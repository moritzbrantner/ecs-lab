use std::{
    collections::{BTreeMap, VecDeque},
    fmt,
};

use ecs_physics::{BodyKind, MATERIAL_SCALE};

use crate::{
    AngularSubstepError3d, AngularSubstepPolicy3d, AngularVelocity3d, OrientedBox3d, RigidBox3d,
    RigidBoxFreeFlightConfig3d, RigidBoxWorldConfig3d, RigidBoxWorldError3d,
    RotatingContactFrontier3d, RotatingContactFrontierError3d, RotatingContactResponseError3d,
    RotatingContactSearchConfig3d, RotatingContactSearchError3d, RotatingContactSet3d,
    advance_to_earliest_rotating_contact_set, earliest_new_rotating_contact_set, obb_contact_seed,
    required_angular_substeps, resolve_rotating_contact_frontier,
    sample_rigid_box_world_free_flight, step_rigid_box_world, step_rigid_box_world_substepped,
};

/// Maximum number of sampled impact frontiers admitted inside one requested frame.
pub const MAX_REPEATED_ROTATING_EVENTS: u16 = 64;
/// Maximum number of bounded discrete tail segments used after the last sampled event.
pub const MAX_REPEATED_ROTATING_TAIL_SEGMENTS: u16 = 64;

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct RepeatedRotatingFrameStep3d {
    /// Final world state after the complete requested frame has been consumed.
    pub boxes: Vec<RigidBox3d>,
    /// First sampled contact set, preserving the existing first-impact inspection seam.
    pub first_contact_set: Option<RotatingContactSet3d>,
    /// Coupled response passes evaluated for the first sampled frontier.
    pub first_response_passes: u8,
    /// Number of sampled event frontiers resolved in this requested frame.
    pub sampled_events: u16,
    /// Number of bounded discrete tail segments executed after the final sampled event.
    pub tail_segments: u16,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RepeatedRotatingFrameError3d {
    NegativeTimestepNumerator(i32),
    NonPositiveTimestepDenominator(i32),
    DampingOutOfRange(u16),
    EventLimit { maximum: u16 },
    TailSegmentLimit { maximum: u16 },
    Search(RotatingContactSearchError3d),
    Frontier(RotatingContactFrontierError3d),
    Response(RotatingContactResponseError3d),
    Stabilization(RigidBoxWorldError3d),
    Tail(AngularSubstepError3d),
    ArithmeticOverflow,
}

impl fmt::Display for RepeatedRotatingFrameError3d {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NegativeTimestepNumerator(value) => write!(
                formatter,
                "repeated rotating frame timestep numerator must be non-negative, got {value}"
            ),
            Self::NonPositiveTimestepDenominator(value) => write!(
                formatter,
                "repeated rotating frame timestep denominator must be positive, got {value}"
            ),
            Self::DampingOutOfRange(value) => write!(
                formatter,
                "repeated rotating frame angular damping must be 0..={MATERIAL_SCALE}, got {value}"
            ),
            Self::EventLimit { maximum } => write!(
                formatter,
                "repeated rotating frame exceeded the bounded {maximum}-event budget"
            ),
            Self::TailSegmentLimit { maximum } => write!(
                formatter,
                "repeated rotating frame tail exceeded the bounded {maximum}-segment budget"
            ),
            Self::Search(error) => write!(
                formatter,
                "repeated rotating frame positive-progress search failed: {error}"
            ),
            Self::Frontier(error) => write!(
                formatter,
                "repeated rotating frame frontier reconstruction failed: {error}"
            ),
            Self::Response(error) => write!(
                formatter,
                "repeated rotating frame coupled response failed: {error}"
            ),
            Self::Stabilization(error) => write!(
                formatter,
                "repeated rotating frame persistent-contact stabilization failed: {error}"
            ),
            Self::Tail(error) => write!(
                formatter,
                "repeated rotating frame bounded tail failed: {error}"
            ),
            Self::ArithmeticOverflow => {
                write!(formatter, "repeated rotating frame arithmetic overflowed")
            }
        }
    }
}

impl std::error::Error for RepeatedRotatingFrameError3d {}

impl From<RotatingContactSearchError3d> for RepeatedRotatingFrameError3d {
    fn from(value: RotatingContactSearchError3d) -> Self {
        Self::Search(value)
    }
}

impl From<RotatingContactFrontierError3d> for RepeatedRotatingFrameError3d {
    fn from(value: RotatingContactFrontierError3d) -> Self {
        Self::Frontier(value)
    }
}

impl From<RotatingContactResponseError3d> for RepeatedRotatingFrameError3d {
    fn from(value: RotatingContactResponseError3d) -> Self {
        Self::Response(value)
    }
}

impl From<RigidBoxWorldError3d> for RepeatedRotatingFrameError3d {
    fn from(value: RigidBoxWorldError3d) -> Self {
        Self::Stabilization(value)
    }
}

impl From<AngularSubstepError3d> for RepeatedRotatingFrameError3d {
    fn from(value: AngularSubstepError3d) -> Self {
        Self::Tail(value)
    }
}

/// Completes one OBB frame with bounded repeated sampled rotating-impact events.
///
/// The first frontier uses the ordinary sampled rotating search so existing zero-time constraints remain
/// visible and can be stabilized. After each response, the unconsumed exact rational remainder is searched
/// for the next strictly-positive event. This guarantees sampled time progress rather than rediscovering a
/// resting pair forever. Every admitted event is advanced from one common segment-start state to one shared
/// frontier and resolved with the existing deterministic equal-time Jacobi response.
///
/// Existing contacts are not discarded while later impacts are considered. Immediately after each sampled
/// response, a zero-time, unity-damping rigid-box solve re-stabilizes every contact present at that frontier.
/// This keeps resting/support constraints in the solver instead of allowing free-flight toward an unrelated
/// later impact to turn them into unbounded penetration. When no further positive sampled event exists, the
/// remaining time is consumed through the existing bounded angular-substep solver, with exact rational
/// segment splitting if the current angular velocity exceeds one admitted tail segment.
///
/// Angular damping is applied exactly once for the requested frame: by the final discrete tail segment, or
/// explicitly when an event lands exactly at frame end. Intermediate event/frontier work always uses unity
/// damping. Event count and tail-segment count are independently bounded and fail closed on pathological
/// motion rather than silently dropping simulation work.
///
/// This remains bounded sampled rotational collision handling, not analytic rotational CCD. A contact island
/// can still exist entirely between configured samples, and a pair that is already touching at a segment
/// start is not separately searched for a later clear-then-recontact transition inside that same segment.
/// Those limitations are explicit; the improvement here is that multiple distinct sampled impacts in one
/// frame are no longer collapsed into a discrete remainder.
///
/// # Errors
///
/// Returns [`RepeatedRotatingFrameError3d`] for malformed configuration, bounded search/frontier/response
/// failures, event/tail budget exhaustion, persistent-contact stabilization failure, or checked arithmetic
/// overflow.
pub fn step_rigid_box_world_repeated_rotating(
    boxes: &[RigidBox3d],
    config: RigidBoxWorldConfig3d,
    search_config: RotatingContactSearchConfig3d,
    substep_policy: AngularSubstepPolicy3d,
) -> Result<RepeatedRotatingFrameStep3d, RepeatedRotatingFrameError3d> {
    validate_frame_config(config)?;

    let Some(frontier) = advance_to_earliest_rotating_contact_set(boxes, config, search_config)?
    else {
        let step = step_rigid_box_world_substepped(boxes, config, substep_policy)?;
        return Ok(RepeatedRotatingFrameStep3d {
            boxes: step.boxes,
            first_contact_set: None,
            first_response_passes: 0,
            sampled_events: 0,
            tail_segments: 0,
        });
    };

    let response = resolve_rotating_contact_frontier(frontier, config.solver_passes)?;
    let first_contact_set = response.contact_set.clone();
    let first_response_passes = response.passes_used;
    let mut sampled_events = 1_u16;
    let mut current = stabilize_current_contacts(&response.boxes, config)?;

    if response.remaining_numerator == 0 {
        damp_world_once(&mut current, config.angular_damping_milli)?;
        return Ok(RepeatedRotatingFrameStep3d {
            boxes: current,
            first_contact_set: Some(first_contact_set),
            first_response_passes,
            sampled_events,
            tail_segments: 0,
        });
    }

    let mut remaining_config = scaled_remainder_config(
        config,
        response.remaining_numerator,
        response.contact_set.denominator,
    )?;

    loop {
        let Some(contact_set) =
            earliest_new_rotating_contact_set(&current, remaining_config, search_config)?
        else {
            let (boxes, tail_segments) =
                step_bounded_tail_segments(&current, remaining_config, substep_policy)?;
            return Ok(RepeatedRotatingFrameStep3d {
                boxes,
                first_contact_set: Some(first_contact_set),
                first_response_passes,
                sampled_events,
                tail_segments,
            });
        };

        if sampled_events >= MAX_REPEATED_ROTATING_EVENTS {
            return Err(RepeatedRotatingFrameError3d::EventLimit {
                maximum: MAX_REPEATED_ROTATING_EVENTS,
            });
        }

        let frontier = advance_to_contact_set(&current, remaining_config, contact_set)?;
        let response = resolve_rotating_contact_frontier(frontier, config.solver_passes)?;
        sampled_events = sampled_events
            .checked_add(1)
            .ok_or(RepeatedRotatingFrameError3d::ArithmeticOverflow)?;
        current = stabilize_current_contacts(&response.boxes, config)?;

        if response.remaining_numerator == 0 {
            damp_world_once(&mut current, config.angular_damping_milli)?;
            return Ok(RepeatedRotatingFrameStep3d {
                boxes: current,
                first_contact_set: Some(first_contact_set),
                first_response_passes,
                sampled_events,
                tail_segments: 0,
            });
        }

        remaining_config = scaled_remainder_config(
            remaining_config,
            response.remaining_numerator,
            response.contact_set.denominator,
        )?;
    }
}

fn validate_frame_config(
    config: RigidBoxWorldConfig3d,
) -> Result<(), RepeatedRotatingFrameError3d> {
    if config.timestep_numerator < 0 {
        return Err(RepeatedRotatingFrameError3d::NegativeTimestepNumerator(
            config.timestep_numerator,
        ));
    }
    if config.timestep_denominator <= 0 {
        return Err(
            RepeatedRotatingFrameError3d::NonPositiveTimestepDenominator(
                config.timestep_denominator,
            ),
        );
    }
    if config.angular_damping_milli > MATERIAL_SCALE {
        return Err(RepeatedRotatingFrameError3d::DampingOutOfRange(
            config.angular_damping_milli,
        ));
    }
    Ok(())
}

fn advance_to_contact_set(
    boxes: &[RigidBox3d],
    frame_config: RigidBoxWorldConfig3d,
    contact_set: RotatingContactSet3d,
) -> Result<RotatingContactFrontier3d, RepeatedRotatingFrameError3d> {
    let denominator = contact_set.denominator;
    let numerator = contact_set.contact_numerator;
    let remaining_numerator = denominator
        .checked_sub(numerator)
        .ok_or(RepeatedRotatingFrameError3d::ArithmeticOverflow)?;
    let sampled = sample_rigid_box_world_free_flight(
        boxes,
        RigidBoxFreeFlightConfig3d {
            gravity: frame_config.gravity,
            timestep_numerator: frame_config.timestep_numerator,
            timestep_denominator: frame_config.timestep_denominator,
        },
        numerator,
        denominator,
    )
    .map_err(RotatingContactFrontierError3d::from)?;
    verify_contact_set(&sampled, &contact_set)?;

    Ok(RotatingContactFrontier3d {
        boxes: sampled,
        contact_set,
        remaining_numerator,
    })
}

fn verify_contact_set(
    boxes: &[RigidBox3d],
    contact_set: &RotatingContactSet3d,
) -> Result<(), RepeatedRotatingFrameError3d> {
    let indices = boxes
        .iter()
        .enumerate()
        .map(|(index, body)| (body.body.entity, index))
        .collect::<BTreeMap<_, _>>();
    for expected in &contact_set.contacts {
        let left_index = *indices
            .get(&expected.left)
            .ok_or(RotatingContactFrontierError3d::MissingEntity(expected.left))?;
        let right_index =
            *indices
                .get(&expected.right)
                .ok_or(RotatingContactFrontierError3d::MissingEntity(
                    expected.right,
                ))?;
        let left = boxes[left_index];
        let right = boxes[right_index];
        let actual = obb_contact_seed(
            OrientedBox3d::new(
                left.state.center,
                left.body.half_extents,
                left.state.angular.orientation,
            ),
            OrientedBox3d::new(
                right.state.center,
                right.body.half_extents,
                right.state.angular.orientation,
            ),
        )
        .map_err(RotatingContactFrontierError3d::from)?
        .ok_or(RotatingContactFrontierError3d::ContactMissingAtFrontier(
            expected.left,
            expected.right,
        ))?;
        if actual != expected.contact {
            return Err(RotatingContactFrontierError3d::ContactChangedAtFrontier(
                expected.left,
                expected.right,
            )
            .into());
        }
    }
    Ok(())
}

fn stabilize_current_contacts(
    boxes: &[RigidBox3d],
    config: RigidBoxWorldConfig3d,
) -> Result<Vec<RigidBox3d>, RepeatedRotatingFrameError3d> {
    let step = step_rigid_box_world(
        boxes,
        RigidBoxWorldConfig3d {
            gravity: ecs_workload::Velocity::new3(0, 0, 0),
            timestep_numerator: 0,
            timestep_denominator: config.timestep_denominator,
            angular_damping_milli: MATERIAL_SCALE,
            solver_passes: config.solver_passes,
        },
    )?;
    Ok(step.boxes)
}

fn step_bounded_tail_segments(
    boxes: &[RigidBox3d],
    remainder_config: RigidBoxWorldConfig3d,
    policy: AngularSubstepPolicy3d,
) -> Result<(Vec<RigidBox3d>, u16), RepeatedRotatingFrameError3d> {
    let mut current = boxes.to_vec();
    let mut segments = VecDeque::from([remainder_config]);
    let mut planned_segments = 1_u16;
    let mut executed_segments = 0_u16;

    while let Some(segment) = segments.pop_front() {
        match required_angular_substeps(&current, segment, policy) {
            Ok(_) => {
                let final_segment = segments.is_empty();
                let mut step_config = segment;
                if !final_segment {
                    step_config.angular_damping_milli = MATERIAL_SCALE;
                }
                current = step_rigid_box_world_substepped(&current, step_config, policy)?.boxes;
                executed_segments = executed_segments
                    .checked_add(1)
                    .ok_or(RepeatedRotatingFrameError3d::ArithmeticOverflow)?;
            }
            Err(AngularSubstepError3d::RequiredSubstepsExceeded { .. }) => {
                if planned_segments >= MAX_REPEATED_ROTATING_TAIL_SEGMENTS {
                    return Err(RepeatedRotatingFrameError3d::TailSegmentLimit {
                        maximum: MAX_REPEATED_ROTATING_TAIL_SEGMENTS,
                    });
                }
                let half = scale_timestep_config(segment, 1, 2)?;
                segments.push_front(half);
                segments.push_front(half);
                planned_segments = planned_segments
                    .checked_add(1)
                    .ok_or(RepeatedRotatingFrameError3d::ArithmeticOverflow)?;
            }
            Err(error) => return Err(error.into()),
        }
    }

    Ok((current, executed_segments))
}

fn scaled_remainder_config(
    config: RigidBoxWorldConfig3d,
    remaining_numerator: u32,
    sampled_denominator: u32,
) -> Result<RigidBoxWorldConfig3d, RepeatedRotatingFrameError3d> {
    if sampled_denominator == 0 || remaining_numerator > sampled_denominator {
        return Err(RepeatedRotatingFrameError3d::ArithmeticOverflow);
    }
    scale_timestep_config(config, remaining_numerator, sampled_denominator)
}

fn scale_timestep_config(
    config: RigidBoxWorldConfig3d,
    fraction_numerator: u32,
    fraction_denominator: u32,
) -> Result<RigidBoxWorldConfig3d, RepeatedRotatingFrameError3d> {
    if fraction_denominator == 0 || fraction_numerator > fraction_denominator {
        return Err(RepeatedRotatingFrameError3d::ArithmeticOverflow);
    }

    let numerator = u128::from(config.timestep_numerator.unsigned_abs())
        .checked_mul(u128::from(fraction_numerator))
        .ok_or(RepeatedRotatingFrameError3d::ArithmeticOverflow)?;
    let denominator = u128::from(
        u32::try_from(config.timestep_denominator)
            .map_err(|_| RepeatedRotatingFrameError3d::ArithmeticOverflow)?,
    )
    .checked_mul(u128::from(fraction_denominator))
    .ok_or(RepeatedRotatingFrameError3d::ArithmeticOverflow)?;
    let divisor = greatest_common_divisor(numerator, denominator);
    let numerator = numerator / divisor;
    let denominator = denominator / divisor;

    Ok(RigidBoxWorldConfig3d {
        gravity: config.gravity,
        timestep_numerator: i32::try_from(numerator)
            .map_err(|_| RepeatedRotatingFrameError3d::ArithmeticOverflow)?,
        timestep_denominator: i32::try_from(denominator)
            .map_err(|_| RepeatedRotatingFrameError3d::ArithmeticOverflow)?,
        angular_damping_milli: config.angular_damping_milli,
        solver_passes: config.solver_passes,
    })
}

fn greatest_common_divisor(mut left: u128, mut right: u128) -> u128 {
    while right != 0 {
        let remainder = left % right;
        left = right;
        right = remainder;
    }
    left.max(1)
}

fn damp_world_once(
    boxes: &mut [RigidBox3d],
    damping_milli: u16,
) -> Result<(), RepeatedRotatingFrameError3d> {
    if damping_milli > MATERIAL_SCALE {
        return Err(RepeatedRotatingFrameError3d::DampingOutOfRange(
            damping_milli,
        ));
    }
    for rigid_box in boxes {
        if rigid_box.body.kind != BodyKind::Dynamic {
            continue;
        }
        let velocity = rigid_box.state.angular.angular_velocity;
        rigid_box.state.angular.angular_velocity = AngularVelocity3d::new(
            damp_axis(velocity.x, damping_milli)?,
            damp_axis(velocity.y, damping_milli)?,
            damp_axis(velocity.z, damping_milli)?,
        );
    }
    Ok(())
}

fn damp_axis(value: i32, damping_milli: u16) -> Result<i32, RepeatedRotatingFrameError3d> {
    let numerator = i128::from(value)
        .checked_mul(i128::from(damping_milli))
        .ok_or(RepeatedRotatingFrameError3d::ArithmeticOverflow)?;
    let damped = div_round_nearest(numerator, i128::from(MATERIAL_SCALE))?;
    i32::try_from(damped).map_err(|_| RepeatedRotatingFrameError3d::ArithmeticOverflow)
}

fn div_round_nearest(
    numerator: i128,
    denominator: i128,
) -> Result<i128, RepeatedRotatingFrameError3d> {
    if denominator <= 0 {
        return Err(RepeatedRotatingFrameError3d::ArithmeticOverflow);
    }
    let half = denominator / 2;
    let adjusted = if numerator >= 0 {
        numerator.checked_add(half)
    } else {
        numerator.checked_sub(half)
    }
    .ok_or(RepeatedRotatingFrameError3d::ArithmeticOverflow)?;
    Ok(adjusted / denominator)
}

#[cfg(test)]
mod tests {
    use ecs_physics::{BodyKind, MATERIAL_SCALE};
    use ecs_workload::{EntityId, Position, Velocity};

    use crate::{AngularState3d, Orientation3d, PhysicsBody3d, RigidBoxState3d};

    use super::*;

    fn frame_config() -> RigidBoxWorldConfig3d {
        RigidBoxWorldConfig3d {
            gravity: Velocity::new3(0, 0, 0),
            timestep_numerator: 1,
            timestep_denominator: 60,
            angular_damping_milli: MATERIAL_SCALE,
            solver_passes: 8,
        }
    }

    fn body(
        entity: u32,
        kind: BodyKind,
        half_extents: [i32; 3],
        center: Position,
        linear_velocity: Velocity,
        angular_velocity: AngularVelocity3d,
    ) -> RigidBox3d {
        let physics_body = match kind {
            BodyKind::Dynamic => PhysicsBody3d::dynamic(EntityId(entity), half_extents),
            BodyKind::Fixed => PhysicsBody3d::fixed(EntityId(entity), half_extents),
        };
        RigidBox3d::new(
            physics_body,
            RigidBoxState3d::new(
                center,
                linear_velocity,
                AngularState3d::new(Orientation3d::IDENTITY, angular_velocity),
            ),
        )
    }

    fn moving_box(entity: u32, y: i64) -> RigidBox3d {
        body(
            entity,
            BodyKind::Dynamic,
            [2, 2, 2],
            Position::new3(0, y, 0),
            Velocity::new3(6_000, 0, 0),
            AngularVelocity3d::default(),
        )
    }

    fn obstacle(entity: u32, x: i64, y: i64) -> RigidBox3d {
        body(
            entity,
            BodyKind::Fixed,
            [8, 8, 8],
            Position::new3(x, y, 0),
            Velocity::new3(0, 0, 0),
            AngularVelocity3d::default(),
        )
    }

    #[test]
    fn resolves_two_distinct_sampled_impacts_in_one_requested_frame() {
        let boxes = [
            moving_box(1, 0),
            obstacle(2, 30, 0),
            moving_box(3, 40),
            obstacle(4, 70, 40),
        ];
        let fixed_first = boxes[1];
        let fixed_second = boxes[3];

        let step = step_rigid_box_world_repeated_rotating(
            &boxes,
            frame_config(),
            RotatingContactSearchConfig3d::default(),
            AngularSubstepPolicy3d::default(),
        )
        .expect("two independent impacts should fit the repeated event budget");

        assert_eq!(step.sampled_events, 2);
        let first = step
            .first_contact_set
            .as_ref()
            .expect("first impact should be retained");
        assert_eq!(first.contacts.len(), 1);
        assert_eq!(first.contacts[0].left, EntityId(1));
        assert_eq!(first.contacts[0].right, EntityId(2));
        assert_eq!(step.boxes[1], fixed_first);
        assert_eq!(step.boxes[3], fixed_second);
    }

    #[test]
    fn existing_start_contact_does_not_block_a_later_independent_impact() {
        let boxes = [
            body(
                1,
                BodyKind::Dynamic,
                [2, 2, 2],
                Position::new3(0, 0, 0),
                Velocity::new3(0, 0, 0),
                AngularVelocity3d::default(),
            ),
            obstacle(2, 10, 0),
            moving_box(3, 40),
            obstacle(4, 30, 40),
        ];

        let step = step_rigid_box_world_repeated_rotating(
            &boxes,
            frame_config(),
            RotatingContactSearchConfig3d::default(),
            AngularSubstepPolicy3d::default(),
        )
        .expect("resting start contact should not stall positive progress");

        assert_eq!(step.sampled_events, 2);
        let first = step
            .first_contact_set
            .as_ref()
            .expect("start constraint should be retained");
        assert_eq!(first.contact_numerator, 0);
        assert_eq!(first.contacts[0].left, EntityId(1));
        assert_eq!(first.contacts[0].right, EntityId(2));
    }

    #[test]
    fn no_sampled_contact_matches_existing_substepped_solver_bit_for_bit() {
        let boxes = [
            body(
                1,
                BodyKind::Dynamic,
                [4, 4, 4],
                Position::new3(0, 0, 0),
                Velocity::new3(30, 20, -10),
                AngularVelocity3d::new(10_000, -20_000, 30_000),
            ),
            obstacle(2, 500, 500),
        ];
        let expected = step_rigid_box_world_substepped(
            &boxes,
            frame_config(),
            AngularSubstepPolicy3d::default(),
        )
        .expect("valid existing bounded angular step");
        let actual = step_rigid_box_world_repeated_rotating(
            &boxes,
            frame_config(),
            RotatingContactSearchConfig3d::default(),
            AngularSubstepPolicy3d::default(),
        )
        .expect("valid repeated-event no-contact fallback");

        assert_eq!(actual.boxes, expected.boxes);
        assert_eq!(actual.first_contact_set, None);
        assert_eq!(actual.first_response_passes, 0);
        assert_eq!(actual.sampled_events, 0);
        assert_eq!(actual.tail_segments, 0);
    }

    #[test]
    fn repeated_event_frame_is_bitwise_repeatable() {
        let boxes = [
            moving_box(1, 0),
            obstacle(2, 30, 0),
            moving_box(3, 40),
            obstacle(4, 70, 40),
        ];
        let first = step_rigid_box_world_repeated_rotating(
            &boxes,
            frame_config(),
            RotatingContactSearchConfig3d::default(),
            AngularSubstepPolicy3d::default(),
        )
        .expect("valid first repeated frame");
        let second = step_rigid_box_world_repeated_rotating(
            &boxes,
            frame_config(),
            RotatingContactSearchConfig3d::default(),
            AngularSubstepPolicy3d::default(),
        )
        .expect("valid repeated replay");

        assert_eq!(first, second);
    }

    #[test]
    fn malformed_configuration_fails_before_sampling() {
        assert_eq!(
            step_rigid_box_world_repeated_rotating(
                &[moving_box(1, 0)],
                RigidBoxWorldConfig3d {
                    timestep_denominator: 0,
                    ..frame_config()
                },
                RotatingContactSearchConfig3d::default(),
                AngularSubstepPolicy3d::default(),
            ),
            Err(RepeatedRotatingFrameError3d::NonPositiveTimestepDenominator(0))
        );
    }
}
