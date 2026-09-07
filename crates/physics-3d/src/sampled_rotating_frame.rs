use std::fmt;

use ecs_physics::{BodyKind, MATERIAL_SCALE};

use crate::{
    AngularSubstepError3d, AngularSubstepPolicy3d, AngularVelocity3d, RigidBox3d,
    RigidBoxWorldConfig3d, RotatingContactFrontierError3d, RotatingContactResponseError3d,
    RotatingContactSearchConfig3d, RotatingContactSet3d, advance_to_earliest_rotating_contact_set,
    resolve_rotating_contact_frontier, step_rigid_box_world_substepped,
};

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct SampledRotatingFrameStep3d {
    /// Final world state after the full requested frame has been consumed.
    pub boxes: Vec<RigidBox3d>,
    /// First sampled rotating contact set, when the bounded search found one.
    pub first_contact_set: Option<RotatingContactSet3d>,
    /// Coupled response passes evaluated at the first sampled frontier.
    pub first_response_passes: u8,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SampledRotatingFrameError3d {
    NegativeTimestepNumerator(i32),
    NonPositiveTimestepDenominator(i32),
    DampingOutOfRange(u16),
    Frontier(RotatingContactFrontierError3d),
    Response(RotatingContactResponseError3d),
    Remainder(AngularSubstepError3d),
    ArithmeticOverflow,
}

impl fmt::Display for SampledRotatingFrameError3d {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NegativeTimestepNumerator(value) => write!(
                formatter,
                "sampled rotating frame timestep numerator must be non-negative, got {value}"
            ),
            Self::NonPositiveTimestepDenominator(value) => write!(
                formatter,
                "sampled rotating frame timestep denominator must be positive, got {value}"
            ),
            Self::DampingOutOfRange(value) => write!(
                formatter,
                "sampled rotating frame angular damping must be 0..={MATERIAL_SCALE}, got {value}"
            ),
            Self::Frontier(error) => write!(
                formatter,
                "sampled rotating frame frontier search failed: {error}"
            ),
            Self::Response(error) => write!(
                formatter,
                "sampled rotating frame contact response failed: {error}"
            ),
            Self::Remainder(error) => write!(
                formatter,
                "sampled rotating frame remainder step failed: {error}"
            ),
            Self::ArithmeticOverflow => {
                write!(formatter, "sampled rotating frame arithmetic overflowed")
            }
        }
    }
}

impl std::error::Error for SampledRotatingFrameError3d {}

impl From<RotatingContactFrontierError3d> for SampledRotatingFrameError3d {
    fn from(value: RotatingContactFrontierError3d) -> Self {
        Self::Frontier(value)
    }
}

impl From<RotatingContactResponseError3d> for SampledRotatingFrameError3d {
    fn from(value: RotatingContactResponseError3d) -> Self {
        Self::Response(value)
    }
}

impl From<AngularSubstepError3d> for SampledRotatingFrameError3d {
    fn from(value: AngularSubstepError3d) -> Self {
        Self::Remainder(value)
    }
}

/// Completes one requested OBB frame using sampled first-impact timing plus bounded remainder stepping.
///
/// The frame begins with the existing conservative sweep + bounded sampled OBB search. If that search
/// finds an earliest equal-time contact set, every body is advanced to one shared sampled frontier and
/// the set is resolved through the order-invariant coupled response stage. The unconsumed fraction of
/// the original frame is then reduced to an exact rational timestep and handed to the existing bounded
/// angular-substep world solver. Because neither the sampled frontier nor its response applies angular
/// damping, the remainder solver applies damping only on its final substep: once for the requested frame.
///
/// When the bounded sampled search finds no contact, the whole frame is delegated directly to the bounded
/// angular-substep solver. That preserves its existing conservative intermediate-orientation sampling and
/// once-per-frame damping behavior instead of replacing it with endpoint-only free flight.
///
/// A contact exactly at the end of the requested frame has no remainder to step. In that case this wrapper
/// applies the same integer angular-damping rule once without performing another zero-time collision pass.
///
/// This is intentionally not full rotational CCD. Only the first rotating impact uses the sampled
/// bracket/frontier search. Contacts in the remainder are handled by bounded discrete angular substeps;
/// the function does not recursively search another sampled time of impact after every response.
///
/// # Errors
///
/// Returns [`SampledRotatingFrameError3d`] for malformed frame configuration, sampled-frontier/search
/// failures, coupled response failure, bounded remainder failure, or checked rational arithmetic overflow.
pub fn step_rigid_box_world_sampled_rotating(
    boxes: &[RigidBox3d],
    config: RigidBoxWorldConfig3d,
    search_config: RotatingContactSearchConfig3d,
    substep_policy: AngularSubstepPolicy3d,
) -> Result<SampledRotatingFrameStep3d, SampledRotatingFrameError3d> {
    validate_frame_config(config)?;

    let Some(frontier) = advance_to_earliest_rotating_contact_set(boxes, config, search_config)?
    else {
        let step = step_rigid_box_world_substepped(boxes, config, substep_policy)?;
        return Ok(SampledRotatingFrameStep3d {
            boxes: step.boxes,
            first_contact_set: None,
            first_response_passes: 0,
        });
    };

    let response = resolve_rotating_contact_frontier(frontier, config.solver_passes)?;
    let first_contact_set = response.contact_set.clone();
    let first_response_passes = response.passes_used;

    let boxes = if response.remaining_numerator == 0 {
        let mut boxes = response.boxes;
        damp_world_once(&mut boxes, config.angular_damping_milli)?;
        boxes
    } else {
        let remainder_config = scaled_remainder_config(
            config,
            response.remaining_numerator,
            response.contact_set.denominator,
        )?;
        step_rigid_box_world_substepped(&response.boxes, remainder_config, substep_policy)?.boxes
    };

    Ok(SampledRotatingFrameStep3d {
        boxes,
        first_contact_set: Some(first_contact_set),
        first_response_passes,
    })
}

fn validate_frame_config(config: RigidBoxWorldConfig3d) -> Result<(), SampledRotatingFrameError3d> {
    if config.timestep_numerator < 0 {
        return Err(SampledRotatingFrameError3d::NegativeTimestepNumerator(
            config.timestep_numerator,
        ));
    }
    if config.timestep_denominator <= 0 {
        return Err(SampledRotatingFrameError3d::NonPositiveTimestepDenominator(
            config.timestep_denominator,
        ));
    }
    if config.angular_damping_milli > MATERIAL_SCALE {
        return Err(SampledRotatingFrameError3d::DampingOutOfRange(
            config.angular_damping_milli,
        ));
    }
    Ok(())
}

fn scaled_remainder_config(
    config: RigidBoxWorldConfig3d,
    remaining_numerator: u32,
    sampled_denominator: u32,
) -> Result<RigidBoxWorldConfig3d, SampledRotatingFrameError3d> {
    if sampled_denominator == 0 || remaining_numerator > sampled_denominator {
        return Err(SampledRotatingFrameError3d::ArithmeticOverflow);
    }

    let numerator = u128::from(config.timestep_numerator.unsigned_abs())
        .checked_mul(u128::from(remaining_numerator))
        .ok_or(SampledRotatingFrameError3d::ArithmeticOverflow)?;
    let denominator = u128::from(
        u32::try_from(config.timestep_denominator)
            .map_err(|_| SampledRotatingFrameError3d::ArithmeticOverflow)?,
    )
    .checked_mul(u128::from(sampled_denominator))
    .ok_or(SampledRotatingFrameError3d::ArithmeticOverflow)?;
    let divisor = greatest_common_divisor(numerator, denominator);
    let numerator = numerator / divisor;
    let denominator = denominator / divisor;

    Ok(RigidBoxWorldConfig3d {
        gravity: config.gravity,
        timestep_numerator: i32::try_from(numerator)
            .map_err(|_| SampledRotatingFrameError3d::ArithmeticOverflow)?,
        timestep_denominator: i32::try_from(denominator)
            .map_err(|_| SampledRotatingFrameError3d::ArithmeticOverflow)?,
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
) -> Result<(), SampledRotatingFrameError3d> {
    if damping_milli > MATERIAL_SCALE {
        return Err(SampledRotatingFrameError3d::DampingOutOfRange(
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

fn damp_axis(value: i32, damping_milli: u16) -> Result<i32, SampledRotatingFrameError3d> {
    let numerator = i128::from(value)
        .checked_mul(i128::from(damping_milli))
        .ok_or(SampledRotatingFrameError3d::ArithmeticOverflow)?;
    let damped = div_round_nearest(numerator, i128::from(MATERIAL_SCALE))?;
    i32::try_from(damped).map_err(|_| SampledRotatingFrameError3d::ArithmeticOverflow)
}

fn div_round_nearest(
    numerator: i128,
    denominator: i128,
) -> Result<i128, SampledRotatingFrameError3d> {
    if denominator <= 0 {
        return Err(SampledRotatingFrameError3d::ArithmeticOverflow);
    }
    let half = denominator / 2;
    let adjusted = if numerator >= 0 {
        numerator.checked_add(half)
    } else {
        numerator.checked_sub(half)
    }
    .ok_or(SampledRotatingFrameError3d::ArithmeticOverflow)?;
    Ok(adjusted / denominator)
}

#[cfg(test)]
mod tests {
    use ecs_physics::{BodyKind, MATERIAL_SCALE};
    use ecs_workload::{EntityId, Position, Velocity};

    use crate::{
        ANGULAR_VELOCITY_SCALE, AngularState3d, Orientation3d, PhysicsBody3d, RigidBoxState3d,
        step_rigid_box_world,
    };

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
                Velocity::new3(0, 0, 0),
                AngularState3d::new(Orientation3d::IDENTITY, angular_velocity),
            ),
        )
    }

    fn rotating_rod() -> RigidBox3d {
        body(
            1,
            BodyKind::Dynamic,
            [20, 2, 2],
            Position::new3(0, 0, 0),
            AngularVelocity3d::new(0, 0, 230 * ANGULAR_VELOCITY_SCALE),
        )
    }

    fn obstacle(entity: u32, x: i64, y: i64) -> RigidBox3d {
        body(
            entity,
            BodyKind::Fixed,
            [2, 2, 2],
            Position::new3(x, y, 0),
            AngularVelocity3d::default(),
        )
    }

    #[test]
    fn sampled_first_contact_is_responded_and_remainder_is_consumed() {
        let boxes = [rotating_rod(), obstacle(2, 10, 10)];
        let frontier = advance_to_earliest_rotating_contact_set(
            &boxes,
            frame_config(),
            RotatingContactSearchConfig3d::default(),
        )
        .expect("valid sampled frontier")
        .expect("fixture should have an intermediate sampled contact");
        let frontier_orientation = frontier.boxes[0].state.angular.orientation;
        let fixed = boxes[1];

        let step = step_rigid_box_world_sampled_rotating(
            &boxes,
            frame_config(),
            RotatingContactSearchConfig3d::default(),
            AngularSubstepPolicy3d::default(),
        )
        .expect("valid complete sampled frame");

        assert!(step.first_contact_set.is_some());
        assert!(step.first_response_passes > 0);
        assert_eq!(step.boxes[1], fixed);
        assert_ne!(
            step.boxes[0].state.angular.orientation,
            frontier_orientation
        );
    }

    #[test]
    fn no_sampled_contact_matches_existing_substepped_frame_bit_for_bit() {
        let boxes = [
            body(
                1,
                BodyKind::Dynamic,
                [10, 10, 10],
                Position::new3(0, 20, 0),
                AngularVelocity3d::new(10_000, 20_000, -10_000),
            ),
            obstacle(2, 500, 500),
        ];
        let expected = step_rigid_box_world_substepped(
            &boxes,
            frame_config(),
            AngularSubstepPolicy3d::default(),
        )
        .expect("valid existing substepped frame");
        let actual = step_rigid_box_world_sampled_rotating(
            &boxes,
            frame_config(),
            RotatingContactSearchConfig3d::default(),
            AngularSubstepPolicy3d::default(),
        )
        .expect("valid sampled wrapper fallback");

        assert_eq!(actual.boxes, expected.boxes);
        assert_eq!(actual.first_contact_set, None);
        assert_eq!(actual.first_response_passes, 0);
    }

    #[test]
    fn reduced_remainder_fraction_preserves_exact_requested_time() {
        let remainder = scaled_remainder_config(frame_config(), 256, 512)
            .expect("valid reduced half-frame remainder");

        assert_eq!(remainder.timestep_numerator, 1);
        assert_eq!(remainder.timestep_denominator, 120);
        assert_eq!(remainder.gravity, frame_config().gravity);
        assert_eq!(
            remainder.angular_damping_milli,
            frame_config().angular_damping_milli
        );
        assert_eq!(remainder.solver_passes, frame_config().solver_passes);
    }

    #[test]
    fn frame_final_damping_matches_existing_rigid_box_rule() {
        let body = body(
            1,
            BodyKind::Dynamic,
            [10, 10, 10],
            Position::new3(0, 0, 0),
            AngularVelocity3d::new(123_456, -654_321, 777_777),
        );
        let damping = 937;
        let mut damped = [body];
        damp_world_once(&mut damped, damping).expect("valid direct frame damping");
        let expected = step_rigid_box_world(
            &[body],
            RigidBoxWorldConfig3d {
                gravity: Velocity::new3(0, 0, 0),
                timestep_numerator: 0,
                timestep_denominator: 60,
                angular_damping_milli: damping,
                solver_passes: 1,
            },
        )
        .expect("valid zero-time isolated damping reference");

        assert_eq!(damped[0], expected.boxes[0]);
    }

    #[test]
    fn malformed_frame_configuration_fails_before_sampling() {
        assert_eq!(
            step_rigid_box_world_sampled_rotating(
                &[rotating_rod()],
                RigidBoxWorldConfig3d {
                    timestep_denominator: 0,
                    ..frame_config()
                },
                RotatingContactSearchConfig3d::default(),
                AngularSubstepPolicy3d::default(),
            ),
            Err(SampledRotatingFrameError3d::NonPositiveTimestepDenominator(
                0
            ))
        );
        assert_eq!(
            step_rigid_box_world_sampled_rotating(
                &[rotating_rod()],
                RigidBoxWorldConfig3d {
                    angular_damping_milli: MATERIAL_SCALE + 1,
                    ..frame_config()
                },
                RotatingContactSearchConfig3d::default(),
                AngularSubstepPolicy3d::default(),
            ),
            Err(SampledRotatingFrameError3d::DampingOutOfRange(
                MATERIAL_SCALE + 1,
            ))
        );
    }
}
