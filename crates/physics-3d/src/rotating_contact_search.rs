use std::{collections::BTreeMap, fmt};

use ecs_physics::BodyKind;
use ecs_workload::{EntityId, Position, Velocity};

use crate::{
    AngularError3d, ObbContactSeed3d, OrientedBox3d, OrientedBoxError3d, RigidBox3d,
    RigidBoxWorldConfig3d, RigidBoxWorldError3d, RotationalSweepError3d, integrate_orientation,
    obb_contact_seed, rotational_sweep_candidate_pairs,
};

pub const MAX_ROTATING_CONTACT_SAMPLES: u16 = 64;
pub const MAX_ROTATING_CONTACT_REFINEMENTS: u8 = 12;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct RotatingContactSearchConfig3d {
    pub coarse_samples: u16,
    pub refinement_steps: u8,
}

impl Default for RotatingContactSearchConfig3d {
    fn default() -> Self {
        Self {
            coarse_samples: 8,
            refinement_steps: 6,
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct RotatingContactBracket3d {
    pub left: EntityId,
    pub right: EntityId,
    /// Last sampled fraction known to be clear, over [`Self::denominator`].
    pub clear_numerator: u32,
    /// First sampled fraction known to be in contact, over [`Self::denominator`].
    pub contact_numerator: u32,
    pub denominator: u32,
    pub contact: ObbContactSeed3d,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct RotatingContactSet3d {
    /// Globally earliest sampled contact fraction numerator for this set.
    pub contact_numerator: u32,
    /// Shared search-grid denominator for every bracket in [`Self::contacts`].
    pub denominator: u32,
    /// Canonically ordered pair brackets that share the earliest sampled contact fraction.
    pub contacts: Vec<RotatingContactBracket3d>,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RotatingContactSearchError3d {
    NonCanonicalPair(EntityId, EntityId),
    NonCanonicalWorld(EntityId, EntityId),
    CoarseSamplesOutOfRange(u16),
    RefinementStepsOutOfRange(u8),
    ArithmeticOverflow,
    Angular(AngularError3d),
    Geometry(OrientedBoxError3d),
    Sweep(RotationalSweepError3d),
    World(RigidBoxWorldError3d),
}

impl fmt::Display for RotatingContactSearchError3d {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NonCanonicalPair(left, right) => write!(
                formatter,
                "rotating contact search expects ascending entity ids, got {} then {}",
                left.0, right.0
            ),
            Self::NonCanonicalWorld(left, right) => write!(
                formatter,
                "rotating contact world expects ascending entity ids, got {} then {}",
                left.0, right.0
            ),
            Self::CoarseSamplesOutOfRange(value) => write!(
                formatter,
                "rotating contact coarse samples must be 1..={MAX_ROTATING_CONTACT_SAMPLES}, got {value}"
            ),
            Self::RefinementStepsOutOfRange(value) => write!(
                formatter,
                "rotating contact refinements must be 0..={MAX_ROTATING_CONTACT_REFINEMENTS}, got {value}"
            ),
            Self::ArithmeticOverflow => {
                write!(formatter, "rotating contact search arithmetic overflowed")
            }
            Self::Angular(error) => write!(
                formatter,
                "rotating contact angular sampling failed: {error}"
            ),
            Self::Geometry(error) => write!(formatter, "rotating contact geometry failed: {error}"),
            Self::Sweep(error) => write!(formatter, "rotating contact sweep failed: {error}"),
            Self::World(error) => write!(
                formatter,
                "rotating contact world configuration failed: {error}"
            ),
        }
    }
}

impl std::error::Error for RotatingContactSearchError3d {}

impl From<AngularError3d> for RotatingContactSearchError3d {
    fn from(value: AngularError3d) -> Self {
        Self::Angular(value)
    }
}

impl From<OrientedBoxError3d> for RotatingContactSearchError3d {
    fn from(value: OrientedBoxError3d) -> Self {
        Self::Geometry(value)
    }
}

impl From<RotationalSweepError3d> for RotatingContactSearchError3d {
    fn from(value: RotationalSweepError3d) -> Self {
        Self::Sweep(value)
    }
}

/// Finds and refines the first *sampled* OBB contact for one canonical entity pair within a frame.
///
/// Each sample is integrated directly from the same start state at an exact rational fraction of the
/// requested frame. The first coarse sample that reports an OBB contact establishes a clear/contact
/// bracket; deterministic bisection then refines only that interval. Contact response is never applied
/// during this search, so the result is pre-impact geometry suitable for a later coupled solver stage.
///
/// This is deliberately not exact rotational CCD. A contact island that begins and ends entirely between
/// two coarse samples can be missed. Callers choose the bounded sampling policy explicitly, and
/// [`search_rotating_contacts`] first uses conservative rotational sweep bounds so possible pairs are not
/// discarded by broad phase before this bounded narrow search runs.
///
/// # Errors
///
/// Returns [`RotatingContactSearchError3d`] for non-canonical pair order, malformed search/frame
/// configuration, invalid OBB geometry, or checked arithmetic failure.
pub fn bracket_rotating_contact(
    left: RigidBox3d,
    right: RigidBox3d,
    frame_config: RigidBoxWorldConfig3d,
    search_config: RotatingContactSearchConfig3d,
) -> Result<Option<RotatingContactBracket3d>, RotatingContactSearchError3d> {
    if left.body.entity >= right.body.entity {
        return Err(RotatingContactSearchError3d::NonCanonicalPair(
            left.body.entity,
            right.body.entity,
        ));
    }
    validate_configs(frame_config, search_config)?;
    let final_denominator = final_search_denominator(search_config)?;

    if let Some(contact) = sampled_contact(left, right, frame_config, 0, 1)? {
        return Ok(Some(RotatingContactBracket3d {
            left: left.body.entity,
            right: right.body.entity,
            clear_numerator: 0,
            contact_numerator: 0,
            denominator: final_denominator,
            contact,
        }));
    }

    let coarse_denominator = u32::from(search_config.coarse_samples);
    let mut bracket = None;
    for sample in 1..=search_config.coarse_samples {
        let numerator = u32::from(sample);
        if let Some(contact) =
            sampled_contact(left, right, frame_config, numerator, coarse_denominator)?
        {
            bracket = Some((numerator - 1, numerator, coarse_denominator, contact));
            break;
        }
    }
    let Some((mut clear, mut contact_at, mut denominator, mut contact)) = bracket else {
        return Ok(None);
    };

    for _ in 0..search_config.refinement_steps {
        clear = clear
            .checked_mul(2)
            .ok_or(RotatingContactSearchError3d::ArithmeticOverflow)?;
        contact_at = contact_at
            .checked_mul(2)
            .ok_or(RotatingContactSearchError3d::ArithmeticOverflow)?;
        denominator = denominator
            .checked_mul(2)
            .ok_or(RotatingContactSearchError3d::ArithmeticOverflow)?;
        let midpoint = clear + (contact_at - clear) / 2;
        if let Some(midpoint_contact) =
            sampled_contact(left, right, frame_config, midpoint, denominator)?
        {
            contact_at = midpoint;
            contact = midpoint_contact;
        } else {
            clear = midpoint;
        }
    }

    Ok(Some(RotatingContactBracket3d {
        left: left.body.entity,
        right: right.body.entity,
        clear_numerator: clear,
        contact_numerator: contact_at,
        denominator,
        contact,
    }))
}

/// Searches all conservative rotational-sweep candidates and returns stable sampled-contact brackets.
///
/// End states are generated by the same free-flight rational sampler used for bracket refinement, then
/// [`rotational_sweep_candidate_pairs`] supplies conservative broad-phase candidates. Candidate pairs are
/// searched independently and results are ordered by sampled contact fraction, then entity ids. This
/// preserves deterministic ownership without yet coupling multiple simultaneous rotating contacts.
///
/// # Errors
///
/// Returns [`RotatingContactSearchError3d`] for malformed world/search inputs, unstable entity order,
/// conservative sweep errors, invalid OBB geometry, or checked arithmetic failure.
pub fn search_rotating_contacts(
    boxes: &[RigidBox3d],
    frame_config: RigidBoxWorldConfig3d,
    search_config: RotatingContactSearchConfig3d,
) -> Result<Vec<RotatingContactBracket3d>, RotatingContactSearchError3d> {
    validate_configs(frame_config, search_config)?;
    for pair in boxes.windows(2) {
        if pair[0].body.entity >= pair[1].body.entity {
            return Err(RotatingContactSearchError3d::NonCanonicalWorld(
                pair[0].body.entity,
                pair[1].body.entity,
            ));
        }
    }

    let end = boxes
        .iter()
        .copied()
        .map(|body| sampled_body(body, frame_config, 1, 1))
        .collect::<Result<Vec<_>, _>>()?;
    let candidates = rotational_sweep_candidate_pairs(boxes, &end)?;
    let indices = boxes
        .iter()
        .enumerate()
        .map(|(index, body)| (body.body.entity, index))
        .collect::<BTreeMap<_, _>>();
    let mut contacts = Vec::new();
    for candidate in candidates {
        let left_index = *indices
            .get(&candidate.left)
            .ok_or(RotatingContactSearchError3d::ArithmeticOverflow)?;
        let right_index = *indices
            .get(&candidate.right)
            .ok_or(RotatingContactSearchError3d::ArithmeticOverflow)?;
        if let Some(contact) = bracket_rotating_contact(
            boxes[left_index],
            boxes[right_index],
            frame_config,
            search_config,
        )? {
            contacts.push(contact);
        }
    }
    contacts.sort_by_key(|contact| (contact.contact_numerator, contact.left, contact.right));
    Ok(contacts)
}

/// Collects the globally earliest sampled rotating contacts into one stable contact-set frontier.
///
/// [`search_rotating_contacts`] already normalizes every bracket to the same configured search grid and
/// orders results by sampled contact fraction followed by entity pair. This function retains only the
/// leading equal-time group so a later solver can advance once and resolve all sampled simultaneous
/// constraints together instead of applying pair-order-dependent response while discovering contacts.
///
/// No impulse, projection, or remaining-frame integration happens here. The set is still bounded sampled
/// evidence rather than exact rotational time-of-impact evidence.
///
/// # Errors
///
/// Returns [`RotatingContactSearchError3d`] for the same malformed world/search inputs and geometry errors
/// as [`search_rotating_contacts`].
pub fn earliest_rotating_contact_set(
    boxes: &[RigidBox3d],
    frame_config: RigidBoxWorldConfig3d,
    search_config: RotatingContactSearchConfig3d,
) -> Result<Option<RotatingContactSet3d>, RotatingContactSearchError3d> {
    let contacts = search_rotating_contacts(boxes, frame_config, search_config)?;
    let Some(first) = contacts.first().copied() else {
        return Ok(None);
    };
    let contact_numerator = first.contact_numerator;
    let denominator = first.denominator;
    let contacts = contacts
        .into_iter()
        .take_while(|contact| {
            contact.contact_numerator == contact_numerator && contact.denominator == denominator
        })
        .collect();
    Ok(Some(RotatingContactSet3d {
        contact_numerator,
        denominator,
        contacts,
    }))
}

fn validate_configs(
    frame_config: RigidBoxWorldConfig3d,
    search_config: RotatingContactSearchConfig3d,
) -> Result<(), RotatingContactSearchError3d> {
    if frame_config.timestep_numerator < 0 {
        return Err(RotatingContactSearchError3d::World(
            RigidBoxWorldError3d::NegativeTimestepNumerator(frame_config.timestep_numerator),
        ));
    }
    if frame_config.timestep_denominator <= 0 {
        return Err(RotatingContactSearchError3d::World(
            RigidBoxWorldError3d::NonPositiveTimestepDenominator(frame_config.timestep_denominator),
        ));
    }
    if search_config.coarse_samples == 0
        || search_config.coarse_samples > MAX_ROTATING_CONTACT_SAMPLES
    {
        return Err(RotatingContactSearchError3d::CoarseSamplesOutOfRange(
            search_config.coarse_samples,
        ));
    }
    if search_config.refinement_steps > MAX_ROTATING_CONTACT_REFINEMENTS {
        return Err(RotatingContactSearchError3d::RefinementStepsOutOfRange(
            search_config.refinement_steps,
        ));
    }
    Ok(())
}

fn final_search_denominator(
    search_config: RotatingContactSearchConfig3d,
) -> Result<u32, RotatingContactSearchError3d> {
    let mut denominator = u32::from(search_config.coarse_samples);
    for _ in 0..search_config.refinement_steps {
        denominator = denominator
            .checked_mul(2)
            .ok_or(RotatingContactSearchError3d::ArithmeticOverflow)?;
    }
    Ok(denominator)
}

fn sampled_contact(
    left: RigidBox3d,
    right: RigidBox3d,
    frame_config: RigidBoxWorldConfig3d,
    fraction_numerator: u32,
    fraction_denominator: u32,
) -> Result<Option<ObbContactSeed3d>, RotatingContactSearchError3d> {
    let left = sampled_body(left, frame_config, fraction_numerator, fraction_denominator)?;
    let right = sampled_body(
        right,
        frame_config,
        fraction_numerator,
        fraction_denominator,
    )?;
    let left = OrientedBox3d::new(
        left.state.center,
        left.body.half_extents,
        left.state.angular.orientation,
    );
    let right = OrientedBox3d::new(
        right.state.center,
        right.body.half_extents,
        right.state.angular.orientation,
    );
    Ok(obb_contact_seed(left, right)?)
}

fn sampled_body(
    mut body: RigidBox3d,
    frame_config: RigidBoxWorldConfig3d,
    fraction_numerator: u32,
    fraction_denominator: u32,
) -> Result<RigidBox3d, RotatingContactSearchError3d> {
    if fraction_denominator == 0 {
        return Err(RotatingContactSearchError3d::ArithmeticOverflow);
    }
    if body.body.kind == BodyKind::Fixed || fraction_numerator == 0 {
        return Ok(body);
    }
    let fraction_numerator = i32::try_from(fraction_numerator)
        .map_err(|_| RotatingContactSearchError3d::ArithmeticOverflow)?;
    let fraction_denominator = i32::try_from(fraction_denominator)
        .map_err(|_| RotatingContactSearchError3d::ArithmeticOverflow)?;
    let timestep_numerator = frame_config
        .timestep_numerator
        .checked_mul(fraction_numerator)
        .ok_or(RotatingContactSearchError3d::ArithmeticOverflow)?;
    let timestep_denominator = frame_config
        .timestep_denominator
        .checked_mul(fraction_denominator)
        .ok_or(RotatingContactSearchError3d::ArithmeticOverflow)?;

    let velocity = Velocity::new3(
        integrate_velocity_axis(
            body.state.linear_velocity.x,
            frame_config.gravity.x,
            timestep_numerator,
            timestep_denominator,
        )?,
        integrate_velocity_axis(
            body.state.linear_velocity.y,
            frame_config.gravity.y,
            timestep_numerator,
            timestep_denominator,
        )?,
        integrate_velocity_axis(
            body.state.linear_velocity.z,
            frame_config.gravity.z,
            timestep_numerator,
            timestep_denominator,
        )?,
    );
    let center = Position::new3(
        integrate_position_axis(
            body.state.center.x,
            velocity.x,
            timestep_numerator,
            timestep_denominator,
        )?,
        integrate_position_axis(
            body.state.center.y,
            velocity.y,
            timestep_numerator,
            timestep_denominator,
        )?,
        integrate_position_axis(
            body.state.center.z,
            velocity.z,
            timestep_numerator,
            timestep_denominator,
        )?,
    );
    let orientation = integrate_orientation(
        body.state.angular.orientation,
        body.state.angular.angular_velocity,
        timestep_numerator,
        timestep_denominator,
    )?;
    body.state.center = center;
    body.state.linear_velocity = velocity;
    body.state.angular.orientation = orientation;
    Ok(body)
}

fn integrate_velocity_axis(
    velocity: i32,
    acceleration: i32,
    numerator: i32,
    denominator: i32,
) -> Result<i32, RotatingContactSearchError3d> {
    let acceleration_step = i128::from(acceleration)
        .checked_mul(i128::from(numerator))
        .ok_or(RotatingContactSearchError3d::ArithmeticOverflow)?;
    let delta = div_round_nearest(acceleration_step, i128::from(denominator))?;
    let next = i128::from(velocity)
        .checked_add(delta)
        .ok_or(RotatingContactSearchError3d::ArithmeticOverflow)?;
    i32::try_from(next).map_err(|_| RotatingContactSearchError3d::ArithmeticOverflow)
}

fn integrate_position_axis(
    position: i64,
    velocity: i32,
    numerator: i32,
    denominator: i32,
) -> Result<i64, RotatingContactSearchError3d> {
    let velocity_step = i128::from(velocity)
        .checked_mul(i128::from(numerator))
        .ok_or(RotatingContactSearchError3d::ArithmeticOverflow)?;
    let delta = div_round_nearest(velocity_step, i128::from(denominator))?;
    let next = i128::from(position)
        .checked_add(delta)
        .ok_or(RotatingContactSearchError3d::ArithmeticOverflow)?;
    i64::try_from(next).map_err(|_| RotatingContactSearchError3d::ArithmeticOverflow)
}

fn div_round_nearest(
    numerator: i128,
    denominator: i128,
) -> Result<i128, RotatingContactSearchError3d> {
    if denominator <= 0 {
        return Err(RotatingContactSearchError3d::ArithmeticOverflow);
    }
    let half = denominator / 2;
    let adjusted = if numerator >= 0 {
        numerator.checked_add(half)
    } else {
        numerator.checked_sub(half)
    }
    .ok_or(RotatingContactSearchError3d::ArithmeticOverflow)?;
    Ok(adjusted / denominator)
}

#[cfg(test)]
mod tests {
    use ecs_physics::{BodyKind, MATERIAL_SCALE};
    use ecs_workload::{EntityId, Position, Velocity};

    use crate::{
        ANGULAR_VELOCITY_SCALE, AngularState3d, AngularVelocity3d, Orientation3d, PhysicsBody3d,
        RigidBoxState3d,
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
    fn intermediate_rotating_contact_is_bracketed_and_refined() {
        let rod = rotating_rod();
        let obstacle = obstacle(2, 10, 10);
        let search = RotatingContactSearchConfig3d {
            coarse_samples: 8,
            refinement_steps: 6,
        };
        let bracket = bracket_rotating_contact(rod, obstacle, frame_config(), search)
            .expect("valid bounded search")
            .expect("intermediate rotation should be sampled");

        assert_eq!(bracket.denominator, 512);
        assert_eq!(bracket.contact_numerator - bracket.clear_numerator, 1);
        assert!(bracket.contact_numerator < bracket.denominator);
        assert!(
            sampled_contact(
                rod,
                obstacle,
                frame_config(),
                bracket.clear_numerator,
                bracket.denominator,
            )
            .expect("valid clear sample")
            .is_none()
        );
        assert!(
            sampled_contact(
                rod,
                obstacle,
                frame_config(),
                bracket.contact_numerator,
                bracket.denominator,
            )
            .expect("valid contact sample")
            .is_some()
        );
        assert!(
            sampled_contact(rod, obstacle, frame_config(), 1, 1)
                .expect("valid endpoint sample")
                .is_none()
        );
    }

    #[test]
    fn one_coarse_endpoint_sample_does_not_overclaim_hidden_contact() {
        let result = bracket_rotating_contact(
            rotating_rod(),
            obstacle(2, 10, 10),
            frame_config(),
            RotatingContactSearchConfig3d {
                coarse_samples: 1,
                refinement_steps: 8,
            },
        )
        .expect("valid intentionally coarse search");

        assert!(result.is_none());
    }

    #[test]
    fn world_search_is_repeatable_and_ordered_by_contact_fraction() {
        let boxes = [
            rotating_rod(),
            obstacle(2, 10, 10),
            obstacle(3, 8, 8),
            obstacle(4, 120, 120),
        ];
        let search = RotatingContactSearchConfig3d::default();
        let first = search_rotating_contacts(&boxes, frame_config(), search)
            .expect("valid rotating world search");
        let second = search_rotating_contacts(&boxes, frame_config(), search)
            .expect("repeatable rotating world search");

        assert_eq!(first, second);
        assert!(!first.is_empty());
        assert!(first.windows(2).all(|window| {
            (window[0].contact_numerator, window[0].left, window[0].right)
                <= (window[1].contact_numerator, window[1].left, window[1].right)
        }));
        assert!(first.iter().all(|contact| contact.right != EntityId(4)));
    }

    #[test]
    fn earliest_contact_set_groups_symmetric_simultaneous_pairs() {
        let boxes = [
            rotating_rod(),
            obstacle(2, 10, 10),
            obstacle(3, -10, -10),
            obstacle(4, 120, 120),
        ];
        let set = earliest_rotating_contact_set(
            &boxes,
            frame_config(),
            RotatingContactSearchConfig3d::default(),
        )
        .expect("valid earliest contact-set search")
        .expect("symmetric rod contacts should be sampled");
        let pairs = set
            .contacts
            .iter()
            .map(|contact| (contact.left, contact.right))
            .collect::<Vec<_>>();

        assert_eq!(pairs, vec![(EntityId(1), EntityId(2)), (EntityId(1), EntityId(3))]);
        assert!(set.contact_numerator > 0);
        assert!(set.contacts.iter().all(|contact| {
            contact.contact_numerator == set.contact_numerator && contact.denominator == set.denominator
        }));
    }

    #[test]
    fn earliest_contact_set_is_none_without_sampled_contacts() {
        let boxes = [rotating_rod(), obstacle(2, 120, 120)];

        assert_eq!(
            earliest_rotating_contact_set(
                &boxes,
                frame_config(),
                RotatingContactSearchConfig3d::default(),
            )
            .expect("valid empty contact-set search"),
            None
        );
    }

    #[test]
    fn malformed_sampling_policy_fails_closed() {
        assert_eq!(
            bracket_rotating_contact(
                rotating_rod(),
                obstacle(2, 10, 10),
                frame_config(),
                RotatingContactSearchConfig3d {
                    coarse_samples: 0,
                    refinement_steps: 0,
                },
            ),
            Err(RotatingContactSearchError3d::CoarseSamplesOutOfRange(0))
        );
    }
}
