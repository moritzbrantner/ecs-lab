use std::{collections::BTreeMap, fmt};

use ecs_physics::BodyKind;
use ecs_workload::{EntityId, Position, Velocity};

use crate::{
    AngularError3d, OrientedBox3d, OrientedBoxError3d, RigidBox3d, RigidBoxWorldConfig3d,
    RotatingContactSearchConfig3d, RotatingContactSearchError3d, RotatingContactSet3d,
    earliest_rotating_contact_set, integrate_orientation, obb_contact_seed,
};

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct RotatingContactFrontier3d {
    /// Free-flight world state sampled at the globally earliest sampled rotating contact fraction.
    pub boxes: Vec<RigidBox3d>,
    /// Equal-time sampled contact constraints discovered from the same start state.
    pub contact_set: RotatingContactSet3d,
    /// Remaining fraction numerator of the requested frame over [`Self::contact_set`]'s denominator.
    pub remaining_numerator: u32,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RotatingContactFrontierError3d {
    MissingEntity(EntityId),
    ContactMissingAtFrontier(EntityId, EntityId),
    ContactChangedAtFrontier(EntityId, EntityId),
    ArithmeticOverflow,
    Search(RotatingContactSearchError3d),
    Angular(AngularError3d),
    Geometry(OrientedBoxError3d),
}

impl fmt::Display for RotatingContactFrontierError3d {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::MissingEntity(entity) => write!(
                formatter,
                "rotating contact frontier cannot find entity {} in the sampled world",
                entity.0
            ),
            Self::ContactMissingAtFrontier(left, right) => write!(
                formatter,
                "rotating contact frontier pair {}-{} is no longer in contact at the set fraction",
                left.0, right.0
            ),
            Self::ContactChangedAtFrontier(left, right) => write!(
                formatter,
                "rotating contact frontier pair {}-{} no longer matches its sampled contact evidence",
                left.0, right.0
            ),
            Self::ArithmeticOverflow => {
                write!(formatter, "rotating contact frontier arithmetic overflowed")
            }
            Self::Search(error) => write!(formatter, "rotating contact frontier search failed: {error}"),
            Self::Angular(error) => write!(formatter, "rotating contact frontier angular sampling failed: {error}"),
            Self::Geometry(error) => write!(formatter, "rotating contact frontier geometry failed: {error}"),
        }
    }
}

impl std::error::Error for RotatingContactFrontierError3d {}

impl From<RotatingContactSearchError3d> for RotatingContactFrontierError3d {
    fn from(value: RotatingContactSearchError3d) -> Self {
        Self::Search(value)
    }
}

impl From<AngularError3d> for RotatingContactFrontierError3d {
    fn from(value: AngularError3d) -> Self {
        Self::Angular(value)
    }
}

impl From<OrientedBoxError3d> for RotatingContactFrontierError3d {
    fn from(value: OrientedBoxError3d) -> Self {
        Self::Geometry(value)
    }
}

/// Advances all bodies from one common start state to the earliest sampled rotating contact frontier.
///
/// The contact search first discovers one equal-time sampled set. This function then re-samples every
/// body directly from the original frame start at that set's exact rational fraction, producing a single
/// authoritative pre-response world state. Every set pair is recomputed against that shared state and
/// must match the contact seed returned by the search; drift fails closed rather than handing inconsistent
/// geometry to a response stage.
///
/// No response, penetration projection, damping, or remaining-frame integration occurs here. This seam
/// only turns independently discovered timing evidence into one shared pre-impact world state. Because
/// the underlying search is bounded sampling, this remains an approximate rotating-contact frontier and
/// does not claim exact rotational CCD.
///
/// # Errors
///
/// Returns [`RotatingContactFrontierError3d`] for search failures, checked sampling arithmetic, missing
/// entities, invalid OBB geometry, or any mismatch between the contact set and the reconstructed shared
/// frontier state.
pub fn advance_to_earliest_rotating_contact_set(
    boxes: &[RigidBox3d],
    frame_config: RigidBoxWorldConfig3d,
    search_config: RotatingContactSearchConfig3d,
) -> Result<Option<RotatingContactFrontier3d>, RotatingContactFrontierError3d> {
    let Some(contact_set) =
        earliest_rotating_contact_set(boxes, frame_config, search_config)?
    else {
        return Ok(None);
    };
    let denominator = contact_set.denominator;
    let numerator = contact_set.contact_numerator;
    let remaining_numerator = denominator
        .checked_sub(numerator)
        .ok_or(RotatingContactFrontierError3d::ArithmeticOverflow)?;
    let sampled = boxes
        .iter()
        .copied()
        .map(|body| sample_body(body, frame_config, numerator, denominator))
        .collect::<Result<Vec<_>, _>>()?;
    verify_contact_set(&sampled, &contact_set)?;

    Ok(Some(RotatingContactFrontier3d {
        boxes: sampled,
        contact_set,
        remaining_numerator,
    }))
}

fn verify_contact_set(
    boxes: &[RigidBox3d],
    contact_set: &RotatingContactSet3d,
) -> Result<(), RotatingContactFrontierError3d> {
    let indices = boxes
        .iter()
        .enumerate()
        .map(|(index, body)| (body.body.entity, index))
        .collect::<BTreeMap<_, _>>();
    for expected in &contact_set.contacts {
        let left_index = *indices
            .get(&expected.left)
            .ok_or(RotatingContactFrontierError3d::MissingEntity(expected.left))?;
        let right_index = *indices
            .get(&expected.right)
            .ok_or(RotatingContactFrontierError3d::MissingEntity(expected.right))?;
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
        )?
        .ok_or(RotatingContactFrontierError3d::ContactMissingAtFrontier(
            expected.left,
            expected.right,
        ))?;
        if actual != expected.contact {
            return Err(RotatingContactFrontierError3d::ContactChangedAtFrontier(
                expected.left,
                expected.right,
            ));
        }
    }
    Ok(())
}

fn sample_body(
    mut body: RigidBox3d,
    frame_config: RigidBoxWorldConfig3d,
    fraction_numerator: u32,
    fraction_denominator: u32,
) -> Result<RigidBox3d, RotatingContactFrontierError3d> {
    if fraction_denominator == 0 {
        return Err(RotatingContactFrontierError3d::ArithmeticOverflow);
    }
    if body.body.kind == BodyKind::Fixed || fraction_numerator == 0 {
        return Ok(body);
    }
    let fraction_numerator = i32::try_from(fraction_numerator)
        .map_err(|_| RotatingContactFrontierError3d::ArithmeticOverflow)?;
    let fraction_denominator = i32::try_from(fraction_denominator)
        .map_err(|_| RotatingContactFrontierError3d::ArithmeticOverflow)?;
    let timestep_numerator = frame_config
        .timestep_numerator
        .checked_mul(fraction_numerator)
        .ok_or(RotatingContactFrontierError3d::ArithmeticOverflow)?;
    let timestep_denominator = frame_config
        .timestep_denominator
        .checked_mul(fraction_denominator)
        .ok_or(RotatingContactFrontierError3d::ArithmeticOverflow)?;

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
    body.state.center = center;
    body.state.linear_velocity = velocity;
    body.state.angular.orientation = integrate_orientation(
        body.state.angular.orientation,
        body.state.angular.angular_velocity,
        timestep_numerator,
        timestep_denominator,
    )?;
    Ok(body)
}

fn integrate_velocity_axis(
    velocity: i32,
    acceleration: i32,
    numerator: i32,
    denominator: i32,
) -> Result<i32, RotatingContactFrontierError3d> {
    let acceleration_step = i128::from(acceleration)
        .checked_mul(i128::from(numerator))
        .ok_or(RotatingContactFrontierError3d::ArithmeticOverflow)?;
    let delta = div_round_nearest(acceleration_step, i128::from(denominator))?;
    let next = i128::from(velocity)
        .checked_add(delta)
        .ok_or(RotatingContactFrontierError3d::ArithmeticOverflow)?;
    i32::try_from(next).map_err(|_| RotatingContactFrontierError3d::ArithmeticOverflow)
}

fn integrate_position_axis(
    position: i64,
    velocity: i32,
    numerator: i32,
    denominator: i32,
) -> Result<i64, RotatingContactFrontierError3d> {
    let velocity_step = i128::from(velocity)
        .checked_mul(i128::from(numerator))
        .ok_or(RotatingContactFrontierError3d::ArithmeticOverflow)?;
    let delta = div_round_nearest(velocity_step, i128::from(denominator))?;
    let next = i128::from(position)
        .checked_add(delta)
        .ok_or(RotatingContactFrontierError3d::ArithmeticOverflow)?;
    i64::try_from(next).map_err(|_| RotatingContactFrontierError3d::ArithmeticOverflow)
}

fn div_round_nearest(
    numerator: i128,
    denominator: i128,
) -> Result<i128, RotatingContactFrontierError3d> {
    if denominator <= 0 {
        return Err(RotatingContactFrontierError3d::ArithmeticOverflow);
    }
    let half = denominator / 2;
    let adjusted = if numerator >= 0 {
        numerator.checked_add(half)
    } else {
        numerator.checked_sub(half)
    }
    .ok_or(RotatingContactFrontierError3d::ArithmeticOverflow)?;
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
    fn frontier_reconstructs_one_shared_equal_time_world() {
        let boxes = [
            rotating_rod(),
            obstacle(2, 10, 10),
            obstacle(3, -10, -10),
        ];
        let frontier = advance_to_earliest_rotating_contact_set(
            &boxes,
            frame_config(),
            RotatingContactSearchConfig3d::default(),
        )
        .expect("valid sampled frontier")
        .expect("rotating rod should reach two sampled contacts");

        assert_eq!(frontier.contact_set.contacts.len(), 2);
        assert_eq!(
            frontier.remaining_numerator + frontier.contact_set.contact_numerator,
            frontier.contact_set.denominator
        );
        assert_eq!(frontier.boxes[1], boxes[1]);
        assert_eq!(frontier.boxes[2], boxes[2]);
        assert_ne!(
            frontier.boxes[0].state.angular.orientation,
            boxes[0].state.angular.orientation
        );
    }

    #[test]
    fn frontier_is_repeatable_bit_for_bit() {
        let boxes = [rotating_rod(), obstacle(2, 10, 10)];
        let first = advance_to_earliest_rotating_contact_set(
            &boxes,
            frame_config(),
            RotatingContactSearchConfig3d::default(),
        )
        .expect("valid first frontier");
        let second = advance_to_earliest_rotating_contact_set(
            &boxes,
            frame_config(),
            RotatingContactSearchConfig3d::default(),
        )
        .expect("valid repeated frontier");

        assert_eq!(first, second);
    }

    #[test]
    fn no_sampled_contact_produces_no_frontier() {
        let boxes = [rotating_rod(), obstacle(2, 120, 120)];

        assert_eq!(
            advance_to_earliest_rotating_contact_set(
                &boxes,
                frame_config(),
                RotatingContactSearchConfig3d::default(),
            )
            .expect("valid empty frontier search"),
            None
        );
    }
}
