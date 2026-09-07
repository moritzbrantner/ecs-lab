use std::{
    collections::{BTreeMap, BTreeSet},
    fmt,
};

use ecs_workload::EntityId;

use crate::{
    AngularState3d, AngularVelocity3d, BoxBoxStabilizationError3d, RigidBox3d, RigidBoxState3d,
    RotatingContactFrontier3d, RotatingContactSet3d, stabilize_box_box_contact,
};

pub const MAX_ROTATING_CONTACT_RESPONSE_PASSES: u8 = 16;

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct RotatingContactResponse3d {
    /// Shared world state after bounded simultaneous response passes at the sampled frontier.
    pub boxes: Vec<RigidBox3d>,
    /// The equal-time sampled contact set consumed by the response passes.
    pub contact_set: RotatingContactSet3d,
    /// Remaining requested-frame fraction numerator over [`Self::contact_set`]'s denominator.
    pub remaining_numerator: u32,
    /// Number of bounded Jacobi-style response passes actually evaluated.
    pub passes_used: u8,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RotatingContactResponseError3d {
    EmptyContactSet,
    NonCanonicalPair(EntityId, EntityId),
    DuplicatePair(EntityId, EntityId),
    FractionMismatch(EntityId, EntityId),
    MissingEntity(EntityId),
    PassesOutOfRange(u8),
    OrientationChanged(EntityId),
    Stabilization(BoxBoxStabilizationError3d),
    ArithmeticOverflow,
}

impl fmt::Display for RotatingContactResponseError3d {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::EmptyContactSet => write!(
                formatter,
                "rotating contact response requires a non-empty contact set"
            ),
            Self::NonCanonicalPair(left, right) => write!(
                formatter,
                "rotating contact response expects ascending entity ids, got {} then {}",
                left.0, right.0
            ),
            Self::DuplicatePair(left, right) => write!(
                formatter,
                "rotating contact response received duplicate pair {}-{}",
                left.0, right.0
            ),
            Self::FractionMismatch(left, right) => write!(
                formatter,
                "rotating contact response pair {}-{} does not share the frontier fraction",
                left.0, right.0
            ),
            Self::MissingEntity(entity) => write!(
                formatter,
                "rotating contact response cannot find entity {} in the frontier world",
                entity.0
            ),
            Self::PassesOutOfRange(value) => write!(
                formatter,
                "rotating contact response passes must be 1..={MAX_ROTATING_CONTACT_RESPONSE_PASSES}, got {value}"
            ),
            Self::OrientationChanged(entity) => write!(
                formatter,
                "rotating contact pair response unexpectedly changed orientation for entity {}",
                entity.0
            ),
            Self::Stabilization(error) => {
                write!(formatter, "rotating contact response failed: {error}")
            }
            Self::ArithmeticOverflow => {
                write!(formatter, "rotating contact response arithmetic overflowed")
            }
        }
    }
}

impl std::error::Error for RotatingContactResponseError3d {}

impl From<BoxBoxStabilizationError3d> for RotatingContactResponseError3d {
    fn from(value: BoxBoxStabilizationError3d) -> Self {
        Self::Stabilization(value)
    }
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
struct StateDelta3d {
    center: [i128; 3],
    linear_velocity: [i128; 3],
    angular_velocity: [i128; 3],
}

impl StateDelta3d {
    fn is_zero(self) -> bool {
        self.center == [0; 3] && self.linear_velocity == [0; 3] && self.angular_velocity == [0; 3]
    }

    fn accumulate(&mut self, other: Self) -> Result<(), RotatingContactResponseError3d> {
        for axis in 0..3 {
            self.center[axis] = self.center[axis]
                .checked_add(other.center[axis])
                .ok_or(RotatingContactResponseError3d::ArithmeticOverflow)?;
            self.linear_velocity[axis] = self.linear_velocity[axis]
                .checked_add(other.linear_velocity[axis])
                .ok_or(RotatingContactResponseError3d::ArithmeticOverflow)?;
            self.angular_velocity[axis] = self.angular_velocity[axis]
                .checked_add(other.angular_velocity[axis])
                .ok_or(RotatingContactResponseError3d::ArithmeticOverflow)?;
        }
        Ok(())
    }
}

/// Applies bounded simultaneous response passes to one verified sampled rotating-contact frontier.
///
/// Every pass snapshots the complete frontier world first. Each equal-time pair is then resolved against
/// that same pass snapshot with the existing Rust-owned OBB normal response, penetration stabilization,
/// and friction helper. Pair-local center, linear-velocity, and angular-velocity changes are accumulated
/// in wide integer deltas and applied to all bodies only after every contact in the set has been evaluated.
/// This Jacobi-style staging prevents discovery/iteration order from changing the representable response.
///
/// Orientation is deliberately frozen at the sampled frontier. The response primitive is allowed to
/// change translational and angular velocity plus discrete penetration correction, but any future change
/// that mutates orientation inside pair response fails closed here. Consuming [`RotatingContactResponse3d::remaining_numerator`]
/// and searching the next contact frontier remain a later solver stage.
///
/// This still does not claim rotational CCD. The contact time comes from the bounded sampled search that
/// produced [`RotatingContactFrontier3d`]; this function only couples response for the admitted equal-time
/// sampled contact set.
///
/// # Errors
///
/// Returns [`RotatingContactResponseError3d`] for malformed pass counts/contact sets, missing entities,
/// underlying pair-response failure, unexpected orientation mutation, or checked arithmetic overflow.
pub fn resolve_rotating_contact_frontier(
    frontier: RotatingContactFrontier3d,
    solver_passes: u8,
) -> Result<RotatingContactResponse3d, RotatingContactResponseError3d> {
    if solver_passes == 0 || solver_passes > MAX_ROTATING_CONTACT_RESPONSE_PASSES {
        return Err(RotatingContactResponseError3d::PassesOutOfRange(
            solver_passes,
        ));
    }
    validate_contact_set(&frontier)?;

    let indices = frontier
        .boxes
        .iter()
        .enumerate()
        .map(|(index, body)| (body.body.entity, index))
        .collect::<BTreeMap<_, _>>();
    for contact in &frontier.contact_set.contacts {
        if !indices.contains_key(&contact.left) {
            return Err(RotatingContactResponseError3d::MissingEntity(contact.left));
        }
        if !indices.contains_key(&contact.right) {
            return Err(RotatingContactResponseError3d::MissingEntity(contact.right));
        }
    }

    let mut boxes = frontier.boxes;
    let mut passes_used = 0_u8;
    for _ in 0..solver_passes {
        passes_used = passes_used
            .checked_add(1)
            .ok_or(RotatingContactResponseError3d::ArithmeticOverflow)?;
        let snapshot = boxes.clone();
        let mut deltas = vec![StateDelta3d::default(); snapshot.len()];
        let mut pair_changed = false;

        for expected in &frontier.contact_set.contacts {
            let left_index = *indices
                .get(&expected.left)
                .ok_or(RotatingContactResponseError3d::MissingEntity(expected.left))?;
            let right_index = *indices.get(&expected.right).ok_or(
                RotatingContactResponseError3d::MissingEntity(expected.right),
            )?;
            let left = snapshot[left_index];
            let right = snapshot[right_index];
            let resolved =
                stabilize_box_box_contact(left.state, left.body, right.state, right.body)?;
            let left_delta = state_delta(left.state, resolved.left, left.body.entity)?;
            let right_delta = state_delta(right.state, resolved.right, right.body.entity)?;
            pair_changed |= !left_delta.is_zero() || !right_delta.is_zero();
            deltas[left_index].accumulate(left_delta)?;
            deltas[right_index].accumulate(right_delta)?;
        }

        if !pair_changed {
            break;
        }

        let mut world_changed = false;
        for (index, delta) in deltas.into_iter().enumerate() {
            let next_state = apply_state_delta(snapshot[index].state, delta)?;
            world_changed |= next_state != snapshot[index].state;
            boxes[index].state = next_state;
        }
        if !world_changed {
            break;
        }
    }

    Ok(RotatingContactResponse3d {
        boxes,
        contact_set: frontier.contact_set,
        remaining_numerator: frontier.remaining_numerator,
        passes_used,
    })
}

fn validate_contact_set(
    frontier: &RotatingContactFrontier3d,
) -> Result<(), RotatingContactResponseError3d> {
    if frontier.contact_set.contacts.is_empty() {
        return Err(RotatingContactResponseError3d::EmptyContactSet);
    }
    let mut pairs = BTreeSet::new();
    for contact in &frontier.contact_set.contacts {
        if contact.left >= contact.right {
            return Err(RotatingContactResponseError3d::NonCanonicalPair(
                contact.left,
                contact.right,
            ));
        }
        if contact.contact_numerator != frontier.contact_set.contact_numerator
            || contact.denominator != frontier.contact_set.denominator
        {
            return Err(RotatingContactResponseError3d::FractionMismatch(
                contact.left,
                contact.right,
            ));
        }
        if !pairs.insert((contact.left, contact.right)) {
            return Err(RotatingContactResponseError3d::DuplicatePair(
                contact.left,
                contact.right,
            ));
        }
    }
    Ok(())
}

fn state_delta(
    before: RigidBoxState3d,
    after: RigidBoxState3d,
    entity: EntityId,
) -> Result<StateDelta3d, RotatingContactResponseError3d> {
    if before.angular.orientation != after.angular.orientation {
        return Err(RotatingContactResponseError3d::OrientationChanged(entity));
    }
    Ok(StateDelta3d {
        center: [
            i128::from(after.center.x) - i128::from(before.center.x),
            i128::from(after.center.y) - i128::from(before.center.y),
            i128::from(after.center.z) - i128::from(before.center.z),
        ],
        linear_velocity: [
            i128::from(after.linear_velocity.x) - i128::from(before.linear_velocity.x),
            i128::from(after.linear_velocity.y) - i128::from(before.linear_velocity.y),
            i128::from(after.linear_velocity.z) - i128::from(before.linear_velocity.z),
        ],
        angular_velocity: [
            i128::from(after.angular.angular_velocity.x)
                - i128::from(before.angular.angular_velocity.x),
            i128::from(after.angular.angular_velocity.y)
                - i128::from(before.angular.angular_velocity.y),
            i128::from(after.angular.angular_velocity.z)
                - i128::from(before.angular.angular_velocity.z),
        ],
    })
}

fn apply_state_delta(
    base: RigidBoxState3d,
    delta: StateDelta3d,
) -> Result<RigidBoxState3d, RotatingContactResponseError3d> {
    Ok(RigidBoxState3d::new(
        ecs_workload::Position::new3(
            add_i64_delta(base.center.x, delta.center[0])?,
            add_i64_delta(base.center.y, delta.center[1])?,
            add_i64_delta(base.center.z, delta.center[2])?,
        ),
        ecs_workload::Velocity::new3(
            add_i32_delta(base.linear_velocity.x, delta.linear_velocity[0])?,
            add_i32_delta(base.linear_velocity.y, delta.linear_velocity[1])?,
            add_i32_delta(base.linear_velocity.z, delta.linear_velocity[2])?,
        ),
        AngularState3d::new(
            base.angular.orientation,
            AngularVelocity3d::new(
                add_i32_delta(base.angular.angular_velocity.x, delta.angular_velocity[0])?,
                add_i32_delta(base.angular.angular_velocity.y, delta.angular_velocity[1])?,
                add_i32_delta(base.angular.angular_velocity.z, delta.angular_velocity[2])?,
            ),
        ),
    ))
}

fn add_i64_delta(value: i64, delta: i128) -> Result<i64, RotatingContactResponseError3d> {
    let result = i128::from(value)
        .checked_add(delta)
        .ok_or(RotatingContactResponseError3d::ArithmeticOverflow)?;
    i64::try_from(result).map_err(|_| RotatingContactResponseError3d::ArithmeticOverflow)
}

fn add_i32_delta(value: i32, delta: i128) -> Result<i32, RotatingContactResponseError3d> {
    let result = i128::from(value)
        .checked_add(delta)
        .ok_or(RotatingContactResponseError3d::ArithmeticOverflow)?;
    i32::try_from(result).map_err(|_| RotatingContactResponseError3d::ArithmeticOverflow)
}

#[cfg(test)]
mod tests {
    use ecs_physics::{BodyKind, MATERIAL_SCALE};
    use ecs_workload::{EntityId, Position, Velocity};

    use crate::{
        ANGULAR_VELOCITY_SCALE, AngularState3d, AngularVelocity3d, Orientation3d, PhysicsBody3d,
        RigidBoxState3d, RigidBoxWorldConfig3d, RotatingContactSearchConfig3d,
        advance_to_earliest_rotating_contact_set,
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

    fn simultaneous_frontier() -> RotatingContactFrontier3d {
        let boxes = [rotating_rod(), obstacle(2, 10, 10), obstacle(3, -10, -10)];
        advance_to_earliest_rotating_contact_set(
            &boxes,
            frame_config(),
            RotatingContactSearchConfig3d::default(),
        )
        .expect("valid frontier search")
        .expect("symmetric rotating contacts should produce a frontier")
    }

    #[test]
    fn coupled_response_is_invariant_to_contact_iteration_order() {
        let frontier = simultaneous_frontier();
        let mut reversed = frontier.clone();
        reversed.contact_set.contacts.reverse();

        let canonical = resolve_rotating_contact_frontier(frontier, 8)
            .expect("valid canonical coupled response");
        let reversed = resolve_rotating_contact_frontier(reversed, 8)
            .expect("valid reversed coupled response");

        assert_eq!(canonical.boxes, reversed.boxes);
        assert_eq!(canonical.remaining_numerator, reversed.remaining_numerator);
    }

    #[test]
    fn fixed_bodies_and_sampled_fraction_are_preserved() {
        let frontier = simultaneous_frontier();
        let fixed_left = frontier.boxes[1];
        let fixed_right = frontier.boxes[2];
        let remaining = frontier.remaining_numerator;
        let denominator = frontier.contact_set.denominator;
        let contact_numerator = frontier.contact_set.contact_numerator;

        let response =
            resolve_rotating_contact_frontier(frontier, 8).expect("valid coupled response");

        assert_eq!(response.boxes[1], fixed_left);
        assert_eq!(response.boxes[2], fixed_right);
        assert_eq!(response.remaining_numerator, remaining);
        assert_eq!(
            response.remaining_numerator + contact_numerator,
            denominator
        );
        assert!(response.passes_used > 0);
    }

    #[test]
    fn response_changes_velocity_without_advancing_frontier_orientation() {
        let frontier = simultaneous_frontier();
        let orientation = frontier.boxes[0].state.angular.orientation;
        let angular_velocity = frontier.boxes[0].state.angular.angular_velocity;

        let response =
            resolve_rotating_contact_frontier(frontier, 8).expect("valid coupled response");

        assert_eq!(response.boxes[0].state.angular.orientation, orientation);
        assert_ne!(
            response.boxes[0].state.angular.angular_velocity,
            angular_velocity
        );
    }

    #[test]
    fn duplicate_contact_pair_fails_closed() {
        let mut frontier = simultaneous_frontier();
        let duplicate = frontier.contact_set.contacts[0];
        frontier.contact_set.contacts.push(duplicate);

        assert_eq!(
            resolve_rotating_contact_frontier(frontier, 8),
            Err(RotatingContactResponseError3d::DuplicatePair(
                duplicate.left,
                duplicate.right,
            ))
        );
    }

    #[test]
    fn zero_response_passes_fail_closed() {
        let frontier = simultaneous_frontier();

        assert_eq!(
            resolve_rotating_contact_frontier(frontier, 0),
            Err(RotatingContactResponseError3d::PassesOutOfRange(0))
        );
    }
}
