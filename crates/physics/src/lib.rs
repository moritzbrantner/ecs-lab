use std::{collections::BTreeMap, fmt};

use ecs_workload::{EntityId, EntitySnapshot, Operation, Position, Velocity, WorldSnapshot};
use geometry_kernels::aabb_aabb;
use spatial_kernels::Aabb;

const MAX_EXACT_F32_INTEGER: i64 = 16_777_216;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BodyKind {
    Fixed,
    Dynamic,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct PhysicsBody {
    pub entity: EntityId,
    pub kind: BodyKind,
    pub half_extents: [i32; 2],
}

impl PhysicsBody {
    #[must_use]
    pub const fn dynamic(entity: EntityId, half_extents: [i32; 2]) -> Self {
        Self {
            entity,
            kind: BodyKind::Dynamic,
            half_extents,
        }
    }

    #[must_use]
    pub const fn fixed(entity: EntityId, half_extents: [i32; 2]) -> Self {
        Self {
            entity,
            kind: BodyKind::Fixed,
            half_extents,
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct PhysicsConfig {
    pub gravity: Velocity,
}

impl Default for PhysicsConfig {
    fn default() -> Self {
        Self {
            gravity: Velocity::new(0, -1),
        }
    }
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct PhysicsStepStats {
    pub body_count: usize,
    pub candidate_pairs: usize,
    pub contacts: usize,
    pub resolved_contacts: usize,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct PhysicsStep {
    operations: Vec<Operation>,
    stats: PhysicsStepStats,
}

impl PhysicsStep {
    #[must_use]
    pub fn operations(&self) -> &[Operation] {
        &self.operations
    }

    #[must_use]
    pub const fn stats(&self) -> PhysicsStepStats {
        self.stats
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum PhysicsError {
    DuplicateBody(EntityId),
    MissingEntity(EntityId),
    MissingPosition(EntityId),
    MissingVelocity(EntityId),
    InvalidHalfExtents(EntityId),
    CoordinateOutOfRange(EntityId),
    NonPositiveTicks(i32),
}

impl fmt::Display for PhysicsError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::DuplicateBody(entity) => write!(formatter, "duplicate physics body {}", entity.0),
            Self::MissingEntity(entity) => {
                write!(formatter, "physics body {} is not alive", entity.0)
            }
            Self::MissingPosition(entity) => {
                write!(formatter, "physics body {} has no position", entity.0)
            }
            Self::MissingVelocity(entity) => {
                write!(
                    formatter,
                    "dynamic physics body {} has no velocity",
                    entity.0
                )
            }
            Self::InvalidHalfExtents(entity) => write!(
                formatter,
                "physics body {} has negative AABB half extents",
                entity.0
            ),
            Self::CoordinateOutOfRange(entity) => write!(
                formatter,
                "physics body {} exceeds the exact f32 collision range",
                entity.0
            ),
            Self::NonPositiveTicks(ticks) => {
                write!(
                    formatter,
                    "physics step requires positive ticks, got {ticks}"
                )
            }
        }
    }
}

impl std::error::Error for PhysicsError {}

/// Produces one deterministic physics step from an observable ECS snapshot.
///
/// Dynamic bodies use semi-implicit Euler integration. Every body pair is then visited in
/// ascending entity-id order. `geometry-kernels` owns the AABB overlap decision; this crate owns
/// the intentionally simple equal-mass, fully inelastic response and positional correction.
/// Returned operations are ordinary ECS workload operations, so storage candidates can consume the
/// same result without implementing a physics-specific trait.
///
/// # Errors
///
/// Returns [`PhysicsError`] for malformed body configuration, missing required ECS components,
/// coordinates that cannot be represented exactly by the reusable f32 collision kernel, or a
/// non-positive timestep.
pub fn step(
    snapshot: &WorldSnapshot,
    bodies: &[PhysicsBody],
    config: PhysicsConfig,
    ticks: i32,
) -> Result<PhysicsStep, PhysicsError> {
    if ticks <= 0 {
        return Err(PhysicsError::NonPositiveTicks(ticks));
    }

    let snapshots = snapshot_by_id(snapshot);
    let mut ordered_bodies = bodies.to_vec();
    ordered_bodies.sort_unstable_by_key(|body| body.entity);
    reject_duplicate_bodies(&ordered_bodies)?;

    let mut states = ordered_bodies
        .into_iter()
        .map(|body| BodyState::from_body(body, &snapshots))
        .collect::<Result<Vec<_>, _>>()?;

    for state in &mut states {
        state.integrate(config.gravity, ticks);
        state.validate_geometry_range()?;
    }

    let mut step_stats = PhysicsStepStats {
        body_count: states.len(),
        ..PhysicsStepStats::default()
    };

    for left_index in 0..states.len() {
        for right_index in (left_index + 1)..states.len() {
            step_stats.candidate_pairs = step_stats.candidate_pairs.saturating_add(1);
            let (left_slice, right_slice) = states.split_at_mut(right_index);
            let left = &mut left_slice[left_index];
            let right = &mut right_slice[0];
            let outcome = resolve_pair(left, right)?;
            if outcome.contact {
                step_stats.contacts = step_stats.contacts.saturating_add(1);
            }
            if outcome.resolved {
                step_stats.resolved_contacts = step_stats.resolved_contacts.saturating_add(1);
            }
        }
    }

    Ok(PhysicsStep {
        operations: changed_operations(&states),
        stats: step_stats,
    })
}

fn snapshot_by_id(snapshot: &WorldSnapshot) -> BTreeMap<EntityId, EntitySnapshot> {
    snapshot
        .entities()
        .iter()
        .map(|entity| (entity.id, *entity))
        .collect()
}

fn reject_duplicate_bodies(bodies: &[PhysicsBody]) -> Result<(), PhysicsError> {
    for pair in bodies.windows(2) {
        if pair[0].entity == pair[1].entity {
            return Err(PhysicsError::DuplicateBody(pair[0].entity));
        }
    }
    Ok(())
}

#[derive(Clone, Copy, Debug)]
struct BodyState {
    entity: EntityId,
    kind: BodyKind,
    half_extents: [i64; 2],
    position: Position,
    velocity: Velocity,
    original_position: Position,
    original_velocity: Velocity,
}

impl BodyState {
    fn from_body(
        body: PhysicsBody,
        snapshots: &BTreeMap<EntityId, EntitySnapshot>,
    ) -> Result<Self, PhysicsError> {
        if body.half_extents.iter().any(|extent| *extent < 0) {
            return Err(PhysicsError::InvalidHalfExtents(body.entity));
        }
        let entity = snapshots
            .get(&body.entity)
            .ok_or(PhysicsError::MissingEntity(body.entity))?;
        let position = entity
            .position
            .ok_or(PhysicsError::MissingPosition(body.entity))?;
        let velocity = match body.kind {
            BodyKind::Fixed => Velocity::new(0, 0),
            BodyKind::Dynamic => entity
                .velocity
                .ok_or(PhysicsError::MissingVelocity(body.entity))?,
        };
        let state = Self {
            entity: body.entity,
            kind: body.kind,
            half_extents: [
                i64::from(body.half_extents[0]),
                i64::from(body.half_extents[1]),
            ],
            position,
            velocity,
            original_position: position,
            original_velocity: velocity,
        };
        state.validate_geometry_range()?;
        Ok(state)
    }

    fn integrate(&mut self, gravity: Velocity, ticks: i32) {
        if self.kind == BodyKind::Fixed {
            return;
        }
        self.velocity.x = self
            .velocity
            .x
            .saturating_add(gravity.x.saturating_mul(ticks));
        self.velocity.y = self
            .velocity
            .y
            .saturating_add(gravity.y.saturating_mul(ticks));
        let ticks = i64::from(ticks);
        self.position.x = self
            .position
            .x
            .saturating_add(i64::from(self.velocity.x).saturating_mul(ticks));
        self.position.y = self
            .position
            .y
            .saturating_add(i64::from(self.velocity.y).saturating_mul(ticks));
    }

    fn validate_geometry_range(&self) -> Result<(), PhysicsError> {
        if axis_fits_exact_f32(self.position.x, self.half_extents[0])
            && axis_fits_exact_f32(self.position.y, self.half_extents[1])
        {
            Ok(())
        } else {
            Err(PhysicsError::CoordinateOutOfRange(self.entity))
        }
    }

    fn geometry_aabb(&self) -> Result<Aabb, PhysicsError> {
        self.validate_geometry_range()?;
        Ok(Aabb::from_center_half_extents(
            [
                exact_i64_to_f32(self.position.x),
                exact_i64_to_f32(self.position.y),
                0.0,
            ],
            [
                exact_i64_to_f32(self.half_extents[0]),
                exact_i64_to_f32(self.half_extents[1]),
                0.5,
            ],
        ))
    }

    const fn axis_position(&self, axis: Axis) -> i64 {
        match axis {
            Axis::X => self.position.x,
            Axis::Y => self.position.y,
        }
    }

    const fn axis_velocity(&self, axis: Axis) -> i32 {
        match axis {
            Axis::X => self.velocity.x,
            Axis::Y => self.velocity.y,
        }
    }

    fn offset_axis(&mut self, axis: Axis, delta: i64) {
        match axis {
            Axis::X => self.position.x = self.position.x.saturating_add(delta),
            Axis::Y => self.position.y = self.position.y.saturating_add(delta),
        }
    }

    const fn set_axis_velocity(&mut self, axis: Axis, value: i32) {
        match axis {
            Axis::X => self.velocity.x = value,
            Axis::Y => self.velocity.y = value,
        }
    }
}

#[derive(Clone, Copy)]
enum Axis {
    X,
    Y,
}

#[derive(Clone, Copy, Debug, Default)]
struct PairOutcome {
    contact: bool,
    resolved: bool,
}

fn resolve_pair(left: &mut BodyState, right: &mut BodyState) -> Result<PairOutcome, PhysicsError> {
    let relation = aabb_aabb(left.geometry_aabb()?, right.geometry_aabb()?);
    if !relation.overlaps {
        return Ok(PairOutcome::default());
    }

    let overlap_x = integer_axis_overlap(left, right, Axis::X);
    let overlap_y = integer_axis_overlap(left, right, Axis::Y);
    let (axis, penetration) = if overlap_x <= overlap_y {
        (Axis::X, overlap_x)
    } else {
        (Axis::Y, overlap_y)
    };
    let normal = if right.axis_position(axis) >= left.axis_position(axis) {
        1_i64
    } else {
        -1_i64
    };
    let relative_velocity =
        i64::from(right.axis_velocity(axis)).saturating_sub(i64::from(left.axis_velocity(axis)));
    let approaching = relative_velocity.saturating_mul(normal) < 0;

    let corrected = correct_penetration(left, right, axis, normal, penetration);
    let impulsed = if approaching {
        apply_inelastic_impulse(left, right, axis)
    } else {
        false
    };

    left.validate_geometry_range()?;
    right.validate_geometry_range()?;

    Ok(PairOutcome {
        contact: true,
        resolved: corrected || impulsed,
    })
}

fn integer_axis_overlap(left: &BodyState, right: &BodyState, axis: Axis) -> i64 {
    let left_center = left.axis_position(axis);
    let right_center = right.axis_position(axis);
    let left_half = match axis {
        Axis::X => left.half_extents[0],
        Axis::Y => left.half_extents[1],
    };
    let right_half = match axis {
        Axis::X => right.half_extents[0],
        Axis::Y => right.half_extents[1],
    };
    let left_max = left_center + left_half;
    let right_max = right_center + right_half;
    let left_min = left_center - left_half;
    let right_min = right_center - right_half;
    left_max.min(right_max) - left_min.max(right_min)
}

fn correct_penetration(
    left: &mut BodyState,
    right: &mut BodyState,
    axis: Axis,
    normal: i64,
    penetration: i64,
) -> bool {
    if penetration <= 0 {
        return false;
    }
    match (left.kind, right.kind) {
        (BodyKind::Fixed, BodyKind::Fixed) => false,
        (BodyKind::Dynamic, BodyKind::Fixed) => {
            left.offset_axis(axis, -normal.saturating_mul(penetration));
            true
        }
        (BodyKind::Fixed, BodyKind::Dynamic) => {
            right.offset_axis(axis, normal.saturating_mul(penetration));
            true
        }
        (BodyKind::Dynamic, BodyKind::Dynamic) => {
            let left_share = penetration / 2;
            let right_share = penetration - left_share;
            left.offset_axis(axis, -normal.saturating_mul(left_share));
            right.offset_axis(axis, normal.saturating_mul(right_share));
            true
        }
    }
}

fn apply_inelastic_impulse(left: &mut BodyState, right: &mut BodyState, axis: Axis) -> bool {
    match (left.kind, right.kind) {
        (BodyKind::Fixed, BodyKind::Fixed) => false,
        (BodyKind::Dynamic, BodyKind::Fixed) => {
            left.set_axis_velocity(axis, 0);
            true
        }
        (BodyKind::Fixed, BodyKind::Dynamic) => {
            right.set_axis_velocity(axis, 0);
            true
        }
        (BodyKind::Dynamic, BodyKind::Dynamic) => {
            let average = average_i32(left.axis_velocity(axis), right.axis_velocity(axis));
            left.set_axis_velocity(axis, average);
            right.set_axis_velocity(axis, average);
            true
        }
    }
}

fn changed_operations(states: &[BodyState]) -> Vec<Operation> {
    let mut operations = Vec::with_capacity(states.len().saturating_mul(2));
    for state in states {
        if state.kind == BodyKind::Fixed {
            continue;
        }
        if state.position != state.original_position {
            operations.push(Operation::SetPosition(state.entity, state.position));
        }
        if state.velocity != state.original_velocity {
            operations.push(Operation::SetVelocity(state.entity, state.velocity));
        }
    }
    operations
}

fn axis_fits_exact_f32(position: i64, half_extent: i64) -> bool {
    (0..=MAX_EXACT_F32_INTEGER).contains(&half_extent)
        && ((-MAX_EXACT_F32_INTEGER + half_extent)..=(MAX_EXACT_F32_INTEGER - half_extent))
            .contains(&position)
}

#[allow(clippy::cast_precision_loss)]
fn exact_i64_to_f32(value: i64) -> f32 {
    value as f32
}

fn average_i32(left: i32, right: i32) -> i32 {
    let average = i64::midpoint(i64::from(left), i64::from(right));
    match i32::try_from(average) {
        Ok(value) => value,
        Err(_) if average.is_negative() => i32::MIN,
        Err(_) => i32::MAX,
    }
}

#[cfg(test)]
mod tests {
    use ecs_reference::ReferenceWorld;
    use ecs_workload::{
        EntityId, EntitySnapshot, Operation, Position, Velocity, Workload, WorldSnapshot,
    };

    use super::{PhysicsBody, PhysicsConfig, PhysicsError, PhysicsStepStats, step};

    const NO_GRAVITY: PhysicsConfig = PhysicsConfig {
        gravity: Velocity::new(0, 0),
    };

    #[test]
    fn integrates_gravity_before_position() {
        let entity = EntityId(1);
        let snapshot = WorldSnapshot::new(vec![EntitySnapshot {
            id: entity,
            position: Some(Position::new(0, 10)),
            velocity: Some(Velocity::new(2, 0)),
        }]);

        let result = step(
            &snapshot,
            &[PhysicsBody::dynamic(entity, [1, 1])],
            PhysicsConfig::default(),
            1,
        );

        let physics = match result {
            Ok(physics) => physics,
            Err(error) => panic!("unexpected physics error: {error}"),
        };
        assert_eq!(
            physics.operations(),
            [
                Operation::SetPosition(entity, Position::new(2, 9)),
                Operation::SetVelocity(entity, Velocity::new(2, -1)),
            ]
        );
        assert_eq!(
            physics.stats(),
            PhysicsStepStats {
                body_count: 1,
                candidate_pairs: 0,
                contacts: 0,
                resolved_contacts: 0,
            }
        );
    }

    #[test]
    fn resolves_dynamic_body_against_fixed_wall() {
        let dynamic = EntityId(1);
        let wall = EntityId(2);
        let snapshot = WorldSnapshot::new(vec![
            EntitySnapshot {
                id: dynamic,
                position: Some(Position::new(0, 0)),
                velocity: Some(Velocity::new(2, 0)),
            },
            EntitySnapshot {
                id: wall,
                position: Some(Position::new(3, 0)),
                velocity: None,
            },
        ]);

        let result = step(
            &snapshot,
            &[
                PhysicsBody::dynamic(dynamic, [1, 1]),
                PhysicsBody::fixed(wall, [1, 1]),
            ],
            NO_GRAVITY,
            1,
        );
        let physics = match result {
            Ok(physics) => physics,
            Err(error) => panic!("unexpected physics error: {error}"),
        };

        assert_eq!(
            physics.operations(),
            [
                Operation::SetPosition(dynamic, Position::new(1, 0)),
                Operation::SetVelocity(dynamic, Velocity::new(0, 0)),
            ]
        );
        assert_eq!(physics.stats().candidate_pairs, 1);
        assert_eq!(physics.stats().contacts, 1);
        assert_eq!(physics.stats().resolved_contacts, 1);
    }

    #[test]
    fn touching_uses_geometry_kernel_contact_semantics() {
        let dynamic = EntityId(1);
        let wall = EntityId(2);
        let snapshot = WorldSnapshot::new(vec![
            EntitySnapshot {
                id: dynamic,
                position: Some(Position::new(0, 0)),
                velocity: Some(Velocity::new(0, 0)),
            },
            EntitySnapshot {
                id: wall,
                position: Some(Position::new(2, 0)),
                velocity: None,
            },
        ]);

        let result = step(
            &snapshot,
            &[
                PhysicsBody::dynamic(dynamic, [1, 1]),
                PhysicsBody::fixed(wall, [1, 1]),
            ],
            NO_GRAVITY,
            1,
        );
        let physics = match result {
            Ok(physics) => physics,
            Err(error) => panic!("unexpected physics error: {error}"),
        };

        assert!(physics.operations().is_empty());
        assert_eq!(physics.stats().contacts, 1);
        assert_eq!(physics.stats().resolved_contacts, 0);
    }

    #[test]
    fn body_input_order_does_not_change_equal_mass_response() {
        let left = EntityId(1);
        let right = EntityId(2);
        let snapshot = WorldSnapshot::new(vec![
            EntitySnapshot {
                id: left,
                position: Some(Position::new(-2, 0)),
                velocity: Some(Velocity::new(2, 0)),
            },
            EntitySnapshot {
                id: right,
                position: Some(Position::new(2, 0)),
                velocity: Some(Velocity::new(-2, 0)),
            },
        ]);
        let forward = [
            PhysicsBody::dynamic(left, [1, 1]),
            PhysicsBody::dynamic(right, [1, 1]),
        ];
        let reverse = [forward[1], forward[0]];

        let forward_result = step(&snapshot, &forward, NO_GRAVITY, 1);
        let reverse_result = step(&snapshot, &reverse, NO_GRAVITY, 1);

        assert_eq!(forward_result, reverse_result);
        let physics = match forward_result {
            Ok(physics) => physics,
            Err(error) => panic!("unexpected physics error: {error}"),
        };
        assert_eq!(
            physics.operations(),
            [
                Operation::SetPosition(left, Position::new(-1, 0)),
                Operation::SetVelocity(left, Velocity::new(0, 0)),
                Operation::SetPosition(right, Position::new(1, 0)),
                Operation::SetVelocity(right, Velocity::new(0, 0)),
            ]
        );
    }

    #[test]
    fn rejects_duplicate_body_configuration() {
        let entity = EntityId(7);
        let snapshot = WorldSnapshot::new(vec![EntitySnapshot {
            id: entity,
            position: Some(Position::new(0, 0)),
            velocity: Some(Velocity::new(0, 0)),
        }]);

        assert_eq!(
            step(
                &snapshot,
                &[
                    PhysicsBody::dynamic(entity, [1, 1]),
                    PhysicsBody::dynamic(entity, [1, 1]),
                ],
                NO_GRAVITY,
                1,
            ),
            Err(PhysicsError::DuplicateBody(entity))
        );
    }

    #[test]
    fn generated_operations_replay_through_reference_ecs() {
        let dynamic = EntityId(1);
        let wall = EntityId(2);
        let workload = Workload::new(vec![
            Operation::Spawn(dynamic),
            Operation::SetPosition(dynamic, Position::new(0, 0)),
            Operation::SetVelocity(dynamic, Velocity::new(2, 0)),
            Operation::Spawn(wall),
            Operation::SetPosition(wall, Position::new(3, 0)),
        ]);
        let mut world = ReferenceWorld::new();
        assert_eq!(world.replay(&workload), Ok(()));

        let physics = match step(
            &world.snapshot(),
            &[
                PhysicsBody::dynamic(dynamic, [1, 1]),
                PhysicsBody::fixed(wall, [1, 1]),
            ],
            NO_GRAVITY,
            1,
        ) {
            Ok(physics) => physics,
            Err(error) => panic!("unexpected physics error: {error}"),
        };
        for operation in physics.operations() {
            assert_eq!(world.apply(*operation), Ok(()));
        }

        assert_eq!(
            world.snapshot(),
            WorldSnapshot::new(vec![
                EntitySnapshot {
                    id: dynamic,
                    position: Some(Position::new(1, 0)),
                    velocity: Some(Velocity::new(0, 0)),
                },
                EntitySnapshot {
                    id: wall,
                    position: Some(Position::new(3, 0)),
                    velocity: None,
                },
            ])
        );
    }
}
