use std::{collections::BTreeMap, fmt};

use ecs_physics::{PhysicsBody, PhysicsConfig, PhysicsError, PhysicsStep, step};
use ecs_reference::ReferenceWorld;
use ecs_workload::{EntityId, EntitySnapshot, Operation, Position, Velocity, Workload, WorkloadError, WorldSnapshot};
use geometry_kernels::aabb_aabb;
use spatial_kernels::Aabb;

const FALLING_BOX_COLUMNS: u32 = 16;
const BOX_HALF_EXTENTS: [i32; 2] = [1, 1];
const FLOOR_HALF_EXTENTS: [i32; 2] = [26, 1];
const MAX_EXACT_F32_INTEGER: i64 = 16_777_216;

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct FallingBoxesScenario {
    dynamic_count: u32,
    setup: Workload,
    bodies: Vec<PhysicsBody>,
}

impl FallingBoxesScenario {
    #[must_use]
    pub fn new(dynamic_count: u32) -> Self {
        let mut operations = Vec::new();
        let mut bodies = Vec::new();

        for raw_id in 0..dynamic_count {
            let entity = EntityId(raw_id);
            let column = raw_id % FALLING_BOX_COLUMNS;
            let row = raw_id / FALLING_BOX_COLUMNS;
            let position = Position::new(i64::from(column) * 3 - 22, i64::from(row) * 3 + 3);
            operations.push(Operation::Spawn(entity));
            operations.push(Operation::SetPosition(entity, position));
            operations.push(Operation::SetVelocity(entity, Velocity::new(0, 0)));
            bodies.push(PhysicsBody::dynamic(entity, BOX_HALF_EXTENTS));
        }

        let floor = EntityId(dynamic_count);
        operations.push(Operation::Spawn(floor));
        operations.push(Operation::SetPosition(floor, Position::new(0, 0)));
        bodies.push(PhysicsBody::fixed(floor, FLOOR_HALF_EXTENTS));

        Self {
            dynamic_count,
            setup: Workload::new(operations),
            bodies,
        }
    }

    #[must_use]
    pub const fn dynamic_count(&self) -> u32 {
        self.dynamic_count
    }

    #[must_use]
    pub fn setup(&self) -> &Workload {
        &self.setup
    }

    #[must_use]
    pub fn bodies(&self) -> &[PhysicsBody] {
        &self.bodies
    }

    #[must_use]
    pub const fn physics_config(&self) -> PhysicsConfig {
        PhysicsConfig {
            gravity: Velocity::new(0, -1),
        }
    }

    /// Produces one canonical physics step for this scenario.
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError`] when the supplied snapshot does not satisfy the physics contract.
    pub fn step(&self, snapshot: &WorldSnapshot) -> Result<PhysicsStep, PhysicsError> {
        step(snapshot, &self.bodies, self.physics_config(), 1)
    }

    /// Runs the named scenario through the reference ECS for `frames` physics steps.
    ///
    /// # Errors
    ///
    /// Returns [`ScenarioError`] if setup, physics, or generated ECS operations fail.
    pub fn reference_after(&self, frames: u32) -> Result<WorldSnapshot, ScenarioError> {
        let mut world = ReferenceWorld::new();
        world.replay(&self.setup).map_err(ScenarioError::Workload)?;

        for _ in 0..frames {
            let physics = self.step(&world.snapshot()).map_err(ScenarioError::Physics)?;
            apply_reference_operations(&mut world, physics.operations())?;
        }

        Ok(world.snapshot())
    }

    /// Builds exact CPU broad-phase evidence from the Rust-owned falling-box state.
    ///
    /// The returned frame is post-physics state after `frames` canonical steps. Its AABBs and
    /// pair bitset are suitable for differential comparison with an optional WebGPU backend.
    ///
    /// # Errors
    ///
    /// Returns [`ScenarioError`] if the reference simulation fails or if a required position cannot
    /// be represented exactly at the `f32` collision boundary.
    pub fn broad_phase_frame_after(&self, frames: u32) -> Result<BroadPhaseFrame, ScenarioError> {
        let snapshot = self.reference_after(frames)?;
        BroadPhaseFrame::from_snapshot(&snapshot, &self.bodies)
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BroadPhaseBody {
    pub entity: EntityId,
    pub min: [f32; 3],
    pub max: [f32; 3],
}

#[derive(Clone, Debug, PartialEq)]
pub struct BroadPhaseFrame {
    bodies: Vec<BroadPhaseBody>,
    pair_words: Vec<u32>,
    overlap_count: u32,
}

impl BroadPhaseFrame {
    fn from_snapshot(snapshot: &WorldSnapshot, bodies: &[PhysicsBody]) -> Result<Self, ScenarioError> {
        let entities = snapshot
            .entities()
            .iter()
            .map(|entity| (entity.id, *entity))
            .collect::<BTreeMap<_, _>>();
        let mut frame_bodies = Vec::with_capacity(bodies.len());
        let mut geometry = Vec::with_capacity(bodies.len());

        for body in bodies {
            let entity = entities
                .get(&body.entity)
                .ok_or(ScenarioError::MissingEntity(body.entity))?;
            let position = entity
                .position
                .ok_or(ScenarioError::MissingPosition(body.entity))?;
            let aabb = body_aabb(*body, position)?;
            frame_bodies.push(BroadPhaseBody {
                entity: body.entity,
                min: aabb.min,
                max: aabb.max,
            });
            geometry.push(aabb);
        }

        let body_count = geometry.len();
        let possible_pairs = body_count.saturating_mul(body_count.saturating_sub(1)) / 2;
        let mut pair_words = vec![0_u32; possible_pairs.div_ceil(32)];
        let mut overlap_count = 0_u32;

        for left in 0..body_count {
            for right in (left + 1)..body_count {
                if !aabb_aabb(geometry[left], geometry[right]).overlaps {
                    continue;
                }
                let pair = pair_index(left, right, body_count);
                pair_words[pair / 32] |= 1_u32 << (pair % 32);
                overlap_count = overlap_count.saturating_add(1);
            }
        }

        Ok(Self {
            bodies: frame_bodies,
            pair_words,
            overlap_count,
        })
    }

    #[must_use]
    pub fn bodies(&self) -> &[BroadPhaseBody] {
        &self.bodies
    }

    #[must_use]
    pub fn pair_words(&self) -> &[u32] {
        &self.pair_words
    }

    #[must_use]
    pub const fn overlap_count(&self) -> u32 {
        self.overlap_count
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ScenarioError {
    Workload(WorkloadError),
    Physics(PhysicsError),
    MissingEntity(EntityId),
    MissingPosition(EntityId),
    CoordinateOutOfRange(EntityId),
}

impl fmt::Display for ScenarioError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Workload(error) => write!(formatter, "scenario workload failed: {error}"),
            Self::Physics(error) => write!(formatter, "scenario physics failed: {error}"),
            Self::MissingEntity(entity) => write!(formatter, "scenario entity {} is missing", entity.0),
            Self::MissingPosition(entity) => {
                write!(formatter, "scenario entity {} has no position", entity.0)
            }
            Self::CoordinateOutOfRange(entity) => write!(
                formatter,
                "scenario entity {} exceeds the exact f32 collision range",
                entity.0
            ),
        }
    }
}

impl std::error::Error for ScenarioError {}

fn apply_reference_operations(
    world: &mut ReferenceWorld,
    operations: &[Operation],
) -> Result<(), ScenarioError> {
    for operation in operations {
        world.apply(*operation).map_err(ScenarioError::Workload)?;
    }
    Ok(())
}

fn body_aabb(body: PhysicsBody, position: Position) -> Result<Aabb, ScenarioError> {
    if !position_in_exact_range(position, body.half_extents) {
        return Err(ScenarioError::CoordinateOutOfRange(body.entity));
    }
    Ok(Aabb::from_center_half_extents(
        [integer_as_f32(position.x), integer_as_f32(position.y), 0.0],
        [
            integer_as_f32(i64::from(body.half_extents[0])),
            integer_as_f32(i64::from(body.half_extents[1])),
            0.5,
        ],
    ))
}

fn position_in_exact_range(position: Position, half_extents: [i32; 2]) -> bool {
    let half_x = i64::from(half_extents[0]);
    let half_y = i64::from(half_extents[1]);
    half_x >= 0
        && half_y >= 0
        && position.x >= -MAX_EXACT_F32_INTEGER + half_x
        && position.x <= MAX_EXACT_F32_INTEGER - half_x
        && position.y >= -MAX_EXACT_F32_INTEGER + half_y
        && position.y <= MAX_EXACT_F32_INTEGER - half_y
}

#[allow(clippy::cast_precision_loss)]
fn integer_as_f32(value: i64) -> f32 {
    value as f32
}

fn pair_index(left: usize, right: usize, count: usize) -> usize {
    left * (2 * count - left - 1) / 2 + (right - left - 1)
}

#[cfg(test)]
mod tests {
    use ecs_reference::ReferenceWorld;
    use ecs_sparse_set::SparseWorld;
    use ecs_workload::{Operation, Position, WorldSnapshot};

    use super::{FallingBoxesScenario, ScenarioError};

    #[test]
    fn falling_boxes_recipe_is_deterministic() {
        assert_eq!(FallingBoxesScenario::new(64), FallingBoxesScenario::new(64));
    }

    #[test]
    fn one_box_touches_the_floor_after_one_step() {
        let scenario = FallingBoxesScenario::new(1);
        let frame = match scenario.broad_phase_frame_after(1) {
            Ok(frame) => frame,
            Err(error) => panic!("unexpected scenario error: {error}"),
        };

        assert_eq!(frame.bodies().len(), 2);
        assert_eq!(frame.overlap_count(), 1);
        assert_eq!(frame.pair_words(), [1]);
    }

    #[test]
    fn broad_phase_evidence_is_repeatable() {
        let scenario = FallingBoxesScenario::new(32);
        assert_eq!(
            scenario.broad_phase_frame_after(6),
            scenario.broad_phase_frame_after(6)
        );
    }

    #[test]
    fn falling_boxes_physics_matches_sparse_set_storage() {
        let scenario = FallingBoxesScenario::new(48);
        let mut reference = ReferenceWorld::new();
        let mut sparse = SparseWorld::new();
        assert_eq!(reference.replay(scenario.setup()), Ok(()));
        assert_eq!(sparse.replay(scenario.setup()), Ok(()));

        for frame in 0..8 {
            assert_eq!(sparse.snapshot(), reference.snapshot(), "before frame {frame}");
            let reference_step = scenario.step(&reference.snapshot());
            let sparse_step = scenario.step(&sparse.snapshot());
            assert_eq!(sparse_step, reference_step, "physics step {frame}");
            let physics = match reference_step {
                Ok(physics) => physics,
                Err(error) => panic!("unexpected physics error: {error}"),
            };
            for operation in physics.operations() {
                assert_eq!(reference.apply(*operation), Ok(()));
                assert_eq!(sparse.apply(*operation), Ok(()));
            }
        }

        assert_eq!(sparse.snapshot(), reference.snapshot());
    }

    #[test]
    fn scenario_reports_missing_positions() {
        let scenario = FallingBoxesScenario::new(1);
        let snapshot = WorldSnapshot::new(vec![ecs_workload::EntitySnapshot {
            id: ecs_workload::EntityId(0),
            position: Some(Position::new(0, 0)),
            velocity: None,
        }]);

        assert_eq!(
            super::BroadPhaseFrame::from_snapshot(&snapshot, scenario.bodies()),
            Err(ScenarioError::MissingEntity(ecs_workload::EntityId(1)))
        );
    }

    #[test]
    fn generated_steps_remain_plain_ecs_operations() {
        let scenario = FallingBoxesScenario::new(1);
        let snapshot = match scenario.reference_after(0) {
            Ok(snapshot) => snapshot,
            Err(error) => panic!("unexpected scenario error: {error}"),
        };
        let physics = match scenario.step(&snapshot) {
            Ok(physics) => physics,
            Err(error) => panic!("unexpected physics error: {error}"),
        };

        assert!(
            physics
                .operations()
                .iter()
                .all(|operation| matches!(operation, Operation::SetPosition(..) | Operation::SetVelocity(..)))
        );
    }
}
