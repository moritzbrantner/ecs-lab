use std::{collections::BTreeMap, fmt};

use ecs_physics::{BodyKind, PhysicsMaterial};
use ecs_reference::ReferenceWorld;
use ecs_workload::{
    EntityId, Operation, Position, Velocity, Workload, WorkloadError, WorldSnapshot,
};
use geometry_kernels::aabb_aabb;
use spatial_kernels::Aabb;

use crate::{
    solver::{axis_fits_exact_f32, exact_i64_to_f32},
    step_3d,
    types::{PhysicsBody3d, PhysicsConfig3d, PhysicsError3d, PhysicsStep3d},
};

const ROOM_DYNAMIC_COUNT: u32 = 48;
const ROOM_COLUMNS: [i64; 4] = [-15, -5, 5, 15];
const ROOM_ROWS: [i64; 4] = [3, 8, 13, 18];
const ROOM_LAYERS: [i64; 3] = [-10, 0, 10];
const ROOM_X_VELOCITIES: [i64; 4] = [-4, -3, 3, 4];
const ROOM_Y_VELOCITIES: [i64; 4] = [-1, 0, 1, 2];
const ROOM_Z_VELOCITIES: [i64; 4] = [4, -3, 3, -4];
const ROOM_MATERIALS: [(u16, u16); 6] = [
    (1_000, 0),
    (850, 150),
    (650, 350),
    (450, 550),
    (250, 750),
    (0, 1_000),
];
const ROOM_SHAPES: [[i32; 3]; 4] = [[2, 1, 1], [1, 1, 2], [2, 1, 2], [1, 1, 1]];
const FLOOR: EntityId = EntityId(ROOM_DYNAMIC_COUNT);
const CEILING: EntityId = EntityId(ROOM_DYNAMIC_COUNT + 1);
const LEFT_WALL: EntityId = EntityId(ROOM_DYNAMIC_COUNT + 2);
const RIGHT_WALL: EntityId = EntityId(ROOM_DYNAMIC_COUNT + 3);
const BACK_WALL: EntityId = EntityId(ROOM_DYNAMIC_COUNT + 4);
const FRONT_WALL: EntityId = EntityId(ROOM_DYNAMIC_COUNT + 5);

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BroadPhaseBody3d {
    pub entity: EntityId,
    pub position: Position,
    pub half_extents: [i32; 3],
    pub kind: BodyKind,
    pub mass_units: u32,
    pub material: PhysicsMaterial,
    pub min: [f32; 3],
    pub max: [f32; 3],
}

#[derive(Clone, Debug, PartialEq)]
pub struct BroadPhaseFrame3d {
    bodies: Vec<BroadPhaseBody3d>,
    pair_words: Vec<u32>,
    overlap_count: u32,
}

impl BroadPhaseFrame3d {
    #[must_use]
    pub fn bodies(&self) -> &[BroadPhaseBody3d] {
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

    fn from_snapshot(
        snapshot: &WorldSnapshot,
        bodies: &[PhysicsBody3d],
    ) -> Result<Self, ScenarioError3d> {
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
                .ok_or(ScenarioError3d::MissingEntity(body.entity))?;
            let position = entity
                .position
                .ok_or(ScenarioError3d::MissingPosition(body.entity))?;
            let aabb = body_aabb(*body, position)?;
            frame_bodies.push(BroadPhaseBody3d {
                entity: body.entity,
                position,
                half_extents: body.half_extents,
                kind: body.kind,
                mass_units: body.mass_units,
                material: body.material,
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
}

fn pair_index(left: usize, right: usize, count: usize) -> usize {
    left * (2 * count - left - 1) / 2 + (right - left - 1)
}

fn body_aabb(body: PhysicsBody3d, position: Position) -> Result<Aabb, ScenarioError3d> {
    let half = body.half_extents.map(i64::from);
    if !axis_fits_exact_f32(position.x, half[0])
        || !axis_fits_exact_f32(position.y, half[1])
        || !axis_fits_exact_f32(position.z, half[2])
    {
        return Err(ScenarioError3d::CoordinateOutOfRange(body.entity));
    }
    Ok(Aabb::from_center_half_extents(
        [
            exact_i64_to_f32(position.x),
            exact_i64_to_f32(position.y),
            exact_i64_to_f32(position.z),
        ],
        [
            exact_i64_to_f32(half[0]),
            exact_i64_to_f32(half[1]),
            exact_i64_to_f32(half[2]),
        ],
    ))
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct BouncingRoom3dScenario {
    setup: Workload,
    bodies: Vec<PhysicsBody3d>,
}

impl Default for BouncingRoom3dScenario {
    fn default() -> Self {
        Self::new()
    }
}

impl BouncingRoom3dScenario {
    #[must_use]
    pub fn new() -> Self {
        let mut operations = Vec::new();
        let mut bodies = Vec::new();

        for (layer_index, z) in ROOM_LAYERS.into_iter().enumerate() {
            for (row_index, y) in ROOM_ROWS.into_iter().enumerate() {
                for (column_index, x) in ROOM_COLUMNS.into_iter().enumerate() {
                    let body_index = bodies.len();
                    let entity = EntityId(body_index as u32);
                    let velocity = Velocity::new3(
                        ROOM_X_VELOCITIES
                            [(column_index + row_index + layer_index) % ROOM_X_VELOCITIES.len()],
                        ROOM_Y_VELOCITIES[(row_index + layer_index) % ROOM_Y_VELOCITIES.len()],
                        ROOM_Z_VELOCITIES
                            [(column_index + row_index * 2 + layer_index) % ROOM_Z_VELOCITIES.len()],
                    );
                    let (restitution, friction) =
                        ROOM_MATERIALS[body_index % ROOM_MATERIALS.len()];
                    let material = PhysicsMaterial::new(restitution, friction);
                    let mass_units = 1 + entity.0 % 4;
                    let half_extents = ROOM_SHAPES[body_index % ROOM_SHAPES.len()];

                    operations.push(Operation::Spawn(entity));
                    operations.push(Operation::SetPosition(entity, Position::new3(x, y, z)));
                    operations.push(Operation::SetVelocity(entity, velocity));
                    bodies.push(
                        PhysicsBody3d::dynamic(entity, half_extents)
                            .with_mass(mass_units)
                            .with_material(material),
                    );
                }
            }
        }

        add_room_boundaries(&mut operations, &mut bodies);
        Self {
            setup: Workload::new(operations),
            bodies,
        }
    }

    #[must_use]
    pub fn setup(&self) -> &Workload {
        &self.setup
    }

    #[must_use]
    pub fn bodies(&self) -> &[PhysicsBody3d] {
        &self.bodies
    }

    #[must_use]
    pub const fn physics_config(&self) -> PhysicsConfig3d {
        PhysicsConfig3d {
            gravity: Velocity::new3(0, -1, 0),
        }
    }

    /// Produces one canonical 3D room step.
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError3d`] when the supplied snapshot violates the 3D physics contract.
    pub fn step(&self, snapshot: &WorldSnapshot) -> Result<PhysicsStep3d, PhysicsError3d> {
        step_3d(snapshot, &self.bodies, self.physics_config(), 1)
    }

    /// Replays the room through the reference ECS for the requested number of steps.
    ///
    /// # Errors
    ///
    /// Returns [`ScenarioError3d`] when setup, physics, or ECS replay fails.
    pub fn reference_after(&self, frames: u32) -> Result<WorldSnapshot, ScenarioError3d> {
        let mut world = ReferenceWorld::new();
        world
            .replay(&self.setup)
            .map_err(ScenarioError3d::Workload)?;
        for _ in 0..frames {
            let physics = self
                .step(&world.snapshot())
                .map_err(ScenarioError3d::Physics)?;
            for operation in physics.operations() {
                world.apply(*operation).map_err(ScenarioError3d::Workload)?;
            }
        }
        Ok(world.snapshot())
    }

    /// Builds exact 3D AABB evidence from the Rust-owned room state.
    ///
    /// # Errors
    ///
    /// Returns [`ScenarioError3d`] when the scenario cannot produce the requested frame.
    pub fn broad_phase_frame_after(
        &self,
        frames: u32,
    ) -> Result<BroadPhaseFrame3d, ScenarioError3d> {
        BroadPhaseFrame3d::from_snapshot(&self.reference_after(frames)?, &self.bodies)
    }
}

fn add_room_boundaries(operations: &mut Vec<Operation>, bodies: &mut Vec<PhysicsBody3d>) {
    let fixed = [
        (FLOOR, Position::new3(0, -1, 0), [20, 1, 16]),
        (CEILING, Position::new3(0, 21, 0), [20, 1, 16]),
        (LEFT_WALL, Position::new3(-21, 10, 0), [1, 12, 16]),
        (RIGHT_WALL, Position::new3(21, 10, 0), [1, 12, 16]),
        (BACK_WALL, Position::new3(0, 10, -17), [20, 12, 1]),
        (FRONT_WALL, Position::new3(0, 10, 17), [20, 12, 1]),
    ];
    for (entity, position, half_extents) in fixed {
        operations.push(Operation::Spawn(entity));
        operations.push(Operation::SetPosition(entity, position));
        bodies.push(PhysicsBody3d::fixed(entity, half_extents));
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ScenarioError3d {
    Workload(WorkloadError),
    Physics(PhysicsError3d),
    MissingEntity(EntityId),
    MissingPosition(EntityId),
    CoordinateOutOfRange(EntityId),
}

impl fmt::Display for ScenarioError3d {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Workload(error) => write!(formatter, "3D scenario workload failed: {error}"),
            Self::Physics(error) => write!(formatter, "3D scenario physics failed: {error}"),
            Self::MissingEntity(entity) => {
                write!(formatter, "3D scenario entity {} is missing", entity.0)
            }
            Self::MissingPosition(entity) => {
                write!(formatter, "3D scenario entity {} has no position", entity.0)
            }
            Self::CoordinateOutOfRange(entity) => write!(
                formatter,
                "3D scenario entity {} exceeds the exact f32 collision range",
                entity.0
            ),
        }
    }
}

impl std::error::Error for ScenarioError3d {}

#[cfg(test)]
mod tests {
    use ecs_reference::ReferenceWorld;
    use ecs_sparse_set::SparseWorld;

    use super::{BouncingRoom3dScenario, ROOM_DYNAMIC_COUNT};

    #[test]
    fn room_moves_bodies_through_depth() {
        let scenario = BouncingRoom3dScenario::new();
        let initial = scenario
            .reference_after(0)
            .expect("initial frame should work");
        let next = scenario.reference_after(1).expect("next frame should work");
        let initial_z = initial.entities()[0]
            .position
            .expect("position should exist")
            .z;
        let next_z = next.entities()[0]
            .position
            .expect("position should exist")
            .z;
        assert_ne!(initial_z, next_z);
    }

    #[test]
    fn room_keeps_reference_and_sparse_storage_in_lockstep() {
        let scenario = BouncingRoom3dScenario::new();
        let mut reference = ReferenceWorld::new();
        let mut sparse = SparseWorld::new();
        assert_eq!(reference.replay(scenario.setup()), Ok(()));
        assert_eq!(sparse.replay(scenario.setup()), Ok(()));

        for frame in 0..24 {
            assert_eq!(
                sparse.snapshot(),
                reference.snapshot(),
                "3D storage snapshots diverged before frame {frame}"
            );
            let physics = scenario
                .step(&reference.snapshot())
                .expect("3D room step should succeed");
            for operation in physics.operations() {
                assert_eq!(reference.apply(*operation), Ok(()));
                assert_eq!(sparse.apply(*operation), Ok(()));
            }
        }
    }

    #[test]
    fn broad_phase_frame_is_repeatable_and_three_dimensional() {
        let scenario = BouncingRoom3dScenario::new();
        let first = scenario
            .broad_phase_frame_after(6)
            .expect("3D broad phase should succeed");
        let second = scenario
            .broad_phase_frame_after(6)
            .expect("repeated 3D broad phase should succeed");
        assert_eq!(first, second);
        assert_eq!(first.bodies().len(), ROOM_DYNAMIC_COUNT as usize + 6);
        assert!(first.bodies()[0].position.z != 0);
    }
}
