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
const ROOM_X_VELOCITIES: [i32; 4] = [-4, -3, 3, 4];
const ROOM_Y_VELOCITIES: [i32; 4] = [-1, 0, 1, 2];
const ROOM_Z_VELOCITIES: [i32; 4] = [4, -3, 3, -4];
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

    /// Rebuilds pair evidence after normalizing the fixed-point room coordinates for display.
    ///
    /// The browser receives normalized `f32` centers and half extents. Recomputing the bitset from the
    /// same normalized values keeps optional WebGPU verification word-for-word comparable even when a
    /// mathematically exact contact lies on an `f32` rounding boundary after normalization.
    #[must_use]
    pub fn pair_words_at_spatial_scale(&self, spatial_scale: i64) -> Option<Vec<u32>> {
        if spatial_scale <= 0 {
            return None;
        }
        let scale = exact_i64_to_f32(spatial_scale);
        let geometry = self
            .bodies
            .iter()
            .map(|body| {
                let half = body.half_extents.map(i64::from);
                Aabb::from_center_half_extents(
                    [
                        exact_i64_to_f32(body.position.x) / scale,
                        exact_i64_to_f32(body.position.y) / scale,
                        exact_i64_to_f32(body.position.z) / scale,
                    ],
                    [
                        exact_i64_to_f32(half[0]) / scale,
                        exact_i64_to_f32(half[1]) / scale,
                        exact_i64_to_f32(half[2]) / scale,
                    ],
                )
            })
            .collect::<Vec<_>>();
        Some(pair_evidence(&geometry).0)
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

        let (pair_words, overlap_count) = pair_evidence(&geometry);
        Ok(Self {
            bodies: frame_bodies,
            pair_words,
            overlap_count,
        })
    }
}

fn pair_evidence(geometry: &[Aabb]) -> (Vec<u32>, u32) {
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
    (pair_words, overlap_count)
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
    spatial_scale: i64,
}

impl Default for BouncingRoom3dScenario {
    fn default() -> Self {
        Self::new()
    }
}

impl BouncingRoom3dScenario {
    #[must_use]
    pub fn new() -> Self {
        let Ok(scenario) = Self::with_substeps_per_tick(1) else {
            unreachable!("the canonical unit timestep is statically representable");
        };
        scenario
    }

    /// Builds the canonical room at a finer integer fixed-step resolution.
    ///
    /// `substeps_per_tick = N` scales positions and extents by `N²` and velocities by `N` while
    /// leaving the integer gravity impulse unchanged. One solver step then represents `1/N` of the
    /// canonical time unit without introducing floating-point ECS state. Dividing exported positions
    /// and extents by [`Self::spatial_scale`] restores the ordinary room coordinates.
    ///
    /// # Errors
    ///
    /// Returns [`ScenarioError3d::InvalidSubstepRate`] when the rate is zero or cannot be represented
    /// by the integer position, velocity, or extent types used by the scenario.
    pub fn with_substeps_per_tick(substeps_per_tick: u32) -> Result<Self, ScenarioError3d> {
        let velocity_scale = i32::try_from(substeps_per_tick)
            .map_err(|_| ScenarioError3d::InvalidSubstepRate(substeps_per_tick))?;
        if velocity_scale <= 0 {
            return Err(ScenarioError3d::InvalidSubstepRate(substeps_per_tick));
        }
        let spatial_scale = i64::from(velocity_scale)
            .checked_mul(i64::from(velocity_scale))
            .ok_or(ScenarioError3d::InvalidSubstepRate(substeps_per_tick))?;
        let extent_scale = i32::try_from(spatial_scale)
            .map_err(|_| ScenarioError3d::InvalidSubstepRate(substeps_per_tick))?;

        let scale_position = |value: i64| {
            value
                .checked_mul(spatial_scale)
                .ok_or(ScenarioError3d::InvalidSubstepRate(substeps_per_tick))
        };
        let scale_velocity = |value: i32| {
            value
                .checked_mul(velocity_scale)
                .ok_or(ScenarioError3d::InvalidSubstepRate(substeps_per_tick))
        };
        let scale_extent = |value: i32| {
            value
                .checked_mul(extent_scale)
                .ok_or(ScenarioError3d::InvalidSubstepRate(substeps_per_tick))
        };

        let mut operations = Vec::new();
        let mut bodies = Vec::new();
        let mut next_entity = 0_u32;

        for (layer_index, z) in ROOM_LAYERS.into_iter().enumerate() {
            for (row_index, y) in ROOM_ROWS.into_iter().enumerate() {
                for (column_index, x) in ROOM_COLUMNS.into_iter().enumerate() {
                    let body_index = bodies.len();
                    let entity = EntityId(next_entity);
                    let x_velocity = ROOM_X_VELOCITIES
                        [(column_index + row_index + layer_index) % ROOM_X_VELOCITIES.len()];
                    let y_velocity =
                        ROOM_Y_VELOCITIES[(row_index + layer_index) % ROOM_Y_VELOCITIES.len()];
                    let z_velocity = ROOM_Z_VELOCITIES
                        [(column_index + row_index * 2 + layer_index) % ROOM_Z_VELOCITIES.len()];
                    let velocity = Velocity::new3(
                        scale_velocity(x_velocity)?,
                        scale_velocity(y_velocity)?,
                        scale_velocity(z_velocity)?,
                    );
                    let (restitution, friction) = ROOM_MATERIALS[body_index % ROOM_MATERIALS.len()];
                    let material = PhysicsMaterial::new(restitution, friction);
                    let mass_units = 1 + next_entity % 4;
                    let half_extents = ROOM_SHAPES[body_index % ROOM_SHAPES.len()]
                        .map(scale_extent)
                        .into_iter()
                        .collect::<Result<Vec<_>, _>>()?
                        .try_into()
                        .map_err(|_| ScenarioError3d::InvalidSubstepRate(substeps_per_tick))?;

                    operations.push(Operation::Spawn(entity));
                    operations.push(Operation::SetPosition(
                        entity,
                        Position::new3(scale_position(x)?, scale_position(y)?, scale_position(z)?),
                    ));
                    operations.push(Operation::SetVelocity(entity, velocity));
                    bodies.push(
                        PhysicsBody3d::dynamic(entity, half_extents)
                            .with_mass(mass_units)
                            .with_material(material),
                    );
                    next_entity += 1;
                }
            }
        }
        debug_assert_eq!(next_entity, ROOM_DYNAMIC_COUNT);

        add_room_boundaries(
            &mut operations,
            &mut bodies,
            spatial_scale,
            extent_scale,
            substeps_per_tick,
        )?;
        Ok(Self {
            setup: Workload::new(operations),
            bodies,
            spatial_scale,
        })
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
    pub const fn spatial_scale(&self) -> i64 {
        self.spatial_scale
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

    /// Builds exact 3D AABB evidence from a supplied Rust-owned room snapshot.
    ///
    /// # Errors
    ///
    /// Returns [`ScenarioError3d`] when the snapshot is missing required room state or exceeds the
    /// exact collision-coordinate range.
    pub fn broad_phase_frame(
        &self,
        snapshot: &WorldSnapshot,
    ) -> Result<BroadPhaseFrame3d, ScenarioError3d> {
        BroadPhaseFrame3d::from_snapshot(snapshot, &self.bodies)
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
        self.broad_phase_frame(&self.reference_after(frames)?)
    }
}

fn add_room_boundaries(
    operations: &mut Vec<Operation>,
    bodies: &mut Vec<PhysicsBody3d>,
    spatial_scale: i64,
    extent_scale: i32,
    substeps_per_tick: u32,
) -> Result<(), ScenarioError3d> {
    let scale_position = |value: i64| {
        value
            .checked_mul(spatial_scale)
            .ok_or(ScenarioError3d::InvalidSubstepRate(substeps_per_tick))
    };
    let scale_extent = |value: i32| {
        value
            .checked_mul(extent_scale)
            .ok_or(ScenarioError3d::InvalidSubstepRate(substeps_per_tick))
    };
    let fixed = [
        (FLOOR, Position::new3(0, -1, 0), [20, 1, 16]),
        (CEILING, Position::new3(0, 21, 0), [20, 1, 16]),
        (LEFT_WALL, Position::new3(-21, 10, 0), [1, 12, 16]),
        (RIGHT_WALL, Position::new3(21, 10, 0), [1, 12, 16]),
        (BACK_WALL, Position::new3(0, 10, -17), [20, 12, 1]),
        (FRONT_WALL, Position::new3(0, 10, 17), [20, 12, 1]),
    ];
    for (entity, position, half_extents) in fixed {
        let scaled_half_extents = half_extents
            .map(scale_extent)
            .into_iter()
            .collect::<Result<Vec<_>, _>>()?
            .try_into()
            .map_err(|_| ScenarioError3d::InvalidSubstepRate(substeps_per_tick))?;
        operations.push(Operation::Spawn(entity));
        operations.push(Operation::SetPosition(
            entity,
            Position::new3(
                scale_position(position.x)?,
                scale_position(position.y)?,
                scale_position(position.z)?,
            ),
        ));
        bodies.push(PhysicsBody3d::fixed(entity, scaled_half_extents));
    }
    Ok(())
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ScenarioError3d {
    Workload(WorkloadError),
    Physics(PhysicsError3d),
    MissingEntity(EntityId),
    MissingPosition(EntityId),
    CoordinateOutOfRange(EntityId),
    InvalidSubstepRate(u32),
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
            Self::InvalidSubstepRate(rate) => {
                write!(
                    formatter,
                    "3D scenario substep rate {rate} cannot be represented"
                )
            }
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
    fn room_supports_sixty_fine_solver_steps_per_tick() {
        let scenario = BouncingRoom3dScenario::with_substeps_per_tick(60)
            .expect("60 Hz room scaling should be representable");
        assert_eq!(scenario.spatial_scale(), 3_600);

        let initial = scenario
            .reference_after(0)
            .expect("initial fine frame should work");
        let next = scenario
            .reference_after(1)
            .expect("next fine frame should work");
        let initial_z = initial.entities()[0]
            .position
            .expect("position should exist")
            .z;
        let next_z = next.entities()[0]
            .position
            .expect("position should exist")
            .z;
        let fine_delta = (next_z - initial_z).abs();
        assert!(fine_delta > 0);
        assert!(fine_delta < scenario.spatial_scale());

        let one_tick = scenario
            .reference_after(60)
            .expect("one nominal tick of fine frames should work");
        let one_tick_z = one_tick.entities()[0]
            .position
            .expect("position should exist")
            .z;
        assert!((one_tick_z - initial_z).abs() > scenario.spatial_scale());
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
