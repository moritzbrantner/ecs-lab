use ecs_physics::{BodyKind, PhysicsMaterial};
use ecs_workload::{EntityId, Operation, Position, Velocity, Workload};

use crate::{
    ANGULAR_VELOCITY_SCALE, AngularState3d, AngularVelocity3d, Orientation3d, PhysicsBody3d,
    RapierWorldConfig3d, RigidBox3d, RigidBoxState3d,
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
const ROOM_FIXED: [(Position, [i32; 3]); 6] = [
    (Position::new3(0, -1, 0), [20, 1, 16]),
    (Position::new3(0, 21, 0), [20, 1, 16]),
    (Position::new3(-21, 10, 0), [1, 12, 16]),
    (Position::new3(21, 10, 0), [1, 12, 16]),
    (Position::new3(0, 10, -17), [20, 12, 1]),
    (Position::new3(0, 10, 17), [20, 12, 1]),
];
/// Rapier angular damping coefficient (1/s) of the room's dynamic boxes.
const ROOM_ANGULAR_DAMPING: f32 = 0.12;
const ROOM_SOLVER_ITERATIONS: u8 = 6;

/// A closed room of 48 spinning dynamic boxes with mixed materials and six fixed slabs.
///
/// Room coordinates are scaled by [`Self::spatial_scale`] into integer ECS units; one room unit
/// is one Rapier metre. Velocities are room units per second, gravity is one room unit per
/// second squared.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct BouncingRoom3dScenario {
    setup: Workload,
    boxes: Vec<RigidBox3d>,
    spatial_scale: i64,
    hz: u32,
}

impl BouncingRoom3dScenario {
    /// Builds the room stepped at `hz` frames per second, with `hz²` integer units per room unit.
    ///
    /// Returns `None` when `hz` is zero or the scaled room does not fit the integer components.
    #[must_use]
    pub fn new(hz: u32) -> Option<Self> {
        let rate = i32::try_from(hz).ok().filter(|rate| *rate > 0)?;
        let spatial_scale = i64::from(rate).checked_mul(i64::from(rate))?;
        let extent_scale = i32::try_from(spatial_scale).ok()?;
        let velocity_scale = extent_scale;
        let scale_position = |value: Position| {
            Some(Position::new3(
                value.x.checked_mul(spatial_scale)?,
                value.y.checked_mul(spatial_scale)?,
                value.z.checked_mul(spatial_scale)?,
            ))
        };
        let scale_extents = |value: [i32; 3]| {
            Some([
                value[0].checked_mul(extent_scale)?,
                value[1].checked_mul(extent_scale)?,
                value[2].checked_mul(extent_scale)?,
            ])
        };

        let mut boxes = Vec::new();
        for (layer_index, z) in ROOM_LAYERS.into_iter().enumerate() {
            for (row_index, y) in ROOM_ROWS.into_iter().enumerate() {
                for (column_index, x) in ROOM_COLUMNS.into_iter().enumerate() {
                    let index = boxes.len();
                    let entity = EntityId(u32::try_from(index).ok()?);
                    let velocity = Velocity::new3(
                        ROOM_X_VELOCITIES
                            [(column_index + row_index + layer_index) % ROOM_X_VELOCITIES.len()]
                        .checked_mul(velocity_scale)?,
                        ROOM_Y_VELOCITIES[(row_index + layer_index) % ROOM_Y_VELOCITIES.len()]
                            .checked_mul(velocity_scale)?,
                        ROOM_Z_VELOCITIES[(column_index + row_index * 2 + layer_index)
                            % ROOM_Z_VELOCITIES.len()]
                        .checked_mul(velocity_scale)?,
                    );
                    let (restitution, friction) = ROOM_MATERIALS[index % ROOM_MATERIALS.len()];
                    let body = PhysicsBody3d::dynamic(
                        entity,
                        scale_extents(ROOM_SHAPES[index % ROOM_SHAPES.len()])?,
                    )
                    .with_mass(1 + entity.0 % 4)
                    .with_material(PhysicsMaterial::new(restitution, friction));
                    boxes.push(RigidBox3d::new(
                        body,
                        RigidBoxState3d::new(
                            scale_position(Position::new3(x, y, z))?,
                            velocity,
                            AngularState3d::new(Orientation3d::IDENTITY, initial_spin(entity)),
                        ),
                    ));
                }
            }
        }
        debug_assert_eq!(boxes.len(), ROOM_DYNAMIC_COUNT as usize);

        for (offset, (center, half_extents)) in ROOM_FIXED.into_iter().enumerate() {
            let entity = EntityId(ROOM_DYNAMIC_COUNT + u32::try_from(offset).ok()?);
            boxes.push(RigidBox3d::new(
                PhysicsBody3d::fixed(entity, scale_extents(half_extents)?),
                RigidBoxState3d::new(
                    scale_position(center)?,
                    Velocity::new3(0, 0, 0),
                    AngularState3d::default(),
                ),
            ));
        }

        let mut operations = Vec::with_capacity(boxes.len() * 3);
        for rigid_box in &boxes {
            let entity = rigid_box.body.entity;
            operations.push(Operation::Spawn(entity));
            operations.push(Operation::SetPosition(entity, rigid_box.state.center));
            if rigid_box.body.kind == BodyKind::Dynamic {
                operations.push(Operation::SetVelocity(
                    entity,
                    rigid_box.state.linear_velocity,
                ));
            }
        }
        Some(Self {
            setup: Workload::new(operations),
            boxes,
            spatial_scale,
            hz,
        })
    }

    /// ECS workload that spawns every room entity with its initial integer state.
    #[must_use]
    pub fn setup(&self) -> &Workload {
        &self.setup
    }

    /// Initial rigid boxes: 48 dynamic boxes followed by the six fixed slabs.
    #[must_use]
    pub fn boxes(&self) -> &[RigidBox3d] {
        &self.boxes
    }

    #[must_use]
    pub const fn spatial_scale(&self) -> i64 {
        self.spatial_scale
    }

    /// Rapier world parameters for the room.
    #[must_use]
    pub fn config(&self) -> RapierWorldConfig3d {
        let gravity = i32::try_from(self.spatial_scale).unwrap_or(i32::MAX);
        RapierWorldConfig3d {
            spatial_scale: self.spatial_scale,
            gravity: Velocity::new3(0, -gravity, 0),
            hz: self.hz,
            angular_damping: ROOM_ANGULAR_DAMPING,
            solver_iterations: ROOM_SOLVER_ITERATIONS,
            lock_rotations: false,
        }
    }
}

fn initial_spin(entity: EntityId) -> AngularVelocity3d {
    let unit = ANGULAR_VELOCITY_SCALE / 5;
    match entity.0 % 8 {
        0 => AngularVelocity3d::new(unit, unit / 2, -unit),
        1 => AngularVelocity3d::new(-unit, 2 * unit, unit / 2),
        2 => AngularVelocity3d::new(unit / 2, -unit, 2 * unit),
        3 => AngularVelocity3d::new(2 * unit, unit, -unit / 2),
        4 => AngularVelocity3d::new(-2 * unit, unit / 2, unit),
        5 => AngularVelocity3d::new(unit, -2 * unit, unit / 2),
        6 => AngularVelocity3d::new(unit / 2, unit, -2 * unit),
        _ => AngularVelocity3d::new(-unit, -unit / 2, 2 * unit),
    }
}

#[cfg(test)]
mod tests {
    use ecs_physics::BodyKind;
    use ecs_reference::ReferenceWorld;
    use ecs_sparse_set::SparseWorld;

    use super::BouncingRoom3dScenario;
    use crate::RapierWorld3d;

    const FRAMES: u32 = 240;

    fn room() -> (BouncingRoom3dScenario, RapierWorld3d) {
        let scenario = BouncingRoom3dScenario::new(60).expect("60 Hz room fits the integer units");
        let world = RapierWorld3d::new(scenario.boxes(), scenario.config()).expect("valid room");
        (scenario, world)
    }

    #[test]
    fn room_stays_inside_its_walls_and_keeps_moving() {
        let (scenario, mut world) = room();
        assert_eq!(scenario.spatial_scale(), 3_600);
        let scale = scenario.spatial_scale();
        let initial_z = world.boxes()[0].state.center.z;
        for _ in 0..FRAMES {
            let frame = world.advance().expect("room step");
            for rigid_box in frame
                .boxes()
                .iter()
                .filter(|rigid_box| rigid_box.body.kind == BodyKind::Dynamic)
            {
                let center = rigid_box.state.center;
                // Interior: |x| < 20, 0 < y < 20, |z| < 16 room units, plus 0.25 units slack.
                let slack = scale / 4;
                assert!(center.x.abs() < 20 * scale + slack, "{rigid_box:?}");
                assert!(
                    center.y > -slack && center.y < 20 * scale + slack,
                    "{rigid_box:?}"
                );
                assert!(center.z.abs() < 16 * scale + slack, "{rigid_box:?}");
            }
            let max_penetration = world.max_penetration();
            assert!(
                max_penetration < 0.2,
                "room penetration {max_penetration} exceeds 0.2 room units"
            );
        }
        assert_ne!(world.boxes()[0].state.center.z, initial_z);
    }

    #[test]
    fn room_replays_identically_on_the_same_machine() {
        let (_, mut first) = room();
        let (_, mut second) = room();
        for frame in 0..FRAMES {
            let left = first.advance().expect("first room step").boxes().to_vec();
            let right = second.advance().expect("second room step").boxes().to_vec();
            assert_eq!(left, right, "same-machine replay diverged at frame {frame}");
        }
    }

    #[test]
    fn quantized_operations_keep_reference_and_sparse_storage_in_lockstep() {
        let (scenario, mut world) = room();
        let mut reference = ReferenceWorld::new();
        let mut sparse = SparseWorld::new();
        assert_eq!(reference.replay(scenario.setup()), Ok(()));
        assert_eq!(sparse.replay(scenario.setup()), Ok(()));
        for frame in 0..60 {
            let step = world.advance().expect("room step");
            for operation in step.operations() {
                assert_eq!(reference.apply(*operation), Ok(()));
                assert_eq!(sparse.apply(*operation), Ok(()));
            }
            assert_eq!(sparse.snapshot(), reference.snapshot(), "frame {frame}");
            for rigid_box in step.boxes() {
                let entity = reference
                    .snapshot()
                    .entities()
                    .iter()
                    .find(|entity| entity.id == rigid_box.body.entity)
                    .copied()
                    .expect("room entity");
                assert_eq!(entity.position, Some(rigid_box.state.center));
                if rigid_box.body.kind == BodyKind::Dynamic {
                    assert_eq!(entity.velocity, Some(rigid_box.state.linear_velocity));
                }
            }
        }
    }
}
