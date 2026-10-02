use std::sync::{Mutex, OnceLock};

use ecs_physics::PhysicsMaterial;
use ecs_physics_3d::{
    AngularState3d, AngularSubstepPolicy3d, AngularVelocity3d, Orientation3d,
    PersistentPhysicsWorld3d, PhysicsBody3d, RigidBox3d, RigidBoxState3d, RigidBoxWorldConfig3d,
    RotatingContactSearchConfig3d, oriented_box_vertices,
};
use ecs_workload::{EntityId, Position, Velocity};

const TOWER_DEMO_FPS: u32 = 60;
const TOWER_DEMO_SECONDS: u32 = 8;
const TOWER_DEMO_MAX_STEPS: u32 = TOWER_DEMO_FPS * TOWER_DEMO_SECONDS;
const TOWER_SCALE: i64 = 3_600;
const TOWER_EXTENT_SCALE: i32 = 3_600;
const TOWER_ROWS: usize = 5;
const TOWER_COLUMNS: usize = 3;
const TOWER_LAYERS: usize = 2;
const TOWER_BLOCK_COUNT: usize = TOWER_ROWS * TOWER_COLUMNS * TOWER_LAYERS;
const FLOOR_INDEX: usize = 0;
const PROJECTILE_INDEX: usize = 1;
const FIRST_BLOCK_INDEX: usize = 2;
const TOWER_BODY_COUNT: usize = FIRST_BLOCK_INDEX + TOWER_BLOCK_COUNT;
const TOWER_CONTACT_SEARCH: RotatingContactSearchConfig3d = RotatingContactSearchConfig3d {
    coarse_samples: 8,
    refinement_steps: 4,
};

static TOWER_DEMO_STATE: OnceLock<Mutex<Option<TowerDemoState>>> = OnceLock::new();

#[derive(Clone, Copy, Debug, Default)]
struct TowerFrameStats {
    spinning_bodies: usize,
    sampled_events: usize,
    tail_contacts: usize,
}

#[derive(Clone, Debug)]
struct TowerFrame {
    #[cfg(test)]
    boxes: Vec<RigidBox3d>,
    vertices: Vec<[Position; 8]>,
    stats: TowerFrameStats,
}

struct TowerDemoState {
    physics: PersistentPhysicsWorld3d,
    frames: Vec<TowerFrame>,
}

impl TowerDemoState {
    fn new() -> Option<Self> {
        let boxes = initial_boxes()?;
        let config = RigidBoxWorldConfig3d {
            gravity: Velocity::new3(0, -10 * TOWER_EXTENT_SCALE, 0),
            timestep_numerator: 1,
            timestep_denominator: i32::try_from(TOWER_DEMO_FPS).ok()?,
            angular_damping_milli: 996,
            solver_passes: 10,
        };
        let physics = PersistentPhysicsWorld3d::new(
            &boxes,
            config,
            TOWER_CONTACT_SEARCH,
            AngularSubstepPolicy3d::default(),
        )
        .ok()?;
        let initial = capture_frame(boxes, TowerFrameStats::default())?;
        Some(Self {
            physics,
            frames: vec![initial],
        })
    }

    fn ensure_frame(&mut self, steps: u32) -> Option<&TowerFrame> {
        if steps > TOWER_DEMO_MAX_STEPS {
            return None;
        }
        let target = usize::try_from(steps).ok()?;
        while self.frames.len() <= target {
            let next = self
                .physics
                .advance()
                .inspect_err(|error| {
                    eprintln!(
                        "tower physics failed while constructing frame {}: {error:?}",
                        self.frames.len()
                    );
                })
                .ok()?;
            let stats = TowerFrameStats {
                spinning_bodies: next
                    .boxes
                    .iter()
                    .filter(|rigid_box| !rigid_box.state.angular.angular_velocity.is_zero())
                    .count(),
                sampled_events: next.sampled_events,
                tail_contacts: next.tail_contacts,
            };
            self.frames.push(capture_frame(next.boxes, stats)?);
        }
        self.frames.get(target)
    }
}

fn initial_boxes() -> Option<Vec<RigidBox3d>> {
    let mut boxes = Vec::with_capacity(TOWER_BODY_COUNT);
    boxes.push(floor_body());
    boxes.push(projectile_body());

    let block_half_extents = [TOWER_EXTENT_SCALE, 2_700, 4_320];
    let block_material = PhysicsMaterial::new(80, 760);
    let mut entity = u32::try_from(FIRST_BLOCK_INDEX).ok()?;
    for layer in 0..TOWER_LAYERS {
        for row in 0..TOWER_ROWS {
            for column in 0..TOWER_COLUMNS {
                let layer_x = i64::try_from(layer).ok()? * 2 * TOWER_SCALE;
                let row_y = 2_700_i64 + i64::try_from(row).ok()? * 5_400;
                let column_z = (i64::try_from(column).ok()? - 1) * 8_640;
                boxes.push(RigidBox3d::new(
                    PhysicsBody3d::dynamic(EntityId(entity), block_half_extents)
                        .with_mass(2)
                        .with_material(block_material),
                    RigidBoxState3d::new(
                        Position::new3(layer_x, row_y, column_z),
                        Velocity::new3(0, 0, 0),
                        AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
                    ),
                ));
                entity = entity.checked_add(1)?;
            }
        }
    }
    (boxes.len() == TOWER_BODY_COUNT).then_some(boxes)
}

fn floor_body() -> RigidBox3d {
    RigidBox3d::new(
        PhysicsBody3d::fixed(
            EntityId(u32::try_from(FLOOR_INDEX).unwrap_or_default()),
            [
                30 * TOWER_EXTENT_SCALE,
                TOWER_EXTENT_SCALE,
                12 * TOWER_EXTENT_SCALE,
            ],
        )
        .with_material(PhysicsMaterial::new(50, 850)),
        RigidBoxState3d::new(
            Position::new3(0, -TOWER_SCALE, 0),
            Velocity::new3(0, 0, 0),
            AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
        ),
    )
}

fn projectile_body() -> RigidBox3d {
    RigidBox3d::new(
        PhysicsBody3d::dynamic(
            EntityId(u32::try_from(PROJECTILE_INDEX).unwrap_or_default()),
            [4_680, 4_680, 4_680],
        )
        .with_mass(24)
        .with_material(PhysicsMaterial::new(120, 420)),
        RigidBoxState3d::new(
            Position::new3(-18 * TOWER_SCALE, 19_800, 0),
            Velocity::new3(24 * TOWER_EXTENT_SCALE, 4 * TOWER_EXTENT_SCALE, 0),
            AngularState3d::new(
                Orientation3d::IDENTITY,
                AngularVelocity3d::new(0, 250_000, 1_250_000),
            ),
        ),
    )
}

fn capture_frame(boxes: Vec<RigidBox3d>, stats: TowerFrameStats) -> Option<TowerFrame> {
    let vertices = boxes
        .iter()
        .map(|rigid_box| {
            oriented_box_vertices(
                rigid_box.state.center,
                rigid_box.body.half_extents,
                rigid_box.state.angular.orientation,
            )
            .ok()
        })
        .collect::<Option<Vec<_>>>()?;
    Some(TowerFrame {
        #[cfg(test)]
        boxes,
        vertices,
        stats,
    })
}

fn tower_demo_state() -> &'static Mutex<Option<TowerDemoState>> {
    TOWER_DEMO_STATE.get_or_init(|| Mutex::new(None))
}

fn ensure_state(state: &mut Option<TowerDemoState>) -> Option<&mut TowerDemoState> {
    if state.is_none() {
        *state = TowerDemoState::new();
    }
    state.as_mut()
}

fn with_frame<T>(steps: u32, read: impl FnOnce(&TowerFrame) -> Option<T>) -> Option<T> {
    let mut state = tower_demo_state().lock().ok()?;
    read(ensure_state(&mut state)?.ensure_frame(steps)?)
}

fn vertex(body_index: u32, vertex_index: u32, steps: u32) -> Option<Position> {
    let body = usize::try_from(body_index).ok()?;
    let vertex = usize::try_from(vertex_index).ok()?;
    with_frame(steps, |frame| {
        frame.vertices.get(body)?.get(vertex).copied()
    })
}

fn display_coordinate(value: i64) -> f32 {
    value as f32 / TOWER_SCALE as f32
}

/// Recreates the engine and its observable rewind cache from the scenario's initial state.
#[unsafe(no_mangle)]
pub extern "C" fn physics_tower_demo_reset() -> u32 {
    let Some(replacement) = TowerDemoState::new() else {
        return 0;
    };
    let Ok(mut state) = tower_demo_state().lock() else {
        return 0;
    };
    *state = Some(replacement);
    1
}

/// Drops the engine and cached views. The next frame read creates a fresh scenario.
#[unsafe(no_mangle)]
pub extern "C" fn physics_tower_demo_dispose() -> u32 {
    let Ok(mut state) = tower_demo_state().lock() else {
        return 0;
    };
    u32::from(state.take().is_some())
}

#[unsafe(no_mangle)]
pub extern "C" fn physics_tower_demo_max_steps() -> u32 {
    TOWER_DEMO_MAX_STEPS
}

#[unsafe(no_mangle)]
pub extern "C" fn physics_tower_demo_body_count() -> u32 {
    u32::try_from(TOWER_BODY_COUNT).unwrap_or_default()
}

#[unsafe(no_mangle)]
pub extern "C" fn physics_tower_demo_body_role(body_index: u32) -> u32 {
    match usize::try_from(body_index).ok() {
        Some(FLOOR_INDEX) => 0,
        Some(PROJECTILE_INDEX) => 1,
        Some(index) if index < TOWER_BODY_COUNT => 2,
        _ => u32::MAX,
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn physics_tower_demo_vertex_x(
    body_index: u32,
    vertex_index: u32,
    steps: u32,
) -> f32 {
    vertex(body_index, vertex_index, steps).map_or(0.0, |value| display_coordinate(value.x))
}

#[unsafe(no_mangle)]
pub extern "C" fn physics_tower_demo_vertex_y(
    body_index: u32,
    vertex_index: u32,
    steps: u32,
) -> f32 {
    vertex(body_index, vertex_index, steps).map_or(0.0, |value| display_coordinate(value.y))
}

#[unsafe(no_mangle)]
pub extern "C" fn physics_tower_demo_vertex_z(
    body_index: u32,
    vertex_index: u32,
    steps: u32,
) -> f32 {
    vertex(body_index, vertex_index, steps).map_or(0.0, |value| display_coordinate(value.z))
}

#[unsafe(no_mangle)]
pub extern "C" fn physics_tower_demo_spinning_bodies(steps: u32) -> u32 {
    with_frame(steps, |frame| {
        u32::try_from(frame.stats.spinning_bodies).ok()
    })
    .unwrap_or_default()
}

#[unsafe(no_mangle)]
pub extern "C" fn physics_tower_demo_sampled_events(steps: u32) -> u32 {
    with_frame(steps, |frame| {
        u32::try_from(frame.stats.sampled_events).ok()
    })
    .unwrap_or_default()
}

#[unsafe(no_mangle)]
pub extern "C" fn physics_tower_demo_tail_contacts(steps: u32) -> u32 {
    with_frame(steps, |frame| u32::try_from(frame.stats.tail_contacts).ok()).unwrap_or_default()
}

#[cfg(test)]
mod tests {
    use ecs_physics_3d::physics_engine_boxes_penetrate;

    use super::*;

    fn assert_no_floor_penetration(frame: &TowerFrame, step: u32) {
        let floor = frame.boxes[FLOOR_INDEX];
        for (body_index, rigid_box) in frame.boxes.iter().enumerate().skip(PROJECTILE_INDEX) {
            let penetrates = physics_engine_boxes_penetrate(floor, *rigid_box)
                .expect("physics-engine floor/body geometry should remain valid");
            assert!(
                !penetrates,
                "body {body_index} / entity {} penetrated the finite floor collider at frame {step}",
                rigid_box.body.entity.0
            );
        }
    }

    #[test]
    fn tower_fast_slice_preserves_preimpact_floor_and_impact_behavior() {
        let mut state = TowerDemoState::new().expect("valid tower fixture");
        let mut maximum_spinning_blocks = 0_usize;
        let mut projectile_passed_front_face = false;

        for step in 0..=60 {
            let frame = state.ensure_frame(step).expect("valid tower frame");
            if step <= 30 {
                for rigid_box in &frame.boxes[FIRST_BLOCK_INDEX..] {
                    assert_eq!(
                        rigid_box.state.angular.angular_velocity,
                        AngularVelocity3d::default(),
                        "tower block {} started spinning before impact at frame {step}",
                        rigid_box.body.entity.0
                    );
                    assert_eq!(
                        rigid_box.state.angular.orientation,
                        Orientation3d::IDENTITY,
                        "tower block {} rotated before impact at frame {step}",
                        rigid_box.body.entity.0
                    );
                }
            }

            let spinning_blocks = frame.boxes[FIRST_BLOCK_INDEX..]
                .iter()
                .filter(|rigid_box| !rigid_box.state.angular.angular_velocity.is_zero())
                .count();
            maximum_spinning_blocks = maximum_spinning_blocks.max(spinning_blocks);
            projectile_passed_front_face |=
                frame.boxes[PROJECTILE_INDEX].state.center.x > TOWER_SCALE;
            assert_no_floor_penetration(frame, step);
        }

        assert!(projectile_passed_front_face);
        assert!(maximum_spinning_blocks >= 4);
    }

    #[test]
    #[ignore = "long deterministic tower acceptance runs in Runtime evidence and Moonlight"]
    fn tower_long_horizon_regression() {
        let mut state = TowerDemoState::new().expect("valid tower fixture");
        let mut maximum_spinning_blocks = 0_usize;
        let mut projectile_passed_front_face = false;

        for step in 0..=TOWER_DEMO_MAX_STEPS {
            let frame = state.ensure_frame(step).expect("valid tower frame");
            if step <= 180 {
                let spinning_blocks = frame.boxes[FIRST_BLOCK_INDEX..]
                    .iter()
                    .filter(|rigid_box| !rigid_box.state.angular.angular_velocity.is_zero())
                    .count();
                maximum_spinning_blocks = maximum_spinning_blocks.max(spinning_blocks);
                projectile_passed_front_face |=
                    frame.boxes[PROJECTILE_INDEX].state.center.x > TOWER_SCALE;
            }
            assert_eq!(frame.boxes.len(), TOWER_BODY_COUNT);
            crate::physics_trace::record(
                "tower",
                step,
                &frame.boxes,
                Some(frame.stats.sampled_events),
                Some(frame.stats.tail_contacts),
            );
            crate::physics_trace::record_vertices(step, &frame.vertices);
            assert_no_floor_penetration(frame, step);
        }

        assert!(projectile_passed_front_face);
        assert!(maximum_spinning_blocks >= 4);
        let work = state.physics.work();
        assert_eq!(work.world_constructions, 1);
        assert_eq!(work.insertions, 32);
        assert_eq!(work.input_conversions, 32);
        assert_eq!(work.angular_scan_visits, 32 * 480);
        assert_eq!(work.output_conversions, 32 * 480);
        assert_eq!(work.descriptor_commands, 0);
        assert_eq!(work.motion_commands, 0);
        let before_rewind = work;
        state.ensure_frame(1).expect("cached rewind view");
        assert_eq!(state.physics.work(), before_rewind);
        let cache_payload = state.frames.capacity() * std::mem::size_of::<super::TowerFrame>()
            + state
                .frames
                .iter()
                .map(|frame| {
                    frame.boxes.capacity() * std::mem::size_of::<ecs_physics_3d::RigidBox3d>()
                        + frame.vertices.capacity()
                            * std::mem::size_of::<[ecs_workload::Position; 8]>()
                })
                .sum::<usize>();
        println!("PERSISTENT_WORK tower {work:?}");
        println!("PERSISTENT_RETAINED tower {:?}", state.physics.retained());
        println!("CACHE_PAYLOAD_BYTES tower {cache_payload}");
    }

    #[test]
    fn tower_body_roles_are_stable() {
        assert_eq!(physics_tower_demo_body_count(), 32);
        assert_eq!(physics_tower_demo_body_role(0), 0);
        assert_eq!(physics_tower_demo_body_role(1), 1);
        assert_eq!(physics_tower_demo_body_role(2), 2);
        assert_eq!(physics_tower_demo_body_role(31), 2);
        assert_eq!(physics_tower_demo_body_role(32), u32::MAX);
    }
}
