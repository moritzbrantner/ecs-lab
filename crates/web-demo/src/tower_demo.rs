use std::sync::{Mutex, OnceLock};

use ecs_physics::PhysicsMaterial;
use ecs_physics_3d::{
    AngularState3d, AngularVelocity3d, Orientation3d, PhysicsBody3d, RapierWorld3d,
    RapierWorldConfig3d, RigidBox3d, RigidBoxState3d, oriented_box_vertices,
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
/// Rapier angular damping coefficient (1/s): about 0.4 % angular speed loss per 60 Hz frame.
const TOWER_ANGULAR_DAMPING: f32 = 0.24;
const TOWER_SOLVER_ITERATIONS: u8 = 10;
/// A body counts as spinning above this angular speed (rad/s); smaller values are solver noise.
const SPINNING_THRESHOLD: f64 = 0.05;

static TOWER_DEMO_STATE: OnceLock<Mutex<Option<TowerDemoState>>> = OnceLock::new();

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
struct TowerFrameStats {
    spinning_bodies: usize,
    contact_pairs: usize,
}

#[derive(Clone, Debug, Eq, PartialEq)]
struct TowerFrame {
    #[cfg(test)]
    boxes: Vec<RigidBox3d>,
    vertices: Vec<[Position; 8]>,
    stats: TowerFrameStats,
}

struct TowerDemoState {
    physics: RapierWorld3d,
    frames: Vec<TowerFrame>,
}

impl TowerDemoState {
    fn new() -> Option<Self> {
        let boxes = initial_boxes()?;
        let config = RapierWorldConfig3d {
            spatial_scale: TOWER_SCALE,
            gravity: Velocity::new3(0, -10 * TOWER_EXTENT_SCALE, 0),
            hz: TOWER_DEMO_FPS,
            angular_damping: TOWER_ANGULAR_DAMPING,
            solver_iterations: TOWER_SOLVER_ITERATIONS,
            lock_rotations: false,
        };
        let physics = RapierWorld3d::new(&boxes, config).ok()?;
        let initial = capture_frame(physics.boxes().to_vec(), 0);
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
                        "tower physics failed while constructing frame {}: {error}",
                        self.frames.len()
                    );
                })
                .ok()?;
            let contact_pairs = next.contact_pairs();
            self.frames
                .push(capture_frame(next.into_boxes(), contact_pairs));
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
                        AngularState3d::default(),
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
            // Wide enough that no debris leaves the floor over the eight-second scene.
            [
                60 * TOWER_EXTENT_SCALE,
                TOWER_EXTENT_SCALE,
                30 * TOWER_EXTENT_SCALE,
            ],
        )
        .with_material(PhysicsMaterial::new(50, 850)),
        RigidBoxState3d::new(
            Position::new3(0, -TOWER_SCALE, 0),
            Velocity::new3(0, 0, 0),
            AngularState3d::default(),
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

fn is_spinning(rigid_box: &RigidBox3d) -> bool {
    rigid_box
        .state
        .angular
        .angular_velocity
        .radians_per_second()
        > SPINNING_THRESHOLD
}

fn capture_frame(boxes: Vec<RigidBox3d>, contact_pairs: usize) -> TowerFrame {
    let vertices = boxes
        .iter()
        .map(|rigid_box| {
            oriented_box_vertices(
                rigid_box.state.center,
                rigid_box.body.half_extents,
                rigid_box.state.angular.orientation,
            )
        })
        .collect();
    let stats = TowerFrameStats {
        spinning_bodies: boxes
            .iter()
            .filter(|rigid_box| is_spinning(rigid_box))
            .count(),
        contact_pairs,
    };
    TowerFrame {
        #[cfg(test)]
        boxes,
        vertices,
        stats,
    }
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

/// Recreates the Rapier world and its rewind cache from the scenario's initial state.
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

/// Drops the Rapier world and cached views. The next frame read creates a fresh scenario.
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

/// Collider pairs in active contact after the frame's Rapier step.
#[unsafe(no_mangle)]
pub extern "C" fn physics_tower_demo_contact_pairs(steps: u32) -> u32 {
    with_frame(steps, |frame| u32::try_from(frame.stats.contact_pairs).ok()).unwrap_or_default()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Allowed overlap between two colliders in any frame, in metres. Rapier's soft contacts let
    /// the 24 m/s projectile impact overlap transiently (about 0.42 m) and push it out over a few
    /// frames; this stays below the smallest block half extent (0.75 m), so nothing tunnels.
    const IMPACT_PENETRATION_TOLERANCE: f32 = 0.5;
    /// Allowed sinking of any corner below the floor plane in any frame, in metres: falling debris
    /// landing on a corner overlaps the floor transiently (about 0.13 m), well under a block's
    /// 0.75 m half height.
    const FLOOR_TOLERANCE: f32 = 0.2;
    /// Allowed overlap once the debris has come to rest, in metres.
    const RESTING_PENETRATION_TOLERANCE: f32 = 0.01;
    /// Allowed tilt of a resting block before the projectile arrives, in radians.
    const PREIMPACT_TILT_TOLERANCE: f64 = 0.01;

    fn max_penetration(state: &TowerDemoState) -> f32 {
        (PROJECTILE_INDEX..TOWER_BODY_COUNT)
            .map(|index| state.physics.penetration_of(index))
            .fold(0.0, f32::max)
    }

    fn assert_physical_quality(state: &TowerDemoState, frame: &TowerFrame, step: u32) {
        let penetration = max_penetration(state);
        assert!(
            penetration < IMPACT_PENETRATION_TOLERANCE,
            "colliders overlap by {penetration} m at frame {step}"
        );
        for (index, rigid_box) in frame.boxes.iter().enumerate().skip(PROJECTILE_INDEX) {
            let lowest = frame.vertices[index]
                .iter()
                .map(|corner| display_coordinate(corner.y))
                .fold(f32::INFINITY, f32::min);
            assert!(
                lowest > -FLOOR_TOLERANCE,
                "body {index} / entity {} sank below the floor ({lowest} m) at frame {step}",
                rigid_box.body.entity.0
            );
        }
    }

    /// Builds frames `0..=steps`, checking each one right after its Rapier step.
    fn run(steps: u32, mut each: impl FnMut(&TowerDemoState, &TowerFrame, u32)) -> TowerDemoState {
        let mut state = TowerDemoState::new().expect("valid tower fixture");
        for step in 0..=steps {
            state.ensure_frame(step).expect("valid tower frame");
            let frame = &state.frames[usize::try_from(step).expect("step index")];
            each(&state, frame, step);
        }
        state
    }

    fn spinning_blocks(frame: &TowerFrame) -> usize {
        frame.boxes[FIRST_BLOCK_INDEX..]
            .iter()
            .filter(|rigid_box| is_spinning(rigid_box))
            .count()
    }

    #[test]
    fn tower_fast_slice_stands_until_impact_then_breaks() {
        let mut maximum_spinning_blocks = 0_usize;
        let mut projectile_passed_front_face = false;
        run(90, |state, frame, step| {
            if step <= 30 {
                for rigid_box in &frame.boxes[FIRST_BLOCK_INDEX..] {
                    let tilt = rigid_box.state.angular.orientation.angle();
                    assert!(
                        tilt < PREIMPACT_TILT_TOLERANCE && !is_spinning(rigid_box),
                        "tower block {} moved before impact at frame {step}: tilt {tilt}",
                        rigid_box.body.entity.0
                    );
                }
            }
            maximum_spinning_blocks = maximum_spinning_blocks.max(spinning_blocks(frame));
            projectile_passed_front_face |=
                frame.boxes[PROJECTILE_INDEX].state.center.x > TOWER_SCALE;
            assert_physical_quality(state, frame, step);
        });
        assert!(projectile_passed_front_face);
        assert!(maximum_spinning_blocks >= 4, "{maximum_spinning_blocks}");
    }

    #[test]
    #[ignore = "long tower acceptance runs in Runtime evidence and Moonlight"]
    fn tower_long_horizon_regression() {
        let mut maximum_spinning_blocks = 0_usize;
        let mut projectile_passed_front_face = false;
        let first = run(TOWER_DEMO_MAX_STEPS, |state, frame, step| {
            if step <= 180 {
                maximum_spinning_blocks = maximum_spinning_blocks.max(spinning_blocks(frame));
                projectile_passed_front_face |=
                    frame.boxes[PROJECTILE_INDEX].state.center.x > TOWER_SCALE;
            }
            assert_eq!(frame.boxes.len(), TOWER_BODY_COUNT);
            crate::physics_trace::record(
                "tower",
                step,
                &frame.boxes,
                Some(frame.stats.contact_pairs),
            );
            crate::physics_trace::record_vertices(step, &frame.vertices);
            assert_physical_quality(state, frame, step);
        });
        assert!(projectile_passed_front_face);
        assert!(maximum_spinning_blocks >= 4, "{maximum_spinning_blocks}");

        // The debris comes to rest by the end of the scene.
        let last = first.frames.last().expect("final tower frame");
        assert_eq!(last.stats.spinning_bodies, 0);
        let resting = max_penetration(&first);
        assert!(
            resting < RESTING_PENETRATION_TOLERANCE,
            "resting overlap {resting} m"
        );

        // Same-machine replay: a fresh world reproduces every quantized frame exactly.
        let second = run(TOWER_DEMO_MAX_STEPS, |_, _, _| {});
        assert!(first.frames == second.frames, "tower replay diverged");
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
