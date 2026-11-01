#[path = "tower_demo.rs"]
mod tower_demo;

use std::sync::{Mutex, OnceLock};

use ecs_physics::PhysicsMaterial;
use ecs_physics_3d::{
    ANGULAR_VELOCITY_SCALE, AngularState3d, AngularVelocity3d, ORIENTATION_SCALE, Orientation3d,
    PairContact3d, PhysicsBody3d, RapierWorld3d, RapierWorldConfig3d, RigidBox3d, RigidBoxState3d,
    oriented_box_vertices,
};
use ecs_workload::{EntityId, Position, Velocity};

const DICE_DEMO_FPS: u32 = 60;
const DICE_DEMO_SECONDS: u32 = 6;
const DICE_DEMO_MAX_STEPS: u32 = DICE_DEMO_FPS * DICE_DEMO_SECONDS;
const DICE_SPATIAL_SCALE: i64 = 3_600;
const DICE_HALF_EXTENT: i32 = 3_600;
/// Initial tilt about +Z, in radians.
const DICE_INITIAL_TILT: f64 = 0.332;
/// Rapier angular damping coefficient (1/s): about 0.5 % angular speed loss per 60 Hz frame.
const DICE_ANGULAR_DAMPING: f32 = 0.3;
const DICE_SOLVER_ITERATIONS: u8 = 4;
const DICE_INDEX: usize = 0;
const FLOOR_INDEX: usize = 1;

static DICE_DEMO_STATE: OnceLock<Mutex<Option<DiceDemoState>>> = OnceLock::new();

#[derive(Clone, Copy, Debug)]
struct DiceFrame {
    state: RigidBoxState3d,
    vertices: [Position; 8],
    contact: Option<PairContact3d>,
}

struct DiceDemoState {
    physics: RapierWorld3d,
    frames: Vec<DiceFrame>,
}

impl DiceDemoState {
    fn new() -> Option<Self> {
        let dice = RigidBox3d::new(
            PhysicsBody3d::dynamic(
                EntityId(0),
                [DICE_HALF_EXTENT, DICE_HALF_EXTENT, DICE_HALF_EXTENT],
            )
            .with_material(PhysicsMaterial::new(550, 350)),
            RigidBoxState3d::new(
                Position::new3(0, 9 * DICE_SPATIAL_SCALE, 0),
                Velocity::new3(2 * DICE_HALF_EXTENT, -DICE_HALF_EXTENT, 0),
                AngularState3d::new(
                    Orientation3d::from_axis_angle([0.0, 0.0, 1.0], DICE_INITIAL_TILT),
                    AngularVelocity3d::default(),
                ),
            ),
        );
        // The plane y = 0 is the top face of a wide fixed slab.
        let floor = RigidBox3d::new(
            PhysicsBody3d::fixed(EntityId(1), [72_000, DICE_HALF_EXTENT, 72_000])
                .with_material(PhysicsMaterial::new(250, 700)),
            RigidBoxState3d::new(
                Position::new3(0, -DICE_SPATIAL_SCALE, 0),
                Velocity::new3(0, 0, 0),
                AngularState3d::default(),
            ),
        );
        let config = RapierWorldConfig3d {
            spatial_scale: DICE_SPATIAL_SCALE,
            gravity: Velocity::new3(0, -36_000, 0),
            hz: DICE_DEMO_FPS,
            angular_damping: DICE_ANGULAR_DAMPING,
            solver_iterations: DICE_SOLVER_ITERATIONS,
            lock_rotations: false,
        };
        let physics = RapierWorld3d::new(&[dice, floor], config).ok()?;
        let initial = capture_frame(physics.boxes()[DICE_INDEX], None);
        Some(Self {
            physics,
            frames: vec![initial],
        })
    }

    fn ensure_frame(&mut self, steps: u32) -> Option<&DiceFrame> {
        if steps > DICE_DEMO_MAX_STEPS {
            return None;
        }
        let target = usize::try_from(steps).ok()?;
        while self.frames.len() <= target {
            let next = self.physics.advance().ok()?;
            let contact = self.physics.contact(DICE_INDEX, FLOOR_INDEX);
            self.frames
                .push(capture_frame(*next.boxes().get(DICE_INDEX)?, contact));
        }
        self.frames.get(target)
    }
}

fn capture_frame(dice: RigidBox3d, contact: Option<PairContact3d>) -> DiceFrame {
    DiceFrame {
        state: dice.state,
        vertices: oriented_box_vertices(
            dice.state.center,
            dice.body.half_extents,
            dice.state.angular.orientation,
        ),
        contact,
    }
}

fn dice_demo_state() -> &'static Mutex<Option<DiceDemoState>> {
    DICE_DEMO_STATE.get_or_init(|| Mutex::new(None))
}

fn ensure_state(state: &mut Option<DiceDemoState>) -> Option<&mut DiceDemoState> {
    if state.is_none() {
        *state = DiceDemoState::new();
    }
    state.as_mut()
}

fn frame(steps: u32) -> Option<DiceFrame> {
    let mut state = dice_demo_state().lock().ok()?;
    ensure_state(&mut state)?.ensure_frame(steps).copied()
}

fn vertex(vertex_index: u32, steps: u32) -> Option<Position> {
    let index = usize::try_from(vertex_index).ok()?;
    frame(steps)?.vertices.get(index).copied()
}

fn display_coordinate(value: i64) -> f32 {
    value as f32 / DICE_SPATIAL_SCALE as f32
}

fn display_orientation(value: i32) -> f32 {
    value as f32 / ORIENTATION_SCALE as f32
}

fn display_angular_velocity(value: i32) -> f32 {
    value as f32 / ANGULAR_VELOCITY_SCALE as f32
}

#[unsafe(no_mangle)]
pub extern "C" fn physics_dice_demo_max_steps() -> u32 {
    DICE_DEMO_MAX_STEPS
}

#[unsafe(no_mangle)]
pub extern "C" fn physics_dice_demo_center_x(steps: u32) -> f32 {
    frame(steps).map_or(0.0, |value| display_coordinate(value.state.center.x))
}

#[unsafe(no_mangle)]
pub extern "C" fn physics_dice_demo_center_y(steps: u32) -> f32 {
    frame(steps).map_or(0.0, |value| display_coordinate(value.state.center.y))
}

#[unsafe(no_mangle)]
pub extern "C" fn physics_dice_demo_center_z(steps: u32) -> f32 {
    frame(steps).map_or(0.0, |value| display_coordinate(value.state.center.z))
}

#[unsafe(no_mangle)]
pub extern "C" fn physics_dice_demo_vertex_x(vertex_index: u32, steps: u32) -> f32 {
    vertex(vertex_index, steps).map_or(0.0, |value| display_coordinate(value.x))
}

#[unsafe(no_mangle)]
pub extern "C" fn physics_dice_demo_vertex_y(vertex_index: u32, steps: u32) -> f32 {
    vertex(vertex_index, steps).map_or(0.0, |value| display_coordinate(value.y))
}

#[unsafe(no_mangle)]
pub extern "C" fn physics_dice_demo_vertex_z(vertex_index: u32, steps: u32) -> f32 {
    vertex(vertex_index, steps).map_or(0.0, |value| display_coordinate(value.z))
}

#[unsafe(no_mangle)]
pub extern "C" fn physics_dice_demo_orientation_x(steps: u32) -> f32 {
    frame(steps).map_or(0.0, |value| {
        display_orientation(value.state.angular.orientation.x)
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn physics_dice_demo_orientation_y(steps: u32) -> f32 {
    frame(steps).map_or(0.0, |value| {
        display_orientation(value.state.angular.orientation.y)
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn physics_dice_demo_orientation_z(steps: u32) -> f32 {
    frame(steps).map_or(0.0, |value| {
        display_orientation(value.state.angular.orientation.z)
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn physics_dice_demo_orientation_w(steps: u32) -> f32 {
    frame(steps).map_or(0.0, |value| {
        display_orientation(value.state.angular.orientation.w)
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn physics_dice_demo_angular_velocity_x(steps: u32) -> f32 {
    frame(steps).map_or(0.0, |value| {
        display_angular_velocity(value.state.angular.angular_velocity.x)
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn physics_dice_demo_angular_velocity_y(steps: u32) -> f32 {
    frame(steps).map_or(0.0, |value| {
        display_angular_velocity(value.state.angular.angular_velocity.y)
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn physics_dice_demo_angular_velocity_z(steps: u32) -> f32 {
    frame(steps).map_or(0.0, |value| {
        display_angular_velocity(value.state.angular.angular_velocity.z)
    })
}

/// Floor contact points Rapier's solver saw in this frame's step.
#[unsafe(no_mangle)]
pub extern "C" fn physics_dice_demo_contact_vertices(steps: u32) -> u32 {
    frame(steps)
        .and_then(|value| value.contact)
        .map_or(0, |contact| contact.points)
}

/// Sum of the floor contact's normal impulses (mass units × metres per second).
#[unsafe(no_mangle)]
pub extern "C" fn physics_dice_demo_normal_impulse(steps: u32) -> f32 {
    frame(steps)
        .and_then(|value| value.contact)
        .map_or(0.0, |contact| contact.normal_impulse)
}

#[cfg(test)]
mod tests {
    use super::{
        DiceDemoState, physics_dice_demo_angular_velocity_z, physics_dice_demo_contact_vertices,
        physics_dice_demo_max_steps, physics_dice_demo_vertex_y,
    };

    /// Allowed sinking of a corner below the floor plane in any frame, in metres (the die is 2 m
    /// wide). Rapier's soft contacts let the first impact overlap about 0.06 m for a frame.
    const FLOOR_TOLERANCE: f32 = 0.1;
    /// Allowed sinking once the die has come to rest, in metres.
    const RESTING_TOLERANCE: f32 = 0.01;

    #[test]
    fn dice_fixture_generates_spin_from_its_off_center_floor_impact() {
        assert_eq!(physics_dice_demo_max_steps(), 360);
        assert_eq!(physics_dice_demo_angular_velocity_z(0), 0.0);
        assert!((1..=180).any(|step| physics_dice_demo_angular_velocity_z(step).abs() > 0.001));
        assert!((1..=180).any(|step| physics_dice_demo_contact_vertices(step) > 0));
    }

    #[test]
    fn exported_oriented_vertices_stay_on_the_plane() {
        for step in 0..=360 {
            for vertex in 0..8 {
                let y = physics_dice_demo_vertex_y(vertex, step);
                assert!(
                    y >= -FLOOR_TOLERANCE,
                    "vertex {vertex} at {y} m, frame {step}"
                );
            }
        }
        for vertex in 0..8 {
            let y = physics_dice_demo_vertex_y(vertex, 360);
            assert!(y >= -RESTING_TOLERANCE, "resting vertex {vertex} at {y} m");
        }
    }

    #[test]
    fn dice_replays_identically_on_the_same_machine() {
        let mut first = DiceDemoState::new().expect("valid dice fixture");
        let mut second = DiceDemoState::new().expect("valid dice fixture");
        first.ensure_frame(360).expect("dice frames");
        second.ensure_frame(360).expect("dice frames");
        for (left, right) in first.frames.iter().zip(&second.frames) {
            assert_eq!(left.state, right.state);
            assert_eq!(left.contact, right.contact);
        }
    }
}
