use std::sync::{Mutex, OnceLock};

use ecs_physics::PhysicsMaterial;
use ecs_physics_3d::{
    ANGULAR_VELOCITY_SCALE, AngularState3d, AngularVelocity3d, BoxPlaneConfig3d, BoxPlaneContact3d,
    BoxPlaneState3d, ORIENTATION_SCALE, Orientation3d, PhysicsBody3d, oriented_box_vertices,
    step_box_on_plane,
};
use ecs_workload::{EntityId, Position, Velocity};

const DICE_DEMO_FPS: u32 = 60;
const DICE_DEMO_SECONDS: u32 = 6;
const DICE_DEMO_MAX_STEPS: u32 = DICE_DEMO_FPS * DICE_DEMO_SECONDS;
const DICE_SPATIAL_SCALE: i64 = 3_600;
const DICE_HALF_EXTENT: i32 = 3_600;
const DICE_INITIAL_TILT_Z: i32 = 180_000_000;

static DICE_DEMO_STATE: OnceLock<Mutex<Option<DiceDemoState>>> = OnceLock::new();

#[derive(Clone, Copy, Debug)]
struct DiceFrame {
    state: BoxPlaneState3d,
    vertices: [Position; 8],
    contact: Option<BoxPlaneContact3d>,
}

struct DiceDemoState {
    body: PhysicsBody3d,
    config: BoxPlaneConfig3d,
    frames: Vec<DiceFrame>,
}

impl DiceDemoState {
    fn new() -> Option<Self> {
        let orientation = Orientation3d::new(0, 0, DICE_INITIAL_TILT_Z, ORIENTATION_SCALE)
            .normalized()
            .ok()?;
        let body = PhysicsBody3d::dynamic(
            EntityId(0),
            [DICE_HALF_EXTENT, DICE_HALF_EXTENT, DICE_HALF_EXTENT],
        )
        .with_material(PhysicsMaterial::new(550, 350));
        let config = BoxPlaneConfig3d {
            gravity: Velocity::new3(0, -36_000, 0),
            plane_y: 0,
            timestep_numerator: 1,
            timestep_denominator: i32::try_from(DICE_DEMO_FPS).ok()?,
            plane_material: PhysicsMaterial::new(250, 700),
            angular_damping_milli: 995,
        };
        let state = BoxPlaneState3d::new(
            Position::new3(0, 9 * DICE_SPATIAL_SCALE, 0),
            Velocity::new3(2 * DICE_HALF_EXTENT, -DICE_HALF_EXTENT, 0),
            AngularState3d::new(orientation, AngularVelocity3d::default()),
        );
        let initial = capture_frame(state, body, None)?;
        Some(Self {
            body,
            config,
            frames: vec![initial],
        })
    }

    fn ensure_frame(&mut self, steps: u32) -> Option<&DiceFrame> {
        if steps > DICE_DEMO_MAX_STEPS {
            return None;
        }
        let target = usize::try_from(steps).ok()?;
        while self.frames.len() <= target {
            let previous = self.frames.last()?.state;
            let next = step_box_on_plane(previous, self.body, self.config).ok()?;
            self.frames
                .push(capture_frame(next.state, self.body, next.contact)?);
        }
        self.frames.get(target)
    }
}

fn capture_frame(
    state: BoxPlaneState3d,
    body: PhysicsBody3d,
    contact: Option<BoxPlaneContact3d>,
) -> Option<DiceFrame> {
    Some(DiceFrame {
        state,
        vertices: oriented_box_vertices(state.center, body.half_extents, state.angular.orientation)
            .ok()?,
        contact,
    })
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

#[unsafe(no_mangle)]
pub extern "C" fn physics_dice_demo_contact_vertices(steps: u32) -> u32 {
    frame(steps)
        .and_then(|value| value.contact)
        .map_or(0, |contact| u32::from(contact.manifold_vertices))
}

#[unsafe(no_mangle)]
pub extern "C" fn physics_dice_demo_normal_impulse(steps: u32) -> f32 {
    frame(steps)
        .and_then(|value| value.contact)
        .map_or(0.0, |contact| {
            contact.normal_impulse_units as f32 / DICE_SPATIAL_SCALE as f32
        })
}

#[cfg(test)]
mod tests {
    use super::{
        physics_dice_demo_angular_velocity_z, physics_dice_demo_max_steps,
        physics_dice_demo_vertex_y,
    };

    #[test]
    fn dice_fixture_generates_spin_from_its_off_center_floor_impact() {
        assert_eq!(physics_dice_demo_max_steps(), 360);
        assert_eq!(physics_dice_demo_angular_velocity_z(0), 0.0);
        assert!((1..=180).any(|step| physics_dice_demo_angular_velocity_z(step).abs() > 0.001));
    }

    #[test]
    fn exported_oriented_vertices_do_not_finish_below_the_plane() {
        for step in 0..=180 {
            for vertex in 0..8 {
                assert!(physics_dice_demo_vertex_y(vertex, step) >= -0.001);
            }
        }
    }
}
