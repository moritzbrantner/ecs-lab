use std::sync::{Mutex, OnceLock};

use ecs_physics::BodyKind;
use ecs_physics_3d::{BouncingRoom3dScenario, BroadPhaseBody3d, BroadPhaseFrame3d};

const PHYSICS_DEMO_MAX_STEPS: u32 = 64;

static PHYSICS_DEMO_STATE: OnceLock<Mutex<PhysicsDemoState>> = OnceLock::new();

#[derive(Clone, Debug, PartialEq)]
struct PhysicsDemoFrame {
    steps: u32,
    frame: BroadPhaseFrame3d,
}

#[derive(Debug, Default)]
struct PhysicsDemoState {
    frame: Option<PhysicsDemoFrame>,
}

fn build_frame(steps: u32) -> Option<PhysicsDemoFrame> {
    if steps > PHYSICS_DEMO_MAX_STEPS {
        return None;
    }
    let frame = BouncingRoom3dScenario::new()
        .broad_phase_frame_after(steps)
        .ok()?;
    Some(PhysicsDemoFrame { steps, frame })
}

fn demo_state() -> &'static Mutex<PhysicsDemoState> {
    PHYSICS_DEMO_STATE.get_or_init(|| Mutex::new(PhysicsDemoState::default()))
}

fn ensure_frame(state: &mut PhysicsDemoState, steps: u32) -> Option<&PhysicsDemoFrame> {
    let needs_rebuild = state
        .frame
        .as_ref()
        .is_none_or(|frame| frame.steps != steps);
    if needs_rebuild {
        state.frame = build_frame(steps);
    }
    state.frame.as_ref()
}

fn frame_body(body_index: u32, steps: u32) -> Option<BroadPhaseBody3d> {
    let index = usize::try_from(body_index).ok()?;
    let mut state = demo_state().lock().ok()?;
    ensure_frame(&mut state, steps)?
        .frame
        .bodies()
        .get(index)
        .copied()
}

#[unsafe(no_mangle)]
pub extern "C" fn physics_demo_max_steps() -> u32 {
    PHYSICS_DEMO_MAX_STEPS
}

#[unsafe(no_mangle)]
pub extern "C" fn physics_demo_body_count(steps: u32) -> u32 {
    let Ok(mut state) = demo_state().lock() else {
        return 0;
    };
    let Some(frame) = ensure_frame(&mut state, steps) else {
        return 0;
    };
    u32::try_from(frame.frame.bodies().len()).unwrap_or_default()
}

#[unsafe(no_mangle)]
pub extern "C" fn physics_demo_entity_id(body_index: u32, steps: u32) -> u32 {
    frame_body(body_index, steps).map_or(u32::MAX, |body| body.entity.0)
}

#[unsafe(no_mangle)]
pub extern "C" fn physics_demo_position_x(body_index: u32, steps: u32) -> i32 {
    frame_body(body_index, steps)
        .and_then(|body| i32::try_from(body.position.x).ok())
        .unwrap_or_default()
}

#[unsafe(no_mangle)]
pub extern "C" fn physics_demo_position_y(body_index: u32, steps: u32) -> i32 {
    frame_body(body_index, steps)
        .and_then(|body| i32::try_from(body.position.y).ok())
        .unwrap_or_default()
}

#[unsafe(no_mangle)]
pub extern "C" fn physics_demo_position_z(body_index: u32, steps: u32) -> i32 {
    frame_body(body_index, steps)
        .and_then(|body| i32::try_from(body.position.z).ok())
        .unwrap_or_default()
}

#[unsafe(no_mangle)]
pub extern "C" fn physics_demo_half_extent_x(body_index: u32, steps: u32) -> i32 {
    frame_body(body_index, steps).map_or(0, |body| body.half_extents[0])
}

#[unsafe(no_mangle)]
pub extern "C" fn physics_demo_half_extent_y(body_index: u32, steps: u32) -> i32 {
    frame_body(body_index, steps).map_or(0, |body| body.half_extents[1])
}

#[unsafe(no_mangle)]
pub extern "C" fn physics_demo_half_extent_z(body_index: u32, steps: u32) -> i32 {
    frame_body(body_index, steps).map_or(0, |body| body.half_extents[2])
}

#[unsafe(no_mangle)]
pub extern "C" fn physics_demo_is_fixed(body_index: u32, steps: u32) -> u32 {
    frame_body(body_index, steps).map_or(0, |body| if body.kind == BodyKind::Fixed { 1 } else { 0 })
}

#[unsafe(no_mangle)]
pub extern "C" fn physics_demo_mass_units(body_index: u32, steps: u32) -> u32 {
    frame_body(body_index, steps).map_or(0, |body| body.mass_units)
}

#[unsafe(no_mangle)]
pub extern "C" fn physics_demo_restitution_milli(body_index: u32, steps: u32) -> u32 {
    frame_body(body_index, steps).map_or(0, |body| u32::from(body.material.restitution_milli))
}

#[unsafe(no_mangle)]
pub extern "C" fn physics_demo_friction_milli(body_index: u32, steps: u32) -> u32 {
    frame_body(body_index, steps).map_or(0, |body| u32::from(body.material.friction_milli))
}

#[unsafe(no_mangle)]
pub extern "C" fn physics_demo_pair_word_count(steps: u32) -> u32 {
    let Ok(mut state) = demo_state().lock() else {
        return 0;
    };
    let Some(frame) = ensure_frame(&mut state, steps) else {
        return 0;
    };
    u32::try_from(frame.frame.pair_words().len()).unwrap_or_default()
}

#[unsafe(no_mangle)]
pub extern "C" fn physics_demo_pair_word(word_index: u32, steps: u32) -> u32 {
    let Ok(index) = usize::try_from(word_index) else {
        return 0;
    };
    let Ok(mut state) = demo_state().lock() else {
        return 0;
    };
    let Some(frame) = ensure_frame(&mut state, steps) else {
        return 0;
    };
    frame.frame.pair_words().get(index).copied().unwrap_or(0)
}

#[unsafe(no_mangle)]
pub extern "C" fn physics_demo_overlap_count(steps: u32) -> u32 {
    let Ok(mut state) = demo_state().lock() else {
        return 0;
    };
    ensure_frame(&mut state, steps).map_or(0, |frame| frame.frame.overlap_count())
}

#[cfg(test)]
mod tests {
    use super::{
        build_frame, physics_demo_body_count, physics_demo_half_extent_z, physics_demo_is_fixed,
        physics_demo_max_steps, physics_demo_position_z,
    };

    #[test]
    fn browser_demo_exposes_the_true_three_dimensional_room() {
        assert_eq!(physics_demo_max_steps(), 64);
        assert_eq!(physics_demo_body_count(0), 54);
        assert_eq!(physics_demo_is_fixed(48, 0), 1);
        assert!(physics_demo_half_extent_z(48, 0) > 1);

        let initial_z = physics_demo_position_z(0, 0);
        let next_z = physics_demo_position_z(0, 1);
        assert_ne!(initial_z, next_z);

        let first = build_frame(6).expect("3D browser frame should be valid");
        let second = build_frame(6).expect("repeated 3D browser frame should be valid");
        assert_eq!(first, second);
    }
}
