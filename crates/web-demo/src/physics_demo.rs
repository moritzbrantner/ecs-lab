use std::sync::{Mutex, OnceLock};

use ecs_physics::BodyKind;
use ecs_physics_3d::{BouncingRoom3dScenario, BroadPhaseBody3d, BroadPhaseFrame3d};
use ecs_reference::ReferenceWorld;

const PHYSICS_DEMO_FPS: u32 = 60;
const PHYSICS_DEMO_SECONDS: u32 = 10;
const PHYSICS_DEMO_MAX_STEPS: u32 = PHYSICS_DEMO_FPS * PHYSICS_DEMO_SECONDS;

static PHYSICS_DEMO_STATE: OnceLock<Mutex<Option<PhysicsDemoState>>> = OnceLock::new();

struct PhysicsDemoFrame {
    frame: BroadPhaseFrame3d,
    pair_words: Vec<u32>,
}

struct PhysicsDemoState {
    scenario: BouncingRoom3dScenario,
    world: ReferenceWorld,
    frames: Vec<PhysicsDemoFrame>,
    spatial_scale: i64,
}

impl PhysicsDemoState {
    fn new() -> Option<Self> {
        let scenario = BouncingRoom3dScenario::with_substeps_per_tick(PHYSICS_DEMO_FPS).ok()?;
        let spatial_scale = scenario.spatial_scale();
        let mut world = ReferenceWorld::new();
        world.replay(scenario.setup()).ok()?;
        let initial_frame = build_demo_frame(&scenario, &world, spatial_scale)?;
        Some(Self {
            scenario,
            world,
            frames: vec![initial_frame],
            spatial_scale,
        })
    }

    fn ensure_frame(&mut self, steps: u32) -> Option<&PhysicsDemoFrame> {
        if steps > PHYSICS_DEMO_MAX_STEPS {
            return None;
        }
        let target = usize::try_from(steps).ok()?;
        while self.frames.len() <= target {
            let physics = self.scenario.step(&self.world.snapshot()).ok()?;
            for operation in physics.operations() {
                self.world.apply(*operation).ok()?;
            }
            let frame = build_demo_frame(&self.scenario, &self.world, self.spatial_scale)?;
            self.frames.push(frame);
        }
        self.frames.get(target)
    }
}

fn build_demo_frame(
    scenario: &BouncingRoom3dScenario,
    world: &ReferenceWorld,
    spatial_scale: i64,
) -> Option<PhysicsDemoFrame> {
    let frame = scenario.broad_phase_frame(&world.snapshot()).ok()?;
    let pair_words = frame.pair_words_at_spatial_scale(spatial_scale)?;
    Some(PhysicsDemoFrame { frame, pair_words })
}

fn demo_state() -> &'static Mutex<Option<PhysicsDemoState>> {
    PHYSICS_DEMO_STATE.get_or_init(|| Mutex::new(None))
}

fn ensure_state(state: &mut Option<PhysicsDemoState>) -> Option<&mut PhysicsDemoState> {
    if state.is_none() {
        *state = PhysicsDemoState::new();
    }
    state.as_mut()
}

fn frame_body(body_index: u32, steps: u32) -> Option<(BroadPhaseBody3d, i64)> {
    let index = usize::try_from(body_index).ok()?;
    let mut state = demo_state().lock().ok()?;
    let state = ensure_state(&mut state)?;
    let spatial_scale = state.spatial_scale;
    let body = state
        .ensure_frame(steps)?
        .frame
        .bodies()
        .get(index)
        .copied()?;
    Some((body, spatial_scale))
}

fn display_coordinate(value: i64, spatial_scale: i64) -> f32 {
    value as f32 / spatial_scale as f32
}

fn display_extent(value: i32, spatial_scale: i64) -> f32 {
    value as f32 / spatial_scale as f32
}

#[unsafe(no_mangle)]
pub extern "C" fn physics_demo_fps() -> u32 {
    PHYSICS_DEMO_FPS
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
    let Some(state) = ensure_state(&mut state) else {
        return 0;
    };
    let Some(frame) = state.ensure_frame(steps) else {
        return 0;
    };
    u32::try_from(frame.frame.bodies().len()).unwrap_or_default()
}

#[unsafe(no_mangle)]
pub extern "C" fn physics_demo_entity_id(body_index: u32, steps: u32) -> u32 {
    frame_body(body_index, steps).map_or(u32::MAX, |(body, _)| body.entity.0)
}

#[unsafe(no_mangle)]
pub extern "C" fn physics_demo_position_x(body_index: u32, steps: u32) -> f32 {
    frame_body(body_index, steps).map_or(0.0, |(body, spatial_scale)| {
        display_coordinate(body.position.x, spatial_scale)
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn physics_demo_position_y(body_index: u32, steps: u32) -> f32 {
    frame_body(body_index, steps).map_or(0.0, |(body, spatial_scale)| {
        display_coordinate(body.position.y, spatial_scale)
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn physics_demo_position_z(body_index: u32, steps: u32) -> f32 {
    frame_body(body_index, steps).map_or(0.0, |(body, spatial_scale)| {
        display_coordinate(body.position.z, spatial_scale)
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn physics_demo_half_extent_x(body_index: u32, steps: u32) -> f32 {
    frame_body(body_index, steps).map_or(0.0, |(body, spatial_scale)| {
        display_extent(body.half_extents[0], spatial_scale)
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn physics_demo_half_extent_y(body_index: u32, steps: u32) -> f32 {
    frame_body(body_index, steps).map_or(0.0, |(body, spatial_scale)| {
        display_extent(body.half_extents[1], spatial_scale)
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn physics_demo_half_extent_z(body_index: u32, steps: u32) -> f32 {
    frame_body(body_index, steps).map_or(0.0, |(body, spatial_scale)| {
        display_extent(body.half_extents[2], spatial_scale)
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn physics_demo_is_fixed(body_index: u32, steps: u32) -> u32 {
    frame_body(body_index, steps).map_or(0, |(body, _)| {
        if body.kind == BodyKind::Fixed { 1 } else { 0 }
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn physics_demo_mass_units(body_index: u32, steps: u32) -> u32 {
    frame_body(body_index, steps).map_or(0, |(body, _)| body.mass_units)
}

#[unsafe(no_mangle)]
pub extern "C" fn physics_demo_restitution_milli(body_index: u32, steps: u32) -> u32 {
    frame_body(body_index, steps)
        .map_or(0, |(body, _)| u32::from(body.material.restitution_milli))
}

#[unsafe(no_mangle)]
pub extern "C" fn physics_demo_friction_milli(body_index: u32, steps: u32) -> u32 {
    frame_body(body_index, steps)
        .map_or(0, |(body, _)| u32::from(body.material.friction_milli))
}

#[unsafe(no_mangle)]
pub extern "C" fn physics_demo_pair_word_count(steps: u32) -> u32 {
    let Ok(mut state) = demo_state().lock() else {
        return 0;
    };
    let Some(state) = ensure_state(&mut state) else {
        return 0;
    };
    let Some(frame) = state.ensure_frame(steps) else {
        return 0;
    };
    u32::try_from(frame.pair_words.len()).unwrap_or_default()
}

#[unsafe(no_mangle)]
pub extern "C" fn physics_demo_pair_word(word_index: u32, steps: u32) -> u32 {
    let Ok(index) = usize::try_from(word_index) else {
        return 0;
    };
    let Ok(mut state) = demo_state().lock() else {
        return 0;
    };
    let Some(state) = ensure_state(&mut state) else {
        return 0;
    };
    let Some(frame) = state.ensure_frame(steps) else {
        return 0;
    };
    frame.pair_words.get(index).copied().unwrap_or(0)
}

#[unsafe(no_mangle)]
pub extern "C" fn physics_demo_overlap_count(steps: u32) -> u32 {
    let Ok(mut state) = demo_state().lock() else {
        return 0;
    };
    let Some(state) = ensure_state(&mut state) else {
        return 0;
    };
    let Some(frame) = state.ensure_frame(steps) else {
        return 0;
    };
    frame
        .pair_words
        .iter()
        .map(|word| word.count_ones())
        .sum()
}

#[cfg(test)]
mod tests {
    use super::{
        physics_demo_body_count, physics_demo_fps, physics_demo_half_extent_z,
        physics_demo_is_fixed, physics_demo_max_steps, physics_demo_position_z,
    };

    #[test]
    fn browser_demo_exposes_true_sixty_hz_three_dimensional_steps() {
        assert_eq!(physics_demo_fps(), 60);
        assert_eq!(physics_demo_max_steps(), 600);
        assert_eq!(physics_demo_body_count(0), 54);
        assert_eq!(physics_demo_is_fixed(48, 0), 1);
        assert!(physics_demo_half_extent_z(48, 0) > 1.0);

        let initial_z = physics_demo_position_z(0, 0);
        let next_z = physics_demo_position_z(0, 1);
        assert_ne!(initial_z, next_z);
        assert!((next_z - initial_z).abs() < 1.0);

        let one_second_z = physics_demo_position_z(0, 60);
        assert!((one_second_z - initial_z).abs() > 1.0);

        let first_repeat = physics_demo_position_z(0, 6);
        let second_repeat = physics_demo_position_z(0, 6);
        assert_eq!(first_repeat, second_repeat);
    }
}
