use std::sync::OnceLock;

use ecs_physics_scenarios::{BroadPhaseFrame, FallingBoxesScenario, ScenarioError};
use ecs_reference::ReferenceWorld;
use ecs_workload::{EntityId, Operation, Position, Velocity};

const ENTITY: EntityId = EntityId(1);
const START: Position = Position::new(8, 6);
const VELOCITY: Velocity = Velocity::new(3, 2);
const WEBGPU_DYNAMIC_BODIES: u32 = 96;
const WEBGPU_FRAME_STEPS: u32 = 6;

static WEBGPU_FRAME: OnceLock<Result<BroadPhaseFrame, ScenarioError>> = OnceLock::new();

fn simulated_position(ticks: i32) -> Position {
    let mut world = ReferenceWorld::new();
    let operations = [
        Operation::Spawn(ENTITY),
        Operation::SetPosition(ENTITY, START),
        Operation::SetVelocity(ENTITY, VELOCITY),
        Operation::Integrate { ticks },
    ];

    for operation in operations {
        if world.apply(operation).is_err() {
            return START;
        }
    }

    world
        .snapshot()
        .entities()
        .first()
        .and_then(|entity| entity.position)
        .unwrap_or(START)
}

fn webgpu_frame() -> Option<&'static BroadPhaseFrame> {
    WEBGPU_FRAME
        .get_or_init(|| {
            FallingBoxesScenario::new(WEBGPU_DYNAMIC_BODIES)
                .broad_phase_frame_after(WEBGPU_FRAME_STEPS)
        })
        .as_ref()
        .ok()
}

#[unsafe(no_mangle)]
pub extern "C" fn start_x() -> i64 {
    START.x
}

#[unsafe(no_mangle)]
pub extern "C" fn start_y() -> i64 {
    START.y
}

#[unsafe(no_mangle)]
pub extern "C" fn velocity_x() -> i32 {
    VELOCITY.x
}

#[unsafe(no_mangle)]
pub extern "C" fn velocity_y() -> i32 {
    VELOCITY.y
}

#[unsafe(no_mangle)]
pub extern "C" fn position_x_after(ticks: i32) -> i64 {
    simulated_position(ticks).x
}

#[unsafe(no_mangle)]
pub extern "C" fn position_y_after(ticks: i32) -> i64 {
    simulated_position(ticks).y
}

#[unsafe(no_mangle)]
pub extern "C" fn webgpu_dynamic_body_count() -> u32 {
    WEBGPU_DYNAMIC_BODIES
}

#[unsafe(no_mangle)]
pub extern "C" fn webgpu_frame_steps() -> u32 {
    WEBGPU_FRAME_STEPS
}

#[unsafe(no_mangle)]
pub extern "C" fn webgpu_body_count() -> u32 {
    let Some(frame) = webgpu_frame() else {
        return 0;
    };
    match u32::try_from(frame.bodies().len()) {
        Ok(count) => count,
        Err(_) => 0,
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn webgpu_cpu_overlap_count() -> u32 {
    webgpu_frame().map_or(0, BroadPhaseFrame::overlap_count)
}

#[unsafe(no_mangle)]
pub extern "C" fn webgpu_pair_word_count() -> u32 {
    let Some(frame) = webgpu_frame() else {
        return 0;
    };
    match u32::try_from(frame.pair_words().len()) {
        Ok(count) => count,
        Err(_) => 0,
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn webgpu_pair_word(index: u32) -> u32 {
    let Some(frame) = webgpu_frame() else {
        return 0;
    };
    let Ok(index) = usize::try_from(index) else {
        return 0;
    };
    frame.pair_words().get(index).copied().unwrap_or(0)
}

#[unsafe(no_mangle)]
pub extern "C" fn webgpu_aabb_value(body_index: u32, lane: u32) -> f32 {
    let Some(frame) = webgpu_frame() else {
        return f32::NAN;
    };
    let Ok(body_index) = usize::try_from(body_index) else {
        return f32::NAN;
    };
    let Some(body) = frame.bodies().get(body_index) else {
        return f32::NAN;
    };

    match lane {
        0 => body.min[0],
        1 => body.min[1],
        2 => body.min[2],
        3 => 0.0,
        4 => body.max[0],
        5 => body.max[1],
        6 => body.max[2],
        7 => 0.0,
        _ => f32::NAN,
    }
}

#[cfg(test)]
mod tests {
    use super::{
        position_x_after, position_y_after, start_x, start_y, velocity_x, velocity_y,
        webgpu_aabb_value, webgpu_body_count, webgpu_cpu_overlap_count, webgpu_dynamic_body_count,
        webgpu_frame_steps, webgpu_pair_word, webgpu_pair_word_count,
    };

    #[test]
    fn exported_fixture_uses_reference_world_integration() {
        assert_eq!((start_x(), start_y()), (8, 6));
        assert_eq!((velocity_x(), velocity_y()), (3, 2));
        assert_eq!((position_x_after(0), position_y_after(0)), (8, 6));
        assert_eq!((position_x_after(5), position_y_after(5)), (23, 16));
    }

    #[test]
    fn webgpu_fixture_is_rust_owned_and_has_exact_cpu_evidence() {
        assert_eq!(webgpu_dynamic_body_count(), 96);
        assert_eq!(webgpu_frame_steps(), 6);
        assert_eq!(webgpu_body_count(), 97);
        assert!(webgpu_pair_word_count() > 0);
        assert!(webgpu_cpu_overlap_count() > 0);

        let set_bits = (0..webgpu_pair_word_count())
            .map(webgpu_pair_word)
            .map(u32::count_ones)
            .sum::<u32>();
        assert_eq!(set_bits, webgpu_cpu_overlap_count());

        for lane in 0..8 {
            assert!(webgpu_aabb_value(0, lane).is_finite());
        }
    }
}
