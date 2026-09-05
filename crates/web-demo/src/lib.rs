use std::sync::{Mutex, OnceLock};

use ecs_physics::{PhysicsBody, PhysicsConfig, PhysicsMaterial, step};
use ecs_physics_scenarios::{BroadPhaseFrame, FallingBoxesScenario, ScenarioError};
use ecs_reference::ReferenceWorld;
use ecs_workload::{EntityId, Operation, Position, Velocity};

const ENTITY: EntityId = EntityId(1);
const PAIR_ENTITY_LEFT: EntityId = EntityId(1);
const PAIR_ENTITY_RIGHT: EntityId = EntityId(2);
const START: Position = Position::new(8, 6);
const VELOCITY: Velocity = Velocity::new(3, 2);
const MAX_INTERACTIVE_ENTITIES: usize = 64;
const WEBGPU_DYNAMIC_BODIES: u32 = 96;
const WEBGPU_FRAME_STEPS: u32 = 6;
const INTERACTIVE_PHYSICS: PhysicsConfig = PhysicsConfig {
    gravity: Velocity::new(0, 0),
};

static INTERACTIVE_STATE: OnceLock<Mutex<InteractiveState>> = OnceLock::new();
static WEBGPU_FRAME: OnceLock<Result<BroadPhaseFrame, ScenarioError>> = OnceLock::new();

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct InteractiveEntity {
    id: EntityId,
    start: Position,
    velocity: Velocity,
    half_extent: i32,
    mass_units: u32,
    material: PhysicsMaterial,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct InteractiveFrameEntity {
    id: EntityId,
    position: Position,
    half_extent: i32,
}

#[derive(Clone, Debug, Eq, PartialEq)]
struct InteractiveFrame {
    revision: u64,
    ticks: i32,
    entities: Vec<InteractiveFrameEntity>,
    pair_words: Vec<u32>,
}

#[derive(Debug, Default)]
struct InteractiveState {
    revision: u64,
    entities: Vec<InteractiveEntity>,
    frame: Option<InteractiveFrame>,
}

impl InteractiveState {
    fn invalidate(&mut self) {
        self.revision = self.revision.saturating_add(1);
        self.frame = None;
    }
}

fn simulated_position(start: Position, velocity: Velocity, ticks: i32) -> Position {
    let mut world = ReferenceWorld::new();
    let operations = [
        Operation::Spawn(ENTITY),
        Operation::SetPosition(ENTITY, start),
        Operation::SetVelocity(ENTITY, velocity),
        Operation::Integrate { ticks },
    ];

    for operation in operations {
        if world.apply(operation).is_err() {
            return start;
        }
    }

    world
        .snapshot()
        .entities()
        .first()
        .and_then(|entity| entity.position)
        .unwrap_or(start)
}

fn pair_overlaps(
    left_position: Position,
    left_half_extent: i32,
    right_position: Position,
    right_half_extent: i32,
) -> bool {
    if left_half_extent < 0 || right_half_extent < 0 {
        return false;
    }

    let mut world = ReferenceWorld::new();
    let operations = [
        Operation::Spawn(PAIR_ENTITY_LEFT),
        Operation::SetPosition(PAIR_ENTITY_LEFT, left_position),
        Operation::Spawn(PAIR_ENTITY_RIGHT),
        Operation::SetPosition(PAIR_ENTITY_RIGHT, right_position),
    ];
    for operation in operations {
        if world.apply(operation).is_err() {
            return false;
        }
    }

    let bodies = [
        PhysicsBody::fixed(PAIR_ENTITY_LEFT, [left_half_extent, left_half_extent]),
        PhysicsBody::fixed(PAIR_ENTITY_RIGHT, [right_half_extent, right_half_extent]),
    ];
    step(&world.snapshot(), &bodies, PhysicsConfig::default(), 1)
        .is_ok_and(|physics| physics.stats().contacts == 1)
}

fn build_interactive_frame(
    entities: &[InteractiveEntity],
    revision: u64,
    ticks: i32,
) -> Option<InteractiveFrame> {
    if ticks < 0 {
        return None;
    }

    let mut world = ReferenceWorld::new();
    let mut bodies = Vec::with_capacity(entities.len());
    for entity in entities {
        for operation in [
            Operation::Spawn(entity.id),
            Operation::SetPosition(entity.id, entity.start),
            Operation::SetVelocity(entity.id, entity.velocity),
        ] {
            world.apply(operation).ok()?;
        }
        bodies.push(
            PhysicsBody::dynamic(entity.id, [entity.half_extent, entity.half_extent])
                .with_mass(entity.mass_units)
                .with_material(entity.material),
        );
    }

    for _ in 0..ticks {
        let physics = step(&world.snapshot(), &bodies, INTERACTIVE_PHYSICS, 1).ok()?;
        for operation in physics.operations() {
            world.apply(*operation).ok()?;
        }
    }

    let snapshot = world.snapshot();
    let frame_entities = entities
        .iter()
        .map(|entity| {
            let position = snapshot
                .entities()
                .iter()
                .find(|snapshot_entity| snapshot_entity.id == entity.id)?
                .position?;
            Some(InteractiveFrameEntity {
                id: entity.id,
                position,
                half_extent: entity.half_extent,
            })
        })
        .collect::<Option<Vec<_>>>()?;
    let pair_words = interactive_pair_words(&frame_entities);

    Some(InteractiveFrame {
        revision,
        ticks,
        entities: frame_entities,
        pair_words,
    })
}

fn interactive_pair_words(entities: &[InteractiveFrameEntity]) -> Vec<u32> {
    let body_count = entities.len();
    let possible_pairs = body_count.saturating_mul(body_count.saturating_sub(1)) / 2;
    let mut pair_words = vec![0_u32; possible_pairs.div_ceil(32)];

    for left in 0..body_count {
        for right in (left + 1)..body_count {
            let left_entity = entities[left];
            let right_entity = entities[right];
            if !pair_overlaps(
                left_entity.position,
                left_entity.half_extent,
                right_entity.position,
                right_entity.half_extent,
            ) {
                continue;
            }
            let pair = pair_index(left, right, body_count);
            pair_words[pair / 32] |= 1_u32 << (pair % 32);
        }
    }

    pair_words
}

fn interactive_state() -> &'static Mutex<InteractiveState> {
    INTERACTIVE_STATE.get_or_init(|| Mutex::new(InteractiveState::default()))
}

fn ensure_interactive_frame(state: &mut InteractiveState, ticks: i32) -> Option<&InteractiveFrame> {
    let needs_rebuild = state
        .frame
        .as_ref()
        .is_none_or(|frame| frame.revision != state.revision || frame.ticks != ticks);
    if needs_rebuild {
        state.frame = build_interactive_frame(&state.entities, state.revision, ticks);
    }
    state.frame.as_ref()
}

fn interactive_position(body_index: u32, ticks: i32) -> Option<Position> {
    let index = usize::try_from(body_index).ok()?;
    let mut state = interactive_state().lock().ok()?;
    ensure_interactive_frame(&mut state, ticks)?
        .entities
        .get(index)
        .map(|entity| entity.position)
}

fn pair_index(left: usize, right: usize, count: usize) -> usize {
    left * (2 * count - left - 1) / 2 + (right - left - 1)
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
    simulated_position(START, VELOCITY, ticks).x
}

#[unsafe(no_mangle)]
pub extern "C" fn position_y_after(ticks: i32) -> i64 {
    simulated_position(START, VELOCITY, ticks).y
}

#[unsafe(no_mangle)]
pub extern "C" fn interactive_clear() -> u32 {
    let Ok(mut state) = interactive_state().lock() else {
        return 0;
    };
    state.entities.clear();
    state.invalidate();
    1
}

#[unsafe(no_mangle)]
pub extern "C" fn interactive_push_entity(
    entity_id: u32,
    start_x: i32,
    start_y: i32,
    velocity_x: i32,
    velocity_y: i32,
    half_extent: i32,
    mass_units: u32,
    restitution_milli: u32,
    friction_milli: u32,
) -> u32 {
    let Ok(restitution_milli) = u16::try_from(restitution_milli) else {
        return 0;
    };
    let Ok(friction_milli) = u16::try_from(friction_milli) else {
        return 0;
    };
    let Ok(mut state) = interactive_state().lock() else {
        return 0;
    };
    if state.entities.len() >= MAX_INTERACTIVE_ENTITIES
        || half_extent < 0
        || mass_units == 0
        || restitution_milli > 1_000
        || friction_milli > 1_000
        || state.entities.iter().any(|entity| entity.id.0 == entity_id)
    {
        return 0;
    }

    state.entities.push(InteractiveEntity {
        id: EntityId(entity_id),
        start: Position::new(i64::from(start_x), i64::from(start_y)),
        velocity: Velocity::new(velocity_x, velocity_y),
        half_extent,
        mass_units,
        material: PhysicsMaterial::new(restitution_milli, friction_milli),
    });
    state.invalidate();
    1
}

#[unsafe(no_mangle)]
pub extern "C" fn interactive_entity_count() -> u32 {
    let Ok(state) = interactive_state().lock() else {
        return 0;
    };
    u32::try_from(state.entities.len()).unwrap_or_default()
}

#[unsafe(no_mangle)]
pub extern "C" fn interactive_position_x(body_index: u32, ticks: i32) -> i32 {
    interactive_position(body_index, ticks)
        .and_then(|position| i32::try_from(position.x).ok())
        .unwrap_or_default()
}

#[unsafe(no_mangle)]
pub extern "C" fn interactive_position_y(body_index: u32, ticks: i32) -> i32 {
    interactive_position(body_index, ticks)
        .and_then(|position| i32::try_from(position.y).ok())
        .unwrap_or_default()
}

#[unsafe(no_mangle)]
pub extern "C" fn interactive_pair_word_count(ticks: i32) -> u32 {
    let Ok(mut state) = interactive_state().lock() else {
        return 0;
    };
    let Some(frame) = ensure_interactive_frame(&mut state, ticks) else {
        return 0;
    };
    u32::try_from(frame.pair_words.len()).unwrap_or_default()
}

#[unsafe(no_mangle)]
pub extern "C" fn interactive_pair_word(word_index: u32, ticks: i32) -> u32 {
    let Ok(index) = usize::try_from(word_index) else {
        return 0;
    };
    let Ok(mut state) = interactive_state().lock() else {
        return 0;
    };
    let Some(frame) = ensure_interactive_frame(&mut state, ticks) else {
        return 0;
    };
    frame.pair_words.get(index).copied().unwrap_or(0)
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
    u32::try_from(frame.bodies().len()).unwrap_or_default()
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
    u32::try_from(frame.pair_words().len()).unwrap_or_default()
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
    use ecs_physics::PhysicsMaterial;
    use ecs_workload::{EntityId, Position, Velocity};

    use super::{
        InteractiveEntity, build_interactive_frame, position_x_after, position_y_after, start_x,
        start_y, velocity_x, velocity_y, webgpu_aabb_value, webgpu_body_count,
        webgpu_cpu_overlap_count, webgpu_dynamic_body_count, webgpu_frame_steps, webgpu_pair_word,
        webgpu_pair_word_count,
    };

    fn interactive_entity(id: u32, x: i64, velocity_x: i32, restitution_milli: u16) -> InteractiveEntity {
        InteractiveEntity {
            id: EntityId(id),
            start: Position::new(x, 0),
            velocity: Velocity::new(velocity_x, 0),
            half_extent: 2,
            mass_units: 1,
            material: PhysicsMaterial::new(restitution_milli, 0),
        }
    }

    #[test]
    fn exported_fixture_uses_reference_world_integration() {
        assert_eq!((start_x(), start_y()), (8, 6));
        assert_eq!((velocity_x(), velocity_y()), (3, 2));
        assert_eq!((position_x_after(0), position_y_after(0)), (8, 6));
        assert_eq!((position_x_after(5), position_y_after(5)), (23, 16));
    }

    #[test]
    fn interactive_frame_uses_rust_material_physics_and_pair_words() {
        let entities = [
            interactive_entity(10, -4, 1, 1_000),
            interactive_entity(20, 4, -1, 1_000),
        ];

        let Some(frame) = build_interactive_frame(&entities, 7, 2) else {
            panic!("interactive frame should be valid");
        };
        assert_eq!(frame.revision, 7);
        assert_eq!(frame.entities[0].position, Position::new(-2, 0));
        assert_eq!(frame.entities[1].position, Position::new(2, 0));
        assert_eq!(frame.pair_words, [1]);

        let Some(after_bounce) = build_interactive_frame(&entities, 7, 3) else {
            panic!("bouncy interactive frame should be valid");
        };
        assert_eq!(after_bounce.entities[0].position, Position::new(-3, 0));
        assert_eq!(after_bounce.entities[1].position, Position::new(3, 0));
    }

    #[test]
    fn restitution_changes_the_interactive_path() {
        let bouncy = [
            interactive_entity(10, -4, 1, 1_000),
            interactive_entity(20, 4, -1, 1_000),
        ];
        let inelastic = [
            interactive_entity(10, -4, 1, 0),
            interactive_entity(20, 4, -1, 0),
        ];

        let Some(bouncy_frame) = build_interactive_frame(&bouncy, 1, 3) else {
            panic!("bouncy frame should be valid");
        };
        let Some(inelastic_frame) = build_interactive_frame(&inelastic, 1, 3) else {
            panic!("inelastic frame should be valid");
        };

        assert_ne!(bouncy_frame.entities, inelastic_frame.entities);
        assert_eq!(inelastic_frame.entities[0].position, Position::new(-2, 0));
        assert_eq!(inelastic_frame.entities[1].position, Position::new(2, 0));
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
