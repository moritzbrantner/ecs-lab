use std::sync::{Mutex, OnceLock};

use ecs_physics::{BodyKind, PhysicsBody, PhysicsConfig, step};
use ecs_physics_scenarios::BouncingRoomScenario;
use ecs_reference::ReferenceWorld;
use ecs_workload::{EntityId, Operation, Position};

const PHYSICS_DEMO_MAX_STEPS: u32 = 12;
const PAIR_ENTITY_LEFT: EntityId = EntityId(u32::MAX - 1);
const PAIR_ENTITY_RIGHT: EntityId = EntityId(u32::MAX);

static PHYSICS_DEMO_STATE: OnceLock<Mutex<PhysicsDemoState>> = OnceLock::new();

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct PhysicsDemoEntity {
    id: EntityId,
    position: Position,
    half_extents: [i32; 2],
    fixed: bool,
    mass_units: u32,
    restitution_milli: u16,
    friction_milli: u16,
}

#[derive(Clone, Debug, Eq, PartialEq)]
struct PhysicsDemoFrame {
    steps: u32,
    entities: Vec<PhysicsDemoEntity>,
    pair_words: Vec<u32>,
}

#[derive(Debug, Default)]
struct PhysicsDemoState {
    frame: Option<PhysicsDemoFrame>,
}

fn build_frame(steps: u32) -> Option<PhysicsDemoFrame> {
    if steps > PHYSICS_DEMO_MAX_STEPS {
        return None;
    }

    let scenario = BouncingRoomScenario::new();
    let snapshot = scenario.reference_after(steps).ok()?;
    let entities = scenario
        .bodies()
        .iter()
        .map(|body| {
            let position = snapshot
                .entities()
                .iter()
                .find(|entity| entity.id == body.entity)?
                .position?;
            Some(PhysicsDemoEntity {
                id: body.entity,
                position,
                half_extents: body.half_extents,
                fixed: body.kind == BodyKind::Fixed,
                mass_units: body.mass_units,
                restitution_milli: body.material.restitution_milli,
                friction_milli: body.material.friction_milli,
            })
        })
        .collect::<Option<Vec<_>>>()?;
    let pair_words = pair_words(&entities);

    Some(PhysicsDemoFrame {
        steps,
        entities,
        pair_words,
    })
}

fn pair_words(entities: &[PhysicsDemoEntity]) -> Vec<u32> {
    let body_count = entities.len();
    let possible_pairs = body_count.saturating_mul(body_count.saturating_sub(1)) / 2;
    let mut words = vec![0_u32; possible_pairs.div_ceil(32)];

    for left in 0..body_count {
        for right in (left + 1)..body_count {
            if !pair_overlaps(entities[left], entities[right]) {
                continue;
            }
            let pair = pair_index(left, right, body_count);
            words[pair / 32] |= 1_u32 << (pair % 32);
        }
    }

    words
}

fn pair_overlaps(left: PhysicsDemoEntity, right: PhysicsDemoEntity) -> bool {
    let mut world = ReferenceWorld::new();
    for operation in [
        Operation::Spawn(PAIR_ENTITY_LEFT),
        Operation::SetPosition(PAIR_ENTITY_LEFT, left.position),
        Operation::Spawn(PAIR_ENTITY_RIGHT),
        Operation::SetPosition(PAIR_ENTITY_RIGHT, right.position),
    ] {
        if world.apply(operation).is_err() {
            return false;
        }
    }

    let bodies = [
        PhysicsBody::fixed(PAIR_ENTITY_LEFT, left.half_extents),
        PhysicsBody::fixed(PAIR_ENTITY_RIGHT, right.half_extents),
    ];
    step(&world.snapshot(), &bodies, PhysicsConfig::default(), 1)
        .is_ok_and(|physics| physics.stats().contacts == 1)
}

fn pair_index(left: usize, right: usize, count: usize) -> usize {
    left * (2 * count - left - 1) / 2 + (right - left - 1)
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

fn frame_entity(body_index: u32, steps: u32) -> Option<PhysicsDemoEntity> {
    let index = usize::try_from(body_index).ok()?;
    let mut state = demo_state().lock().ok()?;
    ensure_frame(&mut state, steps)?
        .entities
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
    u32::try_from(frame.entities.len()).unwrap_or_default()
}

#[unsafe(no_mangle)]
pub extern "C" fn physics_demo_entity_id(body_index: u32, steps: u32) -> u32 {
    frame_entity(body_index, steps).map_or(u32::MAX, |entity| entity.id.0)
}

#[unsafe(no_mangle)]
pub extern "C" fn physics_demo_position_x(body_index: u32, steps: u32) -> i32 {
    frame_entity(body_index, steps)
        .and_then(|entity| i32::try_from(entity.position.x).ok())
        .unwrap_or_default()
}

#[unsafe(no_mangle)]
pub extern "C" fn physics_demo_position_y(body_index: u32, steps: u32) -> i32 {
    frame_entity(body_index, steps)
        .and_then(|entity| i32::try_from(entity.position.y).ok())
        .unwrap_or_default()
}

#[unsafe(no_mangle)]
pub extern "C" fn physics_demo_half_extent_x(body_index: u32, steps: u32) -> i32 {
    frame_entity(body_index, steps).map_or(0, |entity| entity.half_extents[0])
}

#[unsafe(no_mangle)]
pub extern "C" fn physics_demo_half_extent_y(body_index: u32, steps: u32) -> i32 {
    frame_entity(body_index, steps).map_or(0, |entity| entity.half_extents[1])
}

#[unsafe(no_mangle)]
pub extern "C" fn physics_demo_is_fixed(body_index: u32, steps: u32) -> u32 {
    frame_entity(body_index, steps).map_or(0, |entity| if entity.fixed { 1 } else { 0 })
}

#[unsafe(no_mangle)]
pub extern "C" fn physics_demo_mass_units(body_index: u32, steps: u32) -> u32 {
    frame_entity(body_index, steps).map_or(0, |entity| entity.mass_units)
}

#[unsafe(no_mangle)]
pub extern "C" fn physics_demo_restitution_milli(body_index: u32, steps: u32) -> u32 {
    frame_entity(body_index, steps).map_or(0, |entity| u32::from(entity.restitution_milli))
}

#[unsafe(no_mangle)]
pub extern "C" fn physics_demo_friction_milli(body_index: u32, steps: u32) -> u32 {
    frame_entity(body_index, steps).map_or(0, |entity| u32::from(entity.friction_milli))
}

#[unsafe(no_mangle)]
pub extern "C" fn physics_demo_pair_word_count(steps: u32) -> u32 {
    let Ok(mut state) = demo_state().lock() else {
        return 0;
    };
    let Some(frame) = ensure_frame(&mut state, steps) else {
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
    let Some(frame) = ensure_frame(&mut state, steps) else {
        return 0;
    };
    frame.pair_words.get(index).copied().unwrap_or(0)
}

#[cfg(test)]
mod tests {
    use super::{
        build_frame, physics_demo_body_count, physics_demo_half_extent_x,
        physics_demo_half_extent_y, physics_demo_is_fixed, physics_demo_max_steps,
        physics_demo_pair_word, physics_demo_pair_word_count, physics_demo_restitution_milli,
    };

    #[test]
    fn bouncing_room_demo_exposes_gravity_floor_and_exact_pair_evidence() {
        assert_eq!(physics_demo_max_steps(), 12);
        assert_eq!(physics_demo_body_count(0), 4);
        assert_eq!(physics_demo_is_fixed(3, 0), 1);
        assert_eq!(physics_demo_half_extent_x(3, 0), 26);
        assert_eq!(physics_demo_half_extent_y(3, 0), 1);
        assert_eq!(physics_demo_restitution_milli(0, 0), 1_000);

        let initial = build_frame(0).expect("initial bouncing-room frame should be valid");
        let after_gravity = build_frame(1).expect("gravity frame should be valid");
        assert!(after_gravity.entities[0].position.y < initial.entities[0].position.y);

        let contact_frame = build_frame(3).expect("contact frame should be valid");
        assert!(contact_frame.pair_words.iter().any(|word| *word != 0));
        assert_eq!(physics_demo_pair_word_count(3), 1);
        assert_eq!(physics_demo_pair_word(0, 3), contact_frame.pair_words[0]);
    }
}
