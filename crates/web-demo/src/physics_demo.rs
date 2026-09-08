use std::sync::{Mutex, OnceLock};

use ecs_physics::{BodyKind, PhysicsMaterial};
use ecs_physics_3d::{
    ANGULAR_VELOCITY_SCALE, AngularState3d, AngularSubstepPolicy3d, AngularVelocity3d,
    BouncingRoom3dScenario, ORIENTATION_SCALE, Orientation3d, PhysicsBody3d, PhysicsConfig3d,
    RigidBox3d, RigidBoxState3d, RigidBoxWorldConfig3d, RotatingContactSearchConfig3d,
    oriented_box_vertices, step_3d, step_rigid_box_world_repeated_rotating,
};
use ecs_reference::ReferenceWorld;
use ecs_workload::{EntityId, Operation, Position, Velocity, Workload};

const PHYSICS_DEMO_FPS: u32 = 60;
const PHYSICS_DEMO_SECONDS: u32 = 10;
const PHYSICS_DEMO_MAX_STEPS: u32 = PHYSICS_DEMO_FPS * PHYSICS_DEMO_SECONDS;
const PHYSICS_DEMO_SOLVER_PASSES: u8 = 6;
const PHYSICS_DEMO_ANGULAR_DAMPING_MILLI: u16 = 998;
const MATERIAL_DEMO_COUNT: usize = 6;
const MATERIAL_DEMO_RESTITUTION: [u16; MATERIAL_DEMO_COUNT] = [1_000, 850, 650, 450, 250, 0];
const MATERIAL_DEMO_FRICTION: [u16; MATERIAL_DEMO_COUNT] = [0, 150, 350, 550, 750, 1_000];
const MATERIAL_DEMO_SPATIAL_SCALE: i64 = 3_600;
const MATERIAL_DEMO_EXTENT_SCALE: i32 = 3_600;
const MATERIAL_DEMO_VELOCITY_SCALE: i32 = 60;

static PHYSICS_DEMO_STATE: OnceLock<Mutex<Option<PhysicsDemoState>>> = OnceLock::new();
static MATERIAL_DEMO_STATE: OnceLock<Mutex<Option<MaterialDemoState>>> = OnceLock::new();

#[derive(Clone, Copy)]
struct DisplayBounds3d {
    minimum: [f32; 3],
    maximum: [f32; 3],
}

struct PhysicsDemoFrame {
    boxes: Vec<RigidBox3d>,
    broad_bounds: Vec<DisplayBounds3d>,
    pair_words: Vec<u32>,
}

struct PhysicsDemoState {
    frames: Vec<PhysicsDemoFrame>,
    spatial_scale: i64,
    config: RigidBoxWorldConfig3d,
}

impl PhysicsDemoState {
    fn new() -> Option<Self> {
        let scenario = BouncingRoom3dScenario::with_substeps_per_tick(PHYSICS_DEMO_FPS).ok()?;
        let spatial_scale = scenario.spatial_scale();
        let mut world = ReferenceWorld::new();
        world.replay(scenario.setup()).ok()?;
        let snapshot = world.snapshot();
        let velocity_factor = i32::try_from(PHYSICS_DEMO_FPS).ok()?;
        let mut boxes = Vec::with_capacity(scenario.bodies().len());

        for body in scenario.bodies() {
            let entity = snapshot
                .entities()
                .iter()
                .find(|candidate| candidate.id == body.entity)?;
            let center = entity.position?;
            let source_velocity = entity.velocity.unwrap_or(Velocity::new3(0, 0, 0));
            let linear_velocity = if body.kind == BodyKind::Dynamic {
                Velocity::new3(
                    source_velocity.x.checked_mul(velocity_factor)?,
                    source_velocity.y.checked_mul(velocity_factor)?,
                    source_velocity.z.checked_mul(velocity_factor)?,
                )
            } else {
                Velocity::new3(0, 0, 0)
            };
            let angular_velocity = if body.kind == BodyKind::Dynamic {
                initial_angular_velocity(body.entity)
            } else {
                AngularVelocity3d::default()
            };
            boxes.push(RigidBox3d::new(
                *body,
                RigidBoxState3d::new(
                    center,
                    linear_velocity,
                    AngularState3d::new(Orientation3d::IDENTITY, angular_velocity),
                ),
            ));
        }

        let gravity_scale = i32::try_from(spatial_scale).ok()?;
        let config = RigidBoxWorldConfig3d {
            gravity: Velocity::new3(0, gravity_scale.checked_neg()?, 0),
            timestep_numerator: 1,
            timestep_denominator: velocity_factor,
            angular_damping_milli: PHYSICS_DEMO_ANGULAR_DAMPING_MILLI,
            solver_passes: PHYSICS_DEMO_SOLVER_PASSES,
        };
        let initial_frame = build_demo_frame(boxes, spatial_scale)?;
        Some(Self {
            frames: vec![initial_frame],
            spatial_scale,
            config,
        })
    }

    fn ensure_frame(&mut self, steps: u32) -> Option<&PhysicsDemoFrame> {
        if steps > PHYSICS_DEMO_MAX_STEPS {
            return None;
        }
        let target = usize::try_from(steps).ok()?;
        while self.frames.len() <= target {
            let previous = &self.frames.last()?.boxes;
            let next = step_rigid_box_world_repeated_rotating(
                previous,
                self.config,
                RotatingContactSearchConfig3d::default(),
                AngularSubstepPolicy3d::default(),
            )
            .ok()?;
            self.frames
                .push(build_demo_frame(next.boxes, self.spatial_scale)?);
        }
        self.frames.get(target)
    }
}

struct MaterialDemoState {
    world: ReferenceWorld,
    bodies: Vec<PhysicsBody3d>,
    frames: Vec<Vec<Position>>,
}

impl MaterialDemoState {
    fn new() -> Option<Self> {
        let mut operations = Vec::new();
        let mut bodies = Vec::new();

        for index in 0..MATERIAL_DEMO_COUNT {
            let entity = EntityId(u32::try_from(index).ok()?);
            let x = (-15_i64 + i64::try_from(index).ok()? * 6)
                .checked_mul(MATERIAL_DEMO_SPATIAL_SCALE)?;
            operations.push(Operation::Spawn(entity));
            operations.push(Operation::SetPosition(
                entity,
                Position::new3(x, 12 * MATERIAL_DEMO_SPATIAL_SCALE, 0),
            ));
            operations.push(Operation::SetVelocity(
                entity,
                Velocity::new3(
                    MATERIAL_DEMO_VELOCITY_SCALE,
                    -2 * MATERIAL_DEMO_VELOCITY_SCALE,
                    0,
                ),
            ));
            bodies.push(
                PhysicsBody3d::dynamic(
                    entity,
                    [
                        MATERIAL_DEMO_EXTENT_SCALE,
                        MATERIAL_DEMO_EXTENT_SCALE,
                        MATERIAL_DEMO_EXTENT_SCALE,
                    ],
                )
                .with_material(PhysicsMaterial::new(
                    MATERIAL_DEMO_RESTITUTION[index],
                    MATERIAL_DEMO_FRICTION[index],
                )),
            );
        }

        let floor = EntityId(u32::try_from(MATERIAL_DEMO_COUNT).ok()?);
        operations.push(Operation::Spawn(floor));
        operations.push(Operation::SetPosition(
            floor,
            Position::new3(0, -MATERIAL_DEMO_SPATIAL_SCALE, 0),
        ));
        bodies.push(PhysicsBody3d::fixed(
            floor,
            [
                20 * MATERIAL_DEMO_EXTENT_SCALE,
                MATERIAL_DEMO_EXTENT_SCALE,
                8 * MATERIAL_DEMO_EXTENT_SCALE,
            ],
        ));

        let mut world = ReferenceWorld::new();
        world.replay(&Workload::new(operations)).ok()?;
        let initial = capture_material_positions(&world)?;
        Some(Self {
            world,
            bodies,
            frames: vec![initial],
        })
    }

    fn ensure_frame(&mut self, steps: u32) -> Option<&[Position]> {
        if steps > PHYSICS_DEMO_MAX_STEPS {
            return None;
        }
        let target = usize::try_from(steps).ok()?;
        while self.frames.len() <= target {
            let physics = step_3d(
                &self.world.snapshot(),
                &self.bodies,
                PhysicsConfig3d {
                    gravity: Velocity::new3(0, -1, 0),
                },
                1,
            )
            .ok()?;
            for operation in physics.operations() {
                self.world.apply(*operation).ok()?;
            }
            self.frames.push(capture_material_positions(&self.world)?);
        }
        self.frames.get(target).map(Vec::as_slice)
    }
}

fn initial_angular_velocity(entity: EntityId) -> AngularVelocity3d {
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

fn capture_material_positions(world: &ReferenceWorld) -> Option<Vec<Position>> {
    let snapshot = world.snapshot();
    (0..MATERIAL_DEMO_COUNT)
        .map(|index| {
            let entity = EntityId(u32::try_from(index).ok()?);
            snapshot
                .entities()
                .iter()
                .find(|candidate| candidate.id == entity)?
                .position
        })
        .collect()
}

fn build_demo_frame(boxes: Vec<RigidBox3d>, spatial_scale: i64) -> Option<PhysicsDemoFrame> {
    let broad_bounds = boxes
        .iter()
        .map(|rigid_box| display_bounds(*rigid_box, spatial_scale))
        .collect::<Option<Vec<_>>>()?;
    let pair_words = pair_evidence(&broad_bounds);
    Some(PhysicsDemoFrame {
        boxes,
        broad_bounds,
        pair_words,
    })
}

fn display_bounds(rigid_box: RigidBox3d, spatial_scale: i64) -> Option<DisplayBounds3d> {
    let vertices = oriented_box_vertices(
        rigid_box.state.center,
        rigid_box.body.half_extents,
        rigid_box.state.angular.orientation,
    )
    .ok()?;
    let mut minimum = [i64::MAX; 3];
    let mut maximum = [i64::MIN; 3];
    for vertex in vertices {
        for (axis, value) in [vertex.x, vertex.y, vertex.z].into_iter().enumerate() {
            minimum[axis] = minimum[axis].min(value);
            maximum[axis] = maximum[axis].max(value);
        }
    }
    Some(DisplayBounds3d {
        minimum: minimum.map(|value| display_coordinate(value, spatial_scale)),
        maximum: maximum.map(|value| display_coordinate(value, spatial_scale)),
    })
}

fn pair_evidence(bounds: &[DisplayBounds3d]) -> Vec<u32> {
    let possible_pairs = bounds.len().saturating_mul(bounds.len().saturating_sub(1)) / 2;
    let mut pair_words = vec![0_u32; possible_pairs.div_ceil(32)];
    let mut pair = 0_usize;
    for left in 0..bounds.len() {
        for right in (left + 1)..bounds.len() {
            if bounds_overlap(bounds[left], bounds[right]) {
                pair_words[pair / 32] |= 1_u32 << (pair % 32);
            }
            pair += 1;
        }
    }
    pair_words
}

fn bounds_overlap(left: DisplayBounds3d, right: DisplayBounds3d) -> bool {
    (0..3).all(|axis| {
        left.maximum[axis] >= right.minimum[axis] && right.maximum[axis] >= left.minimum[axis]
    })
}

fn demo_state() -> &'static Mutex<Option<PhysicsDemoState>> {
    PHYSICS_DEMO_STATE.get_or_init(|| Mutex::new(None))
}

fn material_demo_state() -> &'static Mutex<Option<MaterialDemoState>> {
    MATERIAL_DEMO_STATE.get_or_init(|| Mutex::new(None))
}

fn ensure_state(state: &mut Option<PhysicsDemoState>) -> Option<&mut PhysicsDemoState> {
    if state.is_none() {
        *state = PhysicsDemoState::new();
    }
    state.as_mut()
}

fn ensure_material_state(state: &mut Option<MaterialDemoState>) -> Option<&mut MaterialDemoState> {
    if state.is_none() {
        *state = MaterialDemoState::new();
    }
    state.as_mut()
}

fn frame_body(body_index: u32, steps: u32) -> Option<(RigidBox3d, i64)> {
    let index = usize::try_from(body_index).ok()?;
    let mut state = demo_state().lock().ok()?;
    let state = ensure_state(&mut state)?;
    let spatial_scale = state.spatial_scale;
    let body = *state.ensure_frame(steps)?.boxes.get(index)?;
    Some((body, spatial_scale))
}

fn frame_bounds(body_index: u32, steps: u32) -> Option<DisplayBounds3d> {
    let index = usize::try_from(body_index).ok()?;
    let mut state = demo_state().lock().ok()?;
    let state = ensure_state(&mut state)?;
    state.ensure_frame(steps)?.broad_bounds.get(index).copied()
}

fn material_position(body_index: u32, steps: u32) -> Option<Position> {
    let index = usize::try_from(body_index).ok()?;
    if index >= MATERIAL_DEMO_COUNT {
        return None;
    }
    let mut state = material_demo_state().lock().ok()?;
    ensure_material_state(&mut state)?
        .ensure_frame(steps)?
        .get(index)
        .copied()
}

fn display_coordinate(value: i64, spatial_scale: i64) -> f32 {
    value as f32 / spatial_scale as f32
}

fn display_extent(value: i32, spatial_scale: i64) -> f32 {
    value as f32 / spatial_scale as f32
}

fn display_orientation(value: i32) -> f32 {
    value as f32 / ORIENTATION_SCALE as f32
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
    u32::try_from(frame.boxes.len()).unwrap_or_default()
}

#[unsafe(no_mangle)]
pub extern "C" fn physics_demo_entity_id(body_index: u32, steps: u32) -> u32 {
    frame_body(body_index, steps).map_or(u32::MAX, |(rigid_box, _)| rigid_box.body.entity.0)
}

#[unsafe(no_mangle)]
pub extern "C" fn physics_demo_position_x(body_index: u32, steps: u32) -> f32 {
    frame_body(body_index, steps).map_or(0.0, |(rigid_box, spatial_scale)| {
        display_coordinate(rigid_box.state.center.x, spatial_scale)
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn physics_demo_position_y(body_index: u32, steps: u32) -> f32 {
    frame_body(body_index, steps).map_or(0.0, |(rigid_box, spatial_scale)| {
        display_coordinate(rigid_box.state.center.y, spatial_scale)
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn physics_demo_position_z(body_index: u32, steps: u32) -> f32 {
    frame_body(body_index, steps).map_or(0.0, |(rigid_box, spatial_scale)| {
        display_coordinate(rigid_box.state.center.z, spatial_scale)
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn physics_demo_half_extent_x(body_index: u32, steps: u32) -> f32 {
    frame_body(body_index, steps).map_or(0.0, |(rigid_box, spatial_scale)| {
        display_extent(rigid_box.body.half_extents[0], spatial_scale)
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn physics_demo_half_extent_y(body_index: u32, steps: u32) -> f32 {
    frame_body(body_index, steps).map_or(0.0, |(rigid_box, spatial_scale)| {
        display_extent(rigid_box.body.half_extents[1], spatial_scale)
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn physics_demo_half_extent_z(body_index: u32, steps: u32) -> f32 {
    frame_body(body_index, steps).map_or(0.0, |(rigid_box, spatial_scale)| {
        display_extent(rigid_box.body.half_extents[2], spatial_scale)
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn physics_demo_orientation_x(body_index: u32, steps: u32) -> f32 {
    frame_body(body_index, steps).map_or(0.0, |(rigid_box, _)| {
        display_orientation(rigid_box.state.angular.orientation.x)
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn physics_demo_orientation_y(body_index: u32, steps: u32) -> f32 {
    frame_body(body_index, steps).map_or(0.0, |(rigid_box, _)| {
        display_orientation(rigid_box.state.angular.orientation.y)
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn physics_demo_orientation_z(body_index: u32, steps: u32) -> f32 {
    frame_body(body_index, steps).map_or(0.0, |(rigid_box, _)| {
        display_orientation(rigid_box.state.angular.orientation.z)
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn physics_demo_orientation_w(body_index: u32, steps: u32) -> f32 {
    frame_body(body_index, steps).map_or(0.0, |(rigid_box, _)| {
        display_orientation(rigid_box.state.angular.orientation.w)
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn physics_demo_angular_velocity_x(body_index: u32, steps: u32) -> i32 {
    frame_body(body_index, steps).map_or(0, |(rigid_box, _)| {
        rigid_box.state.angular.angular_velocity.x
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn physics_demo_angular_velocity_y(body_index: u32, steps: u32) -> i32 {
    frame_body(body_index, steps).map_or(0, |(rigid_box, _)| {
        rigid_box.state.angular.angular_velocity.y
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn physics_demo_angular_velocity_z(body_index: u32, steps: u32) -> i32 {
    frame_body(body_index, steps).map_or(0, |(rigid_box, _)| {
        rigid_box.state.angular.angular_velocity.z
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn physics_demo_broad_min_x(body_index: u32, steps: u32) -> f32 {
    frame_bounds(body_index, steps).map_or(0.0, |bounds| bounds.minimum[0])
}

#[unsafe(no_mangle)]
pub extern "C" fn physics_demo_broad_min_y(body_index: u32, steps: u32) -> f32 {
    frame_bounds(body_index, steps).map_or(0.0, |bounds| bounds.minimum[1])
}

#[unsafe(no_mangle)]
pub extern "C" fn physics_demo_broad_min_z(body_index: u32, steps: u32) -> f32 {
    frame_bounds(body_index, steps).map_or(0.0, |bounds| bounds.minimum[2])
}

#[unsafe(no_mangle)]
pub extern "C" fn physics_demo_broad_max_x(body_index: u32, steps: u32) -> f32 {
    frame_bounds(body_index, steps).map_or(0.0, |bounds| bounds.maximum[0])
}

#[unsafe(no_mangle)]
pub extern "C" fn physics_demo_broad_max_y(body_index: u32, steps: u32) -> f32 {
    frame_bounds(body_index, steps).map_or(0.0, |bounds| bounds.maximum[1])
}

#[unsafe(no_mangle)]
pub extern "C" fn physics_demo_broad_max_z(body_index: u32, steps: u32) -> f32 {
    frame_bounds(body_index, steps).map_or(0.0, |bounds| bounds.maximum[2])
}

#[unsafe(no_mangle)]
pub extern "C" fn physics_demo_is_fixed(body_index: u32, steps: u32) -> u32 {
    frame_body(body_index, steps).map_or(0, |(rigid_box, _)| {
        u32::from(rigid_box.body.kind == BodyKind::Fixed)
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn physics_demo_mass_units(body_index: u32, steps: u32) -> u32 {
    frame_body(body_index, steps).map_or(0, |(rigid_box, _)| rigid_box.body.mass_units)
}

#[unsafe(no_mangle)]
pub extern "C" fn physics_demo_restitution_milli(body_index: u32, steps: u32) -> u32 {
    frame_body(body_index, steps).map_or(0, |(rigid_box, _)| {
        u32::from(rigid_box.body.material.restitution_milli)
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn physics_demo_friction_milli(body_index: u32, steps: u32) -> u32 {
    frame_body(body_index, steps).map_or(0, |(rigid_box, _)| {
        u32::from(rigid_box.body.material.friction_milli)
    })
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
    frame.pair_words.iter().map(|word| word.count_ones()).sum()
}

#[unsafe(no_mangle)]
pub extern "C" fn physics_material_demo_count() -> u32 {
    u32::try_from(MATERIAL_DEMO_COUNT).unwrap_or_default()
}

#[unsafe(no_mangle)]
pub extern "C" fn physics_material_demo_max_steps() -> u32 {
    PHYSICS_DEMO_MAX_STEPS
}

#[unsafe(no_mangle)]
pub extern "C" fn physics_material_demo_restitution_milli(body_index: u32) -> u32 {
    usize::try_from(body_index)
        .ok()
        .and_then(|index| MATERIAL_DEMO_RESTITUTION.get(index).copied())
        .map_or(0, u32::from)
}

#[unsafe(no_mangle)]
pub extern "C" fn physics_material_demo_friction_milli(body_index: u32) -> u32 {
    usize::try_from(body_index)
        .ok()
        .and_then(|index| MATERIAL_DEMO_FRICTION.get(index).copied())
        .map_or(0, u32::from)
}

#[unsafe(no_mangle)]
pub extern "C" fn physics_material_demo_position_x(body_index: u32, steps: u32) -> f32 {
    material_position(body_index, steps).map_or(0.0, |position| {
        display_coordinate(position.x, MATERIAL_DEMO_SPATIAL_SCALE)
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn physics_material_demo_position_y(body_index: u32, steps: u32) -> f32 {
    material_position(body_index, steps).map_or(0.0, |position| {
        display_coordinate(position.y, MATERIAL_DEMO_SPATIAL_SCALE)
    })
}

#[cfg(test)]
mod tests {
    use super::{
        ORIENTATION_SCALE, physics_demo_angular_velocity_x, physics_demo_angular_velocity_y,
        physics_demo_angular_velocity_z, physics_demo_body_count, physics_demo_fps,
        physics_demo_half_extent_z, physics_demo_is_fixed, physics_demo_max_steps,
        physics_demo_orientation_w, physics_demo_orientation_x, physics_demo_orientation_y,
        physics_demo_orientation_z, physics_demo_position_z, physics_material_demo_count,
        physics_material_demo_position_y, physics_material_demo_restitution_milli,
    };

    #[test]
    fn browser_demo_exposes_true_sixty_hz_oriented_three_dimensional_steps() {
        assert_eq!(physics_demo_fps(), 60);
        assert_eq!(physics_demo_max_steps(), 600);
        assert_eq!(physics_demo_body_count(0), 54);
        assert_eq!(physics_demo_is_fixed(48, 0), 1);
        assert!(physics_demo_half_extent_z(48, 0) > 1.0);

        let initial_z = physics_demo_position_z(0, 0);
        let next_z = physics_demo_position_z(0, 1);
        assert_ne!(initial_z, next_z);
        assert!((next_z - initial_z).abs() < 1.0);

        let initial_orientation = [
            physics_demo_orientation_x(0, 0),
            physics_demo_orientation_y(0, 0),
            physics_demo_orientation_z(0, 0),
            physics_demo_orientation_w(0, 0),
        ];
        let later_orientation = [
            physics_demo_orientation_x(0, 60),
            physics_demo_orientation_y(0, 60),
            physics_demo_orientation_z(0, 60),
            physics_demo_orientation_w(0, 60),
        ];
        assert_ne!(initial_orientation, later_orientation);
        assert!(
            physics_demo_angular_velocity_x(0, 0) != 0
                || physics_demo_angular_velocity_y(0, 0) != 0
                || physics_demo_angular_velocity_z(0, 0) != 0
        );

        let fixed_orientation = [
            physics_demo_orientation_x(48, 60),
            physics_demo_orientation_y(48, 60),
            physics_demo_orientation_z(48, 60),
            physics_demo_orientation_w(48, 60),
        ];
        assert_eq!(fixed_orientation, [0.0, 0.0, 0.0, 1.0]);
        assert_eq!(ORIENTATION_SCALE, 1_i32 << 30);

        let first_repeat = physics_demo_position_z(0, 6);
        let second_repeat = physics_demo_position_z(0, 6);
        assert_eq!(first_repeat, second_repeat);
    }

    #[test]
    fn material_fixture_isolates_restitution_response() {
        assert_eq!(physics_material_demo_count(), 6);
        assert_eq!(physics_material_demo_restitution_milli(0), 1_000);
        assert_eq!(physics_material_demo_restitution_milli(5), 0);
        assert_eq!(
            physics_material_demo_position_y(0, 0),
            physics_material_demo_position_y(5, 0)
        );
        assert!(
            physics_material_demo_position_y(0, 300) > physics_material_demo_position_y(5, 300)
        );
    }
}
