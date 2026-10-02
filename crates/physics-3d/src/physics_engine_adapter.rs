use std::fmt;

#[cfg(any(test, feature = "rebuild-reference"))]
use std::collections::BTreeMap;

use ecs_physics::{BodyKind, MATERIAL_SCALE};
use ecs_workload::{EntityId, Position, Velocity};
use physics_engine::{
    AngularState3d as EngineAngularState3d, AngularVelocity3d as EngineAngularVelocity3d,
    BodyId as EngineBodyId, Material as EngineMaterial, Orientation3d as EngineOrientation3d,
    OrientedBoxError3d as EngineOrientedBoxError3d, RigidBody as EngineRigidBody,
    RigidBox3d as EngineRigidBox3d, RigidBoxError3d as EngineRigidBoxError3d,
    RotatingWorldError3d as EngineRotatingWorldError3d, Vec3i as EngineVec3i,
    obb_contact_seed as engine_obb_contact_seed,
};

use crate::{
    AngularState3d, AngularSubstepError3d, AngularVelocity3d, Orientation3d, PhysicsBody3d,
    RigidBox3d, RigidBoxState3d,
};

#[cfg(any(test, feature = "rebuild-reference"))]
use crate::{
    AngularSubstepPolicy3d, RigidBoxWorldConfig3d, RotatingContactSearchConfig3d,
    required_angular_substeps,
};
#[cfg(any(test, feature = "rebuild-reference"))]
use physics_engine::{
    MAX_REPEATED_ROTATING_EVENTS as ENGINE_MAX_REPEATED_ROTATING_EVENTS,
    RotatingWorld3d as EngineRotatingWorld3d, RotatingWorldConfig3d as EngineRotatingWorldConfig3d,
};

/// Result of one ECS-facing frame advanced by the standalone `physics-engine` rotating world.
///
/// The adapter deliberately reports only evidence the standalone engine exposes directly. Legacy
/// frontier/contact-set inspection remains owned by the old experimental solver until that code is
/// deleted after consumer migration.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct PhysicsEngineAdapterStep3d {
    pub boxes: Vec<RigidBox3d>,
    pub sampled_events: usize,
    pub tail_contacts: usize,
    pub substeps: u8,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum PhysicsEngineAdapterError3d {
    DuplicateEntity(EntityId),
    MissingEntity(EntityId),
    MetadataIdentityChange {
        expected: EntityId,
        actual: EntityId,
    },
    BodyIdExhausted,
    CoordinateOutOfRange(EntityId),
    RestitutionOutOfRange(EntityId, u16),
    FrictionOutOfRange(EntityId, u16),
    DampingOutOfRange(u16),
    MissingSourceBody(EngineBodyId),
    ArithmeticOverflow,
    AngularSubstep(AngularSubstepError3d),
    EngineBody(EngineRigidBoxError3d),
    EngineGeometry(EngineOrientedBoxError3d),
    EngineWorld(EngineRotatingWorldError3d),
    EngineInterval(physics_engine::RotatingIntervalError3d),
}

impl fmt::Display for PhysicsEngineAdapterError3d {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::MissingEntity(entity) => {
                write!(formatter, "no physics mapping for entity {}", entity.0)
            }
            Self::MetadataIdentityChange { expected, actual } => write!(
                formatter,
                "entity {} metadata cannot change identity to {}; use remap",
                expected.0, actual.0
            ),
            Self::BodyIdExhausted => formatter.write_str("physics body identity space exhausted"),
            Self::EngineInterval(error) => {
                write!(formatter, "physics-engine interval failed: {error}")
            }
            Self::DuplicateEntity(entity) => {
                write!(
                    formatter,
                    "physics-engine adapter received duplicate entity {}",
                    entity.0
                )
            }
            Self::CoordinateOutOfRange(entity) => write!(
                formatter,
                "physics-engine adapter cannot represent entity {} position in i32 engine coordinates",
                entity.0
            ),
            Self::RestitutionOutOfRange(entity, value) => write!(
                formatter,
                "physics-engine adapter body {} has restitution {value}, expected 0..={MATERIAL_SCALE}",
                entity.0
            ),
            Self::FrictionOutOfRange(entity, value) => write!(
                formatter,
                "physics-engine adapter body {} has friction {value}, expected 0..={MATERIAL_SCALE}",
                entity.0
            ),
            Self::DampingOutOfRange(value) => write!(
                formatter,
                "physics-engine adapter angular damping must be 0..={MATERIAL_SCALE}, got {value}"
            ),
            Self::MissingSourceBody(id) => write!(
                formatter,
                "physics-engine adapter lost ECS metadata for body {}",
                id.0
            ),
            Self::ArithmeticOverflow => {
                write!(formatter, "physics-engine adapter arithmetic overflowed")
            }
            Self::AngularSubstep(error) => {
                write!(
                    formatter,
                    "physics-engine adapter substep policy failed: {error}"
                )
            }
            Self::EngineBody(error) => {
                write!(
                    formatter,
                    "physics-engine adapter body conversion failed: {error}"
                )
            }
            Self::EngineGeometry(error) => {
                write!(
                    formatter,
                    "physics-engine adapter geometry query failed: {error}"
                )
            }
            Self::EngineWorld(error) => {
                write!(
                    formatter,
                    "physics-engine adapter world step failed: {error}"
                )
            }
        }
    }
}

impl std::error::Error for PhysicsEngineAdapterError3d {}

impl From<AngularSubstepError3d> for PhysicsEngineAdapterError3d {
    fn from(value: AngularSubstepError3d) -> Self {
        Self::AngularSubstep(value)
    }
}

impl From<EngineRigidBoxError3d> for PhysicsEngineAdapterError3d {
    fn from(value: EngineRigidBoxError3d) -> Self {
        Self::EngineBody(value)
    }
}

impl From<EngineOrientedBoxError3d> for PhysicsEngineAdapterError3d {
    fn from(value: EngineOrientedBoxError3d) -> Self {
        Self::EngineGeometry(value)
    }
}

impl From<EngineRotatingWorldError3d> for PhysicsEngineAdapterError3d {
    fn from(value: EngineRotatingWorldError3d) -> Self {
        Self::EngineWorld(value)
    }
}

/// Reports whether two ECS-facing rigid boxes positively overlap according to standalone
/// `physics-engine` OBB geometry.
///
/// Zero-depth touching is not penetration. The query deliberately delegates SAT/contact evidence to the
/// standalone engine instead of retaining a second collision implementation in ECS Lab.
///
/// # Errors
///
/// Returns [`PhysicsEngineAdapterError3d`] when either ECS body cannot be represented by the engine or
/// the engine rejects the OBB geometry query.
pub fn physics_engine_boxes_penetrate(
    left: RigidBox3d,
    right: RigidBox3d,
) -> Result<bool, PhysicsEngineAdapterError3d> {
    let left = to_engine_box(left)?;
    let right = to_engine_box(right)?;
    Ok(
        engine_obb_contact_seed(left.oriented_box(), right.oriented_box())?
            .is_some_and(|contact| contact.overlap_numerator > 0),
    )
}

/// Advances ECS-owned rigid-box state through the standalone `physics-engine` authority.
///
/// Entity IDs and component-shaped body metadata remain consumer concerns. The adapter converts that
/// state into engine-local IDs/materials/rigid boxes, executes the engine's bounded sampled rotating
/// event pipeline, then maps the resulting pose and velocities back to the existing ECS-facing types.
/// Angular damping remains explicit consumer policy and is applied exactly once after the requested
/// frame, matching the playground's established contract. If the existing bounded angular policy asks
/// for more than one sample, the requested rational timestep is split evenly and every substep is still
/// executed by `physics-engine`; no collision implementation is duplicated here.
///
/// Rotational event discovery therefore inherits the standalone engine's explicit limitation: it remains
/// sampled rotational collision handling rather than analytic rotational CCD.
///
/// # Errors
///
/// Returns [`PhysicsEngineAdapterError3d`] when ECS state cannot be represented by the standalone engine,
/// material/damping bounds are invalid, the angular substep policy rejects the frame, or the engine fails
/// closed while advancing the world.
#[cfg(any(test, feature = "rebuild-reference"))]
pub fn step_rigid_box_world_with_physics_engine(
    boxes: &[RigidBox3d],
    config: RigidBoxWorldConfig3d,
    search_config: RotatingContactSearchConfig3d,
    substep_policy: AngularSubstepPolicy3d,
) -> Result<PhysicsEngineAdapterStep3d, PhysicsEngineAdapterError3d> {
    if config.angular_damping_milli > MATERIAL_SCALE {
        return Err(PhysicsEngineAdapterError3d::DampingOutOfRange(
            config.angular_damping_milli,
        ));
    }

    let substeps = required_angular_substeps(boxes, config, substep_policy)?;
    let substep_denominator = config
        .timestep_denominator
        .checked_mul(i32::from(substeps))
        .ok_or(PhysicsEngineAdapterError3d::ArithmeticOverflow)?;

    let mut source_bodies = BTreeMap::new();
    let mut world = EngineRotatingWorld3d::new(EngineRotatingWorldConfig3d {
        gravity: engine_velocity(config.gravity),
        sample_count: search_config.coarse_samples,
        refinement_steps: search_config.refinement_steps,
        solver_passes: config.solver_passes,
        max_events: ENGINE_MAX_REPEATED_ROTATING_EVENTS,
    });

    for rigid_box in boxes {
        validate_material(rigid_box.body)?;
        if source_bodies
            .insert(rigid_box.body.entity.0, *rigid_box)
            .is_some()
        {
            return Err(PhysicsEngineAdapterError3d::DuplicateEntity(
                rigid_box.body.entity,
            ));
        }
        world.add_box(to_engine_box(*rigid_box)?)?;
    }

    let mut sampled_events = 0_usize;
    let mut tail_contacts = 0_usize;
    for _ in 0..substeps {
        let report = world.step(config.timestep_numerator, substep_denominator)?;
        sampled_events = sampled_events
            .checked_add(report.stats.sampled_events)
            .ok_or(PhysicsEngineAdapterError3d::ArithmeticOverflow)?;
        tail_contacts = tail_contacts
            .checked_add(report.stats.tail_contacts)
            .ok_or(PhysicsEngineAdapterError3d::ArithmeticOverflow)?;
    }

    let mut converted = Vec::with_capacity(source_bodies.len());
    for engine_box in world.boxes() {
        let id = engine_box.body().id();
        let source = source_bodies
            .get(
                &u32::try_from(id.0)
                    .map_err(|_| PhysicsEngineAdapterError3d::MissingSourceBody(id))?,
            )
            .copied()
            .ok_or(PhysicsEngineAdapterError3d::MissingSourceBody(id))?;
        let mut projected = from_engine_box(engine_box, source.body, source.state.linear_velocity);
        if source.body.kind == BodyKind::Dynamic {
            projected.state.angular.angular_velocity = damp_angular_velocity(
                projected.state.angular.angular_velocity,
                config.angular_damping_milli,
            )?;
        }
        converted.push(projected);
    }

    Ok(PhysicsEngineAdapterStep3d {
        boxes: converted,
        sampled_events,
        tail_contacts,
        substeps,
    })
}

pub(crate) fn validate_material(body: PhysicsBody3d) -> Result<(), PhysicsEngineAdapterError3d> {
    let material = body.material;
    if material.restitution_milli > MATERIAL_SCALE {
        return Err(PhysicsEngineAdapterError3d::RestitutionOutOfRange(
            body.entity,
            material.restitution_milli,
        ));
    }
    if material.friction_milli > MATERIAL_SCALE {
        return Err(PhysicsEngineAdapterError3d::FrictionOutOfRange(
            body.entity,
            material.friction_milli,
        ));
    }
    Ok(())
}

fn to_engine_box(rigid_box: RigidBox3d) -> Result<EngineRigidBox3d, PhysicsEngineAdapterError3d> {
    to_engine_box_with_id(rigid_box, EngineBodyId(u64::from(rigid_box.body.entity.0)))
}

pub(crate) fn to_engine_box_with_id(
    rigid_box: RigidBox3d,
    id: EngineBodyId,
) -> Result<EngineRigidBox3d, PhysicsEngineAdapterError3d> {
    let position = engine_position(rigid_box.body.entity, rigid_box.state.center)?;
    let velocity = engine_velocity(rigid_box.state.linear_velocity);
    let half_extents = EngineVec3i::new(
        rigid_box.body.half_extents[0],
        rigid_box.body.half_extents[1],
        rigid_box.body.half_extents[2],
    );
    let material = EngineMaterial::new(rigid_box.body.material.restitution_milli)
        .with_friction(rigid_box.body.material.friction_milli);
    let body = match rigid_box.body.kind {
        BodyKind::Dynamic => EngineRigidBody::dynamic(id, position, velocity, half_extents)
            .with_mass(rigid_box.body.mass_units)
            .with_material(material),
        BodyKind::Fixed => {
            EngineRigidBody::fixed(id, position, half_extents).with_material(material)
        }
    };
    let angular = EngineAngularState3d::new(
        EngineOrientation3d::new(
            rigid_box.state.angular.orientation.x,
            rigid_box.state.angular.orientation.y,
            rigid_box.state.angular.orientation.z,
            rigid_box.state.angular.orientation.w,
        ),
        EngineAngularVelocity3d::new(
            rigid_box.state.angular.angular_velocity.x,
            rigid_box.state.angular.angular_velocity.y,
            rigid_box.state.angular.angular_velocity.z,
        ),
    );
    EngineRigidBox3d::new(body, angular).map_err(Into::into)
}

pub(crate) fn from_engine_box(
    engine_box: &EngineRigidBox3d,
    source: PhysicsBody3d,
    fixed_velocity: Velocity,
) -> RigidBox3d {
    let body = engine_box.body();
    let angular = engine_box.angular();
    let linear_velocity = if source.kind == BodyKind::Fixed {
        fixed_velocity
    } else {
        Velocity::new3(body.velocity().x, body.velocity().y, body.velocity().z)
    };
    let angular_velocity = AngularVelocity3d::new(
        angular.angular_velocity.x,
        angular.angular_velocity.y,
        angular.angular_velocity.z,
    );
    RigidBox3d::new(
        source,
        RigidBoxState3d::new(
            Position::new3(
                i64::from(body.position().x),
                i64::from(body.position().y),
                i64::from(body.position().z),
            ),
            linear_velocity,
            AngularState3d::new(
                Orientation3d::new(
                    angular.orientation.x,
                    angular.orientation.y,
                    angular.orientation.z,
                    angular.orientation.w,
                ),
                angular_velocity,
            ),
        ),
    )
}

pub(crate) fn engine_position(
    entity: EntityId,
    position: Position,
) -> Result<EngineVec3i, PhysicsEngineAdapterError3d> {
    Ok(EngineVec3i::new(
        coordinate_to_i32(entity, position.x)?,
        coordinate_to_i32(entity, position.y)?,
        coordinate_to_i32(entity, position.z)?,
    ))
}

fn coordinate_to_i32(entity: EntityId, value: i64) -> Result<i32, PhysicsEngineAdapterError3d> {
    i32::try_from(value).map_err(|_| PhysicsEngineAdapterError3d::CoordinateOutOfRange(entity))
}

pub(crate) fn engine_velocity(velocity: Velocity) -> EngineVec3i {
    EngineVec3i::new(velocity.x, velocity.y, velocity.z)
}

#[cfg(any(test, feature = "rebuild-reference"))]
fn damp_angular_velocity(
    velocity: AngularVelocity3d,
    damping_milli: u16,
) -> Result<AngularVelocity3d, PhysicsEngineAdapterError3d> {
    Ok(AngularVelocity3d::new(
        damp_axis(velocity.x, damping_milli)?,
        damp_axis(velocity.y, damping_milli)?,
        damp_axis(velocity.z, damping_milli)?,
    ))
}

#[cfg(any(test, feature = "rebuild-reference"))]
fn damp_axis(value: i32, damping_milli: u16) -> Result<i32, PhysicsEngineAdapterError3d> {
    let numerator = i128::from(value)
        .checked_mul(i128::from(damping_milli))
        .ok_or(PhysicsEngineAdapterError3d::ArithmeticOverflow)?;
    let damped = div_round_nearest(numerator, i128::from(MATERIAL_SCALE))?;
    i32::try_from(damped).map_err(|_| PhysicsEngineAdapterError3d::ArithmeticOverflow)
}

#[cfg(any(test, feature = "rebuild-reference"))]
fn div_round_nearest(
    numerator: i128,
    denominator: i128,
) -> Result<i128, PhysicsEngineAdapterError3d> {
    if denominator <= 0 {
        return Err(PhysicsEngineAdapterError3d::ArithmeticOverflow);
    }
    let half = denominator / 2;
    let adjusted = if numerator >= 0 {
        numerator.checked_add(half)
    } else {
        numerator.checked_sub(half)
    }
    .ok_or(PhysicsEngineAdapterError3d::ArithmeticOverflow)?;
    Ok(adjusted / denominator)
}

#[cfg(test)]
mod tests {
    use ecs_physics::{BodyKind, PhysicsMaterial};
    use ecs_reference::ReferenceWorld;
    use ecs_workload::{EntityId, Position, Velocity};

    use crate::{
        ANGULAR_VELOCITY_SCALE, AngularState3d, AngularSubstepPolicy3d, AngularVelocity3d,
        BouncingRoom3dScenario, Orientation3d, PhysicsBody3d, RigidBox3d, RigidBoxState3d,
        RigidBoxWorldConfig3d, RotatingContactSearchConfig3d,
    };

    use super::{physics_engine_boxes_penetrate, step_rigid_box_world_with_physics_engine};

    fn dynamic(
        entity: u32,
        center: Position,
        velocity: Velocity,
        friction_milli: u16,
    ) -> RigidBox3d {
        RigidBox3d::new(
            PhysicsBody3d::dynamic(EntityId(entity), [10, 10, 10])
                .with_material(PhysicsMaterial::new(0, friction_milli)),
            RigidBoxState3d::new(
                center,
                velocity,
                AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
            ),
        )
    }

    #[test]
    fn engine_geometry_query_distinguishes_touching_from_penetration() {
        let left = dynamic(1, Position::new3(0, 0, 0), Velocity::new3(0, 0, 0), 0);
        let touching = dynamic(2, Position::new3(20, 0, 0), Velocity::new3(0, 0, 0), 0);
        let penetrating = dynamic(3, Position::new3(19, 0, 0), Velocity::new3(0, 0, 0), 0);
        let separated = dynamic(4, Position::new3(21, 0, 0), Velocity::new3(0, 0, 0), 0);

        assert!(
            !physics_engine_boxes_penetrate(left, touching)
                .expect("touching engine geometry should be valid")
        );
        assert!(
            physics_engine_boxes_penetrate(left, penetrating)
                .expect("overlapping engine geometry should be valid")
        );
        assert!(
            !physics_engine_boxes_penetrate(left, separated)
                .expect("separated engine geometry should be valid")
        );
    }

    #[test]
    fn adapter_routes_frictional_obb_response_through_standalone_engine() {
        let boxes = [
            dynamic(1, Position::new3(0, 0, 0), Velocity::new3(60, 0, 40), 1_000),
            dynamic(2, Position::new3(19, 0, 0), Velocity::new3(0, 0, 0), 0),
        ];
        let step = step_rigid_box_world_with_physics_engine(
            &boxes,
            RigidBoxWorldConfig3d {
                gravity: Velocity::new3(0, 0, 0),
                timestep_numerator: 1,
                timestep_denominator: 1,
                angular_damping_milli: 1_000,
                solver_passes: 8,
            },
            RotatingContactSearchConfig3d {
                coarse_samples: 8,
                refinement_steps: 4,
            },
            AngularSubstepPolicy3d::default(),
        )
        .expect("standalone engine adapter should resolve the contact");

        assert_eq!(step.boxes.len(), 2);
        assert!(step.boxes[0].state.linear_velocity.z.abs() < 40);
        assert!(!step.boxes[0].state.angular.angular_velocity.is_zero());
        assert!(step.sampled_events >= 1 || step.tail_contacts >= 1);
    }

    #[test]
    fn fixed_body_velocity_is_preserved_across_adapter_round_trip() {
        let velocity = Velocity::new3(7, -3, 5);
        let fixed = RigidBox3d::new(
            PhysicsBody3d::fixed(EntityId(9), [10, 10, 10]),
            RigidBoxState3d::new(
                Position::new3(20, 0, 0),
                velocity,
                AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
            ),
        );
        let step = step_rigid_box_world_with_physics_engine(
            &[fixed],
            RigidBoxWorldConfig3d {
                gravity: Velocity::new3(0, -10, 0),
                timestep_numerator: 0,
                timestep_denominator: 1,
                angular_damping_milli: 1_000,
                solver_passes: 4,
            },
            RotatingContactSearchConfig3d {
                coarse_samples: 4,
                refinement_steps: 2,
            },
            AngularSubstepPolicy3d::default(),
        )
        .expect("fixed body should round-trip through the adapter");

        assert_eq!(step.boxes[0].state.linear_velocity, velocity);
    }

    fn playground_boxes() -> (Vec<RigidBox3d>, RigidBoxWorldConfig3d) {
        const FPS: u32 = 60;
        let scenario = BouncingRoom3dScenario::with_substeps_per_tick(FPS)
            .expect("60 Hz scenario should be representable");
        let mut world = ReferenceWorld::new();
        world
            .replay(scenario.setup())
            .expect("scenario setup should replay");
        let snapshot = world.snapshot();
        let velocity_factor = i32::try_from(FPS).expect("FPS fits i32");
        let mut boxes = Vec::with_capacity(scenario.bodies().len());

        for body in scenario.bodies() {
            let entity = snapshot
                .entities()
                .iter()
                .find(|candidate| candidate.id == body.entity)
                .expect("scenario entity should exist");
            let center = entity.position.expect("scenario body should have position");
            let source_velocity = entity.velocity.unwrap_or(Velocity::new3(0, 0, 0));
            let linear_velocity = if body.kind == BodyKind::Dynamic {
                Velocity::new3(
                    source_velocity.x * velocity_factor,
                    source_velocity.y * velocity_factor,
                    source_velocity.z * velocity_factor,
                )
            } else {
                Velocity::new3(0, 0, 0)
            };
            let unit = ANGULAR_VELOCITY_SCALE / 5;
            let angular_velocity = if body.kind == BodyKind::Dynamic {
                match body.entity.0 % 8 {
                    0 => AngularVelocity3d::new(unit, unit / 2, -unit),
                    1 => AngularVelocity3d::new(-unit, 2 * unit, unit / 2),
                    2 => AngularVelocity3d::new(unit / 2, -unit, 2 * unit),
                    3 => AngularVelocity3d::new(2 * unit, unit, -unit / 2),
                    4 => AngularVelocity3d::new(-2 * unit, unit / 2, unit),
                    5 => AngularVelocity3d::new(unit, -2 * unit, unit / 2),
                    6 => AngularVelocity3d::new(unit / 2, unit, -2 * unit),
                    _ => AngularVelocity3d::new(-unit, -unit / 2, 2 * unit),
                }
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

        let gravity_scale = i32::try_from(scenario.spatial_scale()).expect("room scale fits i32");
        (
            boxes,
            RigidBoxWorldConfig3d {
                gravity: Velocity::new3(0, -gravity_scale, 0),
                timestep_numerator: 1,
                timestep_denominator: velocity_factor,
                angular_damping_milli: 998,
                solver_passes: 6,
            },
        )
    }

    fn run_playground(frames: usize) -> Vec<RigidBox3d> {
        let (mut boxes, config) = playground_boxes();
        let search = RotatingContactSearchConfig3d {
            coarse_samples: 3,
            refinement_steps: 0,
        };
        for _ in 0..frames {
            boxes = step_rigid_box_world_with_physics_engine(
                &boxes,
                config,
                search,
                AngularSubstepPolicy3d::default(),
            )
            .expect("48-body playground should complete through standalone engine")
            .boxes;
        }
        boxes
    }

    #[test]
    fn forty_eight_body_playground_is_repeatable_through_standalone_engine() {
        let first = run_playground(12);
        let second = run_playground(12);

        assert_eq!(first.len(), 54);
        assert_eq!(first, second);
    }
}
