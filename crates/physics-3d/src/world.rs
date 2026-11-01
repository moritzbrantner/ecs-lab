use std::fmt;

use ecs_physics::{BodyKind, MATERIAL_SCALE};
use ecs_workload::{EntityId, Operation, Position, Velocity};
use rapier3d::prelude::{
    CoefficientCombineRule, ColliderBuilder, ColliderHandle, PhysicsWorld, Pose, RigidBodyBuilder,
    RigidBodyHandle, Rotation, Vector,
};

use crate::{
    ANGULAR_VELOCITY_SCALE, AngularState3d, AngularVelocity3d, Orientation3d, RigidBox3d,
    RigidBoxState3d,
    body::{quantize_i32, quantize_i64},
};

/// Rapier world parameters in the scenario's integer units.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RapierWorldConfig3d {
    /// Integer spatial units per Rapier metre.
    pub spatial_scale: i64,
    /// Gravity in spatial units per second squared.
    pub gravity: Velocity,
    /// Fixed steps per second.
    pub hz: u32,
    /// Rapier angular damping coefficient (1/s) for dynamic bodies.
    pub angular_damping: f32,
    /// Rapier solver iterations per step.
    pub solver_iterations: u8,
    /// Locks the rotation of every dynamic body (translation-only scenarios).
    pub lock_rotations: bool,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum PhysicsError3d {
    InvalidSpatialScale(i64),
    InvalidRate(u32),
    InvalidSolverIterations,
    InvalidHalfExtents(EntityId),
    ZeroMass(EntityId),
    DuplicateEntity(EntityId),
    NonFiniteState(EntityId),
}

impl fmt::Display for PhysicsError3d {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidSpatialScale(scale) => {
                write!(formatter, "spatial scale {scale} must be in 1..=i32::MAX")
            }
            Self::InvalidRate(hz) => write!(formatter, "step rate {hz} Hz must be positive"),
            Self::InvalidSolverIterations => {
                write!(formatter, "Rapier needs at least one solver iteration")
            }
            Self::InvalidHalfExtents(entity) => {
                write!(formatter, "body {} has non-positive half extents", entity.0)
            }
            Self::ZeroMass(entity) => write!(formatter, "dynamic body {} has zero mass", entity.0),
            Self::DuplicateEntity(entity) => {
                write!(formatter, "entity {} appears more than once", entity.0)
            }
            Self::NonFiniteState(entity) => {
                write!(
                    formatter,
                    "Rapier state of body {} became non-finite",
                    entity.0
                )
            }
        }
    }
}

impl std::error::Error for PhysicsError3d {}

/// The contact between two bodies after the last step.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PairContact3d {
    /// Contact points the solver saw.
    pub points: u32,
    /// Sum of normal impulses, in mass units times metres per second.
    pub normal_impulse: f32,
}

/// One quantized frame: the integer ECS view of the Rapier world after a step.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct RapierFrame3d {
    boxes: Vec<RigidBox3d>,
    operations: Vec<Operation>,
    contact_pairs: usize,
}

impl RapierFrame3d {
    /// Every body, in insertion order.
    #[must_use]
    pub fn boxes(&self) -> &[RigidBox3d] {
        &self.boxes
    }

    #[must_use]
    pub fn into_boxes(self) -> Vec<RigidBox3d> {
        self.boxes
    }

    /// `SetPosition`/`SetVelocity` for every dynamic body, in insertion order.
    #[must_use]
    pub fn operations(&self) -> &[Operation] {
        &self.operations
    }

    /// Collider pairs with at least one active contact.
    #[must_use]
    pub const fn contact_pairs(&self) -> usize {
        self.contact_pairs
    }
}

struct Entry {
    body: RigidBodyHandle,
    collider: ColliderHandle,
}

/// A persistent Rapier world over integer rigid boxes.
///
/// Rapier owns the motion; [`Self::advance`] steps it once and quantizes the result. The integer
/// view never feeds back into Rapier, so quantization error does not accumulate.
pub struct RapierWorld3d {
    world: PhysicsWorld,
    entries: Vec<Entry>,
    boxes: Vec<RigidBox3d>,
    spatial_scale: i64,
    velocity_scale: i32,
}

impl fmt::Debug for RapierWorld3d {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter
            .debug_struct("RapierWorld3d")
            .field("boxes", &self.boxes.len())
            .finish_non_exhaustive()
    }
}

impl RapierWorld3d {
    /// Builds the world from integer boxes.
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError3d`] for an invalid configuration, a malformed body or a duplicate
    /// entity.
    pub fn new(boxes: &[RigidBox3d], config: RapierWorldConfig3d) -> Result<Self, PhysicsError3d> {
        let velocity_scale = i32::try_from(config.spatial_scale)
            .ok()
            .filter(|scale| *scale > 0)
            .ok_or(PhysicsError3d::InvalidSpatialScale(config.spatial_scale))?;
        if config.hz == 0 {
            return Err(PhysicsError3d::InvalidRate(config.hz));
        }
        if config.solver_iterations == 0 {
            return Err(PhysicsError3d::InvalidSolverIterations);
        }
        let mut entities = boxes
            .iter()
            .map(|rigid_box| rigid_box.body.entity)
            .collect::<Vec<_>>();
        entities.sort_unstable();
        if let Some(pair) = entities.windows(2).find(|pair| pair[0] == pair[1]) {
            return Err(PhysicsError3d::DuplicateEntity(pair[0]));
        }

        let scale = config.spatial_scale as f32;
        let mut world = PhysicsWorld::new();
        world.gravity = Vector::new(
            config.gravity.x as f32 / scale,
            config.gravity.y as f32 / scale,
            config.gravity.z as f32 / scale,
        );
        world.integration_parameters.dt = 1.0 / config.hz as f32;
        world.integration_parameters.num_solver_iterations = usize::from(config.solver_iterations);

        let mut entries = Vec::with_capacity(boxes.len());
        for rigid_box in boxes {
            let body = rigid_box.body;
            if body.half_extents.iter().any(|extent| *extent <= 0) {
                return Err(PhysicsError3d::InvalidHalfExtents(body.entity));
            }
            let state = rigid_box.state;
            let [qx, qy, qz, qw] = state
                .angular
                .orientation
                .to_unit()
                .map(|value| value as f32);
            let pose = Pose::from_parts(
                Vector::new(
                    state.center.x as f32 / scale,
                    state.center.y as f32 / scale,
                    state.center.z as f32 / scale,
                ),
                Rotation::from_xyzw(qx, qy, qz, qw).normalize(),
            );
            let builder = match body.kind {
                BodyKind::Fixed => RigidBodyBuilder::fixed().pose(pose),
                BodyKind::Dynamic => {
                    if body.mass_units == 0 {
                        return Err(PhysicsError3d::ZeroMass(body.entity));
                    }
                    let velocity = state.linear_velocity;
                    let omega = state.angular.angular_velocity;
                    let angular = ANGULAR_VELOCITY_SCALE as f32;
                    let builder = RigidBodyBuilder::dynamic()
                        .pose(pose)
                        .linvel(Vector::new(
                            velocity.x as f32 / scale,
                            velocity.y as f32 / scale,
                            velocity.z as f32 / scale,
                        ))
                        .angvel(Vector::new(
                            omega.x as f32 / angular,
                            omega.y as f32 / angular,
                            omega.z as f32 / angular,
                        ))
                        .angular_damping(config.angular_damping)
                        .ccd_enabled(true);
                    if config.lock_rotations {
                        builder.lock_rotations()
                    } else {
                        builder
                    }
                }
            };
            let material = body.material;
            let milli = f32::from(MATERIAL_SCALE);
            let half = body.half_extents.map(|extent| extent as f32 / scale);
            let mut collider = ColliderBuilder::cuboid(half[0], half[1], half[2])
                .restitution(f32::from(material.restitution_milli) / milli)
                .friction(f32::from(material.friction_milli) / milli)
                .restitution_combine_rule(CoefficientCombineRule::Max)
                .friction_combine_rule(CoefficientCombineRule::Max);
            if body.kind == BodyKind::Dynamic {
                collider = collider.mass(body.mass_units as f32);
            }
            let (body, collider) = world.insert(builder, collider);
            entries.push(Entry { body, collider });
        }

        let mut rapier = Self {
            world,
            entries,
            boxes: boxes.to_vec(),
            spatial_scale: config.spatial_scale,
            velocity_scale,
        };
        // Quantize the initial state too, so frame zero and later frames share one rounding.
        rapier.boxes = rapier.quantize()?;
        Ok(rapier)
    }

    /// The integer view of the current state, in insertion order.
    #[must_use]
    pub fn boxes(&self) -> &[RigidBox3d] {
        &self.boxes
    }

    /// Steps Rapier once and returns the quantized frame.
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError3d::NonFiniteState`] when a body's state is no longer finite.
    pub fn advance(&mut self) -> Result<RapierFrame3d, PhysicsError3d> {
        self.world.step();
        self.boxes = self.quantize()?;
        let mut operations = Vec::new();
        for rigid_box in &self.boxes {
            if rigid_box.body.kind == BodyKind::Dynamic {
                let entity = rigid_box.body.entity;
                operations.push(Operation::SetPosition(entity, rigid_box.state.center));
                operations.push(Operation::SetVelocity(
                    entity,
                    rigid_box.state.linear_velocity,
                ));
            }
        }
        let contact_pairs = self
            .world
            .narrow_phase
            .contact_pairs()
            .filter(|pair| pair.has_any_active_contact())
            .count();
        Ok(RapierFrame3d {
            boxes: self.boxes.clone(),
            operations,
            contact_pairs,
        })
    }

    /// The contact between bodies `left` and `right` (insertion indices) after the last step.
    #[must_use]
    pub fn contact(&self, left: usize, right: usize) -> Option<PairContact3d> {
        let left = self.entries.get(left)?.collider;
        let right = self.entries.get(right)?.collider;
        let pair = self.world.narrow_phase.contact_pair(left, right)?;
        if !pair.has_any_active_contact() {
            return None;
        }
        let points = pair
            .solver_manifolds()
            .iter()
            .map(|manifold| manifold.data.num_active_contacts())
            .sum::<usize>();
        Some(PairContact3d {
            points: u32::try_from(points).unwrap_or(u32::MAX),
            normal_impulse: pair.total_impulse_magnitude(),
        })
    }

    /// Deepest penetration between any two colliders after the last step, in metres.
    #[must_use]
    pub fn max_penetration(&self) -> f32 {
        self.world
            .narrow_phase
            .contact_pairs()
            .filter_map(|pair| pair.find_deepest_contact())
            .map(|(_, contact)| -contact.dist)
            .fold(0.0, f32::max)
    }

    /// Deepest penetration of body `index` into any other collider, in metres.
    #[must_use]
    pub fn penetration_of(&self, index: usize) -> f32 {
        let Some(entry) = self.entries.get(index) else {
            return 0.0;
        };
        self.world
            .narrow_phase
            .contact_pairs_with(entry.collider)
            .filter_map(|pair| pair.find_deepest_contact())
            .map(|(_, contact)| -contact.dist)
            .fold(0.0, f32::max)
    }

    fn quantize(&self) -> Result<Vec<RigidBox3d>, PhysicsError3d> {
        self.entries
            .iter()
            .zip(&self.boxes)
            .map(|(entry, previous)| {
                // Fixed bodies never move; keep their exact integer pose.
                if previous.body.kind == BodyKind::Fixed {
                    return Ok(*previous);
                }
                let body = &self.world.bodies[entry.body];
                let translation = body.translation();
                let rotation = body.rotation();
                let linvel = body.linvel();
                let angvel = body.angvel();
                let finite = translation.is_finite()
                    && rotation.is_finite()
                    && linvel.is_finite()
                    && angvel.is_finite();
                if !finite {
                    return Err(PhysicsError3d::NonFiniteState(previous.body.entity));
                }
                let position = |value: f32| quantize_i64(f64::from(value), self.spatial_scale);
                let velocity = |value: f32| quantize_i32(f64::from(value), self.velocity_scale);
                let spin = |value: f32| quantize_i32(f64::from(value), ANGULAR_VELOCITY_SCALE);
                Ok(RigidBox3d::new(
                    previous.body,
                    RigidBoxState3d::new(
                        Position::new3(
                            position(translation.x),
                            position(translation.y),
                            position(translation.z),
                        ),
                        Velocity::new3(velocity(linvel.x), velocity(linvel.y), velocity(linvel.z)),
                        AngularState3d::new(
                            Orientation3d::from_unit(
                                [rotation.x, rotation.y, rotation.z, rotation.w].map(f64::from),
                            ),
                            AngularVelocity3d::new(spin(angvel.x), spin(angvel.y), spin(angvel.z)),
                        ),
                    ),
                ))
            })
            .collect()
    }
}
