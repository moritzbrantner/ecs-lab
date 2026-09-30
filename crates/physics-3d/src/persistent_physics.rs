use std::collections::BTreeMap;

use ecs_physics::{BodyKind, MATERIAL_SCALE};
use ecs_workload::{EntityId, Position, Velocity};
use physics_engine::{
    AngularState3d as EngineAngularState, AngularVelocity3d as EngineAngularVelocity, BodyId,
    BodyKind as EngineBodyKind, MAX_REPEATED_ROTATING_EVENTS, Material,
    Orientation3d as EngineOrientation, RigidBody, RigidBox3d as EngineBox,
    RotatingIntervalConfig3d, RotatingIntervalError3d, RotatingIntervalWork3d, RotatingWorld3d,
    RotatingWorldConfig3d, RotatingWorldStepReport3d,
};

pub use physics_engine::MotionAuthority3d as PhysicsEngineMotionAuthority3d;

use crate::{
    AngularSubstepPolicy3d, AngularVelocity3d, Orientation3d, PhysicsBody3d,
    PhysicsEngineAdapterError3d as Error, PhysicsEngineAdapterStep3d, RigidBox3d,
    RigidBoxWorldConfig3d, RotatingContactSearchConfig3d,
    angular_substep::required_substeps_for_velocities,
    physics_engine_adapter::{
        engine_position, engine_velocity, from_engine_box, to_engine_box_with_id, validate_material,
    },
    required_angular_substeps,
};

/// Logical work at the complete adapter boundary, including failed frame attempts.
/// Counts exclude allocator overhead; engine contact work is a diagnostic subset, not total geometry work.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct PhysicsEngineAdapterWork3d {
    pub world_constructions: u64,
    pub insertions: u64,
    pub removals: u64,
    pub remaps: u64,
    pub descriptor_commands: u64,
    pub changed_descriptors: u64,
    pub metadata_changes: u64,
    pub motion_commands: u64,
    pub changed_motion: u64,
    pub initial_policy_scan_visits: u64,
    pub angular_scan_visits: u64,
    pub input_conversions: u64,
    pub geometry_validations: u64,
    pub fixed_insert_queries: u64,
    pub output_conversions: u64,
    pub output_sorts: u64,
    pub attempted_frames: u64,
    pub completed_frames: u64,
    pub completed_substeps: u64,
    pub motion_before_images: u64,
    pub sleep_before_images: u64,
    pub parked_before_images: u64,
    pub damping_visits: u64,
    pub engine_contact_work: [u64; 4],
}

/// Retained logical entries/capacity. This is not allocator bytes or process RSS.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct PhysicsEngineAdapterRetained3d {
    pub bodies: usize,
    pub entity_mappings: usize,
    pub body_metadata_entries: usize,
    pub sleeping_bodies: usize,
    pub report_capacity: usize,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct Metadata {
    body: PhysicsBody3d,
    // Fixed input velocity is an ECS compatibility field, not simulated motion.
    fixed_velocity: Velocity,
}

/// One owned engine with stable mappings and metadata, never a second simulated pose/velocity store.
/// Create with `new`, reset explicitly, and dispose by dropping the owner. Converted snapshots are views.
pub struct PersistentPhysicsWorld3d {
    world: RotatingWorld3d,
    entities: BTreeMap<EntityId, BodyId>,
    metadata: BTreeMap<BodyId, Metadata>,
    next_body_id: u64,
    output_needs_sort: bool,
    config: RigidBoxWorldConfig3d,
    search: RotatingContactSearchConfig3d,
    policy: AngularSubstepPolicy3d,
    reports: Vec<RotatingWorldStepReport3d>,
    work: PhysicsEngineAdapterWork3d,
}

impl PersistentPhysicsWorld3d {
    /// Creates a world once. Initial `BodyId` values preserve the reference adapter's entity ordering.
    ///
    /// # Errors
    /// Returns adapter validation/conversion errors; no live world is replaced on failure.
    pub fn new(
        boxes: &[RigidBox3d],
        config: RigidBoxWorldConfig3d,
        search: RotatingContactSearchConfig3d,
        policy: AngularSubstepPolicy3d,
    ) -> Result<Self, Error> {
        if config.angular_damping_milli > MATERIAL_SCALE {
            return Err(Error::DampingOutOfRange(config.angular_damping_milli));
        }
        required_angular_substeps(boxes, config, policy)?;
        let mut result = Self {
            world: RotatingWorld3d::new(RotatingWorldConfig3d {
                gravity: engine_velocity(config.gravity),
                sample_count: search.coarse_samples,
                refinement_steps: search.refinement_steps,
                solver_passes: config.solver_passes,
                max_events: MAX_REPEATED_ROTATING_EVENTS,
            }),
            entities: BTreeMap::new(),
            metadata: BTreeMap::new(),
            next_body_id: 0,
            output_needs_sort: false,
            config,
            search,
            policy,
            reports: Vec::new(),
            work: PhysicsEngineAdapterWork3d {
                world_constructions: 1,
                initial_policy_scan_visits: boxes.len() as u64,
                ..PhysicsEngineAdapterWork3d::default()
            },
        };
        for rigid_box in boxes {
            let entity = rigid_box.body.entity;
            if result.entities.contains_key(&entity) {
                return Err(Error::DuplicateEntity(entity));
            }
            let id = BodyId(u64::from(entity.0));
            result.insert_at(*rigid_box, id)?;
            result.next_body_id = result.next_body_id.max(id.0 + 1);
        }
        Ok(result)
    }

    /// Builds replacement state before dropping the prior world. Reset is an explicit bulk operation.
    ///
    /// # Errors
    /// Invalid inputs leave the current world, mappings and configuration unchanged.
    pub fn reset(&mut self, boxes: &[RigidBox3d]) -> Result<(), Error> {
        let mut replacement = Self::new(boxes, self.config, self.search, self.policy)?;
        let creation = replacement.work;
        replacement.work = self.work;
        replacement.work.world_constructions += creation.world_constructions;
        replacement.work.insertions += creation.insertions;
        replacement.work.input_conversions += creation.input_conversions;
        replacement.work.geometry_validations += creation.geometry_validations;
        replacement.work.fixed_insert_queries += creation.fixed_insert_queries;
        replacement.work.initial_policy_scan_visits += creation.initial_policy_scan_visits;
        *self = replacement;
        Ok(())
    }

    /// Inserts an actual new member using a fresh engine identity, including after entity-ID reuse.
    ///
    /// # Errors
    /// Duplicate identities or invalid descriptors fail before membership changes.
    pub fn insert(&mut self, body: RigidBox3d) -> Result<(), Error> {
        if self.entities.contains_key(&body.body.entity) {
            return Err(Error::DuplicateEntity(body.body.entity));
        }
        let next = self
            .next_body_id
            .checked_add(1)
            .ok_or(Error::BodyIdExhausted)?;
        self.insert_at(body, BodyId(self.next_body_id))?;
        self.next_body_id = next;
        self.output_needs_sort = true;
        Ok(())
    }

    fn insert_at(&mut self, body: RigidBox3d, id: BodyId) -> Result<(), Error> {
        validate_material(body.body)?;
        self.work.input_conversions += 1;
        let converted = to_engine_box_with_id(body, id)?;
        self.work.geometry_validations += 1;
        physics_engine::rigid_box_free_flight_sweep_bounds(
            &converted,
            physics_engine::RigidBoxFreeFlightConfig3d::new(physics_engine::Vec3i::ZERO, 0, 1),
        )
        .map_err(|error| Error::EngineWorld(error.into()))?;
        if body.body.kind == BodyKind::Fixed {
            self.work.fixed_insert_queries += 1;
            self.world.overlap_query(converted.oriented_box())?;
        }
        self.world.add_box(converted)?;
        self.entities.insert(body.body.entity, id);
        self.metadata.insert(
            id,
            Metadata {
                body: body.body,
                fixed_velocity: if body.body.kind == BodyKind::Fixed {
                    body.state.linear_velocity
                } else {
                    Velocity::new3(0, 0, 0)
                },
            },
        );
        self.work.insertions += 1;
        Ok(())
    }

    /// Removes one actual member and returns its final observable state.
    ///
    /// # Panics
    /// Panics only if the private engine/mapping invariant is broken.
    pub fn remove(&mut self, entity: EntityId) -> Option<RigidBox3d> {
        let id = self.entities.get(&entity).copied()?;
        let metadata = self.metadata[&id];
        let output = from_engine_box(
            self.world.box_by_id(id).expect("mapped body"),
            metadata.body,
            metadata.fixed_velocity,
        );
        self.world.remove_box(id).expect("mapped body");
        self.entities.remove(&entity);
        self.metadata.remove(&id);
        self.work.removals += 1;
        self.work.output_conversions += 1;
        Some(output)
    }

    /// Changes consumer identity while preserving engine identity and physical history.
    ///
    /// # Errors
    /// Missing or duplicate mappings fail without changes.
    ///
    /// # Panics
    /// Panics only if the private engine/mapping invariant is broken.
    pub fn remap(&mut self, entity: EntityId, replacement: EntityId) -> Result<bool, Error> {
        let id = self.id(entity)?;
        if entity == replacement {
            return Ok(false);
        }
        if self.entities.contains_key(&replacement) {
            return Err(Error::DuplicateEntity(replacement));
        }
        self.entities.remove(&entity);
        self.entities.insert(replacement, id);
        self.metadata
            .get_mut(&id)
            .expect("mapped metadata")
            .body
            .entity = replacement;
        self.work.remaps += 1;
        self.output_needs_sort = true;
        Ok(true)
    }

    /// Applies one explicitly authored complete-body change, never an ordinary output replay.
    ///
    /// # Errors
    /// Invalid metadata, pose, geometry or contact admission leaves the member unchanged.
    ///
    /// # Panics
    /// Panics only if the private engine/mapping invariant is broken.
    pub fn replace(&mut self, entity: EntityId, replacement: RigidBox3d) -> Result<bool, Error> {
        let id = self.id(entity)?;
        Self::validate_identity(entity, replacement.body)?;
        validate_material(replacement.body)?;
        let authority = self
            .world
            .box_by_id(id)
            .expect("mapped body")
            .motion_authority();
        let converted = to_engine_box_with_id(replacement, id)?.with_motion_authority(authority);
        let metadata = Metadata {
            body: replacement.body,
            fixed_velocity: if replacement.body.kind == BodyKind::Fixed {
                replacement.state.linear_velocity
            } else {
                Velocity::new3(0, 0, 0)
            },
        };
        self.apply_descriptor(id, converted, metadata)
    }

    /// Updates shape, material, mass or kind while borrowing simulated motion directly from physics.
    ///
    /// # Errors
    /// Invalid identity/geometry/mass or a spinning transition to fixed fails before changes.
    ///
    /// # Panics
    /// Panics only if the private engine/mapping invariant is broken.
    pub fn update_metadata(
        &mut self,
        entity: EntityId,
        replacement: PhysicsBody3d,
    ) -> Result<bool, Error> {
        let id = self.id(entity)?;
        Self::validate_identity(entity, replacement)?;
        validate_material(replacement)?;
        let current = self.world.box_by_id(id).expect("mapped body");
        let translational = Self::body_descriptor(
            id,
            replacement,
            current.body().position(),
            current.body().velocity(),
        );
        let converted = current.clone().with_body(translational)?;
        let metadata = Metadata {
            body: replacement,
            fixed_velocity: if replacement.kind == BodyKind::Fixed {
                self.metadata[&id].fixed_velocity
            } else {
                Velocity::new3(0, 0, 0)
            },
        };
        self.apply_descriptor(id, converted, metadata)
    }

    /// Teleports one body; unchanged orientation preserves its simulated quaternion exactly.
    ///
    /// # Errors
    /// Invalid pose/bounds/contact admission fails before wake or mutation.
    ///
    /// # Panics
    /// Panics only if the private engine/mapping invariant is broken.
    pub fn teleport(
        &mut self,
        entity: EntityId,
        center: Position,
        orientation: Orientation3d,
    ) -> Result<bool, Error> {
        let id = self.id(entity)?;
        let current = self.world.box_by_id(id).expect("mapped body");
        let metadata = self.metadata[&id];
        let translational = Self::body_descriptor(
            id,
            metadata.body,
            engine_position(entity, center)?,
            current.body().velocity(),
        );
        let orientation =
            EngineOrientation::new(orientation.x, orientation.y, orientation.z, orientation.w);
        let converted = if orientation == current.angular().orientation {
            current.clone().with_body(translational)?
        } else {
            EngineBox::new(
                translational,
                EngineAngularState::new(orientation, current.angular().angular_velocity),
            )?
            .with_motion_authority(current.motion_authority())
        };
        self.apply_descriptor(id, converted, metadata)
    }

    /// Changes ownership without writing any simulated pose/velocity back through ECS.
    ///
    /// # Errors
    /// Missing identity or rejected contact admission leaves the body unchanged.
    ///
    /// # Panics
    /// Panics only if the private engine/mapping invariant is broken.
    pub fn set_authority(
        &mut self,
        entity: EntityId,
        authority: PhysicsEngineMotionAuthority3d,
    ) -> Result<bool, Error> {
        let id = self.id(entity)?;
        let converted = self
            .world
            .box_by_id(id)
            .expect("mapped body")
            .clone()
            .with_motion_authority(authority);
        self.apply_descriptor(id, converted, self.metadata[&id])
    }

    /// Applies one intended motion command; output snapshots never call this automatically.
    ///
    /// # Errors
    /// Missing or fixed-body nonzero motion fails before mutation.
    pub fn set_motion(
        &mut self,
        entity: EntityId,
        linear: Velocity,
        angular: AngularVelocity3d,
    ) -> Result<bool, Error> {
        let id = self.id(entity)?;
        self.work.motion_commands += 1;
        let changed = self.world.set_motion(
            id,
            engine_velocity(linear),
            EngineAngularVelocity::new(angular.x, angular.y, angular.z),
        )?;
        self.work.changed_motion += u64::from(changed);
        Ok(changed)
    }

    fn apply_descriptor(
        &mut self,
        id: BodyId,
        replacement: EngineBox,
        metadata: Metadata,
    ) -> Result<bool, Error> {
        self.work.descriptor_commands += 1;
        let changed = self.world.replace_box(replacement)?;
        let metadata_changed = self.metadata[&id] != metadata;
        self.metadata.insert(id, metadata);
        self.work.changed_descriptors += u64::from(changed);
        self.work.metadata_changes += u64::from(metadata_changed);
        Ok(changed || metadata_changed)
    }

    fn body_descriptor(
        id: BodyId,
        metadata: PhysicsBody3d,
        position: physics_engine::Vec3i,
        velocity: physics_engine::Vec3i,
    ) -> RigidBody {
        let half = physics_engine::Vec3i::new(
            metadata.half_extents[0],
            metadata.half_extents[1],
            metadata.half_extents[2],
        );
        let body = match metadata.kind {
            BodyKind::Dynamic => {
                RigidBody::dynamic(id, position, velocity, half).with_mass(metadata.mass_units)
            }
            BodyKind::Fixed => RigidBody::fixed(id, position, half),
        };
        body.with_material(
            Material::new(metadata.material.restitution_milli)
                .with_friction(metadata.material.friction_milli),
        )
    }

    fn validate_identity(entity: EntityId, metadata: PhysicsBody3d) -> Result<(), Error> {
        if metadata.entity == entity {
            Ok(())
        } else {
            Err(Error::MetadataIdentityChange {
                expected: entity,
                actual: metadata.entity,
            })
        }
    }

    fn id(&self, entity: EntityId) -> Result<BodyId, Error> {
        self.entities
            .get(&entity)
            .copied()
            .ok_or(Error::MissingEntity(entity))
    }

    /// Advances the original requested frame with identical angular policy and once-per-frame damping.
    ///
    /// # Errors
    /// Complete returned-error rollback includes late substeps and checked diagnostic event sums.
    pub fn advance(&mut self) -> Result<PhysicsEngineAdapterStep3d, Error> {
        self.work.attempted_frames += 1;
        let mut visits = 0;
        let velocities = self
            .world
            .boxes()
            .inspect(|_| visits += 1)
            .filter(|body| body.body().kind() == EngineBodyKind::Dynamic)
            .map(|body| {
                let omega = body.angular().angular_velocity;
                AngularVelocity3d::new(omega.x, omega.y, omega.z)
            });
        let substeps = required_substeps_for_velocities(velocities, self.config, self.policy);
        self.work.angular_scan_visits += visits;
        let substeps = substeps?;
        self.config
            .timestep_denominator
            .checked_mul(i32::from(substeps))
            .ok_or(Error::ArithmeticOverflow)?;
        let interval = RotatingIntervalConfig3d {
            timestep_numerator: self.config.timestep_numerator,
            timestep_denominator: self.config.timestep_denominator,
            substeps,
            angular_damping_milli: self.config.angular_damping_milli,
        };
        let work = match self.world.advance_interval(interval, &mut self.reports) {
            Ok(work) => work,
            Err(failure) => {
                self.record_physics_work(failure.work);
                return Err(match failure.error {
                    RotatingIntervalError3d::World(error) => Error::EngineWorld(error),
                    RotatingIntervalError3d::EventCountOverflow => Error::ArithmeticOverflow,
                    error => Error::EngineInterval(error),
                });
            }
        };
        self.record_physics_work(work);
        // Projection is infallible: engine coordinates widen to ECS integers and metadata mappings
        // are maintained by the mutation seam. No fallible validation remains after interval commit.
        let boxes = self.snapshot();
        self.work.completed_frames += 1;
        Ok(PhysicsEngineAdapterStep3d {
            boxes,
            sampled_events: work.sampled_events,
            tail_contacts: work.tail_contacts,
            substeps,
        })
    }

    fn record_physics_work(&mut self, work: RotatingIntervalWork3d) {
        self.work.completed_substeps += u64::from(work.completed_substeps);
        self.work.motion_before_images += work.motion_before_images as u64;
        self.work.sleep_before_images += work.sleep_before_images as u64;
        self.work.parked_before_images += work.parked_before_images as u64;
        self.work.damping_visits += work.damping_body_visits as u64;
        for (sum, count) in self
            .work
            .engine_contact_work
            .iter_mut()
            .zip(work.contact_work)
        {
            *sum = sum.saturating_add(count);
        }
    }

    /// Converts the complete observable view. This is counted separately from engine maintenance.
    pub fn snapshot(&mut self) -> Vec<RigidBox3d> {
        let mut output = Vec::with_capacity(self.metadata.len());
        for body in self.world.boxes() {
            let metadata = self.metadata[&body.body().id()];
            output.push(from_engine_box(
                body,
                metadata.body,
                metadata.fixed_velocity,
            ));
        }
        self.work.output_conversions += output.len() as u64;
        if self.output_needs_sort {
            output.sort_unstable_by_key(|body| body.body.entity);
            self.work.output_sorts += 1;
        }
        output
    }

    #[must_use]
    pub const fn work(&self) -> PhysicsEngineAdapterWork3d {
        self.work
    }

    #[must_use]
    pub fn body_id(&self, entity: EntityId) -> Option<BodyId> {
        self.entities.get(&entity).copied()
    }

    #[must_use]
    pub fn retained(&self) -> PhysicsEngineAdapterRetained3d {
        PhysicsEngineAdapterRetained3d {
            bodies: self.world.entity_count(),
            entity_mappings: self.entities.len(),
            body_metadata_entries: self.metadata.len(),
            sleeping_bodies: self.world.sleeping_body_count(),
            report_capacity: self.reports.capacity(),
        }
    }
}
