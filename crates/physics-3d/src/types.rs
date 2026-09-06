use std::fmt;

use ecs_physics::{BodyKind, MATERIAL_SCALE, PhysicsMaterial};
use ecs_workload::{EntityId, Operation, Velocity};

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct PhysicsBody3d {
    pub entity: EntityId,
    pub kind: BodyKind,
    pub half_extents: [i32; 3],
    pub mass_units: u32,
    pub material: PhysicsMaterial,
}

impl PhysicsBody3d {
    #[must_use]
    pub const fn dynamic(entity: EntityId, half_extents: [i32; 3]) -> Self {
        Self {
            entity,
            kind: BodyKind::Dynamic,
            half_extents,
            mass_units: 1,
            material: PhysicsMaterial::new(0, 0),
        }
    }

    #[must_use]
    pub const fn fixed(entity: EntityId, half_extents: [i32; 3]) -> Self {
        Self {
            entity,
            kind: BodyKind::Fixed,
            half_extents,
            mass_units: 0,
            material: PhysicsMaterial::new(0, 0),
        }
    }

    #[must_use]
    pub const fn with_mass(mut self, mass_units: u32) -> Self {
        self.mass_units = mass_units;
        self
    }

    #[must_use]
    pub const fn with_material(mut self, material: PhysicsMaterial) -> Self {
        self.material = material;
        self
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct PhysicsConfig3d {
    pub gravity: Velocity,
}

impl Default for PhysicsConfig3d {
    fn default() -> Self {
        Self {
            gravity: Velocity::new3(0, -1, 0),
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ContactNormal3d {
    pub x: i8,
    pub y: i8,
    pub z: i8,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct PhysicsContact3d {
    pub left: EntityId,
    pub right: EntityId,
    pub normal: ContactNormal3d,
    pub penetration: i64,
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct PhysicsStep3dStats {
    pub body_count: usize,
    pub candidate_pairs: usize,
    pub ccd_candidate_pairs: usize,
    pub ccd_contacts: usize,
    pub contacts: usize,
    pub resolved_contacts: usize,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct PhysicsStep3d {
    pub(crate) operations: Vec<Operation>,
    pub(crate) stats: PhysicsStep3dStats,
    pub(crate) contacts: Vec<PhysicsContact3d>,
    pub(crate) supporting_entities: Vec<EntityId>,
}

impl PhysicsStep3d {
    #[must_use]
    pub fn operations(&self) -> &[Operation] {
        &self.operations
    }

    #[must_use]
    pub const fn stats(&self) -> PhysicsStep3dStats {
        self.stats
    }

    #[must_use]
    pub fn contacts(&self) -> &[PhysicsContact3d] {
        &self.contacts
    }

    #[must_use]
    pub fn supporting_entities(&self) -> &[EntityId] {
        &self.supporting_entities
    }

    #[must_use]
    pub fn is_supported(&self, entity: EntityId) -> bool {
        self.supporting_entities.binary_search(&entity).is_ok()
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum PhysicsError3d {
    DuplicateBody(EntityId),
    MissingEntity(EntityId),
    MissingPosition(EntityId),
    MissingVelocity(EntityId),
    InvalidHalfExtents(EntityId),
    ZeroMass(EntityId),
    RestitutionOutOfRange(EntityId, u16),
    FrictionOutOfRange(EntityId, u16),
    CoordinateOutOfRange(EntityId),
    NonPositiveTicks(i32),
    CcdIterationLimit(EntityId),
}

impl fmt::Display for PhysicsError3d {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::DuplicateBody(entity) => {
                write!(formatter, "duplicate 3D physics body {}", entity.0)
            }
            Self::MissingEntity(entity) => {
                write!(formatter, "3D physics body {} is not alive", entity.0)
            }
            Self::MissingPosition(entity) => {
                write!(formatter, "3D physics body {} has no position", entity.0)
            }
            Self::MissingVelocity(entity) => write!(
                formatter,
                "dynamic 3D physics body {} has no velocity",
                entity.0
            ),
            Self::InvalidHalfExtents(entity) => write!(
                formatter,
                "3D physics body {} has negative AABB half extents",
                entity.0
            ),
            Self::ZeroMass(entity) => write!(
                formatter,
                "dynamic 3D physics body {} has zero mass",
                entity.0
            ),
            Self::RestitutionOutOfRange(entity, value) => write!(
                formatter,
                "3D physics body {} has restitution {value}, expected 0..={MATERIAL_SCALE}",
                entity.0
            ),
            Self::FrictionOutOfRange(entity, value) => write!(
                formatter,
                "3D physics body {} has friction {value}, expected 0..={MATERIAL_SCALE}",
                entity.0
            ),
            Self::CoordinateOutOfRange(entity) => write!(
                formatter,
                "3D physics body {} exceeds the exact f32 collision range",
                entity.0
            ),
            Self::NonPositiveTicks(ticks) => write!(
                formatter,
                "3D physics step requires positive ticks, got {ticks}"
            ),
            Self::CcdIterationLimit(entity) => write!(
                formatter,
                "3D physics body {} exceeded the bounded CCD/contact iteration limit",
                entity.0
            ),
        }
    }
}

impl std::error::Error for PhysicsError3d {}
