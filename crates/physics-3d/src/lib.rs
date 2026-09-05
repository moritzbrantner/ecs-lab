use std::{
    collections::{BTreeMap, BTreeSet},
    fmt,
};

use ecs_physics::{BodyKind, PhysicsMaterial, MATERIAL_SCALE};
use ecs_reference::ReferenceWorld;
use ecs_workload::{
    EntityId, EntitySnapshot, Operation, Position, Velocity, Workload, WorkloadError, WorldSnapshot,
};
use geometry_kernels::aabb_aabb;
use spatial_kernels::Aabb;

const MAX_EXACT_F32_INTEGER: i64 = 16_777_216;
const ROOM_DYNAMIC_COUNT: u32 = 3;
const FLOOR: EntityId = EntityId(ROOM_DYNAMIC_COUNT);
const CEILING: EntityId = EntityId(ROOM_DYNAMIC_COUNT + 1);
const LEFT_WALL: EntityId = EntityId(ROOM_DYNAMIC_COUNT + 2);
const RIGHT_WALL: EntityId = EntityId(ROOM_DYNAMIC_COUNT + 3);
const BACK_WALL: EntityId = EntityId(ROOM_DYNAMIC_COUNT + 4);
const FRONT_WALL: EntityId = EntityId(ROOM_DYNAMIC_COUNT + 5);

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
    pub contacts: usize,
    pub resolved_contacts: usize,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct PhysicsStep3d {
    operations: Vec<Operation>,
    stats: PhysicsStep3dStats,
    contacts: Vec<PhysicsContact3d>,
    supporting_entities: Vec<EntityId>,
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
}

impl fmt::Display for PhysicsError3d {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::DuplicateBody(entity) => write!(formatter, "duplicate 3D physics body {}", entity.0),
            Self::MissingEntity(entity) => write!(formatter, "3D physics body {} is not alive", entity.0),
            Self::MissingPosition(entity) => write!(formatter, "3D physics body {} has no position", entity.0),
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
            Self::ZeroMass(entity) => write!(formatter, "dynamic 3D physics body {} has zero mass", entity.0),
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
            Self::NonPositiveTicks(ticks) => {
                write!(formatter, "3D physics step requires positive ticks, got {ticks}")
            }
        }
    }
}

impl std::error::Error for PhysicsError3d {}

/// Produces one deterministic three-dimensional AABB physics step.
///
/// Positions and velocities remain integer ECS components. Candidate pairs are visited in ascending
/// entity-id order, the reusable geometry kernel owns the 3D AABB overlap decision, and this crate
/// owns deterministic positional correction plus mass, restitution, and two-axis tangent friction.
/// Returned mutations remain ordinary ECS workload operations.
///
/// # Errors
///
/// Returns [`PhysicsError3d`] for malformed body configuration, missing ECS components, coordinates
/// outside the exact f32 collision range, or a non-positive timestep.
pub fn step_3d(
    snapshot: &WorldSnapshot,
    bodies: &[PhysicsBody3d],
    config: PhysicsConfig3d,
    ticks: i32,
) -> Result<PhysicsStep3d, PhysicsError3d> {
    if ticks <= 0 {
        return Err(PhysicsError3d::NonPositiveTicks(ticks));
    }

    let snapshots = snapshot_by_id(snapshot);
    let mut ordered_bodies = bodies.to_vec();
    ordered_bodies.sort_unstable_by_key(|body| body.entity);
    reject_duplicate_bodies(&ordered_bodies)?;

    let mut states = ordered_bodies
        .into_iter()
        .map(|body| BodyState::from_body(body, &snapshots))
        .collect::<Result<Vec<_>, _>>()?;

    for state in &mut states {
        state.integrate(config.gravity, ticks);
        state.validate_geometry_range()?;
    }

    let mut stats = PhysicsStep3dStats {
        body_count: states.len(),
        ..PhysicsStep3dStats::default()
    };
    let mut contacts = Vec::new();
    let mut supporting_entities = BTreeSet::new();

    for left_index in 0..states.len() {
        for right_index in (left_index + 1)..states.len() {
            stats.candidate_pairs = stats.candidate_pairs.saturating_add(1);
            let (left_slice, right_slice) = states.split_at_mut(right_index);
            let left = &mut left_slice[left_index];
            let right = &mut right_slice[0];
            let outcome = resolve_pair(left, right)?;
            if let Some(contact) = outcome.contact {
                contacts.push(contact);
                stats.contacts = stats.contacts.saturating_add(1);
            }
            if outcome.resolved {
                stats.resolved_contacts = stats.resolved_contacts.saturating_add(1);
            }
            if let Some(entity) = outcome.supported_entity {
                supporting_entities.insert(entity);
            }
        }
    }

    Ok(PhysicsStep3d {
        operations: changed_operations(&states),
        stats,
        contacts,
        supporting_entities: supporting_entities.into_iter().collect(),
    })
}

fn snapshot_by_id(snapshot: &WorldSnapshot) -> BTreeMap<EntityId, EntitySnapshot> {
    snapshot
        .entities()
        .iter()
        .map(|entity| (entity.id, *entity))
        .collect()
}

fn reject_duplicate_bodies(bodies: &[PhysicsBody3d]) -> Result<(), PhysicsError3d> {
    for pair in bodies.windows(2) {
        if pair[0].entity == pair[1].entity {
            return Err(PhysicsError3d::DuplicateBody(pair[0].entity));
        }
    }
    Ok(())
}

#[derive(Clone, Copy, Debug)]
struct BodyState {
    entity: EntityId,
    kind: BodyKind,
    half_extents: [i64; 3],
    mass_units: u32,
    material: PhysicsMaterial,
    position: Position,
    velocity: Velocity,
    original_position: Position,
    original_velocity: Velocity,
}

impl BodyState {
    fn from_body(
        body: PhysicsBody3d,
        snapshots: &BTreeMap<EntityId, EntitySnapshot>,
    ) -> Result<Self, PhysicsError3d> {
        if body.half_extents.iter().any(|extent| *extent < 0) {
            return Err(PhysicsError3d::InvalidHalfExtents(body.entity));
        }
        if body.kind == BodyKind::Dynamic && body.mass_units == 0 {
            return Err(PhysicsError3d::ZeroMass(body.entity));
        }
        if body.material.restitution_milli > MATERIAL_SCALE {
            return Err(PhysicsError3d::RestitutionOutOfRange(
                body.entity,
                body.material.restitution_milli,
            ));
        }
        if body.material.friction_milli > MATERIAL_SCALE {
            return Err(PhysicsError3d::FrictionOutOfRange(
                body.entity,
                body.material.friction_milli,
            ));
        }

        let entity = snapshots
            .get(&body.entity)
            .ok_or(PhysicsError3d::MissingEntity(body.entity))?;
        let position = entity
            .position
            .ok_or(PhysicsError3d::MissingPosition(body.entity))?;
        let velocity = match body.kind {
            BodyKind::Fixed => Velocity::new3(0, 0, 0),
            BodyKind::Dynamic => entity
                .velocity
                .ok_or(PhysicsError3d::MissingVelocity(body.entity))?,
        };
        let state = Self {
            entity: body.entity,
            kind: body.kind,
            half_extents: body.half_extents.map(i64::from),
            mass_units: body.mass_units,
            material: body.material,
            position,
            velocity,
            original_position: position,
            original_velocity: velocity,
        };
        state.validate_geometry_range()?;
        Ok(state)
    }

    fn integrate(&mut self, gravity: Velocity, ticks: i32) {
        if self.kind == BodyKind::Fixed {
            return;
        }
        self.velocity.x = self
            .velocity
            .x
            .saturating_add(gravity.x.saturating_mul(ticks));
        self.velocity.y = self
            .velocity
            .y
            .saturating_add(gravity.y.saturating_mul(ticks));
        self.velocity.z = self
            .velocity
            .z
            .saturating_add(gravity.z.saturating_mul(ticks));
        let ticks = i64::from(ticks);
        self.position.x = self
            .position
            .x
            .saturating_add(i64::from(self.velocity.x).saturating_mul(ticks));
        self.position.y = self
            .position
            .y
            .saturating_add(i64::from(self.velocity.y).saturating_mul(ticks));
        self.position.z = self
            .position
            .z
            .saturating_add(i64::from(self.velocity.z).saturating_mul(ticks));
    }

    fn validate_geometry_range(&self) -> Result<(), PhysicsError3d> {
        if axis_fits_exact_f32(self.position.x, self.half_extents[0])
            && axis_fits_exact_f32(self.position.y, self.half_extents[1])
            && axis_fits_exact_f32(self.position.z, self.half_extents[2])
        {
            Ok(())
        } else {
            Err(PhysicsError3d::CoordinateOutOfRange(self.entity))
        }
    }

    fn geometry_aabb(&self) -> Result<Aabb, PhysicsError3d> {
        self.validate_geometry_range()?;
        Ok(Aabb::from_center_half_extents(
            [
                exact_i64_to_f32(self.position.x),
                exact_i64_to_f32(self.position.y),
                exact_i64_to_f32(self.position.z),
            ],
            [
                exact_i64_to_f32(self.half_extents[0]),
                exact_i64_to_f32(self.half_extents[1]),
                exact_i64_to_f32(self.half_extents[2]),
            ],
        ))
    }

    const fn axis_position(&self, axis: Axis) -> i64 {
        match axis {
            Axis::X => self.position.x,
            Axis::Y => self.position.y,
            Axis::Z => self.position.z,
        }
    }

    const fn axis_velocity(&self, axis: Axis) -> i32 {
        match axis {
            Axis::X => self.velocity.x,
            Axis::Y => self.velocity.y,
            Axis::Z => self.velocity.z,
        }
    }

    fn offset_axis(&mut self, axis: Axis, delta: i64) {
        match axis {
            Axis::X => self.position.x = self.position.x.saturating_add(delta),
            Axis::Y => self.position.y = self.position.y.saturating_add(delta),
            Axis::Z => self.position.z = self.position.z.saturating_add(delta),
        }
    }

    fn set_axis_velocity(&mut self, axis: Axis, value: i32) -> bool {
        let previous = self.axis_velocity(axis);
        match axis {
            Axis::X => self.velocity.x = value,
            Axis::Y => self.velocity.y = value,
            Axis::Z => self.velocity.z = value,
        }
        previous != value
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum Axis {
    X,
    Y,
    Z,
}

impl Axis {
    const fn index(self) -> usize {
        match self {
            Self::X => 0,
            Self::Y => 1,
            Self::Z => 2,
        }
    }

    const fn tangents(self) -> [Self; 2] {
        match self {
            Self::X => [Self::Y, Self::Z],
            Self::Y => [Self::X, Self::Z],
            Self::Z => [Self::X, Self::Y],
        }
    }
}

#[derive(Clone, Copy, Debug, Default)]
struct PairOutcome {
    contact: Option<PhysicsContact3d>,
    resolved: bool,
    supported_entity: Option<EntityId>,
}

fn resolve_pair(left: &mut BodyState, right: &mut BodyState) -> Result<PairOutcome, PhysicsError3d> {
    if !aabb_aabb(left.geometry_aabb()?, right.geometry_aabb()?).overlaps {
        return Ok(PairOutcome::default());
    }

    let overlap_x = integer_axis_overlap(left, right, Axis::X);
    let overlap_y = integer_axis_overlap(left, right, Axis::Y);
    let overlap_z = integer_axis_overlap(left, right, Axis::Z);
    let (axis, penetration) = if overlap_x <= overlap_y && overlap_x <= overlap_z {
        (Axis::X, overlap_x)
    } else if overlap_y <= overlap_z {
        (Axis::Y, overlap_y)
    } else {
        (Axis::Z, overlap_z)
    };
    let normal = if right.axis_position(axis) >= left.axis_position(axis) {
        1_i64
    } else {
        -1_i64
    };
    let relative_velocity =
        i64::from(right.axis_velocity(axis)).saturating_sub(i64::from(left.axis_velocity(axis)));
    let approaching = relative_velocity.saturating_mul(normal) < 0;

    let restitution_milli = combined_restitution(left, right);
    let friction_milli = combined_friction(left, right);
    let corrected = correct_penetration(left, right, axis, normal, penetration);
    let normal_impulsed = approaching
        && apply_normal_response(left, right, axis, restitution_milli);
    let [tangent_a, tangent_b] = axis.tangents();
    let friction_a = apply_contact_friction(left, right, tangent_a, friction_milli);
    let friction_b = apply_contact_friction(left, right, tangent_b, friction_milli);

    left.validate_geometry_range()?;
    right.validate_geometry_range()?;

    Ok(PairOutcome {
        contact: Some(PhysicsContact3d {
            left: left.entity,
            right: right.entity,
            normal: contact_normal(axis, normal),
            penetration,
        }),
        resolved: corrected || normal_impulsed || friction_a || friction_b,
        supported_entity: supported_entity(left, right, axis, normal),
    })
}

fn contact_normal(axis: Axis, normal: i64) -> ContactNormal3d {
    let component = if normal >= 0 { 1 } else { -1 };
    match axis {
        Axis::X => ContactNormal3d {
            x: component,
            y: 0,
            z: 0,
        },
        Axis::Y => ContactNormal3d {
            x: 0,
            y: component,
            z: 0,
        },
        Axis::Z => ContactNormal3d {
            x: 0,
            y: 0,
            z: component,
        },
    }
}

fn supported_entity(
    left: &BodyState,
    right: &BodyState,
    axis: Axis,
    normal: i64,
) -> Option<EntityId> {
    if axis != Axis::Y {
        return None;
    }
    if normal < 0 && left.kind == BodyKind::Dynamic {
        return Some(left.entity);
    }
    if normal > 0 && right.kind == BodyKind::Dynamic {
        return Some(right.entity);
    }
    None
}

fn integer_axis_overlap(left: &BodyState, right: &BodyState, axis: Axis) -> i64 {
    let index = axis.index();
    let left_center = left.axis_position(axis);
    let right_center = right.axis_position(axis);
    let left_half = left.half_extents[index];
    let right_half = right.half_extents[index];
    let left_max = left_center + left_half;
    let right_max = right_center + right_half;
    let left_min = left_center - left_half;
    let right_min = right_center - right_half;
    left_max.min(right_max) - left_min.max(right_min)
}

fn correct_penetration(
    left: &mut BodyState,
    right: &mut BodyState,
    axis: Axis,
    normal: i64,
    penetration: i64,
) -> bool {
    if penetration <= 0 {
        return false;
    }
    match (left.kind, right.kind) {
        (BodyKind::Fixed, BodyKind::Fixed) => false,
        (BodyKind::Dynamic, BodyKind::Fixed) => {
            left.offset_axis(axis, -normal.saturating_mul(penetration));
            true
        }
        (BodyKind::Fixed, BodyKind::Dynamic) => {
            right.offset_axis(axis, normal.saturating_mul(penetration));
            true
        }
        (BodyKind::Dynamic, BodyKind::Dynamic) => {
            let (left_share, right_share) =
                dynamic_penetration_shares(penetration, left.mass_units, right.mass_units);
            left.offset_axis(axis, -normal.saturating_mul(left_share));
            right.offset_axis(axis, normal.saturating_mul(right_share));
            true
        }
    }
}

fn dynamic_penetration_shares(penetration: i64, left_mass: u32, right_mass: u32) -> (i64, i64) {
    let total_mass = u64::from(left_mass) + u64::from(right_mass);
    let denominator = i128::from(total_mass);
    let mut left_share =
        i64_from_i128(i128::from(penetration).saturating_mul(i128::from(right_mass)) / denominator);
    let mut right_share =
        i64_from_i128(i128::from(penetration).saturating_mul(i128::from(left_mass)) / denominator);
    let remainder = penetration.saturating_sub(left_share.saturating_add(right_share));
    if remainder > 0 {
        if right_mass > left_mass {
            left_share = left_share.saturating_add(remainder);
        } else {
            right_share = right_share.saturating_add(remainder);
        }
    }
    (left_share, right_share)
}

fn combined_restitution(left: &BodyState, right: &BodyState) -> u16 {
    left.material
        .restitution_milli
        .max(right.material.restitution_milli)
}

fn combined_friction(left: &BodyState, right: &BodyState) -> u16 {
    left.material
        .friction_milli
        .max(right.material.friction_milli)
}

fn apply_normal_response(
    left: &mut BodyState,
    right: &mut BodyState,
    axis: Axis,
    restitution_milli: u16,
) -> bool {
    match (left.kind, right.kind) {
        (BodyKind::Fixed, BodyKind::Fixed) => false,
        (BodyKind::Dynamic, BodyKind::Fixed) => {
            let next = scaled_i32(-i64::from(left.axis_velocity(axis)), restitution_milli);
            left.set_axis_velocity(axis, next)
        }
        (BodyKind::Fixed, BodyKind::Dynamic) => {
            let next = scaled_i32(-i64::from(right.axis_velocity(axis)), restitution_milli);
            right.set_axis_velocity(axis, next)
        }
        (BodyKind::Dynamic, BodyKind::Dynamic) => {
            let scale = i128::from(MATERIAL_SCALE);
            let restitution = i128::from(restitution_milli);
            let left_mass = i128::from(left.mass_units);
            let right_mass = i128::from(right.mass_units);
            let left_velocity = i128::from(left.axis_velocity(axis));
            let right_velocity = i128::from(right.axis_velocity(axis));
            let denominator = (left_mass + right_mass).saturating_mul(scale);

            let left_numerator = (left_mass.saturating_mul(scale)
                - restitution.saturating_mul(right_mass))
            .saturating_mul(left_velocity)
            .saturating_add(
                (scale + restitution)
                    .saturating_mul(right_mass)
                    .saturating_mul(right_velocity),
            );
            let right_numerator = (scale + restitution)
                .saturating_mul(left_mass)
                .saturating_mul(left_velocity)
                .saturating_add(
                    (right_mass.saturating_mul(scale) - restitution.saturating_mul(left_mass))
                        .saturating_mul(right_velocity),
                );

            let left_changed =
                left.set_axis_velocity(axis, rational_i32(left_numerator, denominator));
            let right_changed =
                right.set_axis_velocity(axis, rational_i32(right_numerator, denominator));
            left_changed || right_changed
        }
    }
}

fn apply_contact_friction(
    left: &mut BodyState,
    right: &mut BodyState,
    tangent: Axis,
    friction_milli: u16,
) -> bool {
    if friction_milli == 0 {
        return false;
    }

    match (left.kind, right.kind) {
        (BodyKind::Fixed, BodyKind::Fixed) => false,
        (BodyKind::Dynamic, BodyKind::Fixed) => {
            let retained = MATERIAL_SCALE.saturating_sub(friction_milli);
            let next = scaled_i32(i64::from(left.axis_velocity(tangent)), retained);
            left.set_axis_velocity(tangent, next)
        }
        (BodyKind::Fixed, BodyKind::Dynamic) => {
            let retained = MATERIAL_SCALE.saturating_sub(friction_milli);
            let next = scaled_i32(i64::from(right.axis_velocity(tangent)), retained);
            right.set_axis_velocity(tangent, next)
        }
        (BodyKind::Dynamic, BodyKind::Dynamic) => {
            let scale = i128::from(MATERIAL_SCALE);
            let friction = i128::from(friction_milli);
            let retained = scale - friction;
            let left_mass = i128::from(left.mass_units);
            let right_mass = i128::from(right.mass_units);
            let total_mass = left_mass + right_mass;
            let left_velocity = i128::from(left.axis_velocity(tangent));
            let right_velocity = i128::from(right.axis_velocity(tangent));
            let momentum = left_mass
                .saturating_mul(left_velocity)
                .saturating_add(right_mass.saturating_mul(right_velocity));
            let denominator = scale.saturating_mul(total_mass);

            let left_numerator = left_velocity
                .saturating_mul(retained)
                .saturating_mul(total_mass)
                .saturating_add(friction.saturating_mul(momentum));
            let right_numerator = right_velocity
                .saturating_mul(retained)
                .saturating_mul(total_mass)
                .saturating_add(friction.saturating_mul(momentum));

            let left_changed =
                left.set_axis_velocity(tangent, rational_i32(left_numerator, denominator));
            let right_changed =
                right.set_axis_velocity(tangent, rational_i32(right_numerator, denominator));
            left_changed || right_changed
        }
    }
}

fn scaled_i32(value: i64, coefficient_milli: u16) -> i32 {
    let numerator = i128::from(value).saturating_mul(i128::from(coefficient_milli));
    rational_i32(numerator, i128::from(MATERIAL_SCALE))
}

fn rational_i32(numerator: i128, denominator: i128) -> i32 {
    debug_assert!(denominator > 0);
    let value = numerator / denominator;
    match i32::try_from(value) {
        Ok(value) => value,
        Err(_) if value.is_negative() => i32::MIN,
        Err(_) => i32::MAX,
    }
}

fn i64_from_i128(value: i128) -> i64 {
    match i64::try_from(value) {
        Ok(value) => value,
        Err(_) if value.is_negative() => i64::MIN,
        Err(_) => i64::MAX,
    }
}

fn changed_operations(states: &[BodyState]) -> Vec<Operation> {
    let mut operations = Vec::with_capacity(states.len().saturating_mul(2));
    for state in states {
        if state.kind == BodyKind::Fixed {
            continue;
        }
        if state.position != state.original_position {
            operations.push(Operation::SetPosition(state.entity, state.position));
        }
        if state.velocity != state.original_velocity {
            operations.push(Operation::SetVelocity(state.entity, state.velocity));
        }
    }
    operations
}

fn axis_fits_exact_f32(position: i64, half_extent: i64) -> bool {
    (0..=MAX_EXACT_F32_INTEGER).contains(&half_extent)
        && ((-MAX_EXACT_F32_INTEGER + half_extent)..=(MAX_EXACT_F32_INTEGER - half_extent))
            .contains(&position)
}

#[allow(clippy::cast_precision_loss)]
fn exact_i64_to_f32(value: i64) -> f32 {
    value as f32
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BroadPhaseBody3d {
    pub entity: EntityId,
    pub position: Position,
    pub half_extents: [i32; 3],
    pub kind: BodyKind,
    pub mass_units: u32,
    pub material: PhysicsMaterial,
    pub min: [f32; 3],
    pub max: [f32; 3],
}

#[derive(Clone, Debug, PartialEq)]
pub struct BroadPhaseFrame3d {
    bodies: Vec<BroadPhaseBody3d>,
    pair_words: Vec<u32>,
    overlap_count: u32,
}

impl BroadPhaseFrame3d {
    #[must_use]
    pub fn bodies(&self) -> &[BroadPhaseBody3d] {
        &self.bodies
    }

    #[must_use]
    pub fn pair_words(&self) -> &[u32] {
        &self.pair_words
    }

    #[must_use]
    pub const fn overlap_count(&self) -> u32 {
        self.overlap_count
    }

    fn from_snapshot(
        snapshot: &WorldSnapshot,
        bodies: &[PhysicsBody3d],
    ) -> Result<Self, ScenarioError3d> {
        let entities = snapshot
            .entities()
            .iter()
            .map(|entity| (entity.id, *entity))
            .collect::<BTreeMap<_, _>>();
        let mut frame_bodies = Vec::with_capacity(bodies.len());
        let mut geometry = Vec::with_capacity(bodies.len());

        for body in bodies {
            let entity = entities
                .get(&body.entity)
                .ok_or(ScenarioError3d::MissingEntity(body.entity))?;
            let position = entity
                .position
                .ok_or(ScenarioError3d::MissingPosition(body.entity))?;
            let aabb = body_aabb(*body, position)?;
            frame_bodies.push(BroadPhaseBody3d {
                entity: body.entity,
                position,
                half_extents: body.half_extents,
                kind: body.kind,
                mass_units: body.mass_units,
                material: body.material,
                min: aabb.min,
                max: aabb.max,
            });
            geometry.push(aabb);
        }

        let body_count = geometry.len();
        let possible_pairs = body_count.saturating_mul(body_count.saturating_sub(1)) / 2;
        let mut pair_words = vec![0_u32; possible_pairs.div_ceil(32)];
        let mut overlap_count = 0_u32;
        for left in 0..body_count {
            for right in (left + 1)..body_count {
                if !aabb_aabb(geometry[left], geometry[right]).overlaps {
                    continue;
                }
                let pair = pair_index(left, right, body_count);
                pair_words[pair / 32] |= 1_u32 << (pair % 32);
                overlap_count = overlap_count.saturating_add(1);
            }
        }
        Ok(Self {
            bodies: frame_bodies,
            pair_words,
            overlap_count,
        })
    }
}

fn pair_index(left: usize, right: usize, count: usize) -> usize {
    left * (2 * count - left - 1) / 2 + (right - left - 1)
}

fn body_aabb(body: PhysicsBody3d, position: Position) -> Result<Aabb, ScenarioError3d> {
    let half = body.half_extents.map(i64::from);
    if !axis_fits_exact_f32(position.x, half[0])
        || !axis_fits_exact_f32(position.y, half[1])
        || !axis_fits_exact_f32(position.z, half[2])
    {
        return Err(ScenarioError3d::CoordinateOutOfRange(body.entity));
    }
    Ok(Aabb::from_center_half_extents(
        [
            exact_i64_to_f32(position.x),
            exact_i64_to_f32(position.y),
            exact_i64_to_f32(position.z),
        ],
        [
            exact_i64_to_f32(half[0]),
            exact_i64_to_f32(half[1]),
            exact_i64_to_f32(half[2]),
        ],
    ))
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct BouncingRoom3dScenario {
    setup: Workload,
    bodies: Vec<PhysicsBody3d>,
}

impl Default for BouncingRoom3dScenario {
    fn default() -> Self {
        Self::new()
    }
}

impl BouncingRoom3dScenario {
    #[must_use]
    pub fn new() -> Self {
        let definitions = [
            (
                EntityId(0),
                Position::new3(-10, 12, -8),
                Velocity::new3(3, -1, 4),
                PhysicsMaterial::new(1_000, 0),
                1,
            ),
            (
                EntityId(1),
                Position::new3(4, 15, 2),
                Velocity::new3(-2, -2, -3),
                PhysicsMaterial::new(500, 500),
                2,
            ),
            (
                EntityId(2),
                Position::new3(10, 7, 8),
                Velocity::new3(-4, 0, -3),
                PhysicsMaterial::new(0, 1_000),
                3,
            ),
        ];
        let mut operations = Vec::new();
        let mut bodies = Vec::new();
        for (entity, position, velocity, material, mass_units) in definitions {
            operations.push(Operation::Spawn(entity));
            operations.push(Operation::SetPosition(entity, position));
            operations.push(Operation::SetVelocity(entity, velocity));
            bodies.push(
                PhysicsBody3d::dynamic(entity, [2, 2, 2])
                    .with_mass(mass_units)
                    .with_material(material),
            );
        }

        let fixed = [
            (FLOOR, Position::new3(0, -1, 0), [20, 1, 16]),
            (CEILING, Position::new3(0, 21, 0), [20, 1, 16]),
            (LEFT_WALL, Position::new3(-21, 10, 0), [1, 12, 16]),
            (RIGHT_WALL, Position::new3(21, 10, 0), [1, 12, 16]),
            (BACK_WALL, Position::new3(0, 10, -17), [20, 12, 1]),
            (FRONT_WALL, Position::new3(0, 10, 17), [20, 12, 1]),
        ];
        for (entity, position, half_extents) in fixed {
            operations.push(Operation::Spawn(entity));
            operations.push(Operation::SetPosition(entity, position));
            bodies.push(PhysicsBody3d::fixed(entity, half_extents));
        }

        Self {
            setup: Workload::new(operations),
            bodies,
        }
    }

    #[must_use]
    pub fn setup(&self) -> &Workload {
        &self.setup
    }

    #[must_use]
    pub fn bodies(&self) -> &[PhysicsBody3d] {
        &self.bodies
    }

    #[must_use]
    pub const fn physics_config(&self) -> PhysicsConfig3d {
        PhysicsConfig3d {
            gravity: Velocity::new3(0, -1, 0),
        }
    }

    /// Produces one canonical 3D room step.
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError3d`] when the supplied snapshot violates the 3D physics contract.
    pub fn step(&self, snapshot: &WorldSnapshot) -> Result<PhysicsStep3d, PhysicsError3d> {
        step_3d(snapshot, &self.bodies, self.physics_config(), 1)
    }

    /// Replays the room through the reference ECS for the requested number of steps.
    ///
    /// # Errors
    ///
    /// Returns [`ScenarioError3d`] when setup, physics, or ECS replay fails.
    pub fn reference_after(&self, frames: u32) -> Result<WorldSnapshot, ScenarioError3d> {
        let mut world = ReferenceWorld::new();
        world.replay(&self.setup).map_err(ScenarioError3d::Workload)?;
        for _ in 0..frames {
            let physics = self
                .step(&world.snapshot())
                .map_err(ScenarioError3d::Physics)?;
            for operation in physics.operations() {
                world.apply(*operation).map_err(ScenarioError3d::Workload)?;
            }
        }
        Ok(world.snapshot())
    }

    /// Builds exact 3D AABB evidence from the Rust-owned room state.
    ///
    /// # Errors
    ///
    /// Returns [`ScenarioError3d`] when the scenario cannot produce the requested frame.
    pub fn broad_phase_frame_after(
        &self,
        frames: u32,
    ) -> Result<BroadPhaseFrame3d, ScenarioError3d> {
        BroadPhaseFrame3d::from_snapshot(&self.reference_after(frames)?, &self.bodies)
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ScenarioError3d {
    Workload(WorkloadError),
    Physics(PhysicsError3d),
    MissingEntity(EntityId),
    MissingPosition(EntityId),
    CoordinateOutOfRange(EntityId),
}

impl fmt::Display for ScenarioError3d {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Workload(error) => write!(formatter, "3D scenario workload failed: {error}"),
            Self::Physics(error) => write!(formatter, "3D scenario physics failed: {error}"),
            Self::MissingEntity(entity) => write!(formatter, "3D scenario entity {} is missing", entity.0),
            Self::MissingPosition(entity) => write!(formatter, "3D scenario entity {} has no position", entity.0),
            Self::CoordinateOutOfRange(entity) => write!(
                formatter,
                "3D scenario entity {} exceeds the exact f32 collision range",
                entity.0
            ),
        }
    }
}

impl std::error::Error for ScenarioError3d {}

#[cfg(test)]
mod tests {
    use ecs_physics::PhysicsMaterial;
    use ecs_reference::ReferenceWorld;
    use ecs_sparse_set::SparseWorld;
    use ecs_workload::{EntityId, EntitySnapshot, Operation, Position, Velocity, WorldSnapshot};

    use super::{
        BouncingRoom3dScenario, ContactNormal3d, PhysicsBody3d, PhysicsConfig3d, step_3d,
    };

    const NO_GRAVITY: PhysicsConfig3d = PhysicsConfig3d {
        gravity: Velocity::new3(0, 0, 0),
    };

    #[test]
    fn z_axis_contact_is_resolved_as_real_physics() {
        let left = EntityId(1);
        let right = EntityId(2);
        let snapshot = WorldSnapshot::new(vec![
            EntitySnapshot {
                id: left,
                position: Some(Position::new3(0, 0, -3)),
                velocity: Some(Velocity::new3(0, 0, 2)),
            },
            EntitySnapshot {
                id: right,
                position: Some(Position::new3(0, 0, 3)),
                velocity: Some(Velocity::new3(0, 0, -2)),
            },
        ]);
        let material = PhysicsMaterial::new(1_000, 0);
        let physics = step_3d(
            &snapshot,
            &[
                PhysicsBody3d::dynamic(left, [1, 1, 1]).with_material(material),
                PhysicsBody3d::dynamic(right, [1, 1, 1]).with_material(material),
            ],
            NO_GRAVITY,
            1,
        )
        .expect("valid Z-axis collision should succeed");

        assert_eq!(physics.contacts().len(), 1);
        assert_eq!(
            physics.contacts()[0].normal,
            ContactNormal3d { x: 0, y: 0, z: 1 }
        );
        assert!(physics.operations().contains(&Operation::SetVelocity(
            left,
            Velocity::new3(0, 0, -2)
        )));
        assert!(physics.operations().contains(&Operation::SetVelocity(
            right,
            Velocity::new3(0, 0, 2)
        )));
    }

    #[test]
    fn room_moves_bodies_through_depth() {
        let scenario = BouncingRoom3dScenario::new();
        let initial = scenario.reference_after(0).expect("initial frame should work");
        let next = scenario.reference_after(1).expect("next frame should work");
        let initial_z = initial.entities()[0].position.expect("position should exist").z;
        let next_z = next.entities()[0].position.expect("position should exist").z;
        assert_ne!(initial_z, next_z);
    }

    #[test]
    fn room_keeps_reference_and_sparse_storage_in_lockstep() {
        let scenario = BouncingRoom3dScenario::new();
        let mut reference = ReferenceWorld::new();
        let mut sparse = SparseWorld::new();
        assert_eq!(reference.replay(scenario.setup()), Ok(()));
        assert_eq!(sparse.replay(scenario.setup()), Ok(()));

        for frame in 0..24 {
            assert_eq!(
                sparse.snapshot(),
                reference.snapshot(),
                "3D storage snapshots diverged before frame {frame}"
            );
            let physics = scenario
                .step(&reference.snapshot())
                .expect("3D room step should succeed");
            for operation in physics.operations() {
                assert_eq!(reference.apply(*operation), Ok(()));
                assert_eq!(sparse.apply(*operation), Ok(()));
            }
        }
    }

    #[test]
    fn broad_phase_frame_is_repeatable_and_three_dimensional() {
        let scenario = BouncingRoom3dScenario::new();
        let first = scenario
            .broad_phase_frame_after(6)
            .expect("3D broad phase should succeed");
        let second = scenario
            .broad_phase_frame_after(6)
            .expect("repeated 3D broad phase should succeed");
        assert_eq!(first, second);
        assert_eq!(first.bodies().len(), 9);
        assert!(first.bodies()[0].position.z != 0);
    }
}