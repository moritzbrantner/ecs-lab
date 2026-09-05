use std::{
    collections::{BTreeMap, BTreeSet},
    fmt,
};

use ecs_workload::{EntityId, EntitySnapshot, Operation, Position, Velocity, WorldSnapshot};
use geometry_kernels::aabb_aabb;
use spatial_kernels::Aabb;

const MAX_EXACT_F32_INTEGER: i64 = 16_777_216;
pub const MATERIAL_SCALE: u16 = 1_000;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BodyKind {
    Fixed,
    Dynamic,
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct PhysicsMaterial {
    pub restitution_milli: u16,
    pub friction_milli: u16,
}

impl PhysicsMaterial {
    #[must_use]
    pub const fn new(restitution_milli: u16, friction_milli: u16) -> Self {
        Self {
            restitution_milli,
            friction_milli,
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct PhysicsBody {
    pub entity: EntityId,
    pub kind: BodyKind,
    pub half_extents: [i32; 2],
    pub mass_units: u32,
    pub material: PhysicsMaterial,
}

impl PhysicsBody {
    #[must_use]
    pub const fn dynamic(entity: EntityId, half_extents: [i32; 2]) -> Self {
        Self {
            entity,
            kind: BodyKind::Dynamic,
            half_extents,
            mass_units: 1,
            material: PhysicsMaterial::new(0, 0),
        }
    }

    #[must_use]
    pub const fn fixed(entity: EntityId, half_extents: [i32; 2]) -> Self {
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
pub struct PhysicsConfig {
    pub gravity: Velocity,
}

impl Default for PhysicsConfig {
    fn default() -> Self {
        Self {
            gravity: Velocity::new(0, -1),
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ContactNormal {
    pub x: i8,
    pub y: i8,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct PhysicsContact {
    pub left: EntityId,
    pub right: EntityId,
    pub normal: ContactNormal,
    pub penetration: i64,
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct PhysicsStepStats {
    pub body_count: usize,
    pub candidate_pairs: usize,
    pub contacts: usize,
    pub resolved_contacts: usize,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct PhysicsStep {
    operations: Vec<Operation>,
    stats: PhysicsStepStats,
    contacts: Vec<PhysicsContact>,
    supporting_entities: Vec<EntityId>,
}

impl PhysicsStep {
    #[must_use]
    pub fn operations(&self) -> &[Operation] {
        &self.operations
    }

    #[must_use]
    pub const fn stats(&self) -> PhysicsStepStats {
        self.stats
    }

    #[must_use]
    pub fn contacts(&self) -> &[PhysicsContact] {
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
pub enum PhysicsError {
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

impl fmt::Display for PhysicsError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::DuplicateBody(entity) => write!(formatter, "duplicate physics body {}", entity.0),
            Self::MissingEntity(entity) => {
                write!(formatter, "physics body {} is not alive", entity.0)
            }
            Self::MissingPosition(entity) => {
                write!(formatter, "physics body {} has no position", entity.0)
            }
            Self::MissingVelocity(entity) => {
                write!(
                    formatter,
                    "dynamic physics body {} has no velocity",
                    entity.0
                )
            }
            Self::InvalidHalfExtents(entity) => write!(
                formatter,
                "physics body {} has negative AABB half extents",
                entity.0
            ),
            Self::ZeroMass(entity) => {
                write!(formatter, "dynamic physics body {} has zero mass", entity.0)
            }
            Self::RestitutionOutOfRange(entity, value) => write!(
                formatter,
                "physics body {} has restitution {value}, expected 0..={MATERIAL_SCALE}",
                entity.0
            ),
            Self::FrictionOutOfRange(entity, value) => write!(
                formatter,
                "physics body {} has friction {value}, expected 0..={MATERIAL_SCALE}",
                entity.0
            ),
            Self::CoordinateOutOfRange(entity) => write!(
                formatter,
                "physics body {} exceeds the exact f32 collision range",
                entity.0
            ),
            Self::NonPositiveTicks(ticks) => {
                write!(
                    formatter,
                    "physics step requires positive ticks, got {ticks}"
                )
            }
        }
    }
}

impl std::error::Error for PhysicsError {}

/// Produces one deterministic physics step from an observable ECS snapshot.
///
/// Dynamic bodies use semi-implicit Euler integration. Every body pair is then visited in
/// ascending entity-id order. `geometry-kernels` owns the AABB overlap decision; this crate owns
/// deterministic positional correction plus integer mass, restitution, and contact-friction
/// response. Returned operations are ordinary ECS workload operations, so storage candidates can
/// consume the same result without implementing a physics-specific trait.
///
/// Material coefficients use thousandths: `0` is none and [`MATERIAL_SCALE`] is the full value.
/// Restitution combines by taking the higher coefficient, while friction takes the higher
/// coefficient. The defaults preserve the previous equal-mass, fully inelastic, frictionless
/// response.
///
/// # Errors
///
/// Returns [`PhysicsError`] for malformed body configuration, missing required ECS components,
/// coordinates that cannot be represented exactly by the reusable f32 collision kernel, or a
/// non-positive timestep.
pub fn step(
    snapshot: &WorldSnapshot,
    bodies: &[PhysicsBody],
    config: PhysicsConfig,
    ticks: i32,
) -> Result<PhysicsStep, PhysicsError> {
    if ticks <= 0 {
        return Err(PhysicsError::NonPositiveTicks(ticks));
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

    let mut step_stats = PhysicsStepStats {
        body_count: states.len(),
        ..PhysicsStepStats::default()
    };
    let mut contacts = Vec::new();
    let mut supporting_entities = BTreeSet::new();

    for left_index in 0..states.len() {
        for right_index in (left_index + 1)..states.len() {
            step_stats.candidate_pairs = step_stats.candidate_pairs.saturating_add(1);
            let (left_slice, right_slice) = states.split_at_mut(right_index);
            let left = &mut left_slice[left_index];
            let right = &mut right_slice[0];
            let outcome = resolve_pair(left, right)?;
            if let Some(contact) = outcome.contact {
                contacts.push(contact);
                step_stats.contacts = step_stats.contacts.saturating_add(1);
            }
            if outcome.resolved {
                step_stats.resolved_contacts = step_stats.resolved_contacts.saturating_add(1);
            }
            if let Some(entity) = outcome.supported_entity {
                supporting_entities.insert(entity);
            }
        }
    }

    Ok(PhysicsStep {
        operations: changed_operations(&states),
        stats: step_stats,
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

fn reject_duplicate_bodies(bodies: &[PhysicsBody]) -> Result<(), PhysicsError> {
    for pair in bodies.windows(2) {
        if pair[0].entity == pair[1].entity {
            return Err(PhysicsError::DuplicateBody(pair[0].entity));
        }
    }
    Ok(())
}

#[derive(Clone, Copy, Debug)]
struct BodyState {
    entity: EntityId,
    kind: BodyKind,
    half_extents: [i64; 2],
    mass_units: u32,
    material: PhysicsMaterial,
    position: Position,
    velocity: Velocity,
    original_position: Position,
    original_velocity: Velocity,
}

impl BodyState {
    fn from_body(
        body: PhysicsBody,
        snapshots: &BTreeMap<EntityId, EntitySnapshot>,
    ) -> Result<Self, PhysicsError> {
        if body.half_extents.iter().any(|extent| *extent < 0) {
            return Err(PhysicsError::InvalidHalfExtents(body.entity));
        }
        if body.kind == BodyKind::Dynamic && body.mass_units == 0 {
            return Err(PhysicsError::ZeroMass(body.entity));
        }
        if body.material.restitution_milli > MATERIAL_SCALE {
            return Err(PhysicsError::RestitutionOutOfRange(
                body.entity,
                body.material.restitution_milli,
            ));
        }
        if body.material.friction_milli > MATERIAL_SCALE {
            return Err(PhysicsError::FrictionOutOfRange(
                body.entity,
                body.material.friction_milli,
            ));
        }

        let entity = snapshots
            .get(&body.entity)
            .ok_or(PhysicsError::MissingEntity(body.entity))?;
        let position = entity
            .position
            .ok_or(PhysicsError::MissingPosition(body.entity))?;
        let velocity = match body.kind {
            BodyKind::Fixed => Velocity::new(0, 0),
            BodyKind::Dynamic => entity
                .velocity
                .ok_or(PhysicsError::MissingVelocity(body.entity))?,
        };
        let state = Self {
            entity: body.entity,
            kind: body.kind,
            half_extents: [
                i64::from(body.half_extents[0]),
                i64::from(body.half_extents[1]),
            ],
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
        let ticks = i64::from(ticks);
        self.position.x = self
            .position
            .x
            .saturating_add(i64::from(self.velocity.x).saturating_mul(ticks));
        self.position.y = self
            .position
            .y
            .saturating_add(i64::from(self.velocity.y).saturating_mul(ticks));
    }

    fn validate_geometry_range(&self) -> Result<(), PhysicsError> {
        if axis_fits_exact_f32(self.position.x, self.half_extents[0])
            && axis_fits_exact_f32(self.position.y, self.half_extents[1])
        {
            Ok(())
        } else {
            Err(PhysicsError::CoordinateOutOfRange(self.entity))
        }
    }

    fn geometry_aabb(&self) -> Result<Aabb, PhysicsError> {
        self.validate_geometry_range()?;
        Ok(Aabb::from_center_half_extents(
            [
                exact_i64_to_f32(self.position.x),
                exact_i64_to_f32(self.position.y),
                0.0,
            ],
            [
                exact_i64_to_f32(self.half_extents[0]),
                exact_i64_to_f32(self.half_extents[1]),
                0.5,
            ],
        ))
    }

    const fn axis_position(&self, axis: Axis) -> i64 {
        match axis {
            Axis::X => self.position.x,
            Axis::Y => self.position.y,
        }
    }

    const fn axis_velocity(&self, axis: Axis) -> i32 {
        match axis {
            Axis::X => self.velocity.x,
            Axis::Y => self.velocity.y,
        }
    }

    fn offset_axis(&mut self, axis: Axis, delta: i64) {
        match axis {
            Axis::X => self.position.x = self.position.x.saturating_add(delta),
            Axis::Y => self.position.y = self.position.y.saturating_add(delta),
        }
    }

    fn set_axis_velocity(&mut self, axis: Axis, value: i32) -> bool {
        let previous = self.axis_velocity(axis);
        match axis {
            Axis::X => self.velocity.x = value,
            Axis::Y => self.velocity.y = value,
        }
        previous != value
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum Axis {
    X,
    Y,
}

impl Axis {
    const fn tangent(self) -> Self {
        match self {
            Self::X => Self::Y,
            Self::Y => Self::X,
        }
    }
}

#[derive(Clone, Copy, Debug, Default)]
struct PairOutcome {
    contact: Option<PhysicsContact>,
    resolved: bool,
    supported_entity: Option<EntityId>,
}

fn resolve_pair(left: &mut BodyState, right: &mut BodyState) -> Result<PairOutcome, PhysicsError> {
    let relation = aabb_aabb(left.geometry_aabb()?, right.geometry_aabb()?);
    if !relation.overlaps {
        return Ok(PairOutcome::default());
    }

    let overlap_x = integer_axis_overlap(left, right, Axis::X);
    let overlap_y = integer_axis_overlap(left, right, Axis::Y);
    let (axis, penetration) = if overlap_x <= overlap_y {
        (Axis::X, overlap_x)
    } else {
        (Axis::Y, overlap_y)
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
    let normal_impulsed = if approaching {
        apply_normal_response(left, right, axis, restitution_milli)
    } else {
        false
    };
    let friction_applied = apply_contact_friction(left, right, axis.tangent(), friction_milli);

    left.validate_geometry_range()?;
    right.validate_geometry_range()?;

    Ok(PairOutcome {
        contact: Some(PhysicsContact {
            left: left.entity,
            right: right.entity,
            normal: contact_normal(axis, normal),
            penetration,
        }),
        resolved: corrected || normal_impulsed || friction_applied,
        supported_entity: supported_entity(left, right, axis, normal),
    })
}

fn contact_normal(axis: Axis, normal: i64) -> ContactNormal {
    let component = if normal >= 0 { 1 } else { -1 };
    match axis {
        Axis::X => ContactNormal { x: component, y: 0 },
        Axis::Y => ContactNormal { x: 0, y: component },
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
    let left_center = left.axis_position(axis);
    let right_center = right.axis_position(axis);
    let left_half = match axis {
        Axis::X => left.half_extents[0],
        Axis::Y => left.half_extents[1],
    };
    let right_half = match axis {
        Axis::X => right.half_extents[0],
        Axis::Y => right.half_extents[1],
    };
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

#[cfg(test)]
mod tests {
    use ecs_reference::ReferenceWorld;
    use ecs_workload::{
        EntityId, EntitySnapshot, Operation, Position, Velocity, Workload, WorldSnapshot,
    };

    use super::{
        ContactNormal, PhysicsBody, PhysicsConfig, PhysicsError, PhysicsMaterial, PhysicsStepStats,
        step,
    };

    const NO_GRAVITY: PhysicsConfig = PhysicsConfig {
        gravity: Velocity::new(0, 0),
    };

    #[test]
    fn integrates_gravity_before_position() {
        let entity = EntityId(1);
        let snapshot = WorldSnapshot::new(vec![EntitySnapshot {
            id: entity,
            position: Some(Position::new(0, 10)),
            velocity: Some(Velocity::new(2, 0)),
        }]);

        let physics = step(
            &snapshot,
            &[PhysicsBody::dynamic(entity, [1, 1])],
            PhysicsConfig::default(),
            1,
        )
        .expect("single-body physics should succeed");

        assert_eq!(
            physics.operations(),
            [
                Operation::SetPosition(entity, Position::new(2, 9)),
                Operation::SetVelocity(entity, Velocity::new(2, -1)),
            ]
        );
        assert_eq!(
            physics.stats(),
            PhysicsStepStats {
                body_count: 1,
                candidate_pairs: 0,
                contacts: 0,
                resolved_contacts: 0,
            }
        );
        assert!(physics.contacts().is_empty());
        assert!(physics.supporting_entities().is_empty());
    }

    #[test]
    fn default_material_preserves_inelastic_wall_response() {
        let dynamic = EntityId(1);
        let wall = EntityId(2);
        let snapshot = WorldSnapshot::new(vec![
            EntitySnapshot {
                id: dynamic,
                position: Some(Position::new(0, 0)),
                velocity: Some(Velocity::new(2, 0)),
            },
            EntitySnapshot {
                id: wall,
                position: Some(Position::new(3, 0)),
                velocity: None,
            },
        ]);

        let physics = step(
            &snapshot,
            &[
                PhysicsBody::dynamic(dynamic, [1, 1]),
                PhysicsBody::fixed(wall, [1, 1]),
            ],
            NO_GRAVITY,
            1,
        )
        .expect("wall collision should succeed");

        assert_eq!(
            physics.operations(),
            [
                Operation::SetPosition(dynamic, Position::new(1, 0)),
                Operation::SetVelocity(dynamic, Velocity::new(0, 0)),
            ]
        );
        assert_eq!(physics.stats().candidate_pairs, 1);
        assert_eq!(physics.stats().contacts, 1);
        assert_eq!(physics.stats().resolved_contacts, 1);
    }

    #[test]
    fn restitution_bounces_dynamic_body_from_ordinary_fixed_floor() {
        let dynamic = EntityId(1);
        let floor = EntityId(2);
        let bouncy = PhysicsMaterial::new(1_000, 0);
        let snapshot = WorldSnapshot::new(vec![
            EntitySnapshot {
                id: dynamic,
                position: Some(Position::new(0, 3)),
                velocity: Some(Velocity::new(0, -2)),
            },
            EntitySnapshot {
                id: floor,
                position: Some(Position::new(0, 0)),
                velocity: None,
            },
        ]);

        let physics = step(
            &snapshot,
            &[
                PhysicsBody::dynamic(dynamic, [1, 1]).with_material(bouncy),
                PhysicsBody::fixed(floor, [8, 1]),
            ],
            NO_GRAVITY,
            1,
        )
        .expect("bouncy floor collision should succeed");

        assert_eq!(
            physics.operations(),
            [
                Operation::SetPosition(dynamic, Position::new(0, 2)),
                Operation::SetVelocity(dynamic, Velocity::new(0, 2)),
            ]
        );
        assert!(physics.is_supported(dynamic));
        assert_eq!(physics.contacts()[0].normal, ContactNormal { x: 0, y: -1 });
        assert_eq!(physics.contacts()[0].penetration, 1);
    }

    #[test]
    fn friction_damps_tangent_velocity_and_marks_support() {
        let dynamic = EntityId(1);
        let floor = EntityId(2);
        let snapshot = WorldSnapshot::new(vec![
            EntitySnapshot {
                id: dynamic,
                position: Some(Position::new(0, 3)),
                velocity: Some(Velocity::new(10, -2)),
            },
            EntitySnapshot {
                id: floor,
                position: Some(Position::new(0, 0)),
                velocity: None,
            },
        ]);

        let physics = step(
            &snapshot,
            &[
                PhysicsBody::dynamic(dynamic, [1, 1]).with_material(PhysicsMaterial::new(0, 500)),
                PhysicsBody::fixed(floor, [20, 1]),
            ],
            NO_GRAVITY,
            1,
        )
        .expect("friction contact should succeed");

        assert_eq!(
            physics.operations(),
            [
                Operation::SetPosition(dynamic, Position::new(10, 2)),
                Operation::SetVelocity(dynamic, Velocity::new(5, 0)),
            ]
        );
        assert_eq!(physics.supporting_entities(), [dynamic]);
    }

    #[test]
    fn mass_changes_dynamic_collision_response() {
        let light = EntityId(1);
        let heavy = EntityId(2);
        let perfectly_bouncy = PhysicsMaterial::new(1_000, 0);
        let snapshot = WorldSnapshot::new(vec![
            EntitySnapshot {
                id: light,
                position: Some(Position::new(-6, 0)),
                velocity: Some(Velocity::new(4, 0)),
            },
            EntitySnapshot {
                id: heavy,
                position: Some(Position::new(0, 0)),
                velocity: Some(Velocity::new(0, 0)),
            },
        ]);

        let physics = step(
            &snapshot,
            &[
                PhysicsBody::dynamic(light, [1, 1])
                    .with_mass(1)
                    .with_material(perfectly_bouncy),
                PhysicsBody::dynamic(heavy, [1, 1])
                    .with_mass(3)
                    .with_material(perfectly_bouncy),
            ],
            NO_GRAVITY,
            1,
        )
        .expect("mass-weighted collision should succeed");

        assert_eq!(
            physics.operations(),
            [
                Operation::SetPosition(light, Position::new(-2, 0)),
                Operation::SetVelocity(light, Velocity::new(-2, 0)),
                Operation::SetVelocity(heavy, Velocity::new(2, 0)),
            ]
        );
    }

    #[test]
    fn mass_weights_penetration_correction_toward_lighter_body() {
        let light = EntityId(1);
        let heavy = EntityId(2);
        let snapshot = WorldSnapshot::new(vec![
            EntitySnapshot {
                id: light,
                position: Some(Position::new(0, 0)),
                velocity: Some(Velocity::new(0, 0)),
            },
            EntitySnapshot {
                id: heavy,
                position: Some(Position::new(1, 0)),
                velocity: Some(Velocity::new(0, 0)),
            },
        ]);

        let physics = step(
            &snapshot,
            &[
                PhysicsBody::dynamic(light, [1, 1]).with_mass(1),
                PhysicsBody::dynamic(heavy, [1, 1]).with_mass(3),
            ],
            NO_GRAVITY,
            1,
        )
        .expect("mass-weighted penetration correction should succeed");

        assert_eq!(
            physics.operations(),
            [Operation::SetPosition(light, Position::new(-1, 0))]
        );
    }

    #[test]
    fn touching_uses_geometry_kernel_contact_semantics() {
        let dynamic = EntityId(1);
        let floor = EntityId(2);
        let snapshot = WorldSnapshot::new(vec![
            EntitySnapshot {
                id: dynamic,
                position: Some(Position::new(0, 2)),
                velocity: Some(Velocity::new(0, 0)),
            },
            EntitySnapshot {
                id: floor,
                position: Some(Position::new(0, 0)),
                velocity: None,
            },
        ]);

        let physics = step(
            &snapshot,
            &[
                PhysicsBody::dynamic(dynamic, [1, 1]),
                PhysicsBody::fixed(floor, [8, 1]),
            ],
            NO_GRAVITY,
            1,
        )
        .expect("touching contact should succeed");

        assert!(physics.operations().is_empty());
        assert_eq!(physics.stats().contacts, 1);
        assert_eq!(physics.stats().resolved_contacts, 0);
        assert_eq!(physics.contacts()[0].penetration, 0);
        assert!(physics.is_supported(dynamic));
    }

    #[test]
    fn body_input_order_does_not_change_equal_mass_response() {
        let left = EntityId(1);
        let right = EntityId(2);
        let snapshot = WorldSnapshot::new(vec![
            EntitySnapshot {
                id: left,
                position: Some(Position::new(-2, 0)),
                velocity: Some(Velocity::new(2, 0)),
            },
            EntitySnapshot {
                id: right,
                position: Some(Position::new(2, 0)),
                velocity: Some(Velocity::new(-2, 0)),
            },
        ]);
        let forward = [
            PhysicsBody::dynamic(left, [1, 1]),
            PhysicsBody::dynamic(right, [1, 1]),
        ];
        let reverse = [forward[1], forward[0]];

        let forward_result = step(&snapshot, &forward, NO_GRAVITY, 1);
        let reverse_result = step(&snapshot, &reverse, NO_GRAVITY, 1);

        assert_eq!(forward_result, reverse_result);
        let physics = forward_result.expect("equal-mass collision should succeed");
        assert_eq!(
            physics.operations(),
            [
                Operation::SetPosition(left, Position::new(-1, 0)),
                Operation::SetVelocity(left, Velocity::new(0, 0)),
                Operation::SetPosition(right, Position::new(1, 0)),
                Operation::SetVelocity(right, Velocity::new(0, 0)),
            ]
        );
    }

    #[test]
    fn rejects_duplicate_body_configuration() {
        let entity = EntityId(7);
        let snapshot = WorldSnapshot::new(vec![EntitySnapshot {
            id: entity,
            position: Some(Position::new(0, 0)),
            velocity: Some(Velocity::new(0, 0)),
        }]);

        assert_eq!(
            step(
                &snapshot,
                &[
                    PhysicsBody::dynamic(entity, [1, 1]),
                    PhysicsBody::dynamic(entity, [1, 1]),
                ],
                NO_GRAVITY,
                1,
            ),
            Err(PhysicsError::DuplicateBody(entity))
        );
    }

    #[test]
    fn rejects_invalid_mass_and_material_coefficients() {
        let entity = EntityId(7);
        let snapshot = WorldSnapshot::new(vec![EntitySnapshot {
            id: entity,
            position: Some(Position::new(0, 0)),
            velocity: Some(Velocity::new(0, 0)),
        }]);

        assert_eq!(
            step(
                &snapshot,
                &[PhysicsBody::dynamic(entity, [1, 1]).with_mass(0)],
                NO_GRAVITY,
                1,
            ),
            Err(PhysicsError::ZeroMass(entity))
        );
        assert_eq!(
            step(
                &snapshot,
                &[PhysicsBody::dynamic(entity, [1, 1])
                    .with_material(PhysicsMaterial::new(1_001, 0))],
                NO_GRAVITY,
                1,
            ),
            Err(PhysicsError::RestitutionOutOfRange(entity, 1_001))
        );
        assert_eq!(
            step(
                &snapshot,
                &[PhysicsBody::dynamic(entity, [1, 1])
                    .with_material(PhysicsMaterial::new(0, 1_001))],
                NO_GRAVITY,
                1,
            ),
            Err(PhysicsError::FrictionOutOfRange(entity, 1_001))
        );
    }

    #[test]
    fn generated_operations_replay_through_reference_ecs() {
        let dynamic = EntityId(1);
        let wall = EntityId(2);
        let workload = Workload::new(vec![
            Operation::Spawn(dynamic),
            Operation::SetPosition(dynamic, Position::new(0, 0)),
            Operation::SetVelocity(dynamic, Velocity::new(2, 0)),
            Operation::Spawn(wall),
            Operation::SetPosition(wall, Position::new(3, 0)),
        ]);
        let mut world = ReferenceWorld::new();
        assert_eq!(world.replay(&workload), Ok(()));

        let physics = step(
            &world.snapshot(),
            &[
                PhysicsBody::dynamic(dynamic, [1, 1]),
                PhysicsBody::fixed(wall, [1, 1]),
            ],
            NO_GRAVITY,
            1,
        )
        .expect("generated operations should be valid");
        for operation in physics.operations() {
            assert_eq!(world.apply(*operation), Ok(()));
        }

        assert_eq!(
            world.snapshot(),
            WorldSnapshot::new(vec![
                EntitySnapshot {
                    id: dynamic,
                    position: Some(Position::new(1, 0)),
                    velocity: Some(Velocity::new(0, 0)),
                },
                EntitySnapshot {
                    id: wall,
                    position: Some(Position::new(3, 0)),
                    velocity: None,
                },
            ])
        );
    }
}
