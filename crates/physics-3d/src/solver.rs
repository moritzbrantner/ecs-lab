use std::{
    cmp::Ordering,
    collections::{BTreeMap, BTreeSet},
};

use ecs_physics::{BodyKind, MATERIAL_SCALE, PhysicsMaterial};
use ecs_workload::{EntityId, EntitySnapshot, Operation, Position, Velocity, WorldSnapshot};
use geometry_kernels::aabb_aabb;
use spatial_kernels::Aabb;

use crate::types::{
    ContactNormal3d, PhysicsBody3d, PhysicsConfig3d, PhysicsContact3d, PhysicsError3d,
    PhysicsStep3d, PhysicsStep3dStats,
};

const MAX_EXACT_F32_INTEGER: i64 = 16_777_216;
const SUBTICK_SCALE: i128 = 1_i128 << 32;
const MAX_CCD_EVENTS_PER_BODY: usize = 64;
const MAX_STATIC_CORRECTION_PASSES: usize = 8;

/// Produces one deterministic three-dimensional AABB physics step.
///
/// Dynamic bodies are integrated through a four-dimensional space-time interval `(x, y, z, t)`.
/// Swept AABB tests find the earliest time of impact against fixed bodies before discrete end-of-step
/// overlap handling. Time-of-impact ordering is compared with exact integer fractions while temporal
/// advancement uses deterministic Q32.32 subticks. Dynamic-vs-dynamic response remains the existing
/// deterministic discrete path for this first CCD horizon.
///
/// Positions and velocities remain integer ECS components. Body configuration and pair ordering are
/// canonicalized by entity id, the reusable geometry kernel remains authoritative for discrete 3D
/// overlap/touching, and this crate owns deterministic response and positional correction. Returned
/// mutations remain ordinary ECS workload operations.
///
/// # Errors
///
/// Returns [`PhysicsError3d`] for malformed body configuration, missing ECS components, coordinates
/// outside the exact f32 collision range, a non-positive timestep, or a body that exceeds the bounded
/// static CCD/contact iteration budget.
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

    let mut body_states = ordered_bodies
        .into_iter()
        .map(|body| BodyState::from_body(body, &snapshots))
        .collect::<Result<Vec<_>, _>>()?;

    let body_count = body_states.len();
    let mut step_stats = PhysicsStep3dStats {
        body_count,
        candidate_pairs: body_count.saturating_mul(body_count.saturating_sub(1)) / 2,
        ..PhysicsStep3dStats::default()
    };
    let mut contacts = Vec::new();
    let mut supporting_entities = BTreeSet::new();

    for dynamic_index in 0..body_states.len() {
        if body_states[dynamic_index].kind == BodyKind::Fixed {
            continue;
        }
        integrate_dynamic_with_static_ccd(
            &mut body_states,
            dynamic_index,
            config.gravity,
            ticks,
            &mut step_stats,
            &mut contacts,
            &mut supporting_entities,
        )?;
    }

    // Dynamic-vs-dynamic CCD is intentionally the next horizon. Preserve the existing deterministic
    // discrete response for those pairs while static geometry already uses continuous space-time TOI.
    for left_index in 0..body_states.len() {
        for right_index in (left_index + 1)..body_states.len() {
            if body_states[left_index].kind != BodyKind::Dynamic
                || body_states[right_index].kind != BodyKind::Dynamic
            {
                continue;
            }
            let (left_slice, right_slice) = body_states.split_at_mut(right_index);
            let left = &mut left_slice[left_index];
            let right = &mut right_slice[0];
            let outcome = resolve_pair(left, right)?;
            record_pair_outcome(
                outcome,
                &mut step_stats,
                &mut contacts,
                &mut supporting_entities,
            );
        }
    }

    stabilize_static_penetrations(
        &mut body_states,
        &mut step_stats,
        &mut contacts,
        &mut supporting_entities,
    )?;

    Ok(PhysicsStep3d {
        operations: changed_operations(&body_states),
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

    fn apply_gravity(&mut self, gravity: Velocity, ticks: i32) {
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
    const ALL: [Self; 3] = [Self::X, Self::Y, Self::Z];

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

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct TimeFraction {
    numerator: i128,
    denominator: i128,
}

impl TimeFraction {
    fn new(numerator: i128, denominator: i128) -> Self {
        debug_assert_ne!(denominator, 0);
        if denominator < 0 {
            Self {
                numerator: -numerator,
                denominator: -denominator,
            }
        } else {
            Self {
                numerator,
                denominator,
            }
        }
    }

    const fn is_negative(self) -> bool {
        self.numerator < 0
    }

    fn checked_cmp(self, other: Self, entity: EntityId) -> Result<Ordering, PhysicsError3d> {
        let left = self
            .numerator
            .checked_mul(other.denominator)
            .ok_or(PhysicsError3d::CoordinateOutOfRange(entity))?;
        let right = other
            .numerator
            .checked_mul(self.denominator)
            .ok_or(PhysicsError3d::CoordinateOutOfRange(entity))?;
        Ok(left.cmp(&right))
    }

    fn within_subticks(
        self,
        remaining_subticks: i128,
        entity: EntityId,
    ) -> Result<bool, PhysicsError3d> {
        let remaining = Self::new(remaining_subticks, SUBTICK_SCALE);
        Ok(self.checked_cmp(remaining, entity)? != Ordering::Greater)
    }

    fn to_subticks_floor(self, entity: EntityId) -> Result<i128, PhysicsError3d> {
        let scaled = self
            .numerator
            .checked_mul(SUBTICK_SCALE)
            .ok_or(PhysicsError3d::CoordinateOutOfRange(entity))?;
        Ok((scaled / self.denominator).max(0))
    }
}

#[derive(Clone, Copy, Debug)]
struct ScaledPosition {
    values: [i128; 3],
}

impl ScaledPosition {
    fn from_position(position: Position) -> Self {
        Self {
            values: [
                i128::from(position.x) * SUBTICK_SCALE,
                i128::from(position.y) * SUBTICK_SCALE,
                i128::from(position.z) * SUBTICK_SCALE,
            ],
        }
    }

    const fn axis(self, axis: Axis) -> i128 {
        self.values[axis.index()]
    }

    fn set_axis(&mut self, axis: Axis, value: i128) {
        self.values[axis.index()] = value;
    }

    fn advance(
        &mut self,
        velocity: Velocity,
        subticks: i128,
        entity: EntityId,
    ) -> Result<(), PhysicsError3d> {
        for axis in Axis::ALL {
            let delta = i128::from(velocity_component(velocity, axis))
                .checked_mul(subticks)
                .ok_or(PhysicsError3d::CoordinateOutOfRange(entity))?;
            self.values[axis.index()] = self.values[axis.index()]
                .checked_add(delta)
                .ok_or(PhysicsError3d::CoordinateOutOfRange(entity))?;
        }
        Ok(())
    }

    fn into_position(self, entity: EntityId) -> Result<Position, PhysicsError3d> {
        let x = scaled_axis_to_i64(self.values[0], entity)?;
        let y = scaled_axis_to_i64(self.values[1], entity)?;
        let z = scaled_axis_to_i64(self.values[2], entity)?;
        Ok(Position::new3(x, y, z))
    }
}

fn scaled_axis_to_i64(value: i128, entity: EntityId) -> Result<i64, PhysicsError3d> {
    i64::try_from(value / SUBTICK_SCALE).map_err(|_| PhysicsError3d::CoordinateOutOfRange(entity))
}

const fn velocity_component(velocity: Velocity, axis: Axis) -> i32 {
    match axis {
        Axis::X => velocity.x,
        Axis::Y => velocity.y,
        Axis::Z => velocity.z,
    }
}

#[derive(Clone, Copy, Debug)]
struct StaticSweepHit {
    fixed_index: usize,
    axis: Axis,
    normal_to_fixed: i64,
    contact_coordinate: i128,
    time: TimeFraction,
}

#[allow(clippy::too_many_arguments)]
fn integrate_dynamic_with_static_ccd(
    body_states: &mut [BodyState],
    dynamic_index: usize,
    gravity: Velocity,
    ticks: i32,
    step_stats: &mut PhysicsStep3dStats,
    contacts: &mut Vec<PhysicsContact3d>,
    supporting_entities: &mut BTreeSet<EntityId>,
) -> Result<(), PhysicsError3d> {
    let dynamic_entity = body_states[dynamic_index].entity;
    body_states[dynamic_index].apply_gravity(gravity, ticks);
    let mut position = ScaledPosition::from_position(body_states[dynamic_index].position);
    let mut remaining_subticks = i128::from(ticks)
        .checked_mul(SUBTICK_SCALE)
        .ok_or(PhysicsError3d::CoordinateOutOfRange(dynamic_entity))?;
    let mut events = 0_usize;

    while remaining_subticks > 0 {
        let hit = find_earliest_static_hit(
            body_states,
            dynamic_index,
            position,
            remaining_subticks,
            step_stats,
        )?;
        let Some(hit) = hit else {
            position.advance(
                body_states[dynamic_index].velocity,
                remaining_subticks,
                dynamic_entity,
            )?;
            break;
        };

        if events >= MAX_CCD_EVENTS_PER_BODY {
            return Err(PhysicsError3d::CcdIterationLimit(dynamic_entity));
        }

        let hit_subticks = hit.time.to_subticks_floor(dynamic_entity)?;
        position.advance(
            body_states[dynamic_index].velocity,
            hit_subticks,
            dynamic_entity,
        )?;
        // The slab TOI makes the normal-axis contact coordinate exact even when tangent coordinates
        // lie between integer ECS units. Snap only that axis; tangent axes retain Q32.32 precision.
        position.set_axis(hit.axis, hit.contact_coordinate);
        remaining_subticks = remaining_subticks.saturating_sub(hit_subticks);

        let fixed = body_states[hit.fixed_index];
        let resolved = apply_static_response(&mut body_states[dynamic_index], fixed, hit.axis);
        let contact = static_sweep_contact(body_states[dynamic_index], fixed, hit);
        contacts.push(contact);
        step_stats.contacts = step_stats.contacts.saturating_add(1);
        step_stats.ccd_contacts = step_stats.ccd_contacts.saturating_add(1);
        if resolved {
            step_stats.resolved_contacts = step_stats.resolved_contacts.saturating_add(1);
        }
        if hit.axis == Axis::Y && hit.normal_to_fixed < 0 {
            supporting_entities.insert(dynamic_entity);
        }

        events = events.saturating_add(1);
    }

    body_states[dynamic_index].position = position.into_position(dynamic_entity)?;
    body_states[dynamic_index].validate_geometry_range()
}

fn find_earliest_static_hit(
    body_states: &[BodyState],
    dynamic_index: usize,
    position: ScaledPosition,
    remaining_subticks: i128,
    step_stats: &mut PhysicsStep3dStats,
) -> Result<Option<StaticSweepHit>, PhysicsError3d> {
    let dynamic = body_states[dynamic_index];
    let mut earliest: Option<StaticSweepHit> = None;

    for (fixed_index, fixed) in body_states.iter().copied().enumerate() {
        if fixed.kind != BodyKind::Fixed {
            continue;
        }
        step_stats.ccd_candidate_pairs = step_stats.ccd_candidate_pairs.saturating_add(1);
        let Some(mut hit) = sweep_static_aabb(dynamic, position, fixed, remaining_subticks)? else {
            continue;
        };
        hit.fixed_index = fixed_index;

        let replace = match earliest {
            None => true,
            Some(current) => hit.time.checked_cmp(current.time, dynamic.entity)? == Ordering::Less,
        };
        if replace {
            earliest = Some(hit);
        }
    }

    Ok(earliest)
}

fn sweep_static_aabb(
    dynamic: BodyState,
    position: ScaledPosition,
    fixed: BodyState,
    remaining_subticks: i128,
) -> Result<Option<StaticSweepHit>, PhysicsError3d> {
    if !temporal_broad_phase_overlaps(dynamic, position, fixed, remaining_subticks)? {
        return Ok(None);
    }

    let mut entry: Option<(TimeFraction, Axis, i64, i128)> = None;
    let mut exit: Option<TimeFraction> = None;

    for axis in Axis::ALL {
        let point = position.axis(axis);
        let (minimum, maximum) = expanded_static_bounds(dynamic, fixed, axis);
        let velocity = dynamic.axis_velocity(axis);
        if velocity == 0 {
            if point < minimum || point > maximum {
                return Ok(None);
            }
            continue;
        }

        let velocity_scaled = i128::from(velocity)
            .checked_mul(SUBTICK_SCALE)
            .ok_or(PhysicsError3d::CoordinateOutOfRange(dynamic.entity))?;
        let to_minimum = TimeFraction::new(minimum - point, velocity_scaled);
        let to_maximum = TimeFraction::new(maximum - point, velocity_scaled);
        let (axis_entry, axis_exit, normal_to_fixed, contact_coordinate) = if velocity > 0 {
            (to_minimum, to_maximum, 1_i64, minimum)
        } else {
            (to_maximum, to_minimum, -1_i64, maximum)
        };

        let replace_entry = match entry {
            None => true,
            Some((current, ..)) => {
                axis_entry.checked_cmp(current, dynamic.entity)? == Ordering::Greater
            }
        };
        if replace_entry {
            entry = Some((axis_entry, axis, normal_to_fixed, contact_coordinate));
        }

        let replace_exit = match exit {
            None => true,
            Some(current) => axis_exit.checked_cmp(current, dynamic.entity)? == Ordering::Less,
        };
        if replace_exit {
            exit = Some(axis_exit);
        }

        if let (Some((entry_time, ..)), Some(exit_time)) = (entry, exit)
            && entry_time.checked_cmp(exit_time, dynamic.entity)? == Ordering::Greater
        {
            return Ok(None);
        }
    }

    let Some((entry_time, axis, normal_to_fixed, contact_coordinate)) = entry else {
        return Ok(None);
    };
    let Some(exit_time) = exit else {
        return Ok(None);
    };
    if entry_time.is_negative() || exit_time.is_negative() {
        return Ok(None);
    }
    if !entry_time.within_subticks(remaining_subticks, dynamic.entity)? {
        return Ok(None);
    }

    Ok(Some(StaticSweepHit {
        fixed_index: 0,
        axis,
        normal_to_fixed,
        contact_coordinate,
        time: entry_time,
    }))
}

fn temporal_broad_phase_overlaps(
    dynamic: BodyState,
    position: ScaledPosition,
    fixed: BodyState,
    remaining_subticks: i128,
) -> Result<bool, PhysicsError3d> {
    for axis in Axis::ALL {
        let start = position.axis(axis);
        let delta = i128::from(dynamic.axis_velocity(axis))
            .checked_mul(remaining_subticks)
            .ok_or(PhysicsError3d::CoordinateOutOfRange(dynamic.entity))?;
        let end = start
            .checked_add(delta)
            .ok_or(PhysicsError3d::CoordinateOutOfRange(dynamic.entity))?;
        let swept_min = start.min(end);
        let swept_max = start.max(end);
        let (fixed_min, fixed_max) = expanded_static_bounds(dynamic, fixed, axis);
        if swept_max < fixed_min || swept_min > fixed_max {
            return Ok(false);
        }
    }
    Ok(true)
}

fn expanded_static_bounds(dynamic: BodyState, fixed: BodyState, axis: Axis) -> (i128, i128) {
    let index = axis.index();
    let center = i128::from(fixed.axis_position(axis)) * SUBTICK_SCALE;
    let expanded_half = (i128::from(dynamic.half_extents[index])
        + i128::from(fixed.half_extents[index]))
        * SUBTICK_SCALE;
    (center - expanded_half, center + expanded_half)
}

fn apply_static_response(dynamic: &mut BodyState, fixed: BodyState, axis: Axis) -> bool {
    let restitution_milli = dynamic
        .material
        .restitution_milli
        .max(fixed.material.restitution_milli);
    let friction_milli = dynamic
        .material
        .friction_milli
        .max(fixed.material.friction_milli);

    let normal_velocity = dynamic.axis_velocity(axis);
    let reflected = scaled_i32(-i64::from(normal_velocity), restitution_milli);
    let mut changed = dynamic.set_axis_velocity(axis, reflected);

    if friction_milli > 0 {
        let retained = MATERIAL_SCALE.saturating_sub(friction_milli);
        for tangent in axis.tangents() {
            let next = scaled_i32(i64::from(dynamic.axis_velocity(tangent)), retained);
            changed |= dynamic.set_axis_velocity(tangent, next);
        }
    }

    changed
}

fn static_sweep_contact(
    dynamic: BodyState,
    fixed: BodyState,
    hit: StaticSweepHit,
) -> PhysicsContact3d {
    if dynamic.entity < fixed.entity {
        PhysicsContact3d {
            left: dynamic.entity,
            right: fixed.entity,
            normal: contact_normal(hit.axis, hit.normal_to_fixed),
            penetration: 0,
        }
    } else {
        PhysicsContact3d {
            left: fixed.entity,
            right: dynamic.entity,
            normal: contact_normal(hit.axis, -hit.normal_to_fixed),
            penetration: 0,
        }
    }
}

fn stabilize_static_penetrations(
    body_states: &mut [BodyState],
    step_stats: &mut PhysicsStep3dStats,
    contacts: &mut Vec<PhysicsContact3d>,
    supporting_entities: &mut BTreeSet<EntityId>,
) -> Result<(), PhysicsError3d> {
    for _ in 0..MAX_STATIC_CORRECTION_PASSES {
        let mut corrected_any = false;
        for left_index in 0..body_states.len() {
            for right_index in (left_index + 1)..body_states.len() {
                let kinds = (body_states[left_index].kind, body_states[right_index].kind);
                if !matches!(
                    kinds,
                    (BodyKind::Dynamic, BodyKind::Fixed) | (BodyKind::Fixed, BodyKind::Dynamic)
                ) {
                    continue;
                }
                if !has_positive_penetration(body_states[left_index], body_states[right_index])? {
                    continue;
                }

                let (left_slice, right_slice) = body_states.split_at_mut(right_index);
                let left = &mut left_slice[left_index];
                let right = &mut right_slice[0];
                let outcome = resolve_pair(left, right)?;
                corrected_any |= outcome.resolved;
                record_pair_outcome(outcome, step_stats, contacts, supporting_entities);
            }
        }
        if !corrected_any {
            return Ok(());
        }
    }

    for left_index in 0..body_states.len() {
        for right_index in (left_index + 1)..body_states.len() {
            let (dynamic, fixed) =
                match (body_states[left_index].kind, body_states[right_index].kind) {
                    (BodyKind::Dynamic, BodyKind::Fixed) => {
                        (body_states[left_index], body_states[right_index])
                    }
                    (BodyKind::Fixed, BodyKind::Dynamic) => {
                        (body_states[right_index], body_states[left_index])
                    }
                    _ => continue,
                };
            if has_positive_penetration(dynamic, fixed)? {
                return Err(PhysicsError3d::CcdIterationLimit(dynamic.entity));
            }
        }
    }
    Ok(())
}

fn has_positive_penetration(left: BodyState, right: BodyState) -> Result<bool, PhysicsError3d> {
    if !aabb_aabb(left.geometry_aabb()?, right.geometry_aabb()?).overlaps {
        return Ok(false);
    }
    Ok(integer_axis_overlap(&left, &right, Axis::X) > 0
        && integer_axis_overlap(&left, &right, Axis::Y) > 0
        && integer_axis_overlap(&left, &right, Axis::Z) > 0)
}

#[derive(Clone, Copy, Debug, Default)]
struct PairOutcome {
    contact: Option<PhysicsContact3d>,
    resolved: bool,
    supported_entity: Option<EntityId>,
}

fn record_pair_outcome(
    outcome: PairOutcome,
    step_stats: &mut PhysicsStep3dStats,
    contacts: &mut Vec<PhysicsContact3d>,
    supporting_entities: &mut BTreeSet<EntityId>,
) {
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

fn resolve_pair(
    left: &mut BodyState,
    right: &mut BodyState,
) -> Result<PairOutcome, PhysicsError3d> {
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
    let normal_impulsed =
        approaching && apply_normal_response(left, right, axis, restitution_milli);
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

fn changed_operations(body_states: &[BodyState]) -> Vec<Operation> {
    let mut operations = Vec::with_capacity(body_states.len().saturating_mul(2));
    for state in body_states {
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

pub(crate) fn axis_fits_exact_f32(position: i64, half_extent: i64) -> bool {
    (0..=MAX_EXACT_F32_INTEGER).contains(&half_extent)
        && ((-MAX_EXACT_F32_INTEGER + half_extent)..=(MAX_EXACT_F32_INTEGER - half_extent))
            .contains(&position)
}

#[allow(clippy::cast_precision_loss)]
pub(crate) fn exact_i64_to_f32(value: i64) -> f32 {
    value as f32
}

#[cfg(test)]
mod tests {
    use ecs_physics::PhysicsMaterial;
    use ecs_workload::{EntityId, EntitySnapshot, Operation, Position, Velocity, WorldSnapshot};

    use super::step_3d;
    use crate::types::{ContactNormal3d, PhysicsBody3d, PhysicsConfig3d};

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
        assert!(
            physics
                .operations()
                .contains(&Operation::SetVelocity(left, Velocity::new3(0, 0, -2)))
        );
        assert!(
            physics
                .operations()
                .contains(&Operation::SetVelocity(right, Velocity::new3(0, 0, 2)))
        );
    }

    #[test]
    fn fast_dynamic_body_cannot_tunnel_through_fixed_wall() {
        let dynamic = EntityId(1);
        let wall = EntityId(2);
        let snapshot = WorldSnapshot::new(vec![
            EntitySnapshot {
                id: dynamic,
                position: Some(Position::new3(-10, 0, 0)),
                velocity: Some(Velocity::new3(30, 0, 0)),
            },
            EntitySnapshot {
                id: wall,
                position: Some(Position::new3(0, 0, 0)),
                velocity: None,
            },
        ]);
        let physics = step_3d(
            &snapshot,
            &[
                PhysicsBody3d::dynamic(dynamic, [1, 1, 1]),
                PhysicsBody3d::fixed(wall, [1, 20, 20]),
            ],
            NO_GRAVITY,
            1,
        )
        .expect("swept wall collision should succeed");

        assert!(
            physics
                .operations()
                .contains(&Operation::SetPosition(dynamic, Position::new3(-2, 0, 0)))
        );
        assert!(
            physics
                .operations()
                .contains(&Operation::SetVelocity(dynamic, Velocity::new3(0, 0, 0)))
        );
        assert_eq!(physics.stats().ccd_contacts, 1);
        assert_eq!(
            physics.contacts()[0].normal,
            ContactNormal3d { x: 1, y: 0, z: 0 }
        );
    }

    #[test]
    fn bouncy_static_toi_consumes_the_remaining_fraction_of_the_step() {
        let dynamic = EntityId(1);
        let wall = EntityId(2);
        let snapshot = WorldSnapshot::new(vec![
            EntitySnapshot {
                id: dynamic,
                position: Some(Position::new3(-10, 0, 0)),
                velocity: Some(Velocity::new3(30, 0, 0)),
            },
            EntitySnapshot {
                id: wall,
                position: Some(Position::new3(0, 0, 0)),
                velocity: None,
            },
        ]);
        let bouncy = PhysicsMaterial::new(1_000, 0);
        let physics = step_3d(
            &snapshot,
            &[
                PhysicsBody3d::dynamic(dynamic, [1, 1, 1]).with_material(bouncy),
                PhysicsBody3d::fixed(wall, [1, 20, 20]),
            ],
            NO_GRAVITY,
            1,
        )
        .expect("bouncy swept wall collision should succeed");

        assert!(
            physics
                .operations()
                .contains(&Operation::SetPosition(dynamic, Position::new3(-24, 0, 0)))
        );
        assert!(
            physics
                .operations()
                .contains(&Operation::SetVelocity(dynamic, Velocity::new3(-30, 0, 0)))
        );
    }

    #[test]
    fn swept_aabb_requires_tangent_overlap_at_time_of_impact() {
        let dynamic = EntityId(1);
        let wall = EntityId(2);
        let snapshot = WorldSnapshot::new(vec![
            EntitySnapshot {
                id: dynamic,
                position: Some(Position::new3(-10, 50, 0)),
                velocity: Some(Velocity::new3(30, 0, 0)),
            },
            EntitySnapshot {
                id: wall,
                position: Some(Position::new3(0, 0, 0)),
                velocity: None,
            },
        ]);
        let physics = step_3d(
            &snapshot,
            &[
                PhysicsBody3d::dynamic(dynamic, [1, 1, 1]),
                PhysicsBody3d::fixed(wall, [1, 5, 5]),
            ],
            NO_GRAVITY,
            1,
        )
        .expect("tangent miss should remain collision-free");

        assert!(
            physics
                .operations()
                .contains(&Operation::SetPosition(dynamic, Position::new3(20, 50, 0)))
        );
        assert_eq!(physics.stats().ccd_contacts, 0);
    }

    #[test]
    fn bounded_toi_loop_handles_multiple_wall_impacts_in_one_step() {
        let dynamic = EntityId(1);
        let left_wall = EntityId(2);
        let right_wall = EntityId(3);
        let snapshot = WorldSnapshot::new(vec![
            EntitySnapshot {
                id: dynamic,
                position: Some(Position::new3(0, 0, 0)),
                velocity: Some(Velocity::new3(100, 0, 0)),
            },
            EntitySnapshot {
                id: left_wall,
                position: Some(Position::new3(-10, 0, 0)),
                velocity: None,
            },
            EntitySnapshot {
                id: right_wall,
                position: Some(Position::new3(10, 0, 0)),
                velocity: None,
            },
        ]);
        let bouncy = PhysicsMaterial::new(1_000, 0);
        let physics = step_3d(
            &snapshot,
            &[
                PhysicsBody3d::dynamic(dynamic, [1, 1, 1]).with_material(bouncy),
                PhysicsBody3d::fixed(left_wall, [1, 20, 20]),
                PhysicsBody3d::fixed(right_wall, [1, 20, 20]),
            ],
            NO_GRAVITY,
            1,
        )
        .expect("multiple swept wall impacts should succeed");

        assert!(
            physics
                .operations()
                .contains(&Operation::SetPosition(dynamic, Position::new3(4, 0, 0)))
        );
        assert!(
            !physics.operations().iter().any(
                |operation| matches!(operation, Operation::SetVelocity(entity, _) if *entity == dynamic)
            ),
            "six full-restitution impacts return to the input velocity, so no velocity write is emitted"
        );
        assert_eq!(physics.stats().ccd_contacts, 6);
    }
}
