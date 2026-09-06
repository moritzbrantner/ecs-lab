use std::{
    cmp::Ordering,
    collections::{BTreeMap, BTreeSet},
};

use ecs_physics::{BodyKind, MATERIAL_SCALE, PhysicsMaterial};
use ecs_workload::{EntityId, EntitySnapshot, Operation, Position, Velocity, WorldSnapshot};

use crate::{
    solver::axis_fits_exact_f32,
    types::{
        ContactNormal3d, PhysicsBody3d, PhysicsConfig3d, PhysicsContact3d, PhysicsError3d,
        PhysicsStep3d, PhysicsStep3dStats,
    },
};

const SUBTICK_SCALE: i128 = 1_i128 << 32;
const CCD_EVENTS_PER_BODY: usize = 64;
const MAX_STABILIZATION_PASSES: usize = 8;

/// Produces one deterministic three-dimensional AABB physics step on one global continuous timeline.
///
/// Gravity is applied to every dynamic body first. The solver then searches every pair containing at
/// least one dynamic body for the earliest swept-AABB time of impact over the remaining `(x, y, z, t)`
/// interval. Dynamic-vs-fixed and dynamic-vs-dynamic candidates therefore compete on the same exact
/// integer-fraction timeline. Equal times are ordered by entity pair and then X -> Y -> Z axis order.
/// Q32.32 positions remain private intra-step state and are quantized back to ordinary integer ECS
/// positions before mutations are returned.
///
/// # Errors
///
/// Returns [`PhysicsError3d`] for malformed body configuration, missing ECS components, coordinates
/// outside the exact collision range, a non-positive timestep, arithmetic overflow, or exhaustion of
/// the bounded CCD/contact iteration budget.
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

    for state in &mut body_states {
        state.apply_gravity(config.gravity, ticks);
    }

    let body_count = body_states.len();
    let mut step_stats = PhysicsStep3dStats {
        body_count,
        candidate_pairs: body_count.saturating_mul(body_count.saturating_sub(1)) / 2,
        ..PhysicsStep3dStats::default()
    };
    let mut contacts = Vec::new();
    let mut supporting_entities = BTreeSet::new();

    stabilize_penetrations(
        &mut body_states,
        &mut step_stats,
        &mut contacts,
        &mut supporting_entities,
    )?;

    let mut remaining_subticks = i128::from(ticks)
        .checked_mul(SUBTICK_SCALE)
        .ok_or_else(|| arithmetic_error(&body_states))?;
    let max_events = body_count
        .saturating_mul(CCD_EVENTS_PER_BODY)
        .max(CCD_EVENTS_PER_BODY);
    let mut event_count = 0_usize;

    while remaining_subticks > 0 {
        let hit = find_earliest_hit(&body_states, remaining_subticks, &mut step_stats)?;
        let Some(hit) = hit else {
            advance_all(&mut body_states, remaining_subticks)?;
            break;
        };

        if event_count >= max_events {
            return Err(PhysicsError3d::CcdIterationLimit(
                hit.dynamic_entity(&body_states),
            ));
        }

        let hit_subticks = hit
            .time
            .to_subticks_floor(body_states[hit.left_index].entity)?;
        advance_all(&mut body_states, hit_subticks)?;
        snap_pair_to_contact(&mut body_states, hit)?;
        remaining_subticks = remaining_subticks.saturating_sub(hit_subticks);

        resolve_sweep_hit(
            &mut body_states,
            hit,
            &mut step_stats,
            &mut contacts,
            &mut supporting_entities,
        );
        event_count = event_count.saturating_add(1);
    }

    for state in &mut body_states {
        state.quantize_to_integer_grid()?;
    }
    stabilize_penetrations(
        &mut body_states,
        &mut step_stats,
        &mut contacts,
        &mut supporting_entities,
    )?;
    for state in &body_states {
        state.validate_geometry_range()?;
    }

    Ok(PhysicsStep3d {
        operations: changed_operations(&body_states)?,
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

    fn offset_axis(&mut self, axis: Axis, delta: i128) -> Result<(), ()> {
        self.values[axis.index()] = self.values[axis.index()].checked_add(delta).ok_or(())?;
        Ok(())
    }

    fn advance(&mut self, velocity: Velocity, subticks: i128) -> Result<(), ()> {
        for axis in Axis::ALL {
            let delta = i128::from(velocity_component(velocity, axis))
                .checked_mul(subticks)
                .ok_or(())?;
            self.offset_axis(axis, delta)?;
        }
        Ok(())
    }

    fn into_position(self, entity: EntityId) -> Result<Position, PhysicsError3d> {
        Ok(Position::new3(
            scaled_axis_to_i64(self.values[0], entity)?,
            scaled_axis_to_i64(self.values[1], entity)?,
            scaled_axis_to_i64(self.values[2], entity)?,
        ))
    }
}

#[derive(Clone, Copy, Debug)]
struct BodyState {
    entity: EntityId,
    kind: BodyKind,
    half_extents: [i64; 3],
    mass_units: u32,
    material: PhysicsMaterial,
    position: ScaledPosition,
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
        let original_position = entity
            .position
            .ok_or(PhysicsError3d::MissingPosition(body.entity))?;
        let original_velocity = match body.kind {
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
            position: ScaledPosition::from_position(original_position),
            velocity: original_velocity,
            original_position,
            original_velocity,
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

    fn axis_velocity(self, axis: Axis) -> i32 {
        velocity_component(self.velocity, axis)
    }

    fn offset_axis(&mut self, axis: Axis, delta: i128) -> Result<(), PhysicsError3d> {
        self.position
            .offset_axis(axis, delta)
            .map_err(|()| PhysicsError3d::CoordinateOutOfRange(self.entity))
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

    fn current_position(self) -> Result<Position, PhysicsError3d> {
        self.position.into_position(self.entity)
    }

    fn quantize_to_integer_grid(&mut self) -> Result<(), PhysicsError3d> {
        self.position = ScaledPosition::from_position(self.current_position()?);
        Ok(())
    }

    fn validate_geometry_range(self) -> Result<(), PhysicsError3d> {
        let position = self.current_position()?;
        if axis_fits_exact_f32(position.x, self.half_extents[0])
            && axis_fits_exact_f32(position.y, self.half_extents[1])
            && axis_fits_exact_f32(position.z, self.half_extents[2])
        {
            Ok(())
        } else {
            Err(PhysicsError3d::CoordinateOutOfRange(self.entity))
        }
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
struct SweepHit {
    left_index: usize,
    right_index: usize,
    axis: Axis,
    normal: i64,
    target_relative: i128,
    time: TimeFraction,
}

impl SweepHit {
    fn dynamic_entity(self, states: &[BodyState]) -> EntityId {
        let left = states[self.left_index];
        let right = states[self.right_index];
        if left.kind == BodyKind::Dynamic {
            left.entity
        } else {
            right.entity
        }
    }
}

fn find_earliest_hit(
    states: &[BodyState],
    remaining_subticks: i128,
    step_stats: &mut PhysicsStep3dStats,
) -> Result<Option<SweepHit>, PhysicsError3d> {
    let mut earliest: Option<SweepHit> = None;
    for left_index in 0..states.len() {
        for right_index in (left_index + 1)..states.len() {
            if states[left_index].kind == BodyKind::Fixed
                && states[right_index].kind == BodyKind::Fixed
            {
                continue;
            }
            step_stats.ccd_candidate_pairs = step_stats.ccd_candidate_pairs.saturating_add(1);
            let Some(hit) = sweep_pair(states, left_index, right_index, remaining_subticks)? else {
                continue;
            };
            let replace = match earliest {
                None => true,
                Some(current) => hit_precedes(hit, current, states)?,
            };
            if replace {
                earliest = Some(hit);
            }
        }
    }
    Ok(earliest)
}

fn hit_precedes(
    candidate: SweepHit,
    current: SweepHit,
    states: &[BodyState],
) -> Result<bool, PhysicsError3d> {
    let entity = states[candidate.left_index].entity;
    match candidate.time.checked_cmp(current.time, entity)? {
        Ordering::Less => Ok(true),
        Ordering::Greater => Ok(false),
        Ordering::Equal => {
            let candidate_pair = (
                states[candidate.left_index].entity,
                states[candidate.right_index].entity,
                candidate.axis.index(),
            );
            let current_pair = (
                states[current.left_index].entity,
                states[current.right_index].entity,
                current.axis.index(),
            );
            Ok(candidate_pair < current_pair)
        }
    }
}

fn sweep_pair(
    states: &[BodyState],
    left_index: usize,
    right_index: usize,
    remaining_subticks: i128,
) -> Result<Option<SweepHit>, PhysicsError3d> {
    let left = states[left_index];
    let right = states[right_index];
    if !temporal_broad_phase_overlaps(left, right, remaining_subticks)? {
        return Ok(None);
    }

    let mut entry: Option<(TimeFraction, Axis, i64, i128)> = None;
    let mut exit: Option<TimeFraction> = None;

    for axis in Axis::ALL {
        let relative_position = left.position.axis(axis) - right.position.axis(axis);
        let (minimum, maximum) = expanded_relative_bounds(left, right, axis);
        let relative_velocity = i64::from(left.axis_velocity(axis))
            .saturating_sub(i64::from(right.axis_velocity(axis)));
        if relative_velocity == 0 {
            if relative_position < minimum || relative_position > maximum {
                return Ok(None);
            }
            continue;
        }

        let velocity_scaled = i128::from(relative_velocity)
            .checked_mul(SUBTICK_SCALE)
            .ok_or(PhysicsError3d::CoordinateOutOfRange(left.entity))?;
        let to_minimum = TimeFraction::new(minimum - relative_position, velocity_scaled);
        let to_maximum = TimeFraction::new(maximum - relative_position, velocity_scaled);
        let (axis_entry, axis_exit, normal, target_relative) = if relative_velocity > 0 {
            (to_minimum, to_maximum, 1_i64, minimum)
        } else {
            (to_maximum, to_minimum, -1_i64, maximum)
        };

        let replace_entry = match entry {
            None => true,
            Some((current, ..)) => {
                axis_entry.checked_cmp(current, left.entity)? == Ordering::Greater
            }
        };
        if replace_entry {
            entry = Some((axis_entry, axis, normal, target_relative));
        }

        let replace_exit = match exit {
            None => true,
            Some(current) => axis_exit.checked_cmp(current, left.entity)? == Ordering::Less,
        };
        if replace_exit {
            exit = Some(axis_exit);
        }

        if let (Some((entry_time, ..)), Some(exit_time)) = (entry, exit)
            && entry_time.checked_cmp(exit_time, left.entity)? == Ordering::Greater
        {
            return Ok(None);
        }
    }

    let Some((entry_time, axis, normal, target_relative)) = entry else {
        return Ok(None);
    };
    let Some(exit_time) = exit else {
        return Ok(None);
    };
    if entry_time.is_negative() || exit_time.is_negative() {
        return Ok(None);
    }
    if !entry_time.within_subticks(remaining_subticks, left.entity)? {
        return Ok(None);
    }

    Ok(Some(SweepHit {
        left_index,
        right_index,
        axis,
        normal,
        target_relative,
        time: entry_time,
    }))
}

fn temporal_broad_phase_overlaps(
    left: BodyState,
    right: BodyState,
    remaining_subticks: i128,
) -> Result<bool, PhysicsError3d> {
    for axis in Axis::ALL {
        let start = left.position.axis(axis) - right.position.axis(axis);
        let relative_velocity = i64::from(left.axis_velocity(axis))
            .saturating_sub(i64::from(right.axis_velocity(axis)));
        let delta = i128::from(relative_velocity)
            .checked_mul(remaining_subticks)
            .ok_or(PhysicsError3d::CoordinateOutOfRange(left.entity))?;
        let end = start
            .checked_add(delta)
            .ok_or(PhysicsError3d::CoordinateOutOfRange(left.entity))?;
        let swept_min = start.min(end);
        let swept_max = start.max(end);
        let (minimum, maximum) = expanded_relative_bounds(left, right, axis);
        if swept_max < minimum || swept_min > maximum {
            return Ok(false);
        }
    }
    Ok(true)
}

fn expanded_relative_bounds(left: BodyState, right: BodyState, axis: Axis) -> (i128, i128) {
    let index = axis.index();
    let expanded_half = (i128::from(left.half_extents[index])
        + i128::from(right.half_extents[index]))
        * SUBTICK_SCALE;
    (-expanded_half, expanded_half)
}

fn advance_all(states: &mut [BodyState], subticks: i128) -> Result<(), PhysicsError3d> {
    if subticks == 0 {
        return Ok(());
    }
    for state in states {
        if state.kind == BodyKind::Fixed {
            continue;
        }
        state
            .position
            .advance(state.velocity, subticks)
            .map_err(|()| PhysicsError3d::CoordinateOutOfRange(state.entity))?;
    }
    Ok(())
}

fn snap_pair_to_contact(states: &mut [BodyState], hit: SweepHit) -> Result<(), PhysicsError3d> {
    let (left_slice, right_slice) = states.split_at_mut(hit.right_index);
    let left = &mut left_slice[hit.left_index];
    let right = &mut right_slice[0];
    let current_relative = left.position.axis(hit.axis) - right.position.axis(hit.axis);
    let correction = hit.target_relative - current_relative;
    if correction == 0 {
        return Ok(());
    }

    match (left.kind, right.kind) {
        (BodyKind::Fixed, BodyKind::Fixed) => {}
        (BodyKind::Dynamic, BodyKind::Fixed) => left.offset_axis(hit.axis, correction)?,
        (BodyKind::Fixed, BodyKind::Dynamic) => right.offset_axis(hit.axis, -correction)?,
        (BodyKind::Dynamic, BodyKind::Dynamic) => {
            let total_mass = i128::from(left.mass_units) + i128::from(right.mass_units);
            let left_delta = correction
                .checked_mul(i128::from(right.mass_units))
                .ok_or(PhysicsError3d::CoordinateOutOfRange(left.entity))?
                / total_mass;
            let right_delta = left_delta - correction;
            left.offset_axis(hit.axis, left_delta)?;
            right.offset_axis(hit.axis, right_delta)?;
        }
    }
    Ok(())
}

fn resolve_sweep_hit(
    states: &mut [BodyState],
    hit: SweepHit,
    step_stats: &mut PhysicsStep3dStats,
    contacts: &mut Vec<PhysicsContact3d>,
    supporting_entities: &mut BTreeSet<EntityId>,
) {
    let (left_slice, right_slice) = states.split_at_mut(hit.right_index);
    let left = &mut left_slice[hit.left_index];
    let right = &mut right_slice[0];
    let restitution_milli = combined_restitution(left, right);
    let friction_milli = combined_friction(left, right);
    let normal_changed = apply_normal_response(left, right, hit.axis, restitution_milli);
    let [tangent_a, tangent_b] = hit.axis.tangents();
    let friction_a = apply_contact_friction(left, right, tangent_a, friction_milli);
    let friction_b = apply_contact_friction(left, right, tangent_b, friction_milli);

    contacts.push(PhysicsContact3d {
        left: left.entity,
        right: right.entity,
        normal: contact_normal(hit.axis, hit.normal),
        penetration: 0,
    });
    step_stats.contacts = step_stats.contacts.saturating_add(1);
    step_stats.ccd_contacts = step_stats.ccd_contacts.saturating_add(1);
    if normal_changed || friction_a || friction_b {
        step_stats.resolved_contacts = step_stats.resolved_contacts.saturating_add(1);
    }
    if let Some(entity) = supported_entity(left, right, hit.axis, hit.normal) {
        supporting_entities.insert(entity);
    }
}

fn stabilize_penetrations(
    states: &mut [BodyState],
    step_stats: &mut PhysicsStep3dStats,
    contacts: &mut Vec<PhysicsContact3d>,
    supporting_entities: &mut BTreeSet<EntityId>,
) -> Result<(), PhysicsError3d> {
    for _ in 0..MAX_STABILIZATION_PASSES {
        let mut corrected_any = false;
        for left_index in 0..states.len() {
            for right_index in (left_index + 1)..states.len() {
                if states[left_index].kind == BodyKind::Fixed
                    && states[right_index].kind == BodyKind::Fixed
                {
                    continue;
                }
                let Some((axis, penetration, normal)) =
                    positive_penetration(states[left_index], states[right_index])
                else {
                    continue;
                };
                let outcome = resolve_penetration_pair(
                    states,
                    left_index,
                    right_index,
                    axis,
                    penetration,
                    normal,
                )?;
                corrected_any |= outcome.resolved;
                record_pair_outcome(outcome, step_stats, contacts, supporting_entities);
            }
        }
        if !corrected_any {
            return Ok(());
        }
    }

    for left_index in 0..states.len() {
        for right_index in (left_index + 1)..states.len() {
            if states[left_index].kind == BodyKind::Fixed
                && states[right_index].kind == BodyKind::Fixed
            {
                continue;
            }
            if positive_penetration(states[left_index], states[right_index]).is_some() {
                let entity = if states[left_index].kind == BodyKind::Dynamic {
                    states[left_index].entity
                } else {
                    states[right_index].entity
                };
                return Err(PhysicsError3d::CcdIterationLimit(entity));
            }
        }
    }
    Ok(())
}

fn positive_penetration(left: BodyState, right: BodyState) -> Option<(Axis, i128, i64)> {
    let overlap_x = scaled_axis_overlap(left, right, Axis::X);
    let overlap_y = scaled_axis_overlap(left, right, Axis::Y);
    let overlap_z = scaled_axis_overlap(left, right, Axis::Z);
    if overlap_x <= 0 || overlap_y <= 0 || overlap_z <= 0 {
        return None;
    }
    let (axis, penetration) = if overlap_x <= overlap_y && overlap_x <= overlap_z {
        (Axis::X, overlap_x)
    } else if overlap_y <= overlap_z {
        (Axis::Y, overlap_y)
    } else {
        (Axis::Z, overlap_z)
    };
    let normal = if right.position.axis(axis) >= left.position.axis(axis) {
        1_i64
    } else {
        -1_i64
    };
    Some((axis, penetration, normal))
}

fn scaled_axis_overlap(left: BodyState, right: BodyState, axis: Axis) -> i128 {
    let index = axis.index();
    let left_half = i128::from(left.half_extents[index]) * SUBTICK_SCALE;
    let right_half = i128::from(right.half_extents[index]) * SUBTICK_SCALE;
    let left_center = left.position.axis(axis);
    let right_center = right.position.axis(axis);
    let left_min = left_center - left_half;
    let left_max = left_center + left_half;
    let right_min = right_center - right_half;
    let right_max = right_center + right_half;
    left_max.min(right_max) - left_min.max(right_min)
}

#[derive(Clone, Copy, Debug, Default)]
struct PairOutcome {
    contact: Option<PhysicsContact3d>,
    resolved: bool,
    supported_entity: Option<EntityId>,
}

fn resolve_penetration_pair(
    states: &mut [BodyState],
    left_index: usize,
    right_index: usize,
    axis: Axis,
    penetration: i128,
    normal: i64,
) -> Result<PairOutcome, PhysicsError3d> {
    let (left_slice, right_slice) = states.split_at_mut(right_index);
    let left = &mut left_slice[left_index];
    let right = &mut right_slice[0];
    let relative_velocity =
        i64::from(right.axis_velocity(axis)).saturating_sub(i64::from(left.axis_velocity(axis)));
    let approaching = relative_velocity.saturating_mul(normal) < 0;
    let restitution_milli = combined_restitution(left, right);
    let friction_milli = combined_friction(left, right);
    let corrected = correct_penetration(left, right, axis, normal, penetration)?;
    let normal_changed = approaching && apply_normal_response(left, right, axis, restitution_milli);
    let [tangent_a, tangent_b] = axis.tangents();
    let friction_a = apply_contact_friction(left, right, tangent_a, friction_milli);
    let friction_b = apply_contact_friction(left, right, tangent_b, friction_milli);
    let penetration_units = i64::try_from(penetration / SUBTICK_SCALE)
        .map_err(|_| PhysicsError3d::CoordinateOutOfRange(left.entity))?;

    Ok(PairOutcome {
        contact: Some(PhysicsContact3d {
            left: left.entity,
            right: right.entity,
            normal: contact_normal(axis, normal),
            penetration: penetration_units,
        }),
        resolved: corrected || normal_changed || friction_a || friction_b,
        supported_entity: supported_entity(left, right, axis, normal),
    })
}

fn correct_penetration(
    left: &mut BodyState,
    right: &mut BodyState,
    axis: Axis,
    normal: i64,
    penetration: i128,
) -> Result<bool, PhysicsError3d> {
    if penetration <= 0 {
        return Ok(false);
    }
    let direction = i128::from(normal);
    match (left.kind, right.kind) {
        (BodyKind::Fixed, BodyKind::Fixed) => Ok(false),
        (BodyKind::Dynamic, BodyKind::Fixed) => {
            left.offset_axis(axis, -direction * penetration)?;
            Ok(true)
        }
        (BodyKind::Fixed, BodyKind::Dynamic) => {
            right.offset_axis(axis, direction * penetration)?;
            Ok(true)
        }
        (BodyKind::Dynamic, BodyKind::Dynamic) => {
            let (left_share, right_share) =
                dynamic_penetration_shares(penetration, left.mass_units, right.mass_units);
            left.offset_axis(axis, -direction * left_share)?;
            right.offset_axis(axis, direction * right_share)?;
            Ok(true)
        }
    }
}

fn dynamic_penetration_shares(penetration: i128, left_mass: u32, right_mass: u32) -> (i128, i128) {
    let total_mass = i128::from(left_mass) + i128::from(right_mass);
    let mut left_share = penetration * i128::from(right_mass) / total_mass;
    let mut right_share = penetration * i128::from(left_mass) / total_mass;
    let remainder = penetration - left_share - right_share;
    if remainder > 0 {
        if right_mass > left_mass {
            left_share += remainder;
        } else {
            right_share += remainder;
        }
    }
    (left_share, right_share)
}

fn record_pair_outcome(
    outcome: PairOutcome,
    stats: &mut PhysicsStep3dStats,
    contacts: &mut Vec<PhysicsContact3d>,
    supporting_entities: &mut BTreeSet<EntityId>,
) {
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

fn changed_operations(states: &[BodyState]) -> Result<Vec<Operation>, PhysicsError3d> {
    let mut operations = Vec::with_capacity(states.len().saturating_mul(2));
    for state in states {
        if state.kind == BodyKind::Fixed {
            continue;
        }
        let position = state.current_position()?;
        if position != state.original_position {
            operations.push(Operation::SetPosition(state.entity, position));
        }
        if state.velocity != state.original_velocity {
            operations.push(Operation::SetVelocity(state.entity, state.velocity));
        }
    }
    Ok(operations)
}

fn arithmetic_error(states: &[BodyState]) -> PhysicsError3d {
    let entity = states
        .iter()
        .find(|state| state.kind == BodyKind::Dynamic)
        .map_or(EntityId(0), |state| state.entity);
    PhysicsError3d::CoordinateOutOfRange(entity)
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
    fn fast_dynamic_bodies_cannot_tunnel_through_each_other() {
        let left = EntityId(1);
        let right = EntityId(2);
        let snapshot = WorldSnapshot::new(vec![
            EntitySnapshot {
                id: left,
                position: Some(Position::new3(-10, 0, 0)),
                velocity: Some(Velocity::new3(30, 0, 0)),
            },
            EntitySnapshot {
                id: right,
                position: Some(Position::new3(10, 0, 0)),
                velocity: Some(Velocity::new3(-30, 0, 0)),
            },
        ]);
        let physics = step_3d(
            &snapshot,
            &[
                PhysicsBody3d::dynamic(left, [1, 1, 1]),
                PhysicsBody3d::dynamic(right, [1, 1, 1]),
            ],
            NO_GRAVITY,
            1,
        )
        .expect("relative-motion CCD should catch the crossing bodies");

        assert_eq!(physics.stats().ccd_contacts, 1);
        assert_eq!(physics.contacts().len(), 1);
        assert_eq!(
            physics.contacts()[0].normal,
            ContactNormal3d { x: 1, y: 0, z: 0 }
        );
        assert!(
            physics
                .operations()
                .contains(&Operation::SetPosition(left, Position::new3(-1, 0, 0)))
        );
        assert!(
            physics
                .operations()
                .contains(&Operation::SetPosition(right, Position::new3(1, 0, 0)))
        );
        assert!(
            physics
                .operations()
                .contains(&Operation::SetVelocity(left, Velocity::new3(0, 0, 0)))
        );
        assert!(
            physics
                .operations()
                .contains(&Operation::SetVelocity(right, Velocity::new3(0, 0, 0)))
        );
    }

    #[test]
    fn dynamic_bounce_consumes_remaining_time_after_toi() {
        let left = EntityId(1);
        let right = EntityId(2);
        let snapshot = WorldSnapshot::new(vec![
            EntitySnapshot {
                id: left,
                position: Some(Position::new3(-10, 0, 0)),
                velocity: Some(Velocity::new3(30, 0, 0)),
            },
            EntitySnapshot {
                id: right,
                position: Some(Position::new3(10, 0, 0)),
                velocity: Some(Velocity::new3(-30, 0, 0)),
            },
        ]);
        let bouncy = PhysicsMaterial::new(1_000, 0);
        let physics = step_3d(
            &snapshot,
            &[
                PhysicsBody3d::dynamic(left, [1, 1, 1]).with_material(bouncy),
                PhysicsBody3d::dynamic(right, [1, 1, 1]).with_material(bouncy),
            ],
            NO_GRAVITY,
            1,
        )
        .expect("bouncy dynamic CCD should consume the remainder");

        assert!(
            physics
                .operations()
                .contains(&Operation::SetVelocity(left, Velocity::new3(-30, 0, 0)))
        );
        assert!(
            physics
                .operations()
                .contains(&Operation::SetVelocity(right, Velocity::new3(30, 0, 0)))
        );
        let left_position = physics
            .operations()
            .iter()
            .find_map(|operation| match operation {
                Operation::SetPosition(entity, position) if *entity == left => Some(*position),
                _ => None,
            });
        let right_position = physics
            .operations()
            .iter()
            .find_map(|operation| match operation {
                Operation::SetPosition(entity, position) if *entity == right => Some(*position),
                _ => None,
            });
        assert!(left_position.is_some_and(|position| position.x <= -21));
        assert!(right_position.is_some_and(|position| position.x >= 21));
    }

    #[test]
    fn glancing_dynamic_sweep_requires_tangent_overlap() {
        let left = EntityId(1);
        let right = EntityId(2);
        let snapshot = WorldSnapshot::new(vec![
            EntitySnapshot {
                id: left,
                position: Some(Position::new3(-10, 20, 0)),
                velocity: Some(Velocity::new3(30, 0, 0)),
            },
            EntitySnapshot {
                id: right,
                position: Some(Position::new3(10, 0, 0)),
                velocity: Some(Velocity::new3(-30, 0, 0)),
            },
        ]);
        let physics = step_3d(
            &snapshot,
            &[
                PhysicsBody3d::dynamic(left, [1, 1, 1]),
                PhysicsBody3d::dynamic(right, [1, 1, 1]),
            ],
            NO_GRAVITY,
            1,
        )
        .expect("tangent-separated bodies should not collide");

        assert_eq!(physics.stats().ccd_contacts, 0);
        assert!(
            physics
                .operations()
                .contains(&Operation::SetPosition(left, Position::new3(20, 20, 0)))
        );
        assert!(
            physics
                .operations()
                .contains(&Operation::SetPosition(right, Position::new3(-20, 0, 0)))
        );
    }

    #[test]
    fn equal_toi_events_use_entity_pair_order() {
        let snapshot = WorldSnapshot::new(vec![
            EntitySnapshot {
                id: EntityId(1),
                position: Some(Position::new3(-10, 0, 0)),
                velocity: Some(Velocity::new3(30, 0, 0)),
            },
            EntitySnapshot {
                id: EntityId(2),
                position: Some(Position::new3(10, 0, 0)),
                velocity: Some(Velocity::new3(-30, 0, 0)),
            },
            EntitySnapshot {
                id: EntityId(3),
                position: Some(Position::new3(-10, 10, 0)),
                velocity: Some(Velocity::new3(30, 0, 0)),
            },
            EntitySnapshot {
                id: EntityId(4),
                position: Some(Position::new3(10, 10, 0)),
                velocity: Some(Velocity::new3(-30, 0, 0)),
            },
        ]);
        let bodies = [
            PhysicsBody3d::dynamic(EntityId(1), [1, 1, 1]),
            PhysicsBody3d::dynamic(EntityId(2), [1, 1, 1]),
            PhysicsBody3d::dynamic(EntityId(3), [1, 1, 1]),
            PhysicsBody3d::dynamic(EntityId(4), [1, 1, 1]),
        ];
        let physics = step_3d(&snapshot, &bodies, NO_GRAVITY, 1)
            .expect("equal-time independent contacts should remain deterministic");

        assert!(physics.contacts().len() >= 2);
        assert_eq!(
            (physics.contacts()[0].left, physics.contacts()[0].right),
            (EntityId(1), EntityId(2))
        );
        assert_eq!(
            (physics.contacts()[1].left, physics.contacts()[1].right),
            (EntityId(3), EntityId(4))
        );
    }

    #[test]
    fn static_and_dynamic_hits_share_one_timeline() {
        let mover = EntityId(1);
        let target = EntityId(2);
        let wall = EntityId(3);
        let snapshot = WorldSnapshot::new(vec![
            EntitySnapshot {
                id: mover,
                position: Some(Position::new3(-10, 0, 0)),
                velocity: Some(Velocity::new3(30, 0, 0)),
            },
            EntitySnapshot {
                id: target,
                position: Some(Position::new3(0, 0, 0)),
                velocity: Some(Velocity::new3(0, 0, 0)),
            },
            EntitySnapshot {
                id: wall,
                position: Some(Position::new3(10, 0, 0)),
                velocity: None,
            },
        ]);
        let physics = step_3d(
            &snapshot,
            &[
                PhysicsBody3d::dynamic(mover, [1, 1, 1]),
                PhysicsBody3d::dynamic(target, [1, 1, 1]),
                PhysicsBody3d::fixed(wall, [1, 10, 10]),
            ],
            NO_GRAVITY,
            1,
        )
        .expect("dynamic and fixed events should be ordered on one timeline");

        assert!(!physics.contacts().is_empty());
        assert_eq!(
            (physics.contacts()[0].left, physics.contacts()[0].right),
            (mover, target)
        );
    }
}
