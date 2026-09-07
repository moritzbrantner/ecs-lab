use std::fmt;

use ecs_physics::{BodyKind, MATERIAL_SCALE};
use ecs_workload::{EntityId, Position, Velocity};

use crate::{
    AngularError3d, AngularState3d, AngularVelocity3d, BoxBoxStabilizationError3d, BoxPlaneError3d,
    PhysicsBody3d, RigidBoxState3d, integrate_orientation, oriented_box_vertices,
    stabilize_box_box_contact,
};

const MAX_SOLVER_PASSES: u8 = 16;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct RigidBox3d {
    pub body: PhysicsBody3d,
    pub state: RigidBoxState3d,
}

impl RigidBox3d {
    #[must_use]
    pub const fn new(body: PhysicsBody3d, state: RigidBoxState3d) -> Self {
        Self { body, state }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct RigidBoxWorldConfig3d {
    pub gravity: Velocity,
    pub timestep_numerator: i32,
    pub timestep_denominator: i32,
    pub angular_damping_milli: u16,
    pub solver_passes: u8,
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct RigidBoxWorldStats3d {
    /// Number of oriented-AABB pairs admitted to OBB narrow phase across all solver passes.
    pub candidate_pairs: usize,
    /// Number of SAT contacts observed across all solver passes, including zero-overlap touching pairs.
    pub contacts: usize,
    /// Contacts that applied a non-zero normal impulse.
    pub impulsive_contacts: usize,
    /// Dynamic bodies carrying non-zero angular velocity after response and damping.
    pub spinning_bodies: usize,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct RigidBoxWorldStep3d {
    pub boxes: Vec<RigidBox3d>,
    pub stats: RigidBoxWorldStats3d,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RigidBoxWorldError3d {
    NonCanonicalEntityOrder(EntityId, EntityId),
    InvalidHalfExtents(EntityId),
    ZeroMass(EntityId),
    RestitutionOutOfRange(EntityId, u16),
    FrictionOutOfRange(EntityId, u16),
    NegativeTimestepNumerator(i32),
    NonPositiveTimestepDenominator(i32),
    DampingOutOfRange(u16),
    SolverPassesOutOfRange(u8),
    Angular(AngularError3d),
    Geometry(BoxPlaneError3d),
    Stabilization(BoxBoxStabilizationError3d),
    ArithmeticOverflow,
}

impl fmt::Display for RigidBoxWorldError3d {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NonCanonicalEntityOrder(left, right) => write!(
                formatter,
                "rigid-box world requires strictly ascending entity ids, got {} then {}",
                left.0, right.0
            ),
            Self::InvalidHalfExtents(entity) => write!(
                formatter,
                "rigid-box world body {} requires strictly positive half extents",
                entity.0
            ),
            Self::ZeroMass(entity) => write!(
                formatter,
                "dynamic rigid-box world body {} requires non-zero mass",
                entity.0
            ),
            Self::RestitutionOutOfRange(entity, value) => write!(
                formatter,
                "rigid-box world body {} has restitution {value}, expected 0..={MATERIAL_SCALE}",
                entity.0
            ),
            Self::FrictionOutOfRange(entity, value) => write!(
                formatter,
                "rigid-box world body {} has friction {value}, expected 0..={MATERIAL_SCALE}",
                entity.0
            ),
            Self::NegativeTimestepNumerator(value) => write!(
                formatter,
                "rigid-box world timestep numerator must be non-negative, got {value}"
            ),
            Self::NonPositiveTimestepDenominator(value) => write!(
                formatter,
                "rigid-box world timestep denominator must be positive, got {value}"
            ),
            Self::DampingOutOfRange(value) => write!(
                formatter,
                "rigid-box world angular damping must be 0..={MATERIAL_SCALE}, got {value}"
            ),
            Self::SolverPassesOutOfRange(value) => write!(
                formatter,
                "rigid-box world solver passes must be 1..={MAX_SOLVER_PASSES}, got {value}"
            ),
            Self::Angular(error) => write!(
                formatter,
                "rigid-box world angular integration failed: {error}"
            ),
            Self::Geometry(error) => write!(formatter, "rigid-box world geometry failed: {error}"),
            Self::Stabilization(error) => {
                write!(formatter, "rigid-box world contact failed: {error}")
            }
            Self::ArithmeticOverflow => write!(formatter, "rigid-box world arithmetic overflowed"),
        }
    }
}

impl std::error::Error for RigidBoxWorldError3d {}

impl From<AngularError3d> for RigidBoxWorldError3d {
    fn from(value: AngularError3d) -> Self {
        Self::Angular(value)
    }
}

impl From<BoxPlaneError3d> for RigidBoxWorldError3d {
    fn from(value: BoxPlaneError3d) -> Self {
        Self::Geometry(value)
    }
}

impl From<BoxBoxStabilizationError3d> for RigidBoxWorldError3d {
    fn from(value: BoxBoxStabilizationError3d) -> Self {
        Self::Stabilization(value)
    }
}

/// Advances a small deterministic world of oriented rigid cuboids by one rational timestep.
///
/// The step intentionally mirrors the staging of the existing continuous AABB solver without replacing
/// it: integrate each dynamic body once, use Rust-computed oriented vertices for a conservative AABB
/// candidate cull, then run exact OBB SAT response/stabilization in stable entity order for a bounded
/// number of passes. A ground plane can therefore be represented as an ordinary fixed OBB and shares the
/// exact same collision truth as tower blocks and projectiles.
///
/// This is a discrete 60 Hz-oriented rigid-body path. It does not claim rotational CCD, and its candidate
/// cull is not a replacement for the existing swept spatial-hash broad phase used by the dense room. The
/// intended next consumer is the bounded trebuchet/tower fixture where object count is small and exact
/// Rust-owned angular geometry matters more than broad-phase scale.
///
/// # Errors
///
/// Returns [`RigidBoxWorldError3d`] for non-canonical body ordering, malformed bodies/configuration,
/// invalid orientation geometry, contact response failure, or checked arithmetic overflow.
pub fn step_rigid_box_world(
    boxes: &[RigidBox3d],
    config: RigidBoxWorldConfig3d,
) -> Result<RigidBoxWorldStep3d, RigidBoxWorldError3d> {
    validate_world(boxes, config)?;
    let mut next = boxes.to_vec();
    for rigid_box in &mut next {
        integrate_body(rigid_box, config)?;
    }

    let mut stats = RigidBoxWorldStats3d::default();
    for _ in 0..config.solver_passes {
        let bounds = next
            .iter()
            .map(oriented_bounds)
            .collect::<Result<Vec<_>, _>>()?;
        for left_index in 0..next.len() {
            for right_index in left_index + 1..next.len() {
                if next[left_index].body.kind == BodyKind::Fixed
                    && next[right_index].body.kind == BodyKind::Fixed
                {
                    continue;
                }
                if !bounds_overlap(bounds[left_index], bounds[right_index]) {
                    continue;
                }
                stats.candidate_pairs = stats
                    .candidate_pairs
                    .checked_add(1)
                    .ok_or(RigidBoxWorldError3d::ArithmeticOverflow)?;

                let (left_slice, right_slice) = next.split_at_mut(right_index);
                let left = &mut left_slice[left_index];
                let right = &mut right_slice[0];
                let resolved =
                    stabilize_box_box_contact(left.state, left.body, right.state, right.body)?;
                if let Some(contact) = resolved.contact {
                    stats.contacts = stats
                        .contacts
                        .checked_add(1)
                        .ok_or(RigidBoxWorldError3d::ArithmeticOverflow)?;
                    if contact.normal_impulse_units != 0 {
                        stats.impulsive_contacts = stats
                            .impulsive_contacts
                            .checked_add(1)
                            .ok_or(RigidBoxWorldError3d::ArithmeticOverflow)?;
                    }
                }
                left.state = resolved.left;
                right.state = resolved.right;
            }
        }
    }

    for rigid_box in &mut next {
        if rigid_box.body.kind == BodyKind::Dynamic {
            rigid_box.state.angular.angular_velocity = damp_angular_velocity(
                rigid_box.state.angular.angular_velocity,
                config.angular_damping_milli,
            )?;
            if !rigid_box.state.angular.angular_velocity.is_zero() {
                stats.spinning_bodies = stats
                    .spinning_bodies
                    .checked_add(1)
                    .ok_or(RigidBoxWorldError3d::ArithmeticOverflow)?;
            }
        }
    }

    Ok(RigidBoxWorldStep3d { boxes: next, stats })
}

fn validate_world(
    boxes: &[RigidBox3d],
    config: RigidBoxWorldConfig3d,
) -> Result<(), RigidBoxWorldError3d> {
    if config.timestep_numerator < 0 {
        return Err(RigidBoxWorldError3d::NegativeTimestepNumerator(
            config.timestep_numerator,
        ));
    }
    if config.timestep_denominator <= 0 {
        return Err(RigidBoxWorldError3d::NonPositiveTimestepDenominator(
            config.timestep_denominator,
        ));
    }
    if config.angular_damping_milli > MATERIAL_SCALE {
        return Err(RigidBoxWorldError3d::DampingOutOfRange(
            config.angular_damping_milli,
        ));
    }
    if config.solver_passes == 0 || config.solver_passes > MAX_SOLVER_PASSES {
        return Err(RigidBoxWorldError3d::SolverPassesOutOfRange(
            config.solver_passes,
        ));
    }

    for window in boxes.windows(2) {
        let left = window[0].body.entity;
        let right = window[1].body.entity;
        if left >= right {
            return Err(RigidBoxWorldError3d::NonCanonicalEntityOrder(left, right));
        }
    }
    for rigid_box in boxes {
        let body = rigid_box.body;
        if body.half_extents.iter().any(|extent| *extent <= 0) {
            return Err(RigidBoxWorldError3d::InvalidHalfExtents(body.entity));
        }
        if body.kind == BodyKind::Dynamic && body.mass_units == 0 {
            return Err(RigidBoxWorldError3d::ZeroMass(body.entity));
        }
        if body.material.restitution_milli > MATERIAL_SCALE {
            return Err(RigidBoxWorldError3d::RestitutionOutOfRange(
                body.entity,
                body.material.restitution_milli,
            ));
        }
        if body.material.friction_milli > MATERIAL_SCALE {
            return Err(RigidBoxWorldError3d::FrictionOutOfRange(
                body.entity,
                body.material.friction_milli,
            ));
        }
        rigid_box.state.angular.orientation.normalized()?;
    }
    Ok(())
}

fn integrate_body(
    rigid_box: &mut RigidBox3d,
    config: RigidBoxWorldConfig3d,
) -> Result<(), RigidBoxWorldError3d> {
    if rigid_box.body.kind == BodyKind::Fixed {
        return Ok(());
    }
    let velocity = Velocity::new3(
        integrate_velocity_axis(
            rigid_box.state.linear_velocity.x,
            config.gravity.x,
            config.timestep_numerator,
            config.timestep_denominator,
        )?,
        integrate_velocity_axis(
            rigid_box.state.linear_velocity.y,
            config.gravity.y,
            config.timestep_numerator,
            config.timestep_denominator,
        )?,
        integrate_velocity_axis(
            rigid_box.state.linear_velocity.z,
            config.gravity.z,
            config.timestep_numerator,
            config.timestep_denominator,
        )?,
    );
    let center = Position::new3(
        integrate_position_axis(
            rigid_box.state.center.x,
            velocity.x,
            config.timestep_numerator,
            config.timestep_denominator,
        )?,
        integrate_position_axis(
            rigid_box.state.center.y,
            velocity.y,
            config.timestep_numerator,
            config.timestep_denominator,
        )?,
        integrate_position_axis(
            rigid_box.state.center.z,
            velocity.z,
            config.timestep_numerator,
            config.timestep_denominator,
        )?,
    );
    let orientation = integrate_orientation(
        rigid_box.state.angular.orientation,
        rigid_box.state.angular.angular_velocity,
        config.timestep_numerator,
        config.timestep_denominator,
    )?;
    rigid_box.state = RigidBoxState3d::new(
        center,
        velocity,
        AngularState3d::new(orientation, rigid_box.state.angular.angular_velocity),
    );
    Ok(())
}

fn integrate_velocity_axis(
    velocity: i32,
    acceleration: i32,
    numerator: i32,
    denominator: i32,
) -> Result<i32, RigidBoxWorldError3d> {
    let acceleration_step = i128::from(acceleration)
        .checked_mul(i128::from(numerator))
        .ok_or(RigidBoxWorldError3d::ArithmeticOverflow)?;
    let delta = div_round_nearest(acceleration_step, i128::from(denominator))?;
    let next = i128::from(velocity)
        .checked_add(delta)
        .ok_or(RigidBoxWorldError3d::ArithmeticOverflow)?;
    i32::try_from(next).map_err(|_| RigidBoxWorldError3d::ArithmeticOverflow)
}

fn integrate_position_axis(
    position: i64,
    velocity: i32,
    numerator: i32,
    denominator: i32,
) -> Result<i64, RigidBoxWorldError3d> {
    let velocity_step = i128::from(velocity)
        .checked_mul(i128::from(numerator))
        .ok_or(RigidBoxWorldError3d::ArithmeticOverflow)?;
    let delta = div_round_nearest(velocity_step, i128::from(denominator))?;
    let next = i128::from(position)
        .checked_add(delta)
        .ok_or(RigidBoxWorldError3d::ArithmeticOverflow)?;
    i64::try_from(next).map_err(|_| RigidBoxWorldError3d::ArithmeticOverflow)
}

fn damp_angular_velocity(
    velocity: AngularVelocity3d,
    damping_milli: u16,
) -> Result<AngularVelocity3d, RigidBoxWorldError3d> {
    Ok(AngularVelocity3d::new(
        damp_axis(velocity.x, damping_milli)?,
        damp_axis(velocity.y, damping_milli)?,
        damp_axis(velocity.z, damping_milli)?,
    ))
}

fn damp_axis(value: i32, damping_milli: u16) -> Result<i32, RigidBoxWorldError3d> {
    let numerator = i128::from(value)
        .checked_mul(i128::from(damping_milli))
        .ok_or(RigidBoxWorldError3d::ArithmeticOverflow)?;
    let damped = div_round_nearest(numerator, i128::from(MATERIAL_SCALE))?;
    i32::try_from(damped).map_err(|_| RigidBoxWorldError3d::ArithmeticOverflow)
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct OrientedBounds3d {
    minimum: [i64; 3],
    maximum: [i64; 3],
}

fn oriented_bounds(rigid_box: &RigidBox3d) -> Result<OrientedBounds3d, RigidBoxWorldError3d> {
    let vertices = oriented_box_vertices(
        rigid_box.state.center,
        rigid_box.body.half_extents,
        rigid_box.state.angular.orientation,
    )?;
    let mut minimum = [i64::MAX; 3];
    let mut maximum = [i64::MIN; 3];
    for vertex in vertices {
        for (axis, value) in [vertex.x, vertex.y, vertex.z].into_iter().enumerate() {
            minimum[axis] = minimum[axis].min(value);
            maximum[axis] = maximum[axis].max(value);
        }
    }
    Ok(OrientedBounds3d { minimum, maximum })
}

fn bounds_overlap(left: OrientedBounds3d, right: OrientedBounds3d) -> bool {
    (0..3).all(|axis| {
        left.maximum[axis] >= right.minimum[axis] && right.maximum[axis] >= left.minimum[axis]
    })
}

fn div_round_nearest(numerator: i128, denominator: i128) -> Result<i128, RigidBoxWorldError3d> {
    if denominator <= 0 {
        return Err(RigidBoxWorldError3d::ArithmeticOverflow);
    }
    let half = denominator / 2;
    let adjusted = if numerator >= 0 {
        numerator.checked_add(half)
    } else {
        numerator.checked_sub(half)
    }
    .ok_or(RigidBoxWorldError3d::ArithmeticOverflow)?;
    Ok(adjusted / denominator)
}

#[cfg(test)]
mod tests {
    use ecs_physics::PhysicsMaterial;

    use crate::{AngularState3d, AngularVelocity3d, Orientation3d};

    use super::*;

    const SCALE: i32 = 3_600;

    fn config(gravity_y: i32) -> RigidBoxWorldConfig3d {
        RigidBoxWorldConfig3d {
            gravity: Velocity::new3(0, gravity_y, 0),
            timestep_numerator: 1,
            timestep_denominator: 60,
            angular_damping_milli: 995,
            solver_passes: 8,
        }
    }

    fn state(x: i64, y: i64, velocity: Velocity) -> RigidBoxState3d {
        RigidBoxState3d::new(
            Position::new3(x, y, 0),
            velocity,
            AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
        )
    }

    fn floor() -> RigidBox3d {
        RigidBox3d::new(
            PhysicsBody3d::fixed(EntityId(0), [30 * SCALE, SCALE, 10 * SCALE]),
            state(0, -i64::from(SCALE), Velocity::new3(0, 0, 0)),
        )
    }

    fn block(entity: u32, x: i64, y: i64) -> RigidBox3d {
        RigidBox3d::new(
            PhysicsBody3d::dynamic(EntityId(entity), [SCALE, SCALE, SCALE])
                .with_material(PhysicsMaterial::new(100, 600)),
            state(x, y, Velocity::new3(0, 0, 0)),
        )
    }

    #[test]
    fn stacked_boxes_remain_out_of_the_fixed_floor() {
        let mut boxes = vec![
            floor(),
            block(1, 0, i64::from(SCALE)),
            block(2, 0, 3 * i64::from(SCALE)),
        ];
        for _ in 0..120 {
            boxes = step_rigid_box_world(&boxes, config(-10 * SCALE))
                .expect("valid stacked world")
                .boxes;
            for rigid_box in boxes
                .iter()
                .filter(|value| value.body.kind == BodyKind::Dynamic)
            {
                let vertices = oriented_box_vertices(
                    rigid_box.state.center,
                    rigid_box.body.half_extents,
                    rigid_box.state.angular.orientation,
                )
                .expect("valid vertices");
                assert!(vertices.iter().all(|vertex| vertex.y >= 0));
            }
        }
    }

    #[test]
    fn off_center_projectile_hit_spins_an_ordinary_target_box() {
        let projectile = RigidBox3d::new(
            PhysicsBody3d::dynamic(EntityId(1), [SCALE, SCALE, SCALE])
                .with_mass(8)
                .with_material(PhysicsMaterial::new(100, 0)),
            state(
                -5 * i64::from(SCALE),
                2 * i64::from(SCALE),
                Velocity::new3(12 * SCALE, 0, 0),
            ),
        );
        let target = block(2, 0, i64::from(SCALE));
        let mut boxes = vec![floor(), projectile, target];
        let mut target_spun = false;
        for _ in 0..30 {
            let step = step_rigid_box_world(&boxes, config(0)).expect("valid impact world");
            boxes = step.boxes;
            target_spun |= !boxes[2].state.angular.angular_velocity.is_zero();
        }
        assert!(target_spun);
    }

    #[test]
    fn replay_is_exactly_deterministic() {
        let initial = vec![
            floor(),
            block(1, -i64::from(SCALE), i64::from(SCALE)),
            block(2, i64::from(SCALE), 3 * i64::from(SCALE)),
        ];
        let advance = |mut boxes: Vec<RigidBox3d>| {
            for _ in 0..90 {
                boxes = step_rigid_box_world(&boxes, config(-10 * SCALE))
                    .expect("valid replay")
                    .boxes;
            }
            boxes
        };

        assert_eq!(advance(initial.clone()), advance(initial));
    }

    #[test]
    fn oriented_aabb_cull_skips_far_pairs() {
        let boxes = vec![
            block(1, -20 * i64::from(SCALE), 10 * i64::from(SCALE)),
            block(2, 20 * i64::from(SCALE), 10 * i64::from(SCALE)),
        ];
        let step = step_rigid_box_world(&boxes, config(0)).expect("valid separated world");

        assert_eq!(step.stats.candidate_pairs, 0);
        assert_eq!(step.stats.contacts, 0);
    }
}
