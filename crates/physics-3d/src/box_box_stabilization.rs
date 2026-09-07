use std::fmt;

use ecs_physics::BodyKind;
use ecs_workload::{EntityId, Position};

use crate::{BoxBoxError3d, BoxBoxStep3d, PhysicsBody3d, RigidBoxState3d, resolve_box_box_contact};

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BoxBoxStabilizationError3d {
    NonCanonicalPair(EntityId, EntityId),
    Response(BoxBoxError3d),
    ArithmeticOverflow,
}

impl fmt::Display for BoxBoxStabilizationError3d {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NonCanonicalPair(left, right) => write!(
                formatter,
                "OBB stabilization expects ascending entity ids, got {} then {}",
                left.0, right.0
            ),
            Self::Response(error) => {
                write!(formatter, "OBB stabilization response failed: {error}")
            }
            Self::ArithmeticOverflow => {
                write!(formatter, "OBB stabilization arithmetic overflowed")
            }
        }
    }
}

impl std::error::Error for BoxBoxStabilizationError3d {}

impl From<BoxBoxError3d> for BoxBoxStabilizationError3d {
    fn from(value: BoxBoxError3d) -> Self {
        Self::Response(value)
    }
}

/// Resolves an OBB contact and projects any remaining discrete penetration out of the pair.
///
/// Pair ordering is deliberately canonical (`left.entity < right.entity`) because integer-grid mass
/// weighting can leave a one-unit remainder. The stable entity order owns that remainder, matching the
/// existing continuous solver's ordered contact-set policy.
///
/// The response impulse is calculated from the original overlapping contact before projection. Position
/// correction then moves the two centers along the same Rust-owned SAT axis, weighted by inverse mass;
/// fixed bodies receive no correction. The correction vector is the nearest integer-grid approximation
/// to the SAT minimum translation vector, with a deterministic dominant-axis remainder that guarantees
/// the chosen separating axis closes its reported overlap.
///
/// This is discrete stabilization, not rotational CCD. It exists so a later 60 Hz multi-body angular
/// world can stack and topple boxes without rendering a collision pose different from Rust geometry.
///
/// # Errors
///
/// Returns [`BoxBoxStabilizationError3d`] for non-canonical pair order, underlying response errors, or
/// checked arithmetic overflow.
pub fn stabilize_box_box_contact(
    left_state: RigidBoxState3d,
    left_body: PhysicsBody3d,
    right_state: RigidBoxState3d,
    right_body: PhysicsBody3d,
) -> Result<BoxBoxStep3d, BoxBoxStabilizationError3d> {
    if left_body.entity >= right_body.entity {
        return Err(BoxBoxStabilizationError3d::NonCanonicalPair(
            left_body.entity,
            right_body.entity,
        ));
    }

    let mut step = resolve_box_box_contact(left_state, left_body, right_state, right_body)?;
    let Some(contact) = step.contact else {
        return Ok(step);
    };
    if contact.overlap_numerator == 0
        || (left_body.kind == BodyKind::Fixed && right_body.kind == BodyKind::Fixed)
    {
        return Ok(step);
    }

    let correction = minimum_translation_vector(
        contact.axis,
        contact.overlap_numerator,
        contact.axis_length_squared,
    )?;
    project_pair(
        &mut step.left,
        left_body,
        &mut step.right,
        right_body,
        correction,
    )?;
    Ok(step)
}

fn minimum_translation_vector(
    axis: [i128; 3],
    overlap_numerator: u128,
    axis_length_squared: u128,
) -> Result<[i64; 3], BoxBoxStabilizationError3d> {
    let overlap = i128::try_from(overlap_numerator)
        .map_err(|_| BoxBoxStabilizationError3d::ArithmeticOverflow)?;
    let length_squared = i128::try_from(axis_length_squared)
        .map_err(|_| BoxBoxStabilizationError3d::ArithmeticOverflow)?;
    if overlap <= 0 || length_squared <= 0 {
        return Err(BoxBoxStabilizationError3d::ArithmeticOverflow);
    }

    let mut correction = [0_i128; 3];
    for (target, component) in correction.iter_mut().zip(axis) {
        *target = div_round_nearest(checked_mul(overlap, component)?, length_squared)?;
    }

    let achieved = checked_dot(correction, axis)?;
    if achieved < overlap {
        let dominant = dominant_axis(axis)?;
        let magnitude = axis[dominant]
            .checked_abs()
            .ok_or(BoxBoxStabilizationError3d::ArithmeticOverflow)?;
        let residual = overlap
            .checked_sub(achieved)
            .ok_or(BoxBoxStabilizationError3d::ArithmeticOverflow)?;
        let extra = div_ceil_positive(residual, magnitude)?;
        let signed_extra = if axis[dominant] < 0 { -extra } else { extra };
        correction[dominant] = correction[dominant]
            .checked_add(signed_extra)
            .ok_or(BoxBoxStabilizationError3d::ArithmeticOverflow)?;
    }

    Ok([
        i64::try_from(correction[0]).map_err(|_| BoxBoxStabilizationError3d::ArithmeticOverflow)?,
        i64::try_from(correction[1]).map_err(|_| BoxBoxStabilizationError3d::ArithmeticOverflow)?,
        i64::try_from(correction[2]).map_err(|_| BoxBoxStabilizationError3d::ArithmeticOverflow)?,
    ])
}

fn dominant_axis(axis: [i128; 3]) -> Result<usize, BoxBoxStabilizationError3d> {
    let mut best = 0_usize;
    let mut magnitude = axis[0]
        .checked_abs()
        .ok_or(BoxBoxStabilizationError3d::ArithmeticOverflow)?;
    for (index, component) in axis.into_iter().enumerate().skip(1) {
        let candidate = component
            .checked_abs()
            .ok_or(BoxBoxStabilizationError3d::ArithmeticOverflow)?;
        if candidate > magnitude {
            best = index;
            magnitude = candidate;
        }
    }
    if magnitude == 0 {
        return Err(BoxBoxStabilizationError3d::ArithmeticOverflow);
    }
    Ok(best)
}

fn project_pair(
    left: &mut RigidBoxState3d,
    left_body: PhysicsBody3d,
    right: &mut RigidBoxState3d,
    right_body: PhysicsBody3d,
    correction: [i64; 3],
) -> Result<(), BoxBoxStabilizationError3d> {
    match (left_body.kind, right_body.kind) {
        (BodyKind::Fixed, BodyKind::Fixed) => Ok(()),
        (BodyKind::Fixed, BodyKind::Dynamic) => {
            right.center = offset_position(right.center, correction)?;
            Ok(())
        }
        (BodyKind::Dynamic, BodyKind::Fixed) => {
            left.center = offset_position(left.center, negate_vector(correction)?)?;
            Ok(())
        }
        (BodyKind::Dynamic, BodyKind::Dynamic) => {
            let total_mass = u64::from(left_body.mass_units)
                .checked_add(u64::from(right_body.mass_units))
                .ok_or(BoxBoxStabilizationError3d::ArithmeticOverflow)?;
            let mut left_move = [0_i64; 3];
            let mut right_move = [0_i64; 3];
            for axis in 0..3 {
                let weighted = checked_mul(
                    i128::from(correction[axis]),
                    i128::from(right_body.mass_units),
                )?;
                let share = div_round_nearest(weighted, i128::from(total_mass))?;
                left_move[axis] = i64::try_from(
                    share
                        .checked_neg()
                        .ok_or(BoxBoxStabilizationError3d::ArithmeticOverflow)?,
                )
                .map_err(|_| BoxBoxStabilizationError3d::ArithmeticOverflow)?;
                right_move[axis] = correction[axis]
                    .checked_add(left_move[axis])
                    .ok_or(BoxBoxStabilizationError3d::ArithmeticOverflow)?;
            }
            left.center = offset_position(left.center, left_move)?;
            right.center = offset_position(right.center, right_move)?;
            Ok(())
        }
    }
}

fn negate_vector(vector: [i64; 3]) -> Result<[i64; 3], BoxBoxStabilizationError3d> {
    Ok([
        vector[0]
            .checked_neg()
            .ok_or(BoxBoxStabilizationError3d::ArithmeticOverflow)?,
        vector[1]
            .checked_neg()
            .ok_or(BoxBoxStabilizationError3d::ArithmeticOverflow)?,
        vector[2]
            .checked_neg()
            .ok_or(BoxBoxStabilizationError3d::ArithmeticOverflow)?,
    ])
}

fn offset_position(
    position: Position,
    delta: [i64; 3],
) -> Result<Position, BoxBoxStabilizationError3d> {
    Ok(Position::new3(
        position
            .x
            .checked_add(delta[0])
            .ok_or(BoxBoxStabilizationError3d::ArithmeticOverflow)?,
        position
            .y
            .checked_add(delta[1])
            .ok_or(BoxBoxStabilizationError3d::ArithmeticOverflow)?,
        position
            .z
            .checked_add(delta[2])
            .ok_or(BoxBoxStabilizationError3d::ArithmeticOverflow)?,
    ))
}

fn checked_mul(left: i128, right: i128) -> Result<i128, BoxBoxStabilizationError3d> {
    left.checked_mul(right)
        .ok_or(BoxBoxStabilizationError3d::ArithmeticOverflow)
}

fn checked_dot(left: [i128; 3], right: [i128; 3]) -> Result<i128, BoxBoxStabilizationError3d> {
    left.into_iter()
        .zip(right)
        .try_fold(0_i128, |sum, (left, right)| {
            sum.checked_add(checked_mul(left, right)?)
                .ok_or(BoxBoxStabilizationError3d::ArithmeticOverflow)
        })
}

fn div_round_nearest(
    numerator: i128,
    denominator: i128,
) -> Result<i128, BoxBoxStabilizationError3d> {
    if denominator <= 0 {
        return Err(BoxBoxStabilizationError3d::ArithmeticOverflow);
    }
    let half = denominator / 2;
    let adjusted = if numerator >= 0 {
        numerator.checked_add(half)
    } else {
        numerator.checked_sub(half)
    }
    .ok_or(BoxBoxStabilizationError3d::ArithmeticOverflow)?;
    Ok(adjusted / denominator)
}

fn div_ceil_positive(
    numerator: i128,
    denominator: i128,
) -> Result<i128, BoxBoxStabilizationError3d> {
    if numerator < 0 || denominator <= 0 {
        return Err(BoxBoxStabilizationError3d::ArithmeticOverflow);
    }
    numerator
        .checked_add(denominator - 1)
        .map(|value| value / denominator)
        .ok_or(BoxBoxStabilizationError3d::ArithmeticOverflow)
}

#[cfg(test)]
mod tests {
    use ecs_physics::PhysicsMaterial;
    use ecs_workload::{EntityId, Velocity};

    use crate::{
        AngularState3d, AngularVelocity3d, Orientation3d, OrientedBox3d, obb_contact_seed,
    };

    use super::*;

    fn state(center: Position, velocity: Velocity) -> RigidBoxState3d {
        RigidBoxState3d::new(
            center,
            velocity,
            AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
        )
    }

    fn dynamic(entity: u32) -> PhysicsBody3d {
        PhysicsBody3d::dynamic(EntityId(entity), [10, 10, 10])
            .with_material(PhysicsMaterial::new(0, 0))
    }

    fn remaining_overlap(
        left: RigidBoxState3d,
        left_body: PhysicsBody3d,
        right: RigidBoxState3d,
        right_body: PhysicsBody3d,
    ) -> u128 {
        let left = OrientedBox3d::new(
            left.center,
            left_body.half_extents,
            left.angular.orientation,
        );
        let right = OrientedBox3d::new(
            right.center,
            right_body.half_extents,
            right.angular.orientation,
        );
        obb_contact_seed(left, right)
            .expect("valid geometry")
            .map_or(0, |contact| contact.overlap_numerator)
    }

    #[test]
    fn fixed_wall_projection_closes_reported_penetration() {
        let left_body = dynamic(1);
        let right_body = PhysicsBody3d::fixed(EntityId(2), [10, 10, 10]);
        let left = state(Position::new3(0, 0, 0), Velocity::new3(30, 0, 0));
        let right = state(Position::new3(19, 0, 0), Velocity::new3(0, 0, 0));

        let step = stabilize_box_box_contact(left, left_body, right, right_body)
            .expect("valid stabilization");

        assert_eq!(
            remaining_overlap(step.left, left_body, step.right, right_body),
            0
        );
        assert_eq!(step.left.center.x, -1);
        assert_eq!(step.right.center, right.center);
    }

    #[test]
    fn equal_dynamic_masses_preserve_exact_total_separation() {
        let left_body = dynamic(1);
        let right_body = dynamic(2);
        let left = state(Position::new3(0, 0, 0), Velocity::new3(30, 0, 0));
        let right = state(Position::new3(18, 0, 0), Velocity::new3(0, 0, 0));

        let step = stabilize_box_box_contact(left, left_body, right, right_body)
            .expect("valid stabilization");

        assert_eq!(
            remaining_overlap(step.left, left_body, step.right, right_body),
            0
        );
        assert_eq!(step.right.center.x - step.left.center.x, 20);
    }

    #[test]
    fn off_center_projection_preserves_collision_generated_spin() {
        let left_body = dynamic(1);
        let right_body = dynamic(2);
        let left = state(Position::new3(0, 6, 0), Velocity::new3(90, 0, 0));
        let right = state(Position::new3(19, 0, 0), Velocity::new3(0, 0, 0));

        let step = stabilize_box_box_contact(left, left_body, right, right_body)
            .expect("valid stabilization");

        assert_ne!(step.left.angular.angular_velocity.z, 0);
        assert_ne!(step.right.angular.angular_velocity.z, 0);
        assert_eq!(
            remaining_overlap(step.left, left_body, step.right, right_body),
            0
        );
    }

    #[test]
    fn non_canonical_pair_order_fails_closed() {
        let error = stabilize_box_box_contact(
            state(Position::new3(0, 0, 0), Velocity::new3(0, 0, 0)),
            dynamic(2),
            state(Position::new3(19, 0, 0), Velocity::new3(0, 0, 0)),
            dynamic(1),
        )
        .expect_err("pair order should be explicit");

        assert_eq!(
            error,
            BoxBoxStabilizationError3d::NonCanonicalPair(EntityId(2), EntityId(1))
        );
    }
}
