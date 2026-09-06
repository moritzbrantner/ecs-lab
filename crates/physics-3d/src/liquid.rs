use ecs_physics::{BodyKind, MATERIAL_SCALE};
use ecs_workload::{EntityId, Operation, Position, Velocity, WorldSnapshot};

use crate::PhysicsBody3d;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct LiquidVolume3d {
    pub center: Position,
    pub half_extents: [i32; 3],
    /// Full-submersion buoyancy ratio in thousandths before dividing by body mass units.
    /// `1000` exactly counteracts the configured gravity for a mass-1 body.
    pub density_milli: u16,
    /// Full-submersion linear drag in thousandths.
    pub drag_milli: u16,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum LiquidError3d {
    NegativeVolumeHalfExtent,
    InvalidDensity(u16),
    InvalidDrag(u16),
    MissingEntity(EntityId),
    MissingPosition(EntityId),
    MissingVelocity(EntityId),
    InvalidBodyVolume(EntityId),
    ZeroMass(EntityId),
    ArithmeticOverflow,
}

/// Computes one deterministic buoyancy/drag response over a bounded AABB liquid volume.
///
/// The returned velocity mutations are ordinary ECS operations. Apply them before the next physics step:
/// the buoyancy term then opposes that step's configured gravity, while drag damps existing motion in
/// proportion to exact integer submerged volume. This is rigid-body↔volume interaction, not a fluid
/// solver; the liquid has no particles, pressure field, or body-to-body flow state.
///
/// # Errors
///
/// Returns [`LiquidError3d`] for malformed volume/body data, missing ECS components, or arithmetic
/// overflow.
pub fn liquid_operations(
    snapshot: &WorldSnapshot,
    bodies: &[PhysicsBody3d],
    volume: LiquidVolume3d,
    gravity: Velocity,
) -> Result<Vec<Operation>, LiquidError3d> {
    validate_volume(volume)?;
    let mut operations = Vec::new();

    for body in bodies {
        if body.kind == BodyKind::Fixed {
            continue;
        }
        if body.mass_units == 0 {
            return Err(LiquidError3d::ZeroMass(body.entity));
        }
        let state = snapshot
            .entities()
            .iter()
            .find(|candidate| candidate.id == body.entity)
            .ok_or(LiquidError3d::MissingEntity(body.entity))?;
        let position = state
            .position
            .ok_or(LiquidError3d::MissingPosition(body.entity))?;
        let velocity = state
            .velocity
            .ok_or(LiquidError3d::MissingVelocity(body.entity))?;
        let body_volume =
            aabb_volume(body.half_extents).ok_or(LiquidError3d::InvalidBodyVolume(body.entity))?;
        if body_volume == 0 {
            return Err(LiquidError3d::InvalidBodyVolume(body.entity));
        }
        let submerged = intersection_volume(
            position,
            body.half_extents,
            volume.center,
            volume.half_extents,
        )?;
        if submerged == 0 {
            continue;
        }

        let drag_numerator = i128::from(volume.drag_milli)
            .checked_mul(submerged)
            .ok_or(LiquidError3d::ArithmeticOverflow)?;
        let drag_denominator = i128::from(MATERIAL_SCALE)
            .checked_mul(body_volume)
            .ok_or(LiquidError3d::ArithmeticOverflow)?;
        let retained_numerator = drag_denominator.saturating_sub(drag_numerator);
        let dragged = Velocity::new3(
            scale_velocity(velocity.x, retained_numerator, drag_denominator)?,
            scale_velocity(velocity.y, retained_numerator, drag_denominator)?,
            scale_velocity(velocity.z, retained_numerator, drag_denominator)?,
        );

        let buoyancy_denominator = i128::from(MATERIAL_SCALE)
            .checked_mul(body_volume)
            .and_then(|value| value.checked_mul(i128::from(body.mass_units)))
            .ok_or(LiquidError3d::ArithmeticOverflow)?;
        let buoyancy_numerator = i128::from(volume.density_milli)
            .checked_mul(submerged)
            .ok_or(LiquidError3d::ArithmeticOverflow)?;
        let buoyancy = Velocity::new3(
            opposite_gravity_delta(gravity.x, buoyancy_numerator, buoyancy_denominator)?,
            opposite_gravity_delta(gravity.y, buoyancy_numerator, buoyancy_denominator)?,
            opposite_gravity_delta(gravity.z, buoyancy_numerator, buoyancy_denominator)?,
        );
        let next = Velocity::new3(
            dragged.x.saturating_add(buoyancy.x),
            dragged.y.saturating_add(buoyancy.y),
            dragged.z.saturating_add(buoyancy.z),
        );
        if next != velocity {
            operations.push(Operation::SetVelocity(body.entity, next));
        }
    }
    Ok(operations)
}

fn validate_volume(volume: LiquidVolume3d) -> Result<(), LiquidError3d> {
    if volume.half_extents.iter().any(|extent| *extent < 0) {
        return Err(LiquidError3d::NegativeVolumeHalfExtent);
    }
    if volume.density_milli > MATERIAL_SCALE {
        return Err(LiquidError3d::InvalidDensity(volume.density_milli));
    }
    if volume.drag_milli > MATERIAL_SCALE {
        return Err(LiquidError3d::InvalidDrag(volume.drag_milli));
    }
    Ok(())
}

fn aabb_volume(half_extents: [i32; 3]) -> Option<i128> {
    if half_extents.iter().any(|extent| *extent <= 0) {
        return None;
    }
    half_extents.into_iter().try_fold(1_i128, |volume, half| {
        volume.checked_mul(i128::from(half).checked_mul(2)?)
    })
}

fn intersection_volume(
    left_center: Position,
    left_half: [i32; 3],
    right_center: Position,
    right_half: [i32; 3],
) -> Result<i128, LiquidError3d> {
    if left_half.iter().any(|extent| *extent < 0) {
        return Err(LiquidError3d::ArithmeticOverflow);
    }
    let left_axes = [left_center.x, left_center.y, left_center.z];
    let right_axes = [right_center.x, right_center.y, right_center.z];
    let mut volume = 1_i128;
    for axis in 0..3 {
        let left_min = i128::from(left_axes[axis]) - i128::from(left_half[axis]);
        let left_max = i128::from(left_axes[axis]) + i128::from(left_half[axis]);
        let right_min = i128::from(right_axes[axis]) - i128::from(right_half[axis]);
        let right_max = i128::from(right_axes[axis]) + i128::from(right_half[axis]);
        let width = left_max
            .min(right_max)
            .saturating_sub(left_min.max(right_min));
        if width <= 0 {
            return Ok(0);
        }
        volume = volume
            .checked_mul(width)
            .ok_or(LiquidError3d::ArithmeticOverflow)?;
    }
    Ok(volume)
}

fn scale_velocity(velocity: i32, numerator: i128, denominator: i128) -> Result<i32, LiquidError3d> {
    let scaled = i128::from(velocity)
        .checked_mul(numerator)
        .ok_or(LiquidError3d::ArithmeticOverflow)?
        / denominator;
    i32::try_from(scaled).map_err(|_| LiquidError3d::ArithmeticOverflow)
}

fn opposite_gravity_delta(
    gravity: i32,
    numerator: i128,
    denominator: i128,
) -> Result<i32, LiquidError3d> {
    let scaled = i128::from(gravity)
        .checked_neg()
        .and_then(|value| value.checked_mul(numerator))
        .ok_or(LiquidError3d::ArithmeticOverflow)?
        / denominator;
    i32::try_from(scaled).map_err(|_| LiquidError3d::ArithmeticOverflow)
}

#[cfg(test)]
mod tests {
    use ecs_workload::EntitySnapshot;

    use super::*;

    const BODY: EntityId = EntityId(1);

    fn state(position: Position, velocity: Velocity) -> WorldSnapshot {
        WorldSnapshot::new(vec![EntitySnapshot {
            id: BODY,
            position: Some(position),
            velocity: Some(velocity),
        }])
    }

    fn body(mass_units: u32) -> PhysicsBody3d {
        PhysicsBody3d::dynamic(BODY, [1, 1, 1]).with_mass(mass_units)
    }

    fn water(drag_milli: u16) -> LiquidVolume3d {
        LiquidVolume3d {
            center: Position::new3(0, 0, 0),
            half_extents: [10, 10, 10],
            density_milli: 1_000,
            drag_milli,
        }
    }

    #[test]
    fn mass_one_full_submersion_cancels_one_gravity_step() {
        let operations = liquid_operations(
            &state(Position::new3(0, 0, 0), Velocity::new3(0, 0, 0)),
            &[body(1)],
            water(0),
            Velocity::new3(0, -10, 0),
        )
        .expect("valid liquid response");
        assert_eq!(
            operations,
            vec![Operation::SetVelocity(BODY, Velocity::new3(0, 10, 0))]
        );
    }

    #[test]
    fn heavier_body_receives_less_buoyancy() {
        let operations = liquid_operations(
            &state(Position::new3(0, 0, 0), Velocity::new3(0, 0, 0)),
            &[body(2)],
            water(0),
            Velocity::new3(0, -10, 0),
        )
        .expect("valid liquid response");
        assert_eq!(
            operations,
            vec![Operation::SetVelocity(BODY, Velocity::new3(0, 5, 0))]
        );
    }

    #[test]
    fn full_drag_stops_horizontal_motion_when_submerged() {
        let operations = liquid_operations(
            &state(Position::new3(0, 0, 0), Velocity::new3(12, 0, 0)),
            &[body(1)],
            water(1_000),
            Velocity::new3(0, 0, 0),
        )
        .expect("valid liquid response");
        assert_eq!(
            operations,
            vec![Operation::SetVelocity(BODY, Velocity::new3(0, 0, 0))]
        );
    }

    #[test]
    fn body_outside_volume_is_unchanged() {
        let operations = liquid_operations(
            &state(Position::new3(40, 0, 0), Velocity::new3(5, -2, 1)),
            &[body(1)],
            water(1_000),
            Velocity::new3(0, -10, 0),
        )
        .expect("valid liquid response");
        assert!(operations.is_empty());
    }

    #[test]
    fn half_submersion_scales_buoyancy_and_drag() {
        let volume = LiquidVolume3d {
            center: Position::new3(0, -1, 0),
            half_extents: [10, 1, 10],
            density_milli: 1_000,
            drag_milli: 500,
        };
        let operations = liquid_operations(
            &state(Position::new3(0, 0, 0), Velocity::new3(8, 0, 0)),
            &[body(1)],
            volume,
            Velocity::new3(0, -10, 0),
        )
        .expect("valid partial liquid response");
        assert_eq!(
            operations,
            vec![Operation::SetVelocity(BODY, Velocity::new3(6, 5, 0))]
        );
    }
}
