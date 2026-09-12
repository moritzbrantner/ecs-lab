use std::fmt;

use ecs_physics::BodyKind;

use crate::{RigidBox3d, RigidBoxWorldConfig3d};

/// Maximum bounded angular substeps accepted by the ECS-to-engine adapter policy.
pub const MAX_ANGULAR_SUBSTEPS: u8 = 16;
/// Default maximum conservative angular displacement per substep, in `1e-6` radians.
pub const DEFAULT_MAX_ANGULAR_STEP_UNITS: u32 = 125_000;
/// Default cap keeps the approximation bounded even for pathological spin.
pub const DEFAULT_MAX_ANGULAR_SUBSTEPS: u8 = 8;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct AngularSubstepPolicy3d {
    /// Maximum L1 angular displacement per substep in [`crate::ANGULAR_VELOCITY_SCALE`] units of one radian.
    pub max_angular_step_units: u32,
    /// Maximum number of equal rational substeps allowed for one requested world frame.
    pub max_substeps: u8,
}

impl Default for AngularSubstepPolicy3d {
    fn default() -> Self {
        Self {
            max_angular_step_units: DEFAULT_MAX_ANGULAR_STEP_UNITS,
            max_substeps: DEFAULT_MAX_ANGULAR_SUBSTEPS,
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum AngularSubstepError3d {
    ZeroAngularStep,
    SubstepCapOutOfRange(u8),
    NegativeTimestepNumerator(i32),
    NonPositiveTimestepDenominator(i32),
    RequiredSubstepsExceeded { required: u128, maximum: u8 },
    ArithmeticOverflow,
}

impl fmt::Display for AngularSubstepError3d {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::ZeroAngularStep => write!(
                formatter,
                "angular substep policy requires a non-zero maximum angular displacement"
            ),
            Self::SubstepCapOutOfRange(value) => write!(
                formatter,
                "angular substep cap must be 1..={MAX_ANGULAR_SUBSTEPS}, got {value}"
            ),
            Self::NegativeTimestepNumerator(value) => write!(
                formatter,
                "angular substep timestep numerator must be non-negative, got {value}"
            ),
            Self::NonPositiveTimestepDenominator(value) => write!(
                formatter,
                "angular substep timestep denominator must be positive, got {value}"
            ),
            Self::RequiredSubstepsExceeded { required, maximum } => write!(
                formatter,
                "angular motion requires {required} substeps but the configured cap is {maximum}"
            ),
            Self::ArithmeticOverflow => write!(formatter, "angular substep arithmetic overflowed"),
        }
    }
}

impl std::error::Error for AngularSubstepError3d {}

/// Returns the deterministic equal-substep count required by `policy` for one requested frame.
///
/// ECS Lab retains this count because frame-level damping is consumer policy. Every resulting substep is
/// executed by `physics-engine`; this function does not integrate motion or resolve contacts.
///
/// # Errors
///
/// Returns [`AngularSubstepError3d`] for malformed policy/timestep values, arithmetic overflow, or motion
/// that would require more than the configured bounded substep cap.
pub fn required_angular_substeps(
    boxes: &[RigidBox3d],
    config: RigidBoxWorldConfig3d,
    policy: AngularSubstepPolicy3d,
) -> Result<u8, AngularSubstepError3d> {
    validate_policy(policy)?;
    if config.timestep_numerator < 0 {
        return Err(AngularSubstepError3d::NegativeTimestepNumerator(
            config.timestep_numerator,
        ));
    }
    if config.timestep_denominator <= 0 {
        return Err(AngularSubstepError3d::NonPositiveTimestepDenominator(
            config.timestep_denominator,
        ));
    }

    let maximum_speed = boxes
        .iter()
        .filter(|rigid_box| rigid_box.body.kind == BodyKind::Dynamic)
        .try_fold(0_u128, |maximum, rigid_box| {
            angular_l1_units(rigid_box)
                .map(|speed| maximum.max(speed))
                .ok_or(AngularSubstepError3d::ArithmeticOverflow)
        })?;
    if maximum_speed == 0 || config.timestep_numerator == 0 {
        return Ok(1);
    }

    let timestep_numerator = u128::from(config.timestep_numerator.unsigned_abs());
    let timestep_denominator = u128::from(
        u32::try_from(config.timestep_denominator)
            .map_err(|_| AngularSubstepError3d::ArithmeticOverflow)?,
    );
    let numerator = maximum_speed
        .checked_mul(timestep_numerator)
        .ok_or(AngularSubstepError3d::ArithmeticOverflow)?;
    let denominator = timestep_denominator
        .checked_mul(u128::from(policy.max_angular_step_units))
        .ok_or(AngularSubstepError3d::ArithmeticOverflow)?;
    let required = numerator.div_ceil(denominator).max(1);
    if required > u128::from(policy.max_substeps) {
        return Err(AngularSubstepError3d::RequiredSubstepsExceeded {
            required,
            maximum: policy.max_substeps,
        });
    }
    u8::try_from(required).map_err(|_| AngularSubstepError3d::ArithmeticOverflow)
}

fn validate_policy(policy: AngularSubstepPolicy3d) -> Result<(), AngularSubstepError3d> {
    if policy.max_angular_step_units == 0 {
        return Err(AngularSubstepError3d::ZeroAngularStep);
    }
    if policy.max_substeps == 0 || policy.max_substeps > MAX_ANGULAR_SUBSTEPS {
        return Err(AngularSubstepError3d::SubstepCapOutOfRange(
            policy.max_substeps,
        ));
    }
    Ok(())
}

fn angular_l1_units(rigid_box: &RigidBox3d) -> Option<u128> {
    let velocity = rigid_box.state.angular.angular_velocity;
    u128::from(velocity.x.unsigned_abs())
        .checked_add(u128::from(velocity.y.unsigned_abs()))?
        .checked_add(u128::from(velocity.z.unsigned_abs()))
}

#[cfg(test)]
mod tests {
    use ecs_physics::MATERIAL_SCALE;
    use ecs_workload::{EntityId, Position, Velocity};

    use crate::{
        ANGULAR_VELOCITY_SCALE, AngularState3d, AngularVelocity3d, Orientation3d, PhysicsBody3d,
        RigidBoxState3d,
    };

    use super::*;

    fn config() -> RigidBoxWorldConfig3d {
        RigidBoxWorldConfig3d {
            gravity: Velocity::new3(0, 0, 0),
            timestep_numerator: 1,
            timestep_denominator: 60,
            angular_damping_milli: MATERIAL_SCALE,
            solver_passes: 8,
        }
    }

    fn dynamic_box(angular_velocity: AngularVelocity3d) -> RigidBox3d {
        RigidBox3d::new(
            PhysicsBody3d::dynamic(EntityId(1), [10, 10, 10]),
            RigidBoxState3d::new(
                Position::new3(0, 20, 0),
                Velocity::new3(0, 0, 0),
                AngularState3d::new(Orientation3d::IDENTITY, angular_velocity),
            ),
        )
    }

    #[test]
    fn slow_spin_remains_one_engine_substep() {
        let boxes = [dynamic_box(AngularVelocity3d::new(10_000, 20_000, -10_000))];

        assert_eq!(
            required_angular_substeps(&boxes, config(), AngularSubstepPolicy3d::default())
                .expect("valid policy"),
            1
        );
    }

    #[test]
    fn high_spin_uses_multiple_equal_engine_substeps() {
        let boxes = [dynamic_box(AngularVelocity3d::new(
            0,
            0,
            12 * ANGULAR_VELOCITY_SCALE,
        ))];
        let policy = AngularSubstepPolicy3d {
            max_angular_step_units: 50_000,
            max_substeps: 8,
        };

        assert_eq!(
            required_angular_substeps(&boxes, config(), policy).expect("bounded high spin"),
            4
        );
    }

    #[test]
    fn excessive_spin_fails_closed_at_the_configured_cap() {
        let boxes = [dynamic_box(AngularVelocity3d::new(
            0,
            0,
            60 * ANGULAR_VELOCITY_SCALE,
        ))];
        let policy = AngularSubstepPolicy3d {
            max_angular_step_units: 50_000,
            max_substeps: 8,
        };

        assert_eq!(
            required_angular_substeps(&boxes, config(), policy),
            Err(AngularSubstepError3d::RequiredSubstepsExceeded {
                required: 20,
                maximum: 8,
            })
        );
    }

    #[test]
    fn malformed_timestep_is_rejected_before_engine_dispatch() {
        let boxes = [dynamic_box(AngularVelocity3d::default())];
        let mut invalid = config();
        invalid.timestep_denominator = 0;

        assert_eq!(
            required_angular_substeps(&boxes, invalid, AngularSubstepPolicy3d::default()),
            Err(AngularSubstepError3d::NonPositiveTimestepDenominator(0))
        );
    }
}
