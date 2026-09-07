use std::fmt;

use ecs_physics::{BodyKind, MATERIAL_SCALE};

use crate::{
    RigidBox3d, RigidBoxWorldConfig3d, RigidBoxWorldError3d, RigidBoxWorldStats3d,
    RigidBoxWorldStep3d, rigid_box_world::step_rigid_box_world as step_rigid_box_world_once,
};

/// Maximum bounded angular substeps accepted by this approximation layer.
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
    RequiredSubstepsExceeded { required: u128, maximum: u8 },
    ArithmeticOverflow,
    World(RigidBoxWorldError3d),
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
            Self::RequiredSubstepsExceeded { required, maximum } => write!(
                formatter,
                "angular motion requires {required} substeps but the configured cap is {maximum}"
            ),
            Self::ArithmeticOverflow => write!(formatter, "angular substep arithmetic overflowed"),
            Self::World(error) => write!(formatter, "angular substep world step failed: {error}"),
        }
    }
}

impl std::error::Error for AngularSubstepError3d {}

impl From<RigidBoxWorldError3d> for AngularSubstepError3d {
    fn from(value: RigidBoxWorldError3d) -> Self {
        Self::World(value)
    }
}

/// Returns the deterministic equal-substep count required by `policy` for one requested frame.
///
/// The bound uses the L1 norm of dynamic angular velocity as a conservative, square-root-free upper
/// bound on angular speed. The calculation stays in exact integer/rational units and therefore does not
/// make platform floating point part of collision timing. Slow or non-rotating scenes remain one step.
///
/// This is a bounded discrete approximation policy, not rotational CCD. A substep count greater than one
/// increases the number of orientation/contact samples inside the frame but does not solve an analytic
/// earliest time of impact.
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
        return Err(
            RigidBoxWorldError3d::NegativeTimestepNumerator(config.timestep_numerator).into(),
        );
    }
    if config.timestep_denominator <= 0 {
        return Err(RigidBoxWorldError3d::NonPositiveTimestepDenominator(
            config.timestep_denominator,
        )
        .into());
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

/// Advances one requested rigid-box frame through a bounded number of deterministic angular substeps.
///
/// Each substep uses an equal rational fraction of the original timestep. Angular damping is deliberately
/// disabled on intermediate samples and applied only on the final sample, preserving its existing
/// once-per-requested-frame meaning. Candidate/contact counters accumulate across samples while
/// `spinning_bodies` reports the final state only.
///
/// The ordinary [`crate::step_rigid_box_world`] API remains available and unchanged. Callers opt into
/// this approximation explicitly until the rotational-CCD horizon has enough evidence to justify making
/// it the canonical stepping policy.
///
/// # Errors
///
/// Returns [`AngularSubstepError3d`] when the policy cannot represent the requested angular motion, the
/// rational substep denominator overflows, accumulated evidence overflows, or an underlying world step
/// fails.
pub fn step_rigid_box_world_substepped(
    boxes: &[RigidBox3d],
    config: RigidBoxWorldConfig3d,
    policy: AngularSubstepPolicy3d,
) -> Result<RigidBoxWorldStep3d, AngularSubstepError3d> {
    let substeps = required_angular_substeps(boxes, config, policy)?;
    if substeps == 1 {
        return step_rigid_box_world_once(boxes, config).map_err(Into::into);
    }

    let substep_denominator = config
        .timestep_denominator
        .checked_mul(i32::from(substeps))
        .ok_or(AngularSubstepError3d::ArithmeticOverflow)?;
    let mut current = boxes.to_vec();
    let mut accumulated = RigidBoxWorldStats3d::default();

    for index in 0..substeps {
        let final_substep = index + 1 == substeps;
        let mut substep_config = config;
        substep_config.timestep_denominator = substep_denominator;
        if !final_substep {
            substep_config.angular_damping_milli = MATERIAL_SCALE;
        }

        let step = step_rigid_box_world_once(&current, substep_config)?;
        accumulated.candidate_pairs = accumulated
            .candidate_pairs
            .checked_add(step.stats.candidate_pairs)
            .ok_or(AngularSubstepError3d::ArithmeticOverflow)?;
        accumulated.contacts = accumulated
            .contacts
            .checked_add(step.stats.contacts)
            .ok_or(AngularSubstepError3d::ArithmeticOverflow)?;
        accumulated.impulsive_contacts = accumulated
            .impulsive_contacts
            .checked_add(step.stats.impulsive_contacts)
            .ok_or(AngularSubstepError3d::ArithmeticOverflow)?;
        if final_substep {
            accumulated.spinning_bodies = step.stats.spinning_bodies;
        }
        current = step.boxes;
    }

    Ok(RigidBoxWorldStep3d {
        boxes: current,
        stats: accumulated,
    })
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

    fn dynamic_box(
        entity: u32,
        half_extents: [i32; 3],
        center: Position,
        angular_velocity: AngularVelocity3d,
    ) -> RigidBox3d {
        RigidBox3d::new(
            PhysicsBody3d::dynamic(EntityId(entity), half_extents),
            RigidBoxState3d::new(
                center,
                Velocity::new3(0, 0, 0),
                AngularState3d::new(Orientation3d::IDENTITY, angular_velocity),
            ),
        )
    }

    #[test]
    fn slow_spin_matches_the_existing_single_step_bit_for_bit() {
        let boxes = [dynamic_box(
            1,
            [10, 10, 10],
            Position::new3(0, 20, 0),
            AngularVelocity3d::new(10_000, 20_000, -10_000),
        )];
        let expected =
            step_rigid_box_world_once(&boxes, config()).expect("valid direct world step");
        let actual =
            step_rigid_box_world_substepped(&boxes, config(), AngularSubstepPolicy3d::default())
                .expect("slow spin should remain one step");

        assert_eq!(
            required_angular_substeps(&boxes, config(), AngularSubstepPolicy3d::default())
                .expect("valid policy"),
            1
        );
        assert_eq!(actual, expected);
    }

    #[test]
    fn high_spin_uses_multiple_equal_substeps() {
        let boxes = [dynamic_box(
            1,
            [10, 10, 10],
            Position::new3(0, 20, 0),
            AngularVelocity3d::new(0, 0, 12 * ANGULAR_VELOCITY_SCALE),
        )];
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
        let boxes = [dynamic_box(
            1,
            [10, 10, 10],
            Position::new3(0, 20, 0),
            AngularVelocity3d::new(0, 0, 60 * ANGULAR_VELOCITY_SCALE),
        )];
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
    fn substeps_observe_a_rotational_contact_missed_by_the_single_endpoint() {
        let boxes = [
            dynamic_box(
                1,
                [20, 2, 2],
                Position::new3(0, 0, 0),
                AngularVelocity3d::new(0, 0, 230 * ANGULAR_VELOCITY_SCALE),
            ),
            RigidBox3d::new(
                PhysicsBody3d::fixed(EntityId(2), [2, 2, 2]),
                RigidBoxState3d::new(
                    Position::new3(10, 10, 0),
                    Velocity::new3(0, 0, 0),
                    AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
                ),
            ),
        ];
        let direct = step_rigid_box_world_once(&boxes, config()).expect("valid direct world step");
        let substepped = step_rigid_box_world_substepped(
            &boxes,
            config(),
            AngularSubstepPolicy3d {
                max_angular_step_units: 800_000,
                max_substeps: 8,
            },
        )
        .expect("bounded intermediate rotation samples");

        assert_eq!(direct.stats.contacts, 0);
        assert!(substepped.stats.contacts > 0);
        assert_ne!(
            substepped.boxes[0].state.center,
            direct.boxes[0].state.center
        );
    }
}
