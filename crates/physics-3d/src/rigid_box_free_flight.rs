use std::fmt;

use ecs_physics::BodyKind;
use ecs_workload::{Position, Velocity};

use crate::{AngularError3d, AngularState3d, RigidBox3d, RigidBoxState3d, integrate_orientation};

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct RigidBoxFreeFlightConfig3d {
    pub gravity: Velocity,
    pub timestep_numerator: i32,
    pub timestep_denominator: i32,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RigidBoxFreeFlightError3d {
    NegativeTimestepNumerator(i32),
    NonPositiveTimestepDenominator(i32),
    InvalidFraction { numerator: u32, denominator: u32 },
    Angular(AngularError3d),
    ArithmeticOverflow,
}

impl fmt::Display for RigidBoxFreeFlightError3d {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NegativeTimestepNumerator(value) => write!(
                formatter,
                "rigid-box free-flight timestep numerator must be non-negative, got {value}"
            ),
            Self::NonPositiveTimestepDenominator(value) => write!(
                formatter,
                "rigid-box free-flight timestep denominator must be positive, got {value}"
            ),
            Self::InvalidFraction {
                numerator,
                denominator,
            } => write!(
                formatter,
                "rigid-box free-flight fraction must satisfy 0 <= numerator <= denominator with denominator > 0, got {numerator}/{denominator}"
            ),
            Self::Angular(error) => write!(
                formatter,
                "rigid-box free-flight angular sampling failed: {error}"
            ),
            Self::ArithmeticOverflow => {
                write!(formatter, "rigid-box free-flight arithmetic overflowed")
            }
        }
    }
}

impl std::error::Error for RigidBoxFreeFlightError3d {}

impl From<AngularError3d> for RigidBoxFreeFlightError3d {
    fn from(value: AngularError3d) -> Self {
        Self::Angular(value)
    }
}

/// Samples one rigid cuboid directly from one segment-start state at an exact rational fraction.
///
/// The sampler owns free-flight arithmetic only: gravity updates linear velocity, the updated velocity
/// advances position with the same semi-implicit integer rule used by the rigid-box world, and angular
/// velocity advances orientation through [`integrate_orientation`]. Fixed bodies are returned unchanged.
///
/// Angular damping, contact response, stabilization, broad phase, and event iteration are deliberately
/// absent. Keeping those policies out of this primitive lets a future sampled-contact replay consume
/// several collision events inside one requested frame while still applying damping exactly once at the
/// frame boundary.
///
/// Every sample is evaluated directly from the supplied segment-start state rather than by chaining
/// smaller samples. This makes the rounding policy explicit and repeatable for contact search, frontier
/// reconstruction, and remaining-frame replay.
///
/// # Errors
///
/// Returns [`RigidBoxFreeFlightError3d`] for malformed timestep/fraction inputs, angular integration
/// failure, or checked integer arithmetic overflow.
pub fn sample_rigid_box_free_flight(
    mut body: RigidBox3d,
    config: RigidBoxFreeFlightConfig3d,
    fraction_numerator: u32,
    fraction_denominator: u32,
) -> Result<RigidBox3d, RigidBoxFreeFlightError3d> {
    validate_config(config, fraction_numerator, fraction_denominator)?;
    if body.body.kind == BodyKind::Fixed || fraction_numerator == 0 {
        return Ok(body);
    }

    let fraction_numerator = i32::try_from(fraction_numerator)
        .map_err(|_| RigidBoxFreeFlightError3d::ArithmeticOverflow)?;
    let fraction_denominator = i32::try_from(fraction_denominator)
        .map_err(|_| RigidBoxFreeFlightError3d::ArithmeticOverflow)?;
    let timestep_numerator = config
        .timestep_numerator
        .checked_mul(fraction_numerator)
        .ok_or(RigidBoxFreeFlightError3d::ArithmeticOverflow)?;
    let timestep_denominator = config
        .timestep_denominator
        .checked_mul(fraction_denominator)
        .ok_or(RigidBoxFreeFlightError3d::ArithmeticOverflow)?;

    let velocity = Velocity::new3(
        integrate_velocity_axis(
            body.state.linear_velocity.x,
            config.gravity.x,
            timestep_numerator,
            timestep_denominator,
        )?,
        integrate_velocity_axis(
            body.state.linear_velocity.y,
            config.gravity.y,
            timestep_numerator,
            timestep_denominator,
        )?,
        integrate_velocity_axis(
            body.state.linear_velocity.z,
            config.gravity.z,
            timestep_numerator,
            timestep_denominator,
        )?,
    );
    let center = Position::new3(
        integrate_position_axis(
            body.state.center.x,
            velocity.x,
            timestep_numerator,
            timestep_denominator,
        )?,
        integrate_position_axis(
            body.state.center.y,
            velocity.y,
            timestep_numerator,
            timestep_denominator,
        )?,
        integrate_position_axis(
            body.state.center.z,
            velocity.z,
            timestep_numerator,
            timestep_denominator,
        )?,
    );
    let orientation = integrate_orientation(
        body.state.angular.orientation,
        body.state.angular.angular_velocity,
        timestep_numerator,
        timestep_denominator,
    )?;
    body.state = RigidBoxState3d::new(
        center,
        velocity,
        AngularState3d::new(orientation, body.state.angular.angular_velocity),
    );
    Ok(body)
}

/// Samples an ordered rigid-box world with the same direct rational free-flight rule for every body.
///
/// Input order is preserved exactly. No collision discovery or response is performed.
///
/// # Errors
///
/// Returns the first [`RigidBoxFreeFlightError3d`] produced while sampling the world.
pub fn sample_rigid_box_world_free_flight(
    boxes: &[RigidBox3d],
    config: RigidBoxFreeFlightConfig3d,
    fraction_numerator: u32,
    fraction_denominator: u32,
) -> Result<Vec<RigidBox3d>, RigidBoxFreeFlightError3d> {
    validate_config(config, fraction_numerator, fraction_denominator)?;
    boxes
        .iter()
        .copied()
        .map(|body| {
            sample_rigid_box_free_flight(body, config, fraction_numerator, fraction_denominator)
        })
        .collect()
}

fn validate_config(
    config: RigidBoxFreeFlightConfig3d,
    fraction_numerator: u32,
    fraction_denominator: u32,
) -> Result<(), RigidBoxFreeFlightError3d> {
    if config.timestep_numerator < 0 {
        return Err(RigidBoxFreeFlightError3d::NegativeTimestepNumerator(
            config.timestep_numerator,
        ));
    }
    if config.timestep_denominator <= 0 {
        return Err(RigidBoxFreeFlightError3d::NonPositiveTimestepDenominator(
            config.timestep_denominator,
        ));
    }
    if fraction_denominator == 0 || fraction_numerator > fraction_denominator {
        return Err(RigidBoxFreeFlightError3d::InvalidFraction {
            numerator: fraction_numerator,
            denominator: fraction_denominator,
        });
    }
    Ok(())
}

fn integrate_velocity_axis(
    velocity: i32,
    acceleration: i32,
    numerator: i32,
    denominator: i32,
) -> Result<i32, RigidBoxFreeFlightError3d> {
    let acceleration_step = i128::from(acceleration)
        .checked_mul(i128::from(numerator))
        .ok_or(RigidBoxFreeFlightError3d::ArithmeticOverflow)?;
    let delta = div_round_nearest(acceleration_step, i128::from(denominator))?;
    let next = i128::from(velocity)
        .checked_add(delta)
        .ok_or(RigidBoxFreeFlightError3d::ArithmeticOverflow)?;
    i32::try_from(next).map_err(|_| RigidBoxFreeFlightError3d::ArithmeticOverflow)
}

fn integrate_position_axis(
    position: i64,
    velocity: i32,
    numerator: i32,
    denominator: i32,
) -> Result<i64, RigidBoxFreeFlightError3d> {
    let velocity_step = i128::from(velocity)
        .checked_mul(i128::from(numerator))
        .ok_or(RigidBoxFreeFlightError3d::ArithmeticOverflow)?;
    let delta = div_round_nearest(velocity_step, i128::from(denominator))?;
    let next = i128::from(position)
        .checked_add(delta)
        .ok_or(RigidBoxFreeFlightError3d::ArithmeticOverflow)?;
    i64::try_from(next).map_err(|_| RigidBoxFreeFlightError3d::ArithmeticOverflow)
}

fn div_round_nearest(
    numerator: i128,
    denominator: i128,
) -> Result<i128, RigidBoxFreeFlightError3d> {
    if denominator <= 0 {
        return Err(RigidBoxFreeFlightError3d::ArithmeticOverflow);
    }
    let half = denominator / 2;
    let adjusted = if numerator >= 0 {
        numerator.checked_add(half)
    } else {
        numerator.checked_sub(half)
    }
    .ok_or(RigidBoxFreeFlightError3d::ArithmeticOverflow)?;
    Ok(adjusted / denominator)
}

#[cfg(test)]
mod tests {
    use ecs_physics::MATERIAL_SCALE;
    use ecs_workload::{EntityId, Position};

    use crate::{
        ANGULAR_VELOCITY_SCALE, AngularVelocity3d, Orientation3d, PhysicsBody3d,
        RigidBoxWorldConfig3d, step_rigid_box_world,
    };

    use super::*;

    fn config() -> RigidBoxFreeFlightConfig3d {
        RigidBoxFreeFlightConfig3d {
            gravity: Velocity::new3(0, -3, 1),
            timestep_numerator: 1,
            timestep_denominator: 60,
        }
    }

    fn dynamic_body() -> RigidBox3d {
        RigidBox3d::new(
            PhysicsBody3d::dynamic(EntityId(1), [5, 6, 7]),
            RigidBoxState3d::new(
                Position::new3(10, 20, -30),
                Velocity::new3(120, -45, 11),
                AngularState3d::new(
                    Orientation3d::IDENTITY,
                    AngularVelocity3d::new(
                        2 * ANGULAR_VELOCITY_SCALE,
                        -3 * ANGULAR_VELOCITY_SCALE,
                        4 * ANGULAR_VELOCITY_SCALE,
                    ),
                ),
            ),
        )
    }

    #[test]
    fn full_fraction_matches_isolated_rigid_box_world_without_damping_loss() {
        let body = dynamic_body();
        let sampled = sample_rigid_box_free_flight(body, config(), 1, 1)
            .expect("valid direct full-frame sample");
        let stepped = step_rigid_box_world(
            &[body],
            RigidBoxWorldConfig3d {
                gravity: config().gravity,
                timestep_numerator: config().timestep_numerator,
                timestep_denominator: config().timestep_denominator,
                angular_damping_milli: MATERIAL_SCALE,
                solver_passes: 1,
            },
        )
        .expect("valid isolated rigid-box step");

        assert_eq!(sampled, stepped.boxes[0]);
    }

    #[test]
    fn fixed_body_is_bit_for_bit_unchanged() {
        let dynamic = dynamic_body();
        let fixed = RigidBox3d::new(
            PhysicsBody3d::fixed(EntityId(2), dynamic.body.half_extents),
            dynamic.state,
        );

        assert_eq!(
            sample_rigid_box_free_flight(fixed, config(), 3, 4).expect("valid fixed sample"),
            fixed
        );
    }

    #[test]
    fn direct_fraction_sample_is_repeatable_and_preserves_angular_velocity() {
        let body = dynamic_body();
        let first = sample_rigid_box_free_flight(body, config(), 3, 8)
            .expect("valid first fractional sample");
        let second = sample_rigid_box_free_flight(body, config(), 3, 8)
            .expect("valid repeated fractional sample");

        assert_eq!(first, second);
        assert_eq!(
            first.state.angular.angular_velocity,
            body.state.angular.angular_velocity
        );
        assert_ne!(
            first.state.angular.orientation,
            body.state.angular.orientation
        );
    }

    #[test]
    fn world_sampler_preserves_order_and_matches_individual_samples() {
        let first = dynamic_body();
        let second = RigidBox3d::new(
            PhysicsBody3d::fixed(EntityId(2), [4, 4, 4]),
            RigidBoxState3d::new(
                Position::new3(500, 0, 0),
                Velocity::new3(0, 0, 0),
                AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
            ),
        );
        let world = sample_rigid_box_world_free_flight(&[first, second], config(), 1, 2)
            .expect("valid world sample");

        assert_eq!(world.len(), 2);
        assert_eq!(world[0].body.entity, EntityId(1));
        assert_eq!(world[1].body.entity, EntityId(2));
        assert_eq!(
            world[0],
            sample_rigid_box_free_flight(first, config(), 1, 2).expect("valid individual sample")
        );
        assert_eq!(world[1], second);
    }

    #[test]
    fn malformed_fraction_and_timestep_fail_closed() {
        assert_eq!(
            sample_rigid_box_free_flight(dynamic_body(), config(), 2, 1),
            Err(RigidBoxFreeFlightError3d::InvalidFraction {
                numerator: 2,
                denominator: 1,
            })
        );
        assert_eq!(
            sample_rigid_box_free_flight(dynamic_body(), config(), 0, 0),
            Err(RigidBoxFreeFlightError3d::InvalidFraction {
                numerator: 0,
                denominator: 0,
            })
        );
        assert_eq!(
            sample_rigid_box_free_flight(
                dynamic_body(),
                RigidBoxFreeFlightConfig3d {
                    timestep_denominator: 0,
                    ..config()
                },
                1,
                1,
            ),
            Err(RigidBoxFreeFlightError3d::NonPositiveTimestepDenominator(0))
        );
    }
}
