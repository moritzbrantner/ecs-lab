use ecs_workload::{Position, Velocity};

use crate::{AngularState3d, PhysicsBody3d};

/// ECS-facing rigid-box state used to map scenario data into the standalone physics engine.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct RigidBoxState3d {
    pub center: Position,
    pub linear_velocity: Velocity,
    pub angular: AngularState3d,
}

impl RigidBoxState3d {
    #[must_use]
    pub const fn new(center: Position, linear_velocity: Velocity, angular: AngularState3d) -> Self {
        Self {
            center,
            linear_velocity,
            angular,
        }
    }
}

/// ECS-owned body metadata paired with its current rigid-box state.
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

/// Frame policy retained by ECS Lab while `physics-engine` owns world stepping and contact semantics.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct RigidBoxWorldConfig3d {
    pub gravity: Velocity,
    pub timestep_numerator: i32,
    pub timestep_denominator: i32,
    pub angular_damping_milli: u16,
    pub solver_passes: u8,
}

/// Sample-grid policy passed through the ECS adapter to `physics-engine`.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct RotatingContactSearchConfig3d {
    pub coarse_samples: u16,
    pub refinement_steps: u8,
}

impl Default for RotatingContactSearchConfig3d {
    fn default() -> Self {
        Self {
            coarse_samples: 8,
            refinement_steps: 6,
        }
    }
}
