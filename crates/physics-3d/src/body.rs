use ecs_physics::{BodyKind, PhysicsMaterial};
use ecs_workload::{EntityId, Position, Velocity};

/// Fixed-point quaternion scale. `1 << 30` represents one unit.
pub const ORIENTATION_SCALE: i32 = 1_i32 << 30;
/// Angular velocity units per radian per second.
pub const ANGULAR_VELOCITY_SCALE: i32 = 1_000_000;

/// ECS-owned rigid-box metadata. Half extents are in the scenario's integer spatial units.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct PhysicsBody3d {
    pub entity: EntityId,
    pub kind: BodyKind,
    pub half_extents: [i32; 3],
    pub mass_units: u32,
    pub material: PhysicsMaterial,
}

impl PhysicsBody3d {
    #[must_use]
    pub const fn dynamic(entity: EntityId, half_extents: [i32; 3]) -> Self {
        Self {
            entity,
            kind: BodyKind::Dynamic,
            half_extents,
            mass_units: 1,
            material: PhysicsMaterial::new(0, 0),
        }
    }

    #[must_use]
    pub const fn fixed(entity: EntityId, half_extents: [i32; 3]) -> Self {
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

/// Fixed-point unit quaternion; components are scaled by [`ORIENTATION_SCALE`].
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Orientation3d {
    pub x: i32,
    pub y: i32,
    pub z: i32,
    pub w: i32,
}

impl Orientation3d {
    pub const IDENTITY: Self = Self {
        x: 0,
        y: 0,
        z: 0,
        w: ORIENTATION_SCALE,
    };

    #[must_use]
    pub const fn new(x: i32, y: i32, z: i32, w: i32) -> Self {
        Self { x, y, z, w }
    }

    /// Quantizes the rotation of `radians` about `axis`. A zero axis yields the identity.
    #[must_use]
    pub fn from_axis_angle(axis: [f64; 3], radians: f64) -> Self {
        let length = axis.iter().map(|value| value * value).sum::<f64>().sqrt();
        if length == 0.0 {
            return Self::IDENTITY;
        }
        let (sin, cos) = (radians / 2.0).sin_cos();
        let scale = sin / length;
        Self::from_unit([axis[0] * scale, axis[1] * scale, axis[2] * scale, cos])
    }

    /// Quantizes a quaternion given as `[x, y, z, w]`, normalizing it first. A zero or non-finite
    /// quaternion yields the identity.
    #[must_use]
    pub fn from_unit(xyzw: [f64; 4]) -> Self {
        let length = xyzw.iter().map(|value| value * value).sum::<f64>().sqrt();
        if !length.is_finite() || length == 0.0 {
            return Self::IDENTITY;
        }
        let [x, y, z, w] = xyzw.map(|value| quantize_i32(value / length, ORIENTATION_SCALE));
        Self { x, y, z, w }
    }

    /// The quaternion as `[x, y, z, w]` floating-point components.
    #[must_use]
    pub fn to_unit(self) -> [f64; 4] {
        [self.x, self.y, self.z, self.w]
            .map(|value| f64::from(value) / f64::from(ORIENTATION_SCALE))
    }

    /// Rotation angle away from the identity, in radians.
    #[must_use]
    pub fn angle(self) -> f64 {
        let w = self.to_unit()[3].abs().min(1.0);
        2.0 * w.acos()
    }
}

impl Default for Orientation3d {
    fn default() -> Self {
        Self::IDENTITY
    }
}

/// Fixed-point angular velocity; components are scaled by [`ANGULAR_VELOCITY_SCALE`].
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct AngularVelocity3d {
    pub x: i32,
    pub y: i32,
    pub z: i32,
}

impl AngularVelocity3d {
    #[must_use]
    pub const fn new(x: i32, y: i32, z: i32) -> Self {
        Self { x, y, z }
    }

    #[must_use]
    pub const fn is_zero(self) -> bool {
        self.x == 0 && self.y == 0 && self.z == 0
    }

    /// Magnitude in radians per second.
    #[must_use]
    pub fn radians_per_second(self) -> f64 {
        let [x, y, z] = [self.x, self.y, self.z].map(f64::from);
        (x * x + y * y + z * z).sqrt() / f64::from(ANGULAR_VELOCITY_SCALE)
    }
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct AngularState3d {
    pub orientation: Orientation3d,
    pub angular_velocity: AngularVelocity3d,
}

impl AngularState3d {
    #[must_use]
    pub const fn new(orientation: Orientation3d, angular_velocity: AngularVelocity3d) -> Self {
        Self {
            orientation,
            angular_velocity,
        }
    }
}

/// Integer rigid-box state: the ECS view of one Rapier body.
///
/// `center` is in integer spatial units, `linear_velocity` in spatial units per second.
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

/// Rounds `value * scale` to the nearest integer, saturating at the `i32` range.
pub(crate) fn quantize_i32(value: f64, scale: i32) -> i32 {
    // `as` saturates and maps NaN to zero, which is the intended quantization.
    (value * f64::from(scale)).round() as i32
}

/// Rounds `value * scale` to the nearest integer, saturating at the `i64` range.
pub(crate) fn quantize_i64(value: f64, scale: i64) -> i64 {
    (value * scale as f64).round() as i64
}

#[cfg(test)]
mod tests {
    use super::{ORIENTATION_SCALE, Orientation3d};

    #[test]
    fn quaternion_quantization_round_trips_within_one_unit() {
        let orientation = Orientation3d::from_axis_angle([1.0, 2.0, -0.5], 0.7);
        let again = Orientation3d::from_unit(orientation.to_unit());
        for (left, right) in [
            (orientation.x, again.x),
            (orientation.y, again.y),
            (orientation.z, again.z),
            (orientation.w, again.w),
        ] {
            assert!((left - right).abs() <= 1);
        }
        assert!((orientation.angle() - 0.7).abs() < 1e-6);
        assert_eq!(Orientation3d::from_unit([0.0; 4]), Orientation3d::IDENTITY);
        assert_eq!(Orientation3d::IDENTITY.w, ORIENTATION_SCALE);
    }
}
