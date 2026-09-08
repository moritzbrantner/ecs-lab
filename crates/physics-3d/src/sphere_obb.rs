use std::fmt;

use ecs_workload::Position;

use crate::{AngularError3d, ORIENTATION_SCALE, Orientation3d, OrientedBox3d};

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Sphere3d {
    pub center: Position,
    pub radius: i32,
}

impl Sphere3d {
    #[must_use]
    pub const fn new(center: Position, radius: i32) -> Self {
        Self { center, radius }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct SphereObbContact3d {
    /// Closest deterministic point on the oriented-box surface.
    pub point: Position,
    /// Unnormalized world-space direction from the oriented box toward the sphere.
    ///
    /// For a sphere center inside the box this is the outward normal of the nearest face, with stable
    /// X -> Y -> Z tie-breaking. For an outside center this is the vector from [`Self::point`] to the
    /// sphere center.
    pub normal: [i64; 3],
    /// Squared distance from the sphere center to [`Self::point`].
    pub distance_squared: i128,
    pub radius_squared: i128,
    /// Whether the sphere center itself lies inside the oriented box.
    pub center_inside: bool,
}

impl SphereObbContact3d {
    #[must_use]
    pub const fn overlaps(self) -> bool {
        self.center_inside || self.distance_squared <= self.radius_squared
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SphereObbError3d {
    InvalidRadius,
    InvalidHalfExtents,
    Angular(AngularError3d),
    ArithmeticOverflow,
}

impl fmt::Display for SphereObbError3d {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidRadius => {
                write!(formatter, "sphere-OBB contact requires a positive radius")
            }
            Self::InvalidHalfExtents => write!(
                formatter,
                "sphere-OBB contact requires strictly positive box half extents"
            ),
            Self::Angular(error) => write!(formatter, "sphere-OBB orientation failed: {error}"),
            Self::ArithmeticOverflow => {
                write!(formatter, "sphere-OBB contact arithmetic overflowed")
            }
        }
    }
}

impl std::error::Error for SphereObbError3d {}

impl From<AngularError3d> for SphereObbError3d {
    fn from(value: AngularError3d) -> Self {
        Self::Angular(value)
    }
}

/// Evaluates deterministic fixed-point sphere-vs-oriented-box contact geometry.
///
/// The sphere center is transformed into the box's local frame using the same normalized fixed-point
/// quaternion convention as the angular box path. The local point is clamped to the cuboid extents and
/// transformed back to world space. A center that lies inside the box chooses the nearest face with
/// stable X -> Y -> Z tie-breaking so later response has a deterministic escape normal.
///
/// This is contact geometry only. It does not integrate time, apply impulses, stabilize penetration, or
/// claim sphere-vs-OBB continuous collision detection.
///
/// # Errors
///
/// Returns [`SphereObbError3d`] for invalid dimensions/orientation or checked arithmetic overflow.
pub fn sphere_obb_contact(
    sphere: Sphere3d,
    box_shape: OrientedBox3d,
) -> Result<SphereObbContact3d, SphereObbError3d> {
    validate(sphere, box_shape)?;
    let orientation = box_shape.orientation.normalized()?;
    let matrix = rotation_matrix(orientation)?;
    let center_delta = position_delta(box_shape.center, sphere.center)?;
    let local_center = rotate_inverse_with_matrix(matrix, center_delta)?;
    let half_extents = box_shape.half_extents.map(i64::from);
    let center_inside = (0..3).all(|axis| {
        local_center[axis] >= -half_extents[axis] && local_center[axis] <= half_extents[axis]
    });

    let (local_point, inside_normal) = if center_inside {
        nearest_surface_point(local_center, half_extents)?
    } else {
        (
            [
                local_center[0].clamp(-half_extents[0], half_extents[0]),
                local_center[1].clamp(-half_extents[1], half_extents[1]),
                local_center[2].clamp(-half_extents[2], half_extents[2]),
            ],
            None,
        )
    };

    let world_offset = rotate_with_matrix(matrix, local_point)?;
    let point = add_offset(box_shape.center, world_offset)?;
    let surface_to_center = position_delta(point, sphere.center)?;
    let distance_squared = squared_length(surface_to_center)?;
    let radius = i128::from(sphere.radius);
    let radius_squared = radius
        .checked_mul(radius)
        .ok_or(SphereObbError3d::ArithmeticOverflow)?;
    let normal = match inside_normal {
        Some(local_normal) => rotate_with_matrix(matrix, local_normal)?,
        None => surface_to_center,
    };

    Ok(SphereObbContact3d {
        point,
        normal,
        distance_squared,
        radius_squared,
        center_inside,
    })
}

fn validate(sphere: Sphere3d, box_shape: OrientedBox3d) -> Result<(), SphereObbError3d> {
    if sphere.radius <= 0 {
        return Err(SphereObbError3d::InvalidRadius);
    }
    if box_shape.half_extents.iter().any(|extent| *extent <= 0) {
        return Err(SphereObbError3d::InvalidHalfExtents);
    }
    Ok(())
}

fn nearest_surface_point(
    local_center: [i64; 3],
    half_extents: [i64; 3],
) -> Result<([i64; 3], Option<[i64; 3]>), SphereObbError3d> {
    let mut axis = 0_usize;
    let mut margin = half_extents[0]
        .checked_sub(local_center[0].abs())
        .ok_or(SphereObbError3d::ArithmeticOverflow)?;
    for candidate_axis in 1..3 {
        let candidate_margin = half_extents[candidate_axis]
            .checked_sub(local_center[candidate_axis].abs())
            .ok_or(SphereObbError3d::ArithmeticOverflow)?;
        if candidate_margin < margin {
            axis = candidate_axis;
            margin = candidate_margin;
        }
    }

    let sign = if local_center[axis] < 0 {
        -1_i64
    } else {
        1_i64
    };
    let mut point = local_center;
    point[axis] = half_extents[axis]
        .checked_mul(sign)
        .ok_or(SphereObbError3d::ArithmeticOverflow)?;
    let mut normal = [0_i64; 3];
    normal[axis] = i64::from(ORIENTATION_SCALE)
        .checked_mul(sign)
        .ok_or(SphereObbError3d::ArithmeticOverflow)?;
    Ok((point, Some(normal)))
}

fn position_delta(left: Position, right: Position) -> Result<[i64; 3], SphereObbError3d> {
    Ok([
        right
            .x
            .checked_sub(left.x)
            .ok_or(SphereObbError3d::ArithmeticOverflow)?,
        right
            .y
            .checked_sub(left.y)
            .ok_or(SphereObbError3d::ArithmeticOverflow)?,
        right
            .z
            .checked_sub(left.z)
            .ok_or(SphereObbError3d::ArithmeticOverflow)?,
    ])
}

fn add_offset(center: Position, offset: [i64; 3]) -> Result<Position, SphereObbError3d> {
    Ok(Position::new3(
        center
            .x
            .checked_add(offset[0])
            .ok_or(SphereObbError3d::ArithmeticOverflow)?,
        center
            .y
            .checked_add(offset[1])
            .ok_or(SphereObbError3d::ArithmeticOverflow)?,
        center
            .z
            .checked_add(offset[2])
            .ok_or(SphereObbError3d::ArithmeticOverflow)?,
    ))
}

fn rotation_matrix(orientation: Orientation3d) -> Result<[[i128; 3]; 3], SphereObbError3d> {
    let x = i128::from(orientation.x);
    let y = i128::from(orientation.y);
    let z = i128::from(orientation.z);
    let w = i128::from(orientation.w);
    let scale = i128::from(ORIENTATION_SCALE);
    let xx = checked_mul(x, x)?;
    let yy = checked_mul(y, y)?;
    let zz = checked_mul(z, z)?;
    let xy = checked_mul(x, y)?;
    let xz = checked_mul(x, z)?;
    let yz = checked_mul(y, z)?;
    let xw = checked_mul(x, w)?;
    let yw = checked_mul(y, w)?;
    let zw = checked_mul(z, w)?;

    Ok([
        [
            checked_sub(scale, scaled_twice(checked_add(yy, zz)?, scale)?)?,
            scaled_twice(checked_sub(xy, zw)?, scale)?,
            scaled_twice(checked_add(xz, yw)?, scale)?,
        ],
        [
            scaled_twice(checked_add(xy, zw)?, scale)?,
            checked_sub(scale, scaled_twice(checked_add(xx, zz)?, scale)?)?,
            scaled_twice(checked_sub(yz, xw)?, scale)?,
        ],
        [
            scaled_twice(checked_sub(xz, yw)?, scale)?,
            scaled_twice(checked_add(yz, xw)?, scale)?,
            checked_sub(scale, scaled_twice(checked_add(xx, yy)?, scale)?)?,
        ],
    ])
}

fn rotate_inverse_with_matrix(
    matrix: [[i128; 3]; 3],
    vector: [i64; 3],
) -> Result<[i64; 3], SphereObbError3d> {
    rotate_with_matrix(
        [
            [matrix[0][0], matrix[1][0], matrix[2][0]],
            [matrix[0][1], matrix[1][1], matrix[2][1]],
            [matrix[0][2], matrix[1][2], matrix[2][2]],
        ],
        vector,
    )
}

fn rotate_with_matrix(
    matrix: [[i128; 3]; 3],
    vector: [i64; 3],
) -> Result<[i64; 3], SphereObbError3d> {
    let scale = i128::from(ORIENTATION_SCALE);
    let mut output = [0_i64; 3];
    for (target, row) in output.iter_mut().zip(matrix) {
        let first = checked_mul(row[0], i128::from(vector[0]))?;
        let second = checked_mul(row[1], i128::from(vector[1]))?;
        let third = checked_mul(row[2], i128::from(vector[2]))?;
        let sum = checked_add(checked_add(first, second)?, third)?;
        *target = i64::try_from(div_round_nearest(sum, scale)?)
            .map_err(|_| SphereObbError3d::ArithmeticOverflow)?;
    }
    Ok(output)
}

fn squared_length(vector: [i64; 3]) -> Result<i128, SphereObbError3d> {
    vector.into_iter().try_fold(0_i128, |sum, component| {
        let component = i128::from(component);
        sum.checked_add(
            component
                .checked_mul(component)
                .ok_or(SphereObbError3d::ArithmeticOverflow)?,
        )
        .ok_or(SphereObbError3d::ArithmeticOverflow)
    })
}

fn scaled_twice(value: i128, scale: i128) -> Result<i128, SphereObbError3d> {
    div_round_nearest(checked_mul(value, 2)?, scale)
}

fn checked_mul(left: i128, right: i128) -> Result<i128, SphereObbError3d> {
    left.checked_mul(right)
        .ok_or(SphereObbError3d::ArithmeticOverflow)
}

fn checked_add(left: i128, right: i128) -> Result<i128, SphereObbError3d> {
    left.checked_add(right)
        .ok_or(SphereObbError3d::ArithmeticOverflow)
}

fn checked_sub(left: i128, right: i128) -> Result<i128, SphereObbError3d> {
    left.checked_sub(right)
        .ok_or(SphereObbError3d::ArithmeticOverflow)
}

fn div_round_nearest(numerator: i128, denominator: i128) -> Result<i128, SphereObbError3d> {
    if denominator <= 0 {
        return Err(SphereObbError3d::ArithmeticOverflow);
    }
    let half = denominator / 2;
    let adjusted = if numerator >= 0 {
        numerator.checked_add(half)
    } else {
        numerator.checked_sub(half)
    }
    .ok_or(SphereObbError3d::ArithmeticOverflow)?;
    Ok(adjusted / denominator)
}

#[cfg(test)]
mod tests {
    use ecs_workload::Position;

    use crate::{ORIENTATION_SCALE, Orientation3d, OrientedBox3d};

    use super::{Sphere3d, SphereObbError3d, sphere_obb_contact};

    #[test]
    fn face_touch_uses_closest_surface_point() {
        let sphere = Sphere3d::new(Position::new3(5, 0, 0), 3);
        let box_shape =
            OrientedBox3d::new(Position::new3(0, 0, 0), [2, 3, 4], Orientation3d::IDENTITY);
        let contact = sphere_obb_contact(sphere, box_shape).expect("valid mixed pair");

        assert!(contact.overlaps());
        assert!(!contact.center_inside);
        assert_eq!(contact.point, Position::new3(2, 0, 0));
        assert_eq!(contact.normal, [3, 0, 0]);
        assert_eq!(contact.distance_squared, 9);
        assert_eq!(contact.radius_squared, 9);
    }

    #[test]
    fn corner_gap_uses_euclidean_distance() {
        let sphere = Sphere3d::new(Position::new3(5, 5, 0), 4);
        let box_shape =
            OrientedBox3d::new(Position::new3(0, 0, 0), [2, 2, 2], Orientation3d::IDENTITY);
        let contact = sphere_obb_contact(sphere, box_shape).expect("valid mixed pair");

        assert!(!contact.overlaps());
        assert_eq!(contact.point, Position::new3(2, 2, 0));
        assert_eq!(contact.distance_squared, 18);
        assert_eq!(contact.radius_squared, 16);
    }

    #[test]
    fn rotated_long_axis_contacts_in_world_y() {
        let sphere = Sphere3d::new(Position::new3(0, 5, 0), 1);
        let box_shape = OrientedBox3d::new(
            Position::new3(0, 0, 0),
            [4, 1, 1],
            Orientation3d::new(0, 0, ORIENTATION_SCALE / 2, ORIENTATION_SCALE / 2),
        );
        let contact = sphere_obb_contact(sphere, box_shape).expect("valid rotated mixed pair");

        assert!(contact.overlaps());
        assert_eq!(contact.point, Position::new3(0, 4, 0));
        assert_eq!(contact.distance_squared, 1);
    }

    #[test]
    fn interior_center_chooses_stable_nearest_face() {
        let sphere = Sphere3d::new(Position::new3(0, 0, 0), 1);
        let box_shape =
            OrientedBox3d::new(Position::new3(0, 0, 0), [2, 3, 4], Orientation3d::IDENTITY);
        let contact = sphere_obb_contact(sphere, box_shape).expect("valid interior pair");

        assert!(contact.overlaps());
        assert!(contact.center_inside);
        assert_eq!(contact.point, Position::new3(2, 0, 0));
        assert_eq!(contact.normal, [i64::from(ORIENTATION_SCALE), 0, 0]);
        assert_eq!(contact.distance_squared, 4);
        assert_eq!(contact.radius_squared, 1);
    }

    #[test]
    fn extreme_center_does_not_overflow_absolute_value() {
        let sphere = Sphere3d::new(Position::new3(i64::MIN, 0, 0), 1);
        let box_shape =
            OrientedBox3d::new(Position::new3(0, 0, 0), [2, 2, 2], Orientation3d::IDENTITY);
        let contact = sphere_obb_contact(sphere, box_shape).expect("representable extreme pair");

        assert!(!contact.overlaps());
        assert!(!contact.center_inside);
    }

    #[test]
    fn invalid_dimensions_fail_closed() {
        let box_shape =
            OrientedBox3d::new(Position::new3(0, 0, 0), [2, 2, 2], Orientation3d::IDENTITY);
        assert_eq!(
            sphere_obb_contact(Sphere3d::new(Position::new3(0, 0, 0), 0), box_shape),
            Err(SphereObbError3d::InvalidRadius)
        );

        let invalid_box =
            OrientedBox3d::new(Position::new3(0, 0, 0), [2, 0, 2], Orientation3d::IDENTITY);
        assert_eq!(
            sphere_obb_contact(Sphere3d::new(Position::new3(0, 0, 0), 1), invalid_box),
            Err(SphereObbError3d::InvalidHalfExtents)
        );
    }
}
