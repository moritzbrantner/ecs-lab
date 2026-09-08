use std::fmt;

use ecs_workload::Position;

use crate::{
    AngularError3d, BoxPlaneError3d, OrientedBox3d, oriented_box_vertices,
};

const FEATURE_SIGNS: [i128; 2] = [1, -1];

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
    /// Nearest surface point rounded deterministically to integer world coordinates.
    ///
    /// Use [`Self::point_numerator`] / [`Self::point_denominator`] when a later response path needs the
    /// exact rational point selected from the quantized OBB surface.
    pub point: Position,
    pub point_numerator: [i128; 3],
    pub point_denominator: i128,
    /// Primitive exact direction from the box surface toward the sphere when the center is outside.
    /// For an interior center it points from the center toward the selected escape feature.
    pub normal: [i128; 3],
    /// Exact squared closest-point distance numerator. Divide by
    /// [`Self::distance_squared_denominator`] for world-space squared distance.
    pub distance_squared_numerator: i128,
    pub distance_squared_denominator: i128,
    /// Sphere-radius-squared threshold expressed over [`Self::distance_squared_denominator`].
    pub threshold_squared_numerator: i128,
    pub center_inside: bool,
}

impl SphereObbContact3d {
    #[must_use]
    pub const fn overlaps(self) -> bool {
        self.center_inside || self.distance_squared_numerator <= self.threshold_squared_numerator
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SphereObbError3d {
    InvalidRadius,
    InvalidHalfExtents,
    DegenerateGeometry,
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
            Self::DegenerateGeometry => write!(
                formatter,
                "sphere-OBB contact geometry collapsed after deterministic quantization"
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

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct Candidate {
    offset_numerator: [i128; 3],
    denominator: i128,
    distance_squared_numerator: i128,
    surface_normal: Option<[i128; 3]>,
}

/// Evaluates deterministic fixed-point sphere-vs-oriented-box contact geometry.
///
/// The query consumes the exact integer vertices returned by [`oriented_box_vertices`] and derives the
/// three already-quantized box basis vectors from them. It then enumerates valid closest projections onto
/// all six faces, twelve edges, and eight vertices of that quantized parallelepiped. This deliberately
/// matches the geometry used by OBB SAT instead of reapplying the pre-quantization quaternion transform.
/// Exact rational point/distance evidence is retained until the final diagnostic point rounding step.
///
/// This is contact geometry only. It does not integrate time, apply impulses, stabilize penetration, or
/// claim sphere-vs-OBB continuous collision detection.
///
/// # Errors
///
/// Returns [`SphereObbError3d`] for invalid/degenerate geometry or checked arithmetic overflow.
pub fn sphere_obb_contact(
    sphere: Sphere3d,
    box_shape: OrientedBox3d,
) -> Result<SphereObbContact3d, SphereObbError3d> {
    if sphere.radius <= 0 {
        return Err(SphereObbError3d::InvalidRadius);
    }

    let vertices = oriented_box_vertices(
        box_shape.center,
        box_shape.half_extents,
        box_shape.orientation,
    )
    .map_err(map_geometry_error)?;
    let basis = quantized_basis(&vertices)?;
    let center_delta = position_delta(box_shape.center, sphere.center)?;
    let center_inside = point_inside_quantized_box(center_delta, basis)?;
    let candidate = closest_surface_candidate(center_delta, basis)?;
    make_contact(sphere, box_shape.center, center_delta, center_inside, candidate)
}

fn quantized_basis(vertices: &[Position; 8]) -> Result<[[i128; 3]; 3], SphereObbError3d> {
    let doubled = [
        position_delta(vertices[0], vertices[1])?,
        position_delta(vertices[0], vertices[2])?,
        position_delta(vertices[0], vertices[4])?,
    ];
    let mut basis = [[0_i128; 3]; 3];
    for axis in 0..3 {
        for component in 0..3 {
            if doubled[axis][component] % 2 != 0 {
                return Err(SphereObbError3d::DegenerateGeometry);
            }
            basis[axis][component] = doubled[axis][component] / 2;
        }
        if dot(basis[axis], basis[axis])? == 0 {
            return Err(SphereObbError3d::DegenerateGeometry);
        }
    }
    if triple_product(basis[0], basis[1], basis[2])? == 0 {
        return Err(SphereObbError3d::DegenerateGeometry);
    }
    Ok(basis)
}

fn point_inside_quantized_box(
    point: [i128; 3],
    basis: [[i128; 3]; 3],
) -> Result<bool, SphereObbError3d> {
    let normals = [
        cross(basis[1], basis[2])?,
        cross(basis[2], basis[0])?,
        cross(basis[0], basis[1])?,
    ];
    let determinant = dot(basis[0], normals[0])?;
    let determinant_magnitude = checked_abs(determinant)?;
    if determinant_magnitude == 0 {
        return Err(SphereObbError3d::DegenerateGeometry);
    }

    for normal in normals {
        let coordinate_numerator = checked_abs(dot(point, normal)?)?;
        if coordinate_numerator > determinant_magnitude {
            return Ok(false);
        }
    }
    Ok(true)
}

fn closest_surface_candidate(
    point: [i128; 3],
    basis: [[i128; 3]; 3],
) -> Result<Candidate, SphereObbError3d> {
    let mut best = None;

    for axis in 0..3 {
        for sign in FEATURE_SIGNS {
            if let Some(candidate) = face_candidate(point, basis, axis, sign)? {
                consider_candidate(&mut best, candidate)?;
            }
        }
    }

    for axis in 0..3 {
        let others = other_axes(axis);
        for first_sign in FEATURE_SIGNS {
            for second_sign in FEATURE_SIGNS {
                if let Some(candidate) = edge_candidate(
                    point,
                    basis,
                    axis,
                    others,
                    first_sign,
                    second_sign,
                )? {
                    consider_candidate(&mut best, candidate)?;
                }
            }
        }
    }

    for first_sign in FEATURE_SIGNS {
        for second_sign in FEATURE_SIGNS {
            for third_sign in FEATURE_SIGNS {
                let candidate = vertex_candidate(
                    point,
                    basis,
                    [first_sign, second_sign, third_sign],
                )?;
                consider_candidate(&mut best, candidate)?;
            }
        }
    }

    best.ok_or(SphereObbError3d::DegenerateGeometry)
}

fn face_candidate(
    point: [i128; 3],
    basis: [[i128; 3]; 3],
    axis: usize,
    sign: i128,
) -> Result<Option<Candidate>, SphereObbError3d> {
    let spans = other_axes(axis);
    let u = basis[spans[0]];
    let v = basis[spans[1]];
    let face_offset = scale_vector(basis[axis], sign)?;
    let relative = subtract_vectors(point, face_offset)?;
    let uu = dot(u, u)?;
    let uv = dot(u, v)?;
    let vv = dot(v, v)?;
    let denominator = checked_sub(checked_mul(uu, vv)?, checked_mul(uv, uv)?)?;
    if denominator <= 0 {
        return Err(SphereObbError3d::DegenerateGeometry);
    }

    let ru = dot(relative, u)?;
    let rv = dot(relative, v)?;
    let u_numerator = checked_sub(checked_mul(ru, vv)?, checked_mul(rv, uv)?)?;
    let v_numerator = checked_sub(checked_mul(rv, uu)?, checked_mul(ru, uv)?)?;
    if checked_abs(u_numerator)? > denominator || checked_abs(v_numerator)? > denominator {
        return Ok(None);
    }

    let mut offset_numerator = scale_vector(face_offset, denominator)?;
    offset_numerator = add_vectors(offset_numerator, scale_vector(u, u_numerator)?)?;
    offset_numerator = add_vectors(offset_numerator, scale_vector(v, v_numerator)?)?;
    let relative_squared = dot(relative, relative)?;
    let projected_numerator = checked_add(
        checked_mul(ru, u_numerator)?,
        checked_mul(rv, v_numerator)?,
    )?;
    let distance_squared_numerator = checked_sub(
        checked_mul(relative_squared, denominator)?,
        projected_numerator,
    )?;
    if distance_squared_numerator < 0 {
        return Err(SphereObbError3d::ArithmeticOverflow);
    }

    let mut surface_normal = cross(u, v)?;
    if dot(basis[axis], surface_normal)? < 0 {
        surface_normal = negate_vector(surface_normal)?;
    }
    if sign < 0 {
        surface_normal = negate_vector(surface_normal)?;
    }

    Ok(Some(Candidate {
        offset_numerator,
        denominator,
        distance_squared_numerator,
        surface_normal: Some(surface_normal),
    }))
}

fn edge_candidate(
    point: [i128; 3],
    basis: [[i128; 3]; 3],
    edge_axis: usize,
    fixed_axes: [usize; 2],
    first_sign: i128,
    second_sign: i128,
) -> Result<Option<Candidate>, SphereObbError3d> {
    let edge = basis[edge_axis];
    let mut edge_offset = scale_vector(basis[fixed_axes[0]], first_sign)?;
    edge_offset = add_vectors(
        edge_offset,
        scale_vector(basis[fixed_axes[1]], second_sign)?,
    )?;
    let relative = subtract_vectors(point, edge_offset)?;
    let denominator = dot(edge, edge)?;
    if denominator <= 0 {
        return Err(SphereObbError3d::DegenerateGeometry);
    }
    let parameter_numerator = dot(relative, edge)?;
    if checked_abs(parameter_numerator)? > denominator {
        return Ok(None);
    }

    let mut offset_numerator = scale_vector(edge_offset, denominator)?;
    offset_numerator = add_vectors(
        offset_numerator,
        scale_vector(edge, parameter_numerator)?,
    )?;
    let relative_squared = dot(relative, relative)?;
    let distance_squared_numerator = checked_sub(
        checked_mul(relative_squared, denominator)?,
        checked_mul(parameter_numerator, parameter_numerator)?,
    )?;
    if distance_squared_numerator < 0 {
        return Err(SphereObbError3d::ArithmeticOverflow);
    }

    Ok(Some(Candidate {
        offset_numerator,
        denominator,
        distance_squared_numerator,
        surface_normal: None,
    }))
}

fn vertex_candidate(
    point: [i128; 3],
    basis: [[i128; 3]; 3],
    signs: [i128; 3],
) -> Result<Candidate, SphereObbError3d> {
    let mut offset = [0_i128; 3];
    for axis in 0..3 {
        offset = add_vectors(offset, scale_vector(basis[axis], signs[axis])?)?;
    }
    let relative = subtract_vectors(point, offset)?;
    Ok(Candidate {
        offset_numerator: offset,
        denominator: 1,
        distance_squared_numerator: dot(relative, relative)?,
        surface_normal: None,
    })
}

fn consider_candidate(
    best: &mut Option<Candidate>,
    candidate: Candidate,
) -> Result<(), SphereObbError3d> {
    let replace = match *best {
        None => true,
        Some(current) => {
            let candidate_scaled = checked_mul(
                candidate.distance_squared_numerator,
                current.denominator,
            )?;
            let current_scaled = checked_mul(
                current.distance_squared_numerator,
                candidate.denominator,
            )?;
            candidate_scaled < current_scaled
        }
    };
    if replace {
        *best = Some(candidate);
    }
    Ok(())
}

fn make_contact(
    sphere: Sphere3d,
    box_center: Position,
    center_delta: [i128; 3],
    center_inside: bool,
    candidate: Candidate,
) -> Result<SphereObbContact3d, SphereObbError3d> {
    let center = position_axes(box_center);
    let mut point_numerator = [0_i128; 3];
    for axis in 0..3 {
        point_numerator[axis] = checked_add(
            checked_mul(center[axis], candidate.denominator)?,
            candidate.offset_numerator[axis],
        )?;
    }
    let point = Position::new3(
        rounded_i64(point_numerator[0], candidate.denominator)?,
        rounded_i64(point_numerator[1], candidate.denominator)?,
        rounded_i64(point_numerator[2], candidate.denominator)?,
    );

    let mut direction = [0_i128; 3];
    for axis in 0..3 {
        let center_scaled = checked_mul(center_delta[axis], candidate.denominator)?;
        direction[axis] = if center_inside {
            checked_sub(candidate.offset_numerator[axis], center_scaled)?
        } else {
            checked_sub(center_scaled, candidate.offset_numerator[axis])?
        };
    }
    if direction == [0, 0, 0] {
        direction = candidate
            .surface_normal
            .ok_or(SphereObbError3d::DegenerateGeometry)?;
    }
    let normal = primitive_direction(direction)?;

    let radius = i128::from(sphere.radius);
    let radius_squared = checked_mul(radius, radius)?;
    let threshold_squared_numerator = checked_mul(radius_squared, candidate.denominator)?;

    Ok(SphereObbContact3d {
        point,
        point_numerator,
        point_denominator: candidate.denominator,
        normal,
        distance_squared_numerator: candidate.distance_squared_numerator,
        distance_squared_denominator: candidate.denominator,
        threshold_squared_numerator,
        center_inside,
    })
}

fn map_geometry_error(error: BoxPlaneError3d) -> SphereObbError3d {
    match error {
        BoxPlaneError3d::InvalidHalfExtents => SphereObbError3d::InvalidHalfExtents,
        BoxPlaneError3d::Angular(error) => SphereObbError3d::Angular(error),
        BoxPlaneError3d::ArithmeticOverflow
        | BoxPlaneError3d::FixedBody
        | BoxPlaneError3d::ZeroMass
        | BoxPlaneError3d::NegativeTimestepNumerator(_)
        | BoxPlaneError3d::NonPositiveTimestepDenominator(_)
        | BoxPlaneError3d::DampingOutOfRange(_)
        | BoxPlaneError3d::FrictionOutOfRange(_) => SphereObbError3d::ArithmeticOverflow,
    }
}

fn other_axes(axis: usize) -> [usize; 2] {
    match axis {
        0 => [1, 2],
        1 => [2, 0],
        2 => [0, 1],
        _ => unreachable!("3D axis index is bounded by the caller"),
    }
}

fn position_delta(left: Position, right: Position) -> Result<[i128; 3], SphereObbError3d> {
    let left = position_axes(left);
    let right = position_axes(right);
    Ok([
        checked_sub(right[0], left[0])?,
        checked_sub(right[1], left[1])?,
        checked_sub(right[2], left[2])?,
    ])
}

const fn position_axes(position: Position) -> [i128; 3] {
    [
        position.x as i128,
        position.y as i128,
        position.z as i128,
    ]
}

fn dot(left: [i128; 3], right: [i128; 3]) -> Result<i128, SphereObbError3d> {
    let mut sum = 0_i128;
    for axis in 0..3 {
        sum = checked_add(sum, checked_mul(left[axis], right[axis])?)?;
    }
    Ok(sum)
}

fn cross(left: [i128; 3], right: [i128; 3]) -> Result<[i128; 3], SphereObbError3d> {
    Ok([
        checked_sub(
            checked_mul(left[1], right[2])?,
            checked_mul(left[2], right[1])?,
        )?,
        checked_sub(
            checked_mul(left[2], right[0])?,
            checked_mul(left[0], right[2])?,
        )?,
        checked_sub(
            checked_mul(left[0], right[1])?,
            checked_mul(left[1], right[0])?,
        )?,
    ])
}

fn triple_product(
    first: [i128; 3],
    second: [i128; 3],
    third: [i128; 3],
) -> Result<i128, SphereObbError3d> {
    dot(first, cross(second, third)?)
}

fn scale_vector(vector: [i128; 3], scale: i128) -> Result<[i128; 3], SphereObbError3d> {
    Ok([
        checked_mul(vector[0], scale)?,
        checked_mul(vector[1], scale)?,
        checked_mul(vector[2], scale)?,
    ])
}

fn add_vectors(
    left: [i128; 3],
    right: [i128; 3],
) -> Result<[i128; 3], SphereObbError3d> {
    Ok([
        checked_add(left[0], right[0])?,
        checked_add(left[1], right[1])?,
        checked_add(left[2], right[2])?,
    ])
}

fn subtract_vectors(
    left: [i128; 3],
    right: [i128; 3],
) -> Result<[i128; 3], SphereObbError3d> {
    Ok([
        checked_sub(left[0], right[0])?,
        checked_sub(left[1], right[1])?,
        checked_sub(left[2], right[2])?,
    ])
}

fn negate_vector(vector: [i128; 3]) -> Result<[i128; 3], SphereObbError3d> {
    Ok([
        vector[0]
            .checked_neg()
            .ok_or(SphereObbError3d::ArithmeticOverflow)?,
        vector[1]
            .checked_neg()
            .ok_or(SphereObbError3d::ArithmeticOverflow)?,
        vector[2]
            .checked_neg()
            .ok_or(SphereObbError3d::ArithmeticOverflow)?,
    ])
}

fn primitive_direction(vector: [i128; 3]) -> Result<[i128; 3], SphereObbError3d> {
    let mut divisor = 0_i128;
    for component in vector {
        divisor = gcd(divisor, checked_abs(component)?);
    }
    if divisor == 0 {
        return Err(SphereObbError3d::DegenerateGeometry);
    }
    Ok(vector.map(|component| component / divisor))
}

fn gcd(mut left: i128, mut right: i128) -> i128 {
    while right != 0 {
        let remainder = left % right;
        left = right;
        right = remainder;
    }
    left
}

fn rounded_i64(numerator: i128, denominator: i128) -> Result<i64, SphereObbError3d> {
    let rounded = div_round_nearest(numerator, denominator)?;
    i64::try_from(rounded).map_err(|_| SphereObbError3d::ArithmeticOverflow)
}

fn checked_abs(value: i128) -> Result<i128, SphereObbError3d> {
    value
        .checked_abs()
        .ok_or(SphereObbError3d::ArithmeticOverflow)
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
        assert_eq!(contact.normal, [1, 0, 0]);
        assert_eq!(
            contact.distance_squared_numerator,
            contact.threshold_squared_numerator
        );
    }

    #[test]
    fn corner_gap_uses_euclidean_distance() {
        let sphere = Sphere3d::new(Position::new3(5, 5, 0), 4);
        let box_shape =
            OrientedBox3d::new(Position::new3(0, 0, 0), [2, 2, 2], Orientation3d::IDENTITY);
        let contact = sphere_obb_contact(sphere, box_shape).expect("valid mixed pair");

        assert!(!contact.overlaps());
        assert_eq!(contact.point, Position::new3(2, 2, 0));
        assert!(contact.distance_squared_numerator > contact.threshold_squared_numerator);
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
        assert_eq!(contact.normal, [0, 1, 0]);
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
        assert_eq!(contact.normal, [1, 0, 0]);
    }

    #[test]
    fn closest_point_matches_quantized_obb_vertices() {
        let sphere = Sphere3d::new(Position::new3(-6, -1, 0), 5);
        let box_shape = OrientedBox3d::new(
            Position::new3(0, 0, 0),
            [1, 1, 1],
            Orientation3d::new(0, 0, 46_835_961, 1_072_719_860),
        );
        let contact = sphere_obb_contact(sphere, box_shape).expect("valid quantized mixed pair");

        assert!(contact.overlaps());
        assert_eq!(contact.point, Position::new3(-1, -1, 0));
        assert_eq!(contact.normal, [-1, 0, 0]);
        assert_eq!(
            contact.distance_squared_numerator,
            contact.threshold_squared_numerator
        );
    }

    #[test]
    fn extreme_center_fails_closed_when_exact_feature_math_overflows() {
        let sphere = Sphere3d::new(Position::new3(i64::MIN, 0, 0), 1);
        let box_shape =
            OrientedBox3d::new(Position::new3(0, 0, 0), [2, 2, 2], Orientation3d::IDENTITY);

        assert_eq!(
            sphere_obb_contact(sphere, box_shape),
            Err(SphereObbError3d::ArithmeticOverflow)
        );
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
