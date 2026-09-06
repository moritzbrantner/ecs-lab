use ecs_workload::Position;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ColliderShape3d {
    Aabb { half_extents: [i32; 3] },
    Sphere { radius: i32 },
}

impl ColliderShape3d {
    #[must_use]
    pub const fn aabb(half_extents: [i32; 3]) -> Self {
        Self::Aabb { half_extents }
    }

    #[must_use]
    pub const fn sphere(radius: i32) -> Self {
        Self::Sphere { radius }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Collider3d {
    pub center: Position,
    pub shape: ColliderShape3d,
}

impl Collider3d {
    #[must_use]
    pub const fn new(center: Position, shape: ColliderShape3d) -> Self {
        Self { center, shape }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ColliderContact3d {
    /// Vector from the left collider toward the right collider at the closest/contact feature.
    ///
    /// It is deliberately not normalized: integer direction evidence remains exact and callers that
    /// need a unit vector may normalize only at their presentation or response boundary.
    pub direction: [i64; 3],
    /// Squared closest-point distance before subtracting the applicable radius.
    pub distance_squared: i128,
    /// Squared contact threshold for the shape pair. Contact exists when
    /// `distance_squared <= threshold_squared`.
    pub threshold_squared: i128,
}

impl ColliderContact3d {
    #[must_use]
    pub const fn overlaps(self) -> bool {
        self.distance_squared <= self.threshold_squared
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ColliderError3d {
    NegativeAabbHalfExtent,
    NegativeSphereRadius,
    ArithmeticOverflow,
}

/// Evaluates deterministic integer contact semantics for AABB and sphere pairs.
///
/// This is a shape-query layer only. It does not integrate time or apply impulses; the continuous solver
/// remains the owner of time-of-impact and response. Touching counts as contact, matching the existing
/// AABB contract.
///
/// # Errors
///
/// Returns [`ColliderError3d`] for invalid dimensions or arithmetic overflow.
pub fn collider_contact(
    left: Collider3d,
    right: Collider3d,
) -> Result<ColliderContact3d, ColliderError3d> {
    validate_shape(left.shape)?;
    validate_shape(right.shape)?;
    match (left.shape, right.shape) {
        (
            ColliderShape3d::Sphere {
                radius: left_radius,
            },
            ColliderShape3d::Sphere {
                radius: right_radius,
            },
        ) => sphere_sphere_contact(left.center, left_radius, right.center, right_radius),
        (ColliderShape3d::Sphere { radius }, ColliderShape3d::Aabb { half_extents }) => {
            sphere_aabb_contact(left.center, radius, right.center, half_extents)
        }
        (ColliderShape3d::Aabb { half_extents }, ColliderShape3d::Sphere { radius }) => {
            let contact = sphere_aabb_contact(right.center, radius, left.center, half_extents)?;
            Ok(ColliderContact3d {
                direction: contact.direction.map(|value| -value),
                ..contact
            })
        }
        (
            ColliderShape3d::Aabb {
                half_extents: left_half,
            },
            ColliderShape3d::Aabb {
                half_extents: right_half,
            },
        ) => aabb_aabb_contact(left.center, left_half, right.center, right_half),
    }
}

fn validate_shape(shape: ColliderShape3d) -> Result<(), ColliderError3d> {
    match shape {
        ColliderShape3d::Aabb { half_extents } if half_extents.iter().any(|extent| *extent < 0) => {
            Err(ColliderError3d::NegativeAabbHalfExtent)
        }
        ColliderShape3d::Sphere { radius } if radius < 0 => {
            Err(ColliderError3d::NegativeSphereRadius)
        }
        _ => Ok(()),
    }
}

fn sphere_sphere_contact(
    left: Position,
    left_radius: i32,
    right: Position,
    right_radius: i32,
) -> Result<ColliderContact3d, ColliderError3d> {
    let direction = checked_delta(left, right)?;
    let distance_squared = squared_length(direction)?;
    let radius = i128::from(left_radius)
        .checked_add(i128::from(right_radius))
        .ok_or(ColliderError3d::ArithmeticOverflow)?;
    let threshold_squared = radius
        .checked_mul(radius)
        .ok_or(ColliderError3d::ArithmeticOverflow)?;
    Ok(ColliderContact3d {
        direction,
        distance_squared,
        threshold_squared,
    })
}

fn sphere_aabb_contact(
    sphere: Position,
    radius: i32,
    aabb: Position,
    half_extents: [i32; 3],
) -> Result<ColliderContact3d, ColliderError3d> {
    let sphere_axes = position_axes(sphere);
    let aabb_axes = position_axes(aabb);
    let mut closest = [0_i64; 3];
    for axis in 0..3 {
        let half = i64::from(half_extents[axis]);
        let minimum = aabb_axes[axis]
            .checked_sub(half)
            .ok_or(ColliderError3d::ArithmeticOverflow)?;
        let maximum = aabb_axes[axis]
            .checked_add(half)
            .ok_or(ColliderError3d::ArithmeticOverflow)?;
        closest[axis] = sphere_axes[axis].clamp(minimum, maximum);
    }

    let direction = [
        closest[0]
            .checked_sub(sphere_axes[0])
            .ok_or(ColliderError3d::ArithmeticOverflow)?,
        closest[1]
            .checked_sub(sphere_axes[1])
            .ok_or(ColliderError3d::ArithmeticOverflow)?,
        closest[2]
            .checked_sub(sphere_axes[2])
            .ok_or(ColliderError3d::ArithmeticOverflow)?,
    ];
    let distance_squared = squared_length(direction)?;
    let radius = i128::from(radius);
    let threshold_squared = radius
        .checked_mul(radius)
        .ok_or(ColliderError3d::ArithmeticOverflow)?;
    Ok(ColliderContact3d {
        direction,
        distance_squared,
        threshold_squared,
    })
}

fn aabb_aabb_contact(
    left: Position,
    left_half: [i32; 3],
    right: Position,
    right_half: [i32; 3],
) -> Result<ColliderContact3d, ColliderError3d> {
    let direction = checked_delta(left, right)?;
    let mut separated_squared = 0_i128;
    for axis in 0..3 {
        let center_distance = i128::from(direction[axis]).abs();
        let combined_half = i128::from(left_half[axis])
            .checked_add(i128::from(right_half[axis]))
            .ok_or(ColliderError3d::ArithmeticOverflow)?;
        let gap = center_distance.saturating_sub(combined_half);
        separated_squared = separated_squared
            .checked_add(
                gap.checked_mul(gap)
                    .ok_or(ColliderError3d::ArithmeticOverflow)?,
            )
            .ok_or(ColliderError3d::ArithmeticOverflow)?;
    }
    Ok(ColliderContact3d {
        direction,
        distance_squared: separated_squared,
        threshold_squared: 0,
    })
}

fn checked_delta(left: Position, right: Position) -> Result<[i64; 3], ColliderError3d> {
    let left_axes = position_axes(left);
    let right_axes = position_axes(right);
    Ok([
        right_axes[0]
            .checked_sub(left_axes[0])
            .ok_or(ColliderError3d::ArithmeticOverflow)?,
        right_axes[1]
            .checked_sub(left_axes[1])
            .ok_or(ColliderError3d::ArithmeticOverflow)?,
        right_axes[2]
            .checked_sub(left_axes[2])
            .ok_or(ColliderError3d::ArithmeticOverflow)?,
    ])
}

fn squared_length(vector: [i64; 3]) -> Result<i128, ColliderError3d> {
    vector.into_iter().try_fold(0_i128, |sum, component| {
        let component = i128::from(component);
        sum.checked_add(
            component
                .checked_mul(component)
                .ok_or(ColliderError3d::ArithmeticOverflow)?,
        )
        .ok_or(ColliderError3d::ArithmeticOverflow)
    })
}

const fn position_axes(position: Position) -> [i64; 3] {
    [position.x, position.y, position.z]
}

#[cfg(test)]
mod tests {
    use ecs_workload::Position;

    use super::{Collider3d, ColliderError3d, ColliderShape3d, collider_contact};

    #[test]
    fn sphere_sphere_touching_is_contact() {
        let left = Collider3d::new(Position::new3(0, 0, 0), ColliderShape3d::sphere(3));
        let right = Collider3d::new(Position::new3(7, 0, 0), ColliderShape3d::sphere(4));
        let contact = collider_contact(left, right).expect("valid sphere pair");
        assert!(contact.overlaps());
        assert_eq!(contact.distance_squared, 49);
        assert_eq!(contact.threshold_squared, 49);
        assert_eq!(contact.direction, [7, 0, 0]);
    }

    #[test]
    fn separated_spheres_do_not_contact() {
        let left = Collider3d::new(Position::new3(0, 0, 0), ColliderShape3d::sphere(2));
        let right = Collider3d::new(Position::new3(5, 0, 0), ColliderShape3d::sphere(2));
        assert!(
            !collider_contact(left, right)
                .expect("valid sphere pair")
                .overlaps()
        );
    }

    #[test]
    fn sphere_aabb_corner_uses_exact_squared_distance() {
        let sphere = Collider3d::new(Position::new3(4, 4, 0), ColliderShape3d::sphere(3));
        let aabb = Collider3d::new(Position::new3(0, 0, 0), ColliderShape3d::aabb([2, 2, 2]));
        let contact = collider_contact(sphere, aabb).expect("valid mixed pair");
        assert!(contact.overlaps());
        assert_eq!(contact.direction, [-2, -2, 0]);
        assert_eq!(contact.distance_squared, 8);
        assert_eq!(contact.threshold_squared, 9);
    }

    #[test]
    fn aabb_sphere_is_symmetric_except_for_direction() {
        let sphere = Collider3d::new(Position::new3(5, 0, 0), ColliderShape3d::sphere(2));
        let aabb = Collider3d::new(Position::new3(0, 0, 0), ColliderShape3d::aabb([3, 3, 3]));
        let left = collider_contact(sphere, aabb).expect("sphere-aabb");
        let right = collider_contact(aabb, sphere).expect("aabb-sphere");
        assert_eq!(left.overlaps(), right.overlaps());
        assert_eq!(left.distance_squared, right.distance_squared);
        assert_eq!(left.threshold_squared, right.threshold_squared);
        assert_eq!(left.direction, right.direction.map(|value| -value));
    }

    #[test]
    fn existing_aabb_touching_semantics_are_preserved() {
        let left = Collider3d::new(Position::new3(0, 0, 0), ColliderShape3d::aabb([2, 2, 2]));
        let right = Collider3d::new(Position::new3(4, 0, 0), ColliderShape3d::aabb([2, 2, 2]));
        assert!(
            collider_contact(left, right)
                .expect("valid AABB pair")
                .overlaps()
        );
    }

    #[test]
    fn invalid_shapes_fail_closed() {
        let sphere = Collider3d::new(Position::new3(0, 0, 0), ColliderShape3d::sphere(-1));
        let box_shape = Collider3d::new(Position::new3(0, 0, 0), ColliderShape3d::aabb([1, 1, 1]));
        assert_eq!(
            collider_contact(sphere, box_shape),
            Err(ColliderError3d::NegativeSphereRadius)
        );
    }
}
