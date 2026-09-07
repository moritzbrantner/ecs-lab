use std::fmt;

use ecs_physics::BodyKind;
use ecs_workload::EntityId;

use crate::{
    RigidBox3d,
    swept_broad_phase::{BroadPhaseBounds3d, aabb_candidate_pairs},
};

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct RotationalSweepBounds3d {
    pub minimum: [i64; 3],
    pub maximum: [i64; 3],
}

#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub struct RotationalSweepPair3d {
    pub left: EntityId,
    pub right: EntityId,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RotationalSweepError3d {
    BodyCountMismatch {
        start: usize,
        end: usize,
    },
    EntityMismatch {
        index: usize,
        start: EntityId,
        end: EntityId,
    },
    BodyKindMismatch(EntityId),
    ShapeMismatch(EntityId),
    InvalidHalfExtents(EntityId),
    ArithmeticOverflow(EntityId),
}

impl fmt::Display for RotationalSweepError3d {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::BodyCountMismatch { start, end } => write!(
                formatter,
                "rotational sweep requires matching body counts, got {start} then {end}"
            ),
            Self::EntityMismatch { index, start, end } => write!(
                formatter,
                "rotational sweep body {index} changed entity id from {} to {}",
                start.0, end.0
            ),
            Self::BodyKindMismatch(entity) => write!(
                formatter,
                "rotational sweep body {} changed fixed/dynamic kind across the interval",
                entity.0
            ),
            Self::ShapeMismatch(entity) => write!(
                formatter,
                "rotational sweep body {} changed half extents across the interval",
                entity.0
            ),
            Self::InvalidHalfExtents(entity) => write!(
                formatter,
                "rotational sweep body {} requires strictly positive half extents",
                entity.0
            ),
            Self::ArithmeticOverflow(entity) => write!(
                formatter,
                "rotational sweep bounds overflowed for body {}",
                entity.0
            ),
        }
    }
}

impl std::error::Error for RotationalSweepError3d {}

/// Conservatively bounds one cuboid over translation plus arbitrary orientation change.
///
/// The cuboid is enclosed by the orientation-independent sphere whose radius is the ceiling of the
/// half-extents' Euclidean norm. Sweeping the corresponding axis-aligned cube between the start and end
/// centers therefore contains every possible intermediate OBB orientation while the center travels
/// between those endpoints. This intentionally over-approximates rotation; it is broad-phase evidence,
/// not a contact manifold or time of impact.
///
/// # Errors
///
/// Returns [`RotationalSweepError3d`] if the paired states do not describe the same rigid shape, contain
/// invalid half extents, or require coordinates outside checked integer arithmetic.
pub fn rotational_sweep_bounds(
    start: RigidBox3d,
    end: RigidBox3d,
) -> Result<RotationalSweepBounds3d, RotationalSweepError3d> {
    validate_same_body(start, end, 0)?;
    let radius = orientation_independent_radius(start)?;
    let start_center = [
        start.state.center.x,
        start.state.center.y,
        start.state.center.z,
    ];
    let end_center = [end.state.center.x, end.state.center.y, end.state.center.z];
    let mut minimum = [0_i64; 3];
    let mut maximum = [0_i64; 3];
    for axis in 0..3 {
        minimum[axis] = start_center[axis]
            .min(end_center[axis])
            .checked_sub(radius)
            .ok_or(RotationalSweepError3d::ArithmeticOverflow(
                start.body.entity,
            ))?;
        maximum[axis] = start_center[axis]
            .max(end_center[axis])
            .checked_add(radius)
            .ok_or(RotationalSweepError3d::ArithmeticOverflow(
                start.body.entity,
            ))?;
    }
    Ok(RotationalSweepBounds3d { minimum, maximum })
}

/// Builds deterministic candidate entity pairs from conservative rotational sweep bounds.
///
/// Every corresponding start/end body is first enclosed by [`rotational_sweep_bounds`]. Those bounds are
/// then submitted to the same shared spatial-hash acceleration structure used by the OBB world. If the
/// hash cannot represent the coordinate range or coverage cheaply and exactly, this function falls back
/// to every non-fixed/fixed pair instead of losing collision truth.
///
/// Returning a pair only means an intermediate rotating contact is possible. Exact OBB narrow phase and
/// eventual earliest-impact search remain separate Rust-owned stages. This function therefore does not
/// claim rotational CCD.
///
/// # Errors
///
/// Returns [`RotationalSweepError3d`] when the start/end slices do not contain the same stable bodies or
/// when one body's conservative bound cannot be represented with checked integer arithmetic.
pub fn rotational_sweep_candidate_pairs(
    start: &[RigidBox3d],
    end: &[RigidBox3d],
) -> Result<Vec<RotationalSweepPair3d>, RotationalSweepError3d> {
    if start.len() != end.len() {
        return Err(RotationalSweepError3d::BodyCountMismatch {
            start: start.len(),
            end: end.len(),
        });
    }

    let mut bounds = Vec::with_capacity(start.len());
    for (index, (start_body, end_body)) in
        start.iter().copied().zip(end.iter().copied()).enumerate()
    {
        validate_same_body(start_body, end_body, index)?;
        let sweep = rotational_sweep_bounds(start_body, end_body)?;
        bounds.push(BroadPhaseBounds3d {
            kind: start_body.body.kind,
            minimum: sweep.minimum,
            maximum: sweep.maximum,
        });
    }

    let pair_indices = aabb_candidate_pairs(&bounds).unwrap_or_else(|| all_pair_indices(start));
    Ok(pair_indices
        .into_iter()
        .map(|(left, right)| RotationalSweepPair3d {
            left: start[left].body.entity,
            right: start[right].body.entity,
        })
        .collect())
}

fn validate_same_body(
    start: RigidBox3d,
    end: RigidBox3d,
    index: usize,
) -> Result<(), RotationalSweepError3d> {
    if start.body.entity != end.body.entity {
        return Err(RotationalSweepError3d::EntityMismatch {
            index,
            start: start.body.entity,
            end: end.body.entity,
        });
    }
    if start.body.kind != end.body.kind {
        return Err(RotationalSweepError3d::BodyKindMismatch(start.body.entity));
    }
    if start.body.half_extents != end.body.half_extents {
        return Err(RotationalSweepError3d::ShapeMismatch(start.body.entity));
    }
    if start.body.half_extents.iter().any(|extent| *extent <= 0) {
        return Err(RotationalSweepError3d::InvalidHalfExtents(
            start.body.entity,
        ));
    }
    Ok(())
}

fn orientation_independent_radius(body: RigidBox3d) -> Result<i64, RotationalSweepError3d> {
    let squared = body
        .body
        .half_extents
        .into_iter()
        .map(|extent| u128::from(extent.unsigned_abs()).pow(2))
        .try_fold(0_u128, u128::checked_add)
        .ok_or(RotationalSweepError3d::ArithmeticOverflow(body.body.entity))?;
    let floor = integer_sqrt(squared);
    let radius = if floor
        .checked_mul(floor)
        .is_some_and(|floor_squared| floor_squared == squared)
    {
        floor
    } else {
        floor
            .checked_add(1)
            .ok_or(RotationalSweepError3d::ArithmeticOverflow(body.body.entity))?
    };
    i64::try_from(radius).map_err(|_| RotationalSweepError3d::ArithmeticOverflow(body.body.entity))
}

fn integer_sqrt(value: u128) -> u128 {
    if value < 2 {
        return value;
    }
    let mut low = 1_u128;
    let mut high = value.div_ceil(2);
    while low < high {
        let middle = low + (high - low).div_ceil(2);
        if middle <= value / middle {
            low = middle;
        } else {
            high = middle - 1;
        }
    }
    low
}

fn all_pair_indices(boxes: &[RigidBox3d]) -> Vec<(usize, usize)> {
    let mut pairs = Vec::new();
    for left in 0..boxes.len() {
        for right in (left + 1)..boxes.len() {
            if boxes[left].body.kind == BodyKind::Fixed && boxes[right].body.kind == BodyKind::Fixed
            {
                continue;
            }
            pairs.push((left, right));
        }
    }
    pairs
}

#[cfg(test)]
mod tests {
    use ecs_workload::{EntityId, Position, Velocity};

    use crate::{
        AngularState3d, AngularVelocity3d, ORIENTATION_SCALE, Orientation3d, PhysicsBody3d,
        RigidBoxState3d, oriented_box_vertices,
    };

    use super::*;

    fn rigid_box(
        entity: u32,
        kind: BodyKind,
        half_extents: [i32; 3],
        center: Position,
        orientation: Orientation3d,
    ) -> RigidBox3d {
        let body = match kind {
            BodyKind::Dynamic => PhysicsBody3d::dynamic(EntityId(entity), half_extents),
            BodyKind::Fixed => PhysicsBody3d::fixed(EntityId(entity), half_extents),
        };
        RigidBox3d::new(
            body,
            RigidBoxState3d::new(
                center,
                Velocity::new3(0, 0, 0),
                AngularState3d::new(orientation, AngularVelocity3d::default()),
            ),
        )
    }

    fn contains(bounds: RotationalSweepBounds3d, position: Position) -> bool {
        [position.x, position.y, position.z]
            .into_iter()
            .enumerate()
            .all(|(axis, value)| value >= bounds.minimum[axis] && value <= bounds.maximum[axis])
    }

    #[test]
    fn orientation_independent_sweep_contains_both_endpoint_vertices() {
        let start = rigid_box(
            1,
            BodyKind::Dynamic,
            [20, 3, 2],
            Position::new3(-12, 5, 4),
            Orientation3d::IDENTITY,
        );
        let end_orientation = Orientation3d::new(0, 0, ORIENTATION_SCALE / 2, ORIENTATION_SCALE)
            .normalized()
            .expect("valid rotated endpoint");
        let end = rigid_box(
            1,
            BodyKind::Dynamic,
            [20, 3, 2],
            Position::new3(17, 9, -6),
            end_orientation,
        );
        let bounds = rotational_sweep_bounds(start, end).expect("valid conservative sweep");

        for vertex in oriented_box_vertices(
            start.state.center,
            start.body.half_extents,
            start.state.angular.orientation,
        )
        .expect("valid start vertices")
        .into_iter()
        .chain(
            oriented_box_vertices(
                end.state.center,
                end.body.half_extents,
                end.state.angular.orientation,
            )
            .expect("valid end vertices"),
        ) {
            assert!(contains(bounds, vertex));
        }
    }

    #[test]
    fn shared_hash_retains_pair_for_possible_intermediate_rotation() {
        let rod_start = rigid_box(
            1,
            BodyKind::Dynamic,
            [20, 2, 2],
            Position::new3(0, 0, 0),
            Orientation3d::IDENTITY,
        );
        let obstacle = rigid_box(
            2,
            BodyKind::Fixed,
            [2, 2, 2],
            Position::new3(0, 15, 0),
            Orientation3d::IDENTITY,
        );
        let start = [rod_start, obstacle];
        let end = [rod_start, obstacle];

        assert_eq!(
            rotational_sweep_candidate_pairs(&start, &end).expect("valid sweep candidates"),
            vec![RotationalSweepPair3d {
                left: EntityId(1),
                right: EntityId(2),
            }]
        );
    }

    #[test]
    fn far_sweep_pair_is_still_culled() {
        let rod = rigid_box(
            1,
            BodyKind::Dynamic,
            [20, 2, 2],
            Position::new3(0, 0, 0),
            Orientation3d::IDENTITY,
        );
        let obstacle = rigid_box(
            2,
            BodyKind::Fixed,
            [2, 2, 2],
            Position::new3(120, 120, 0),
            Orientation3d::IDENTITY,
        );

        assert!(
            rotational_sweep_candidate_pairs(&[rod, obstacle], &[rod, obstacle])
                .expect("valid separated sweep")
                .is_empty()
        );
    }

    #[test]
    fn shape_drift_fails_closed() {
        let start = rigid_box(
            1,
            BodyKind::Dynamic,
            [10, 10, 10],
            Position::new3(0, 0, 0),
            Orientation3d::IDENTITY,
        );
        let end = rigid_box(
            1,
            BodyKind::Dynamic,
            [11, 10, 10],
            Position::new3(0, 0, 0),
            Orientation3d::IDENTITY,
        );

        assert_eq!(
            rotational_sweep_candidate_pairs(&[start], &[end]),
            Err(RotationalSweepError3d::ShapeMismatch(EntityId(1)))
        );
    }
}
