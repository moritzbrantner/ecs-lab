use std::collections::{BTreeMap, BTreeSet};

use ecs_physics::BodyKind;
use spatial_kernels::{Aabb, CellCoord3, SpatialHash3D};

const EXACT_F32_INTEGER_LIMIT: i64 = 1_i64 << 24;
const MAX_CELLS_PER_BODY: usize = 4_096;

#[derive(Clone, Copy, Debug)]
pub(crate) struct SweptBroadPhaseBody {
    pub kind: BodyKind,
    pub center_scaled: [i128; 3],
    pub half_extents: [i64; 3],
    pub velocity: [i32; 3],
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) struct BroadPhaseBounds3d {
    pub kind: BodyKind,
    pub minimum: [i64; 3],
    pub maximum: [i64; 3],
}

/// Builds a deterministic conservative candidate set from swept world-space AABBs.
///
/// `None` means the grid could not represent the supplied range cheaply and the caller must fall back
/// to the exact naive all-pairs path. Returning a fallback instead of dropping a body keeps this layer
/// strictly an acceleration structure rather than simulation authority.
pub(crate) fn swept_candidate_pairs(
    bodies: &[SweptBroadPhaseBody],
    remaining_subticks: i128,
    subtick_scale: i128,
) -> Option<Vec<(usize, usize)>> {
    if remaining_subticks < 0 || subtick_scale <= 0 {
        return None;
    }
    if bodies.len() < 2 {
        return Some(Vec::new());
    }

    let cell_size_units = bodies
        .iter()
        .filter(|body| body.kind == BodyKind::Dynamic)
        .flat_map(|body| body.half_extents)
        .max()
        .unwrap_or(1)
        .max(1)
        .checked_mul(2)?;
    let bounds = bodies
        .iter()
        .copied()
        .map(|body| swept_bounds(body, remaining_subticks, subtick_scale))
        .collect::<Option<Vec<_>>>()?;
    candidate_pairs_for_bounds(&bounds, cell_size_units)
}

/// Builds deterministic spatial-hash candidates for already conservative integer world-space AABBs.
///
/// This is the reusable discrete counterpart to [`swept_candidate_pairs`]. The supplied bounds remain
/// authoritative: the hash only removes pairs that cannot share a covered grid cell. `None` requests an
/// all-pairs fallback when coordinates or cell coverage exceed the exact/cheap grid contract.
pub(crate) fn aabb_candidate_pairs(
    bounds: &[BroadPhaseBounds3d],
) -> Option<Vec<(usize, usize)>> {
    if bounds.len() < 2 {
        return Some(Vec::new());
    }

    let mut cell_size_units = 1_i64;
    for body in bounds.iter().filter(|body| body.kind == BodyKind::Dynamic) {
        for axis in 0..3 {
            let span = body.maximum[axis].checked_sub(body.minimum[axis])?;
            if span < 0 {
                return None;
            }
            cell_size_units = cell_size_units.max(span.max(1));
        }
    }
    candidate_pairs_for_bounds(bounds, cell_size_units)
}

fn candidate_pairs_for_bounds(
    bounds: &[BroadPhaseBounds3d],
    cell_size_units: i64,
) -> Option<Vec<(usize, usize)>> {
    if cell_size_units <= 0 || cell_size_units > EXACT_F32_INTEGER_LIMIT {
        return None;
    }
    let grid = SpatialHash3D::new(exact_i64_to_f32(cell_size_units));
    let mut cells: BTreeMap<CellCoord3, Vec<usize>> = BTreeMap::new();

    for (body_index, body) in bounds.iter().copied().enumerate() {
        let world_bounds = integer_aabb(body)?;
        let (minimum, maximum) = grid.cell_bounds(world_bounds);
        let x_span = inclusive_span(minimum.x, maximum.x)?;
        let y_span = inclusive_span(minimum.y, maximum.y)?;
        let z_span = inclusive_span(minimum.z, maximum.z)?;
        let cell_count = x_span.checked_mul(y_span)?.checked_mul(z_span)?;
        if cell_count > MAX_CELLS_PER_BODY {
            return None;
        }

        for x in minimum.x..=maximum.x {
            for y in minimum.y..=maximum.y {
                for z in minimum.z..=maximum.z {
                    cells
                        .entry(CellCoord3::new(x, y, z))
                        .or_default()
                        .push(body_index);
                }
            }
        }
    }

    let mut pairs = BTreeSet::new();
    for bucket in cells.values() {
        for left_offset in 0..bucket.len() {
            for right_offset in (left_offset + 1)..bucket.len() {
                let left_index = bucket[left_offset];
                let right_index = bucket[right_offset];
                if bounds[left_index].kind == BodyKind::Fixed
                    && bounds[right_index].kind == BodyKind::Fixed
                {
                    continue;
                }
                pairs.insert((left_index.min(right_index), left_index.max(right_index)));
            }
        }
    }
    Some(pairs.into_iter().collect())
}

fn swept_bounds(
    body: SweptBroadPhaseBody,
    remaining_subticks: i128,
    subtick_scale: i128,
) -> Option<BroadPhaseBounds3d> {
    let mut minimum = [0_i64; 3];
    let mut maximum = [0_i64; 3];

    for axis in 0..3 {
        let half_scaled = i128::from(body.half_extents[axis]).checked_mul(subtick_scale)?;
        let end_center = body.center_scaled[axis]
            .checked_add(i128::from(body.velocity[axis]).checked_mul(remaining_subticks)?)?;
        let start_min = body.center_scaled[axis].checked_sub(half_scaled)?;
        let start_max = body.center_scaled[axis].checked_add(half_scaled)?;
        let end_min = end_center.checked_sub(half_scaled)?;
        let end_max = end_center.checked_add(half_scaled)?;
        minimum[axis] = scaled_floor_to_i64(start_min.min(end_min), subtick_scale)?;
        maximum[axis] = scaled_ceil_to_i64(start_max.max(end_max), subtick_scale)?;
    }

    Some(BroadPhaseBounds3d {
        kind: body.kind,
        minimum,
        maximum,
    })
}

fn integer_aabb(bounds: BroadPhaseBounds3d) -> Option<Aabb> {
    let mut minimum = [0.0_f32; 3];
    let mut maximum = [0.0_f32; 3];
    for axis in 0..3 {
        let low = bounds.minimum[axis];
        let high = bounds.maximum[axis];
        if low > high || !exact_f32_integer(low) || !exact_f32_integer(high) {
            return None;
        }
        minimum[axis] = exact_i64_to_f32(low);
        maximum[axis] = exact_i64_to_f32(high);
    }
    Some(Aabb::new(minimum, maximum))
}

fn inclusive_span(minimum: i32, maximum: i32) -> Option<usize> {
    let span = i64::from(maximum)
        .checked_sub(i64::from(minimum))?
        .checked_add(1)?;
    usize::try_from(span).ok()
}

fn scaled_floor_to_i64(value: i128, scale: i128) -> Option<i64> {
    i64::try_from(value.div_euclid(scale)).ok()
}

fn scaled_ceil_to_i64(value: i128, scale: i128) -> Option<i64> {
    let quotient = value.div_euclid(scale);
    let remainder = value.rem_euclid(scale);
    let rounded = if remainder == 0 {
        quotient
    } else {
        quotient.checked_add(1)?
    };
    i64::try_from(rounded).ok()
}

const fn exact_f32_integer(value: i64) -> bool {
    value >= -EXACT_F32_INTEGER_LIMIT && value <= EXACT_F32_INTEGER_LIMIT
}

#[allow(clippy::cast_precision_loss)]
fn exact_i64_to_f32(value: i64) -> f32 {
    value as f32
}

#[cfg(test)]
mod tests {
    use std::collections::BTreeSet;

    use ecs_physics::BodyKind;

    use super::{
        BroadPhaseBounds3d, SweptBroadPhaseBody, aabb_candidate_pairs, swept_bounds,
        swept_candidate_pairs,
    };

    const SCALE: i128 = 1_i128 << 32;

    fn body(
        kind: BodyKind,
        center: [i64; 3],
        half_extents: [i64; 3],
        velocity: [i32; 3],
    ) -> SweptBroadPhaseBody {
        SweptBroadPhaseBody {
            kind,
            center_scaled: center.map(|value| i128::from(value) * SCALE),
            half_extents,
            velocity,
        }
    }

    fn bounds(
        kind: BodyKind,
        minimum: [i64; 3],
        maximum: [i64; 3],
    ) -> BroadPhaseBounds3d {
        BroadPhaseBounds3d {
            kind,
            minimum,
            maximum,
        }
    }

    fn overlaps(left: BroadPhaseBounds3d, right: BroadPhaseBounds3d) -> bool {
        (0..3).all(|axis| {
            left.maximum[axis] >= right.minimum[axis]
                && right.maximum[axis] >= left.minimum[axis]
        })
    }

    #[test]
    fn fast_swept_pair_shares_a_candidate_cell() {
        let bodies = [
            body(BodyKind::Dynamic, [-20, 0, 0], [1, 1, 1], [40, 0, 0]),
            body(BodyKind::Dynamic, [10, 0, 0], [1, 1, 1], [0, 0, 0]),
        ];
        let pairs = swept_candidate_pairs(&bodies, SCALE, SCALE).expect("grid should be usable");
        assert_eq!(pairs, vec![(0, 1)]);
    }

    #[test]
    fn separated_bodies_reduce_candidate_count() {
        let bodies = (0..24)
            .map(|index| {
                body(
                    BodyKind::Dynamic,
                    [i64::from(index) * 20, 0, 0],
                    [1, 1, 1],
                    [0, 0, 0],
                )
            })
            .collect::<Vec<_>>();
        let pairs = swept_candidate_pairs(&bodies, SCALE, SCALE).expect("grid should be usable");
        assert!(pairs.len() < bodies.len() * (bodies.len() - 1) / 2);
        assert!(pairs.is_empty());
    }

    #[test]
    fn arbitrary_world_aabbs_share_the_same_deterministic_hash() {
        let bodies = [
            bounds(BodyKind::Dynamic, [-4, -2, -2], [4, 2, 2]),
            bounds(BodyKind::Dynamic, [3, -2, -2], [9, 2, 2]),
            bounds(BodyKind::Dynamic, [40, -2, -2], [44, 2, 2]),
            bounds(BodyKind::Fixed, [-2, -6, -2], [2, -4, 2]),
        ];
        let pairs = aabb_candidate_pairs(&bodies).expect("grid should be usable");

        assert!(pairs.contains(&(0, 1)));
        assert!(!pairs.contains(&(0, 2)));
        assert!(!pairs.contains(&(1, 2)));
    }

    #[test]
    fn every_overlapping_swept_world_aabb_is_retained() {
        let bodies = vec![
            body(BodyKind::Dynamic, [-12, 0, 0], [2, 2, 2], [8, 0, 0]),
            body(BodyKind::Dynamic, [-2, 0, 0], [2, 2, 2], [0, 0, 0]),
            body(BodyKind::Dynamic, [7, 4, 0], [1, 2, 1], [-5, -2, 0]),
            body(BodyKind::Fixed, [0, -5, 0], [20, 1, 20], [0, 0, 0]),
            body(BodyKind::Dynamic, [30, 20, 30], [1, 1, 1], [0, 0, 0]),
        ];
        let candidate_set = swept_candidate_pairs(&bodies, SCALE, SCALE)
            .expect("grid should be usable")
            .into_iter()
            .collect::<BTreeSet<_>>();
        let bounds = bodies
            .iter()
            .copied()
            .map(|body| swept_bounds(body, SCALE, SCALE).expect("swept bounds"))
            .collect::<Vec<_>>();

        for left in 0..bodies.len() {
            for right in (left + 1)..bodies.len() {
                if bodies[left].kind == BodyKind::Fixed && bodies[right].kind == BodyKind::Fixed {
                    continue;
                }
                if overlaps(bounds[left], bounds[right]) {
                    assert!(
                        candidate_set.contains(&(left, right)),
                        "overlapping swept pair {left}-{right} was dropped"
                    );
                }
            }
        }
    }

    #[test]
    fn oversized_grid_falls_back_instead_of_dropping_pairs() {
        let bodies = [
            body(BodyKind::Dynamic, [0, 0, 0], [1, 1, 1], [i32::MAX, 0, 0]),
            body(BodyKind::Dynamic, [10, 0, 0], [1, 1, 1], [0, 0, 0]),
        ];
        assert!(swept_candidate_pairs(&bodies, SCALE, SCALE).is_none());
    }

    #[test]
    fn arbitrary_bounds_outside_exact_grid_request_fallback() {
        let bodies = [
            bounds(
                BodyKind::Dynamic,
                [-(1_i64 << 25), 0, 0],
                [-(1_i64 << 25) + 2, 2, 2],
            ),
            bounds(BodyKind::Dynamic, [0, 0, 0], [2, 2, 2]),
        ];
        assert!(aabb_candidate_pairs(&bodies).is_none());
    }
}
