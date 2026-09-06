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
    if cell_size_units > EXACT_F32_INTEGER_LIMIT {
        return None;
    }
    let grid = SpatialHash3D::new(exact_i64_to_f32(cell_size_units));
    let mut cells: BTreeMap<CellCoord3, Vec<usize>> = BTreeMap::new();

    for (body_index, body) in bodies.iter().copied().enumerate() {
        let bounds = swept_bounds(body, remaining_subticks, subtick_scale)?;
        let (minimum, maximum) = grid.cell_bounds(bounds);
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
                if bodies[left_index].kind == BodyKind::Fixed
                    && bodies[right_index].kind == BodyKind::Fixed
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
) -> Option<Aabb> {
    let mut minimum = [0.0_f32; 3];
    let mut maximum = [0.0_f32; 3];

    for axis in 0..3 {
        let half_scaled = i128::from(body.half_extents[axis]).checked_mul(subtick_scale)?;
        let end_center = body.center_scaled[axis].checked_add(
            i128::from(body.velocity[axis]).checked_mul(remaining_subticks)?,
        )?;
        let start_min = body.center_scaled[axis].checked_sub(half_scaled)?;
        let start_max = body.center_scaled[axis].checked_add(half_scaled)?;
        let end_min = end_center.checked_sub(half_scaled)?;
        let end_max = end_center.checked_add(half_scaled)?;
        let outward_min = scaled_floor_to_i64(start_min.min(end_min), subtick_scale)?;
        let outward_max = scaled_ceil_to_i64(start_max.max(end_max), subtick_scale)?;
        if !exact_f32_integer(outward_min) || !exact_f32_integer(outward_max) {
            return None;
        }
        minimum[axis] = exact_i64_to_f32(outward_min);
        maximum[axis] = exact_i64_to_f32(outward_max);
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
        SweptBroadPhaseBody, swept_bounds, swept_candidate_pairs,
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

    fn overlaps(left: spatial_kernels::Aabb, right: spatial_kernels::Aabb) -> bool {
        (0..3).all(|axis| left.max[axis] >= right.min[axis] && right.max[axis] >= left.min[axis])
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

        for left in 0..bodies.len() {
            for right in (left + 1)..bodies.len() {
                if bodies[left].kind == BodyKind::Fixed && bodies[right].kind == BodyKind::Fixed {
                    continue;
                }
                let left_bounds = swept_bounds(bodies[left], SCALE, SCALE).expect("left bounds");
                let right_bounds = swept_bounds(bodies[right], SCALE, SCALE).expect("right bounds");
                if overlaps(left_bounds, right_bounds) {
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
            body(
                BodyKind::Dynamic,
                [0, 0, 0],
                [1, 1, 1],
                [i32::MAX, 0, 0],
            ),
            body(BodyKind::Dynamic, [10, 0, 0], [1, 1, 1], [0, 0, 0]),
        ];
        assert!(swept_candidate_pairs(&bodies, SCALE, SCALE).is_none());
    }
}
