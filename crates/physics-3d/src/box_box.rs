use std::fmt;

use ecs_physics::{BodyKind, MATERIAL_SCALE, PhysicsMaterial};
use ecs_workload::{EntityId, Position, Velocity};

use crate::{
    angular::{
        ANGULAR_VELOCITY_SCALE, AngularError3d, AngularState3d, AngularVelocity3d,
        ORIENTATION_SCALE, Orientation3d, box_inertia,
    },
    box_plane::{BoxPlaneError3d, oriented_box_vertices},
    oriented_box::{ObbAxisFeature3d, OrientedBox3d, OrientedBoxError3d, obb_contact_seed},
    types::PhysicsBody3d,
};

const RESPONSE_SCALE: i128 = 1_i128 << 50;

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

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct BoxBoxContact3d {
    /// Stable reduced-manifold contact point derived from the Rust-owned OBB geometry.
    pub point: Position,
    /// Primitive SAT axis oriented from the left body toward the right body.
    pub axis: [i128; 3],
    pub feature: ObbAxisFeature3d,
    pub overlap_numerator: u128,
    pub axis_length_squared: u128,
    pub left_support_mask: u8,
    pub right_support_mask: u8,
    /// Scalar multiplier applied to the primitive SAT axis. The actual impulse vector is
    /// `normal_impulse_units * axis`, with opposite signs on the two bodies.
    pub normal_impulse_units: i64,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct BoxBoxStep3d {
    pub left: RigidBoxState3d,
    pub right: RigidBoxState3d,
    pub contact: Option<BoxBoxContact3d>,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BoxBoxError3d {
    SameEntity(EntityId),
    InvalidHalfExtents(EntityId),
    ZeroMass(EntityId),
    RestitutionOutOfRange(EntityId, u16),
    Angular(AngularError3d),
    Geometry(OrientedBoxError3d),
    ArithmeticOverflow,
}

impl fmt::Display for BoxBoxError3d {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::SameEntity(entity) => write!(
                formatter,
                "OBB response requires two distinct bodies, got entity {} twice",
                entity.0
            ),
            Self::InvalidHalfExtents(entity) => write!(
                formatter,
                "OBB response body {} requires strictly positive half extents",
                entity.0
            ),
            Self::ZeroMass(entity) => write!(
                formatter,
                "dynamic OBB response body {} requires non-zero mass",
                entity.0
            ),
            Self::RestitutionOutOfRange(entity, value) => write!(
                formatter,
                "OBB response body {} has restitution {value}, expected 0..={MATERIAL_SCALE}",
                entity.0
            ),
            Self::Angular(error) => write!(formatter, "OBB angular response failed: {error}"),
            Self::Geometry(error) => write!(formatter, "OBB contact geometry failed: {error}"),
            Self::ArithmeticOverflow => {
                write!(formatter, "OBB contact response arithmetic overflowed")
            }
        }
    }
}

impl std::error::Error for BoxBoxError3d {}

impl From<AngularError3d> for BoxBoxError3d {
    fn from(value: AngularError3d) -> Self {
        Self::Angular(value)
    }
}

impl From<OrientedBoxError3d> for BoxBoxError3d {
    fn from(value: OrientedBoxError3d) -> Self {
        Self::Geometry(value)
    }
}

/// Resolves one overlapping oriented-box pair with a deterministic normal impulse.
///
/// The existing swept-AABB path remains the broad phase. This function consumes the Rust-owned SAT
/// contact seed, reduces the support sets to one stable contact point, measures relative velocity at that
/// point including angular motion, and applies one equal-and-opposite normal impulse. Parallel,
/// world-axis-aligned face supports are first clipped to their actual rectangular overlap so an overhanging
/// body receives its impulse inside the shared patch rather than at either full-face center. Other
/// dynamic↔dynamic pairs keep the midpoint of their support centroids. For dynamic↔fixed contacts with
/// unequal support footprints, the tighter support feature remains the bounded fallback anchor and is
/// projected onto the shared support mid-plane. That prevents a very large boundary face (for example a
/// floor slab) from moving the impulse toward the boundary's own center and inventing torque on an
/// otherwise centered resting box.
///
/// Exact zero-depth contacts tied across more than one SAT axis are deliberately observation-only in this
/// discrete response seam. At an edge/corner coincidence there is no unique normal yet, so applying the
/// first stable tied axis would turn deterministic feature ordering into arbitrary torque. Once a unique
/// axis wins or the pair has real penetration, ordinary coupled linear/angular response applies. This
/// avoids solver-created pre-impact spin without suppressing genuine penetrating edge/corner response.
///
/// This is deliberately a narrow response seam rather than a complete rigid-body world step: it does not
/// integrate gravity/position/orientation, perform rotational CCD, iterate a multi-contact island, or add
/// tangential friction. Those remain separate solver stages so the demo can adopt angular behavior without
/// replacing the conservative swept broad phase.
///
/// # Errors
///
/// Returns [`BoxBoxError3d`] for invalid bodies/orientations, malformed material values, or checked
/// arithmetic overflow.
pub fn resolve_box_box_contact(
    left_state: RigidBoxState3d,
    left_body: PhysicsBody3d,
    right_state: RigidBoxState3d,
    right_body: PhysicsBody3d,
) -> Result<BoxBoxStep3d, BoxBoxError3d> {
    validate_body(left_body)?;
    validate_body(right_body)?;
    if left_body.entity == right_body.entity {
        return Err(BoxBoxError3d::SameEntity(left_body.entity));
    }

    let left_shape = OrientedBox3d::new(
        left_state.center,
        left_body.half_extents,
        left_state.angular.orientation,
    );
    let right_shape = OrientedBox3d::new(
        right_state.center,
        right_body.half_extents,
        right_state.angular.orientation,
    );
    let Some(seed) = obb_contact_seed(left_shape, right_shape)? else {
        return Ok(BoxBoxStep3d {
            left: left_state,
            right: right_state,
            contact: None,
        });
    };

    let left_vertices = oriented_box_vertices(
        left_state.center,
        left_body.half_extents,
        left_state.angular.orientation,
    )
    .map_err(map_vertex_error)?;
    let right_vertices = oriented_box_vertices(
        right_state.center,
        right_body.half_extents,
        right_state.angular.orientation,
    )
    .map_err(map_vertex_error)?;
    let left_support = support_centroid(&left_vertices, seed.left_support_mask)?;
    let right_support = support_centroid(&right_vertices, seed.right_support_mask)?;
    let point = reduced_contact_point(
        &left_vertices,
        seed.left_support_mask,
        left_support,
        left_body,
        &right_vertices,
        seed.right_support_mask,
        right_support,
        right_body,
        seed.axis,
        seed.axis_length_squared,
    )?;
    let left_offset = position_delta(left_state.center, point)?;
    let right_offset = position_delta(right_state.center, point)?;

    let left_velocity = contact_velocity(left_state, left_offset)?;
    let right_velocity = contact_velocity(right_state, right_offset)?;
    let relative_velocity = [
        checked_sub(i128::from(right_velocity[0]), i128::from(left_velocity[0]))?,
        checked_sub(i128::from(right_velocity[1]), i128::from(left_velocity[1]))?,
        checked_sub(i128::from(right_velocity[2]), i128::from(left_velocity[2]))?,
    ];
    let normal_velocity = checked_dot(relative_velocity, seed.axis)?;
    let ambiguous_zero_depth_touch = seed.overlap_numerator == 0 && seed.minimum_axis_ties > 1;

    let mut left = left_state;
    let mut right = right_state;
    let normal_impulse_units = if !ambiguous_zero_depth_touch
        && normal_velocity < 0
        && (left_body.kind == BodyKind::Dynamic || right_body.kind == BodyKind::Dynamic)
    {
        let impulse = normal_impulse(
            left_body,
            left_state.angular.orientation,
            left_offset,
            right_body,
            right_state.angular.orientation,
            right_offset,
            seed.axis,
            seed.axis_length_squared,
            normal_velocity,
        )?;
        if impulse > 0 {
            let left_impulse = scale_axis(seed.axis, -i128::from(impulse))?;
            let right_impulse = scale_axis(seed.axis, i128::from(impulse))?;
            apply_body_impulse(&mut left, left_body, left_offset, left_impulse)?;
            apply_body_impulse(&mut right, right_body, right_offset, right_impulse)?;
        }
        impulse
    } else {
        0
    };

    Ok(BoxBoxStep3d {
        left,
        right,
        contact: Some(BoxBoxContact3d {
            point,
            axis: seed.axis,
            feature: seed.feature,
            overlap_numerator: seed.overlap_numerator,
            axis_length_squared: seed.axis_length_squared,
            left_support_mask: seed.left_support_mask,
            right_support_mask: seed.right_support_mask,
            normal_impulse_units,
        }),
    })
}

fn validate_body(body: PhysicsBody3d) -> Result<(), BoxBoxError3d> {
    if body.half_extents.iter().any(|extent| *extent <= 0) {
        return Err(BoxBoxError3d::InvalidHalfExtents(body.entity));
    }
    if body.kind == BodyKind::Dynamic && body.mass_units == 0 {
        return Err(BoxBoxError3d::ZeroMass(body.entity));
    }
    if body.material.restitution_milli > MATERIAL_SCALE {
        return Err(BoxBoxError3d::RestitutionOutOfRange(
            body.entity,
            body.material.restitution_milli,
        ));
    }
    Ok(())
}

fn map_vertex_error(error: BoxPlaneError3d) -> BoxBoxError3d {
    match error {
        BoxPlaneError3d::Angular(error) => BoxBoxError3d::Angular(error),
        _ => BoxBoxError3d::ArithmeticOverflow,
    }
}

fn support_centroid(vertices: &[Position; 8], mask: u8) -> Result<Position, BoxBoxError3d> {
    let mut sum = [0_i128; 3];
    let mut count = 0_i128;
    for (index, vertex) in vertices.iter().enumerate() {
        if mask & (1_u8 << index) == 0 {
            continue;
        }
        count = checked_add(count, 1)?;
        sum[0] = checked_add(sum[0], i128::from(vertex.x))?;
        sum[1] = checked_add(sum[1], i128::from(vertex.y))?;
        sum[2] = checked_add(sum[2], i128::from(vertex.z))?;
    }
    if count == 0 {
        return Err(BoxBoxError3d::ArithmeticOverflow);
    }
    Ok(Position::new3(
        i64::try_from(div_round_nearest(sum[0], count)?)
            .map_err(|_| BoxBoxError3d::ArithmeticOverflow)?,
        i64::try_from(div_round_nearest(sum[1], count)?)
            .map_err(|_| BoxBoxError3d::ArithmeticOverflow)?,
        i64::try_from(div_round_nearest(sum[2], count)?)
            .map_err(|_| BoxBoxError3d::ArithmeticOverflow)?,
    ))
}

#[allow(clippy::too_many_arguments)]
fn reduced_contact_point(
    left_vertices: &[Position; 8],
    left_mask: u8,
    left_support: Position,
    left_body: PhysicsBody3d,
    right_vertices: &[Position; 8],
    right_mask: u8,
    right_support: Position,
    right_body: PhysicsBody3d,
    axis: [i128; 3],
    axis_length_squared: u128,
) -> Result<Position, BoxBoxError3d> {
    if let Some(point) = axis_aligned_face_overlap_centroid(
        left_vertices,
        left_mask,
        left_support,
        right_vertices,
        right_mask,
        right_support,
        axis,
    )? {
        return Ok(point);
    }

    if left_body.kind == right_body.kind {
        return midpoint(left_support, right_support);
    }

    let left_spread = support_spread_squared(left_vertices, left_mask, left_support)?;
    let right_spread = support_spread_squared(right_vertices, right_mask, right_support)?;
    let anchor = match left_spread.cmp(&right_spread) {
        std::cmp::Ordering::Less => left_support,
        std::cmp::Ordering::Greater => right_support,
        std::cmp::Ordering::Equal => return midpoint(left_support, right_support),
    };
    project_to_support_midplane(
        anchor,
        left_support,
        right_support,
        axis,
        axis_length_squared,
    )
}

#[allow(clippy::too_many_arguments)]
fn axis_aligned_face_overlap_centroid(
    left_vertices: &[Position; 8],
    left_mask: u8,
    left_support: Position,
    right_vertices: &[Position; 8],
    right_mask: u8,
    right_support: Position,
    axis: [i128; 3],
) -> Result<Option<Position>, BoxBoxError3d> {
    if left_mask.count_ones() != 4 || right_mask.count_ones() != 4 {
        return Ok(None);
    }
    let Some(normal_axis) = coordinate_axis(axis) else {
        return Ok(None);
    };
    let tangent_axes = match normal_axis {
        0 => [1, 2],
        1 => [0, 2],
        2 => [0, 1],
        _ => return Ok(None),
    };
    if !support_is_axis_aligned_rectangle(left_vertices, left_mask, tangent_axes)
        || !support_is_axis_aligned_rectangle(right_vertices, right_mask, tangent_axes)
    {
        return Ok(None);
    }

    let mut coordinate = [0_i64; 3];
    coordinate[normal_axis] = midpoint_axis(
        position_component(left_support, normal_axis),
        position_component(right_support, normal_axis),
    )?;
    for tangent_axis in tangent_axes {
        let (left_minimum, left_maximum) =
            support_interval(left_vertices, left_mask, tangent_axis)?;
        let (right_minimum, right_maximum) =
            support_interval(right_vertices, right_mask, tangent_axis)?;
        let overlap_minimum = left_minimum.max(right_minimum);
        let overlap_maximum = left_maximum.min(right_maximum);
        if overlap_minimum > overlap_maximum {
            return Ok(None);
        }
        coordinate[tangent_axis] = midpoint_axis(overlap_minimum, overlap_maximum)?;
    }
    Ok(Some(Position::new3(
        coordinate[0],
        coordinate[1],
        coordinate[2],
    )))
}

fn coordinate_axis(axis: [i128; 3]) -> Option<usize> {
    let mut found = None;
    for (index, component) in axis.into_iter().enumerate() {
        if component == 0 {
            continue;
        }
        if found.is_some() {
            return None;
        }
        found = Some(index);
    }
    found
}

fn support_is_axis_aligned_rectangle(
    vertices: &[Position; 8],
    mask: u8,
    tangent_axes: [usize; 2],
) -> bool {
    tangent_axes.into_iter().all(|axis| {
        let mut unique = [0_i64; 4];
        let mut unique_count = 0_usize;
        for (index, vertex) in vertices.iter().enumerate() {
            if mask & (1_u8 << index) == 0 {
                continue;
            }
            let value = position_component(*vertex, axis);
            if unique[..unique_count].contains(&value) {
                continue;
            }
            if unique_count == unique.len() {
                return false;
            }
            unique[unique_count] = value;
            unique_count += 1;
        }
        unique_count == 2
    })
}

fn support_interval(
    vertices: &[Position; 8],
    mask: u8,
    axis: usize,
) -> Result<(i64, i64), BoxBoxError3d> {
    let mut minimum = None;
    let mut maximum = None;
    for (index, vertex) in vertices.iter().enumerate() {
        if mask & (1_u8 << index) == 0 {
            continue;
        }
        let value = position_component(*vertex, axis);
        minimum = Some(minimum.map_or(value, |current: i64| current.min(value)));
        maximum = Some(maximum.map_or(value, |current: i64| current.max(value)));
    }
    match (minimum, maximum) {
        (Some(minimum), Some(maximum)) => Ok((minimum, maximum)),
        _ => Err(BoxBoxError3d::ArithmeticOverflow),
    }
}

fn position_component(position: Position, axis: usize) -> i64 {
    match axis {
        0 => position.x,
        1 => position.y,
        2 => position.z,
        _ => 0,
    }
}

fn support_spread_squared(
    vertices: &[Position; 8],
    mask: u8,
    center: Position,
) -> Result<u128, BoxBoxError3d> {
    let mut total = 0_u128;
    let mut count = 0_u8;
    for (index, vertex) in vertices.iter().enumerate() {
        if mask & (1_u8 << index) == 0 {
            continue;
        }
        count = count
            .checked_add(1)
            .ok_or(BoxBoxError3d::ArithmeticOverflow)?;
        for component in [
            i128::from(vertex.x) - i128::from(center.x),
            i128::from(vertex.y) - i128::from(center.y),
            i128::from(vertex.z) - i128::from(center.z),
        ] {
            let magnitude = component.unsigned_abs();
            let square = magnitude
                .checked_mul(magnitude)
                .ok_or(BoxBoxError3d::ArithmeticOverflow)?;
            total = total
                .checked_add(square)
                .ok_or(BoxBoxError3d::ArithmeticOverflow)?;
        }
    }
    if count == 0 {
        return Err(BoxBoxError3d::ArithmeticOverflow);
    }
    Ok(total)
}

fn project_to_support_midplane(
    anchor: Position,
    left_support: Position,
    right_support: Position,
    axis: [i128; 3],
    axis_length_squared: u128,
) -> Result<Position, BoxBoxError3d> {
    let length_squared =
        i128::try_from(axis_length_squared).map_err(|_| BoxBoxError3d::ArithmeticOverflow)?;
    if length_squared <= 0 {
        return Err(BoxBoxError3d::ArithmeticOverflow);
    }
    let target_projection = div_round_nearest(
        checked_add(
            dot_position(left_support, axis)?,
            dot_position(right_support, axis)?,
        )?,
        2,
    )?;
    let projection_delta = checked_sub(target_projection, dot_position(anchor, axis)?)?;
    let mut coordinate = [
        i128::from(anchor.x),
        i128::from(anchor.y),
        i128::from(anchor.z),
    ];
    for index in 0..3 {
        coordinate[index] = checked_add(
            coordinate[index],
            div_round_nearest(checked_mul(projection_delta, axis[index])?, length_squared)?,
        )?;
    }
    Ok(Position::new3(
        i64::try_from(coordinate[0]).map_err(|_| BoxBoxError3d::ArithmeticOverflow)?,
        i64::try_from(coordinate[1]).map_err(|_| BoxBoxError3d::ArithmeticOverflow)?,
        i64::try_from(coordinate[2]).map_err(|_| BoxBoxError3d::ArithmeticOverflow)?,
    ))
}

fn dot_position(position: Position, axis: [i128; 3]) -> Result<i128, BoxBoxError3d> {
    checked_dot(
        [
            i128::from(position.x),
            i128::from(position.y),
            i128::from(position.z),
        ],
        axis,
    )
}

fn midpoint(left: Position, right: Position) -> Result<Position, BoxBoxError3d> {
    Ok(Position::new3(
        midpoint_axis(left.x, right.x)?,
        midpoint_axis(left.y, right.y)?,
        midpoint_axis(left.z, right.z)?,
    ))
}

fn midpoint_axis(left: i64, right: i64) -> Result<i64, BoxBoxError3d> {
    let sum = checked_add(i128::from(left), i128::from(right))?;
    i64::try_from(div_round_nearest(sum, 2)?).map_err(|_| BoxBoxError3d::ArithmeticOverflow)
}

fn position_delta(center: Position, point: Position) -> Result<[i64; 3], BoxBoxError3d> {
    Ok([
        point
            .x
            .checked_sub(center.x)
            .ok_or(BoxBoxError3d::ArithmeticOverflow)?,
        point
            .y
            .checked_sub(center.y)
            .ok_or(BoxBoxError3d::ArithmeticOverflow)?,
        point
            .z
            .checked_sub(center.z)
            .ok_or(BoxBoxError3d::ArithmeticOverflow)?,
    ])
}

fn contact_velocity(state: RigidBoxState3d, offset: [i64; 3]) -> Result<[i64; 3], BoxBoxError3d> {
    let omega = state.angular.angular_velocity;
    let rotation_x = checked_sub(
        checked_mul(i128::from(omega.y), i128::from(offset[2]))?,
        checked_mul(i128::from(omega.z), i128::from(offset[1]))?,
    )?;
    let rotation_y = checked_sub(
        checked_mul(i128::from(omega.z), i128::from(offset[0]))?,
        checked_mul(i128::from(omega.x), i128::from(offset[2]))?,
    )?;
    let rotation_z = checked_sub(
        checked_mul(i128::from(omega.x), i128::from(offset[1]))?,
        checked_mul(i128::from(omega.y), i128::from(offset[0]))?,
    )?;
    let scale = i128::from(ANGULAR_VELOCITY_SCALE);
    Ok([
        add_linear_rotation(state.linear_velocity.x, rotation_x, scale)?,
        add_linear_rotation(state.linear_velocity.y, rotation_y, scale)?,
        add_linear_rotation(state.linear_velocity.z, rotation_z, scale)?,
    ])
}

fn add_linear_rotation(
    linear: i32,
    rotational_numerator: i128,
    scale: i128,
) -> Result<i64, BoxBoxError3d> {
    let rotational = i64::try_from(div_round_nearest(rotational_numerator, scale)?)
        .map_err(|_| BoxBoxError3d::ArithmeticOverflow)?;
    i64::from(linear)
        .checked_add(rotational)
        .ok_or(BoxBoxError3d::ArithmeticOverflow)
}

#[allow(clippy::too_many_arguments)]
fn normal_impulse(
    left_body: PhysicsBody3d,
    left_orientation: Orientation3d,
    left_offset: [i64; 3],
    right_body: PhysicsBody3d,
    right_orientation: Orientation3d,
    right_offset: [i64; 3],
    axis: [i128; 3],
    axis_length_squared: u128,
    normal_velocity: i128,
) -> Result<i64, BoxBoxError3d> {
    let effective_inverse_mass = checked_add(
        body_effective_inverse_mass_scaled(
            left_body,
            left_orientation,
            left_offset,
            axis,
            axis_length_squared,
        )?,
        body_effective_inverse_mass_scaled(
            right_body,
            right_orientation,
            right_offset,
            axis,
            axis_length_squared,
        )?,
    )?;
    if effective_inverse_mass <= 0 {
        return Ok(0);
    }

    let restitution = combined_restitution(left_body.material, right_body.material);
    let closing_speed = normal_velocity
        .checked_neg()
        .ok_or(BoxBoxError3d::ArithmeticOverflow)?;
    let bounce_scale = checked_add(i128::from(MATERIAL_SCALE), i128::from(restitution))?;
    let numerator = checked_mul(checked_mul(closing_speed, bounce_scale)?, RESPONSE_SCALE)?;
    let denominator = checked_mul(i128::from(MATERIAL_SCALE), effective_inverse_mass)?;
    i64::try_from(div_round_nearest(numerator, denominator)?)
        .map_err(|_| BoxBoxError3d::ArithmeticOverflow)
}

fn combined_restitution(left: PhysicsMaterial, right: PhysicsMaterial) -> u16 {
    left.restitution_milli.max(right.restitution_milli)
}

fn body_effective_inverse_mass_scaled(
    body: PhysicsBody3d,
    orientation: Orientation3d,
    contact_offset: [i64; 3],
    axis: [i128; 3],
    axis_length_squared: u128,
) -> Result<i128, BoxBoxError3d> {
    if body.kind == BodyKind::Fixed {
        return Ok(0);
    }

    let length_squared =
        i128::try_from(axis_length_squared).map_err(|_| BoxBoxError3d::ArithmeticOverflow)?;
    let translational = checked_mul(RESPONSE_SCALE, length_squared)? / i128::from(body.mass_units);
    let angular_impulse = cross_i64_i128(contact_offset, axis)?;
    let local = rotate_inverse(orientation, angular_impulse)?;
    let inertia = box_inertia(body)?;
    let mut rotational = 0_i128;
    for (index, component) in local.into_iter().enumerate() {
        let inverse_inertia =
            inverse_inertia_scaled(inertia.principal_numerators[index], inertia.denominator)?;
        rotational = checked_add(
            rotational,
            checked_mul(checked_mul(component, component)?, inverse_inertia)?,
        )?;
    }
    checked_add(translational, rotational)
}

fn apply_body_impulse(
    state: &mut RigidBoxState3d,
    body: PhysicsBody3d,
    contact_offset: [i64; 3],
    impulse: [i128; 3],
) -> Result<(), BoxBoxError3d> {
    if body.kind == BodyKind::Fixed {
        return Ok(());
    }

    state.linear_velocity = Velocity::new3(
        add_linear_impulse_axis(state.linear_velocity.x, impulse[0], body.mass_units)?,
        add_linear_impulse_axis(state.linear_velocity.y, impulse[1], body.mass_units)?,
        add_linear_impulse_axis(state.linear_velocity.z, impulse[2], body.mass_units)?,
    );

    let angular_impulse = cross_i64_i128(contact_offset, impulse)?;
    let local_impulse = rotate_inverse(state.angular.orientation, angular_impulse)?;
    let inertia = box_inertia(body)?;
    let mut local_delta = [0_i128; 3];
    for (index, (target, component)) in local_delta.iter_mut().zip(local_impulse).enumerate() {
        let inverse_inertia =
            inverse_inertia_scaled(inertia.principal_numerators[index], inertia.denominator)?;
        let numerator = checked_mul(
            checked_mul(component, inverse_inertia)?,
            i128::from(ANGULAR_VELOCITY_SCALE),
        )?;
        *target = div_round_nearest(numerator, RESPONSE_SCALE)?;
    }
    let world_delta = rotate_forward(state.angular.orientation, local_delta)?;
    state.angular.angular_velocity = AngularVelocity3d::new(
        add_angular_axis(state.angular.angular_velocity.x, world_delta[0])?,
        add_angular_axis(state.angular.angular_velocity.y, world_delta[1])?,
        add_angular_axis(state.angular.angular_velocity.z, world_delta[2])?,
    );
    Ok(())
}

fn add_linear_impulse_axis(
    current: i32,
    impulse: i128,
    mass_units: u32,
) -> Result<i32, BoxBoxError3d> {
    let delta = div_round_nearest(impulse, i128::from(mass_units))?;
    let next = checked_add(i128::from(current), delta)?;
    i32::try_from(next).map_err(|_| BoxBoxError3d::ArithmeticOverflow)
}

fn add_angular_axis(current: i32, delta: i128) -> Result<i32, BoxBoxError3d> {
    let next = checked_add(i128::from(current), delta)?;
    i32::try_from(next).map_err(|_| BoxBoxError3d::ArithmeticOverflow)
}

fn inverse_inertia_scaled(
    principal_numerator: u128,
    denominator: u32,
) -> Result<i128, BoxBoxError3d> {
    if principal_numerator == 0 {
        return Err(BoxBoxError3d::ArithmeticOverflow);
    }
    let principal =
        i128::try_from(principal_numerator).map_err(|_| BoxBoxError3d::ArithmeticOverflow)?;
    Ok(checked_mul(RESPONSE_SCALE, i128::from(denominator))? / principal)
}

fn cross_i64_i128(left: [i64; 3], right: [i128; 3]) -> Result<[i128; 3], BoxBoxError3d> {
    let left = left.map(i128::from);
    Ok([
        cross_component(left[1], right[2], left[2], right[1])?,
        cross_component(left[2], right[0], left[0], right[2])?,
        cross_component(left[0], right[1], left[1], right[0])?,
    ])
}

fn cross_component(
    left_a: i128,
    right_a: i128,
    left_b: i128,
    right_b: i128,
) -> Result<i128, BoxBoxError3d> {
    checked_sub(checked_mul(left_a, right_a)?, checked_mul(left_b, right_b)?)
}

fn checked_dot(left: [i128; 3], right: [i128; 3]) -> Result<i128, BoxBoxError3d> {
    checked_add(
        checked_add(
            checked_mul(left[0], right[0])?,
            checked_mul(left[1], right[1])?,
        )?,
        checked_mul(left[2], right[2])?,
    )
}

fn scale_axis(axis: [i128; 3], scale: i128) -> Result<[i128; 3], BoxBoxError3d> {
    Ok([
        checked_mul(axis[0], scale)?,
        checked_mul(axis[1], scale)?,
        checked_mul(axis[2], scale)?,
    ])
}

fn rotate_inverse(
    orientation: Orientation3d,
    vector: [i128; 3],
) -> Result<[i128; 3], BoxBoxError3d> {
    let matrix = rotation_matrix(orientation.normalized()?)?;
    rotate_with_matrix(
        [
            [matrix[0][0], matrix[1][0], matrix[2][0]],
            [matrix[0][1], matrix[1][1], matrix[2][1]],
            [matrix[0][2], matrix[1][2], matrix[2][2]],
        ],
        vector,
    )
}

fn rotate_forward(
    orientation: Orientation3d,
    vector: [i128; 3],
) -> Result<[i128; 3], BoxBoxError3d> {
    rotate_with_matrix(rotation_matrix(orientation.normalized()?)?, vector)
}

fn rotation_matrix(orientation: Orientation3d) -> Result<[[i128; 3]; 3], BoxBoxError3d> {
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

fn rotate_with_matrix(
    matrix: [[i128; 3]; 3],
    vector: [i128; 3],
) -> Result<[i128; 3], BoxBoxError3d> {
    let scale = i128::from(ORIENTATION_SCALE);
    let mut output = [0_i128; 3];
    for (target, row) in output.iter_mut().zip(matrix) {
        let sum = checked_add(
            checked_add(
                checked_mul(row[0], vector[0])?,
                checked_mul(row[1], vector[1])?,
            )?,
            checked_mul(row[2], vector[2])?,
        )?;
        *target = div_round_nearest(sum, scale)?;
    }
    Ok(output)
}

fn scaled_twice(value: i128, scale: i128) -> Result<i128, BoxBoxError3d> {
    div_round_nearest(checked_mul(value, 2)?, scale)
}

fn checked_mul(left: i128, right: i128) -> Result<i128, BoxBoxError3d> {
    left.checked_mul(right)
        .ok_or(BoxBoxError3d::ArithmeticOverflow)
}

fn checked_add(left: i128, right: i128) -> Result<i128, BoxBoxError3d> {
    left.checked_add(right)
        .ok_or(BoxBoxError3d::ArithmeticOverflow)
}

fn checked_sub(left: i128, right: i128) -> Result<i128, BoxBoxError3d> {
    left.checked_sub(right)
        .ok_or(BoxBoxError3d::ArithmeticOverflow)
}

fn div_round_nearest(numerator: i128, denominator: i128) -> Result<i128, BoxBoxError3d> {
    if denominator <= 0 {
        return Err(BoxBoxError3d::ArithmeticOverflow);
    }
    let half = denominator / 2;
    let adjusted = if numerator >= 0 {
        numerator.checked_add(half)
    } else {
        numerator.checked_sub(half)
    }
    .ok_or(BoxBoxError3d::ArithmeticOverflow)?;
    Ok(adjusted / denominator)
}

#[cfg(test)]
mod tests {
    use ecs_physics::PhysicsMaterial;
    use ecs_workload::EntityId;

    use super::*;

    fn dynamic_body(entity: u32) -> PhysicsBody3d {
        PhysicsBody3d::dynamic(EntityId(entity), [10, 10, 10])
            .with_material(PhysicsMaterial::new(0, 0))
    }

    fn state(center: Position, velocity: Velocity) -> RigidBoxState3d {
        RigidBoxState3d::new(
            center,
            velocity,
            AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
        )
    }

    #[test]
    fn separated_pair_is_idempotently_unchanged() {
        let left = state(Position::new3(0, 0, 0), Velocity::new3(20, 0, 0));
        let right = state(Position::new3(30, 0, 0), Velocity::new3(0, 0, 0));
        let step = resolve_box_box_contact(left, dynamic_body(1), right, dynamic_body(2))
            .expect("valid separated pair");

        assert_eq!(step.contact, None);
        assert_eq!(step.left, left);
        assert_eq!(step.right, right);
    }

    #[test]
    fn centered_face_impact_exchanges_velocity_without_spin() {
        let left = state(Position::new3(0, 0, 0), Velocity::new3(60, 0, 0));
        let right = state(Position::new3(19, 0, 0), Velocity::new3(0, 0, 0));
        let step = resolve_box_box_contact(left, dynamic_body(1), right, dynamic_body(2))
            .expect("valid centered impact");
        let contact = step.contact.expect("overlapping pair should contact");

        assert_eq!(contact.axis, [1, 0, 0]);
        assert!(contact.normal_impulse_units > 0);
        assert_eq!(step.left.linear_velocity.x, 30);
        assert_eq!(step.right.linear_velocity.x, 30);
        assert_eq!(
            step.left.angular.angular_velocity,
            AngularVelocity3d::default()
        );
        assert_eq!(
            step.right.angular.angular_velocity,
            AngularVelocity3d::default()
        );
    }

    #[test]
    fn off_center_face_impact_generates_spin_for_both_boxes() {
        let left = state(Position::new3(0, 6, 0), Velocity::new3(90, 0, 0));
        let right = state(Position::new3(19, 0, 0), Velocity::new3(0, 0, 0));
        let step = resolve_box_box_contact(left, dynamic_body(1), right, dynamic_body(2))
            .expect("valid off-center impact");
        let contact = step.contact.expect("overlapping pair should contact");

        assert_eq!(contact.axis, [1, 0, 0]);
        assert!(contact.normal_impulse_units > 0);
        assert_ne!(step.left.angular.angular_velocity.z, 0);
        assert_ne!(step.right.angular.angular_velocity.z, 0);
        assert!(step.left.linear_velocity.x < left.linear_velocity.x);
        assert!(step.right.linear_velocity.x > right.linear_velocity.x);
    }

    #[test]
    fn fixed_wall_receives_no_velocity_or_spin() {
        let left_body = dynamic_body(1).with_material(PhysicsMaterial::new(MATERIAL_SCALE, 0));
        let right_body = PhysicsBody3d::fixed(EntityId(2), [10, 10, 10]);
        let left = state(Position::new3(0, 0, 0), Velocity::new3(60, 0, 0));
        let right = state(Position::new3(19, 0, 0), Velocity::new3(0, 0, 0));
        let step =
            resolve_box_box_contact(left, left_body, right, right_body).expect("valid wall impact");

        assert!(
            step.contact
                .expect("wall should contact")
                .normal_impulse_units
                > 0
        );
        assert_eq!(step.left.linear_velocity.x, -60);
        assert_eq!(step.right, right);
    }

    #[test]
    fn wide_fixed_support_does_not_invent_torque_from_its_face_center() {
        let floor_body = PhysicsBody3d::fixed(EntityId(1), [100, 10, 100]);
        let block_body = dynamic_body(2);
        let floor = state(Position::new3(0, 0, 0), Velocity::new3(0, 0, 0));
        let block = state(Position::new3(60, 19, 40), Velocity::new3(0, -60, 0));
        let step = resolve_box_box_contact(floor, floor_body, block, block_body)
            .expect("valid wide support contact");
        let contact = step.contact.expect("floor should contact block");

        assert_eq!(contact.axis, [0, 1, 0]);
        assert_eq!(contact.point.x, block.center.x);
        assert_eq!(contact.point.z, block.center.z);
        assert!(contact.normal_impulse_units > 0);
        assert_eq!(
            step.right.angular.angular_velocity,
            AngularVelocity3d::default()
        );
        assert_eq!(step.left, floor);
    }

    #[test]
    fn overhanging_fixed_support_uses_the_shared_face_patch() {
        let floor_body = PhysicsBody3d::fixed(EntityId(1), [100, 10, 100]);
        let block_body = dynamic_body(2);
        let floor = state(Position::new3(0, 0, 0), Velocity::new3(0, 0, 0));
        let block = state(Position::new3(105, 19, 0), Velocity::new3(0, -60, 0));
        let step = resolve_box_box_contact(floor, floor_body, block, block_body)
            .expect("valid overhanging support contact");
        let contact = step.contact.expect("overhang should contact floor");

        assert_eq!(contact.axis, [0, 1, 0]);
        assert_eq!(contact.point.x, 98);
        assert!(contact.point.x <= 100);
        assert!(contact.normal_impulse_units > 0);
        assert_ne!(step.right.angular.angular_velocity.z, 0);
    }

    #[test]
    fn exact_diagonal_edge_touch_does_not_choose_an_arbitrary_torque_axis() {
        let left = state(Position::new3(0, 0, 0), Velocity::new3(0, 0, 0));
        let right = state(Position::new3(0, 20, 20), Velocity::new3(0, -60, 0));
        let step = resolve_box_box_contact(left, dynamic_body(1), right, dynamic_body(2))
            .expect("valid exact edge touch");
        let contact = step.contact.expect("edge touch remains contact evidence");

        assert_eq!(contact.overlap_numerator, 0);
        assert_eq!(contact.normal_impulse_units, 0);
        assert_eq!(step.left, left);
        assert_eq!(step.right, right);
    }

    #[test]
    fn penetrating_diagonal_contact_still_generates_real_torque() {
        let left = state(Position::new3(0, 0, 0), Velocity::new3(0, 0, 0));
        let right = state(Position::new3(0, 19, 19), Velocity::new3(0, -60, 0));
        let step = resolve_box_box_contact(left, dynamic_body(1), right, dynamic_body(2))
            .expect("valid penetrating diagonal contact");
        let contact = step.contact.expect("penetration should contact");

        assert!(contact.overlap_numerator > 0);
        assert!(contact.normal_impulse_units > 0);
        assert_ne!(step.left.angular.angular_velocity.x, 0);
        assert_ne!(step.right.angular.angular_velocity.x, 0);
    }

    #[test]
    fn separating_overlap_reports_contact_without_second_impulse() {
        let left = state(Position::new3(0, 0, 0), Velocity::new3(-20, 0, 0));
        let right = state(Position::new3(19, 0, 0), Velocity::new3(20, 0, 0));
        let step = resolve_box_box_contact(left, dynamic_body(1), right, dynamic_body(2))
            .expect("valid separating overlap");
        let contact = step.contact.expect("overlap remains contact evidence");

        assert_eq!(contact.normal_impulse_units, 0);
        assert_eq!(step.left, left);
        assert_eq!(step.right, right);
    }
}
