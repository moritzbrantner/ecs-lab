use ecs_physics::{BodyKind, MATERIAL_SCALE};
use ecs_workload::{Position, Velocity};

use crate::{
    angular::{
        ANGULAR_VELOCITY_SCALE, AngularVelocity3d, ORIENTATION_SCALE, Orientation3d, box_inertia,
    },
    box_box::{BoxBoxContact3d, BoxBoxStep3d, RigidBoxState3d},
    box_box_stabilization::{
        BoxBoxStabilizationError3d, stabilize_box_box_contact as stabilize_without_friction,
    },
    box_plane::oriented_box_vertices,
    oriented_box::ObbAxisFeature3d,
    types::PhysicsBody3d,
};

const RESPONSE_SCALE: i128 = 1_i128 << 50;

type Result3d<T> = Result<T, BoxBoxStabilizationError3d>;

#[derive(Clone, Copy)]
struct TangentResponse3d {
    left_body: PhysicsBody3d,
    right_body: PhysicsBody3d,
    left_offset: [i64; 3],
    right_offset: [i64; 3],
    tangent: [i128; 3],
    contact: BoxBoxContact3d,
    friction_milli: u16,
}

/// Resolves and stabilizes one OBB pair, then applies one deterministic Coulomb-limited tangent impulse.
///
/// The underlying normal response and discrete penetration projection remain unchanged. Friction uses the
/// same original Rust-owned contact point and SAT feature as the normal impulse, measures the post-normal
/// relative contact velocity including spin, and chooses the dominant slip direction from the exact box
/// edges that span the contact tangent plane. Keeping the tangent on those bounded geometry edges avoids
/// inventing an enormous projected integer direction as boxes rotate. The Coulomb bound compares squared
/// impulse magnitudes, so no floating-point normalization or square root becomes solver truth.
///
/// The existing swept-AABB path is still only a conservative broad phase for the dense room. This helper
/// changes the bounded OBB response seam used by the angular rigid-box world; it does not claim rotational
/// CCD.
///
/// # Errors
///
/// Returns [`BoxBoxStabilizationError3d`] for invalid pair ordering/response, out-of-range friction,
/// malformed contact geometry, or checked arithmetic overflow.
pub fn stabilize_box_box_contact(
    left_state: RigidBoxState3d,
    left_body: PhysicsBody3d,
    right_state: RigidBoxState3d,
    right_body: PhysicsBody3d,
) -> Result3d<BoxBoxStep3d> {
    validate_friction(left_body)?;
    validate_friction(right_body)?;

    let mut step = stabilize_without_friction(left_state, left_body, right_state, right_body)?;
    let Some(contact) = step.contact else {
        return Ok(step);
    };
    if contact.normal_impulse_units <= 0 {
        return Ok(step);
    }

    let friction_milli = left_body
        .material
        .friction_milli
        .max(right_body.material.friction_milli);
    if friction_milli == 0 {
        return Ok(step);
    }

    let left_offset = position_delta(left_state.center, contact.point)?;
    let right_offset = position_delta(right_state.center, contact.point)?;
    let relative_velocity =
        relative_contact_velocity(step.left, left_offset, step.right, right_offset)?;
    let tangents = contact_tangents(left_state, left_body, right_state, right_body, contact)?;
    let Some(tangent) = dominant_slip_tangent(relative_velocity, tangents)? else {
        return Ok(step);
    };
    apply_tangent_response(
        &mut step,
        TangentResponse3d {
            left_body,
            right_body,
            left_offset,
            right_offset,
            tangent,
            contact,
            friction_milli,
        },
    )?;
    Ok(step)
}

fn validate_friction(body: PhysicsBody3d) -> Result3d<()> {
    if body.material.friction_milli > MATERIAL_SCALE {
        return Err(BoxBoxStabilizationError3d::ArithmeticOverflow);
    }
    Ok(())
}

fn contact_tangents(
    left_state: RigidBoxState3d,
    left_body: PhysicsBody3d,
    right_state: RigidBoxState3d,
    right_body: PhysicsBody3d,
    contact: BoxBoxContact3d,
) -> Result3d<[[i128; 3]; 2]> {
    let left_edges = oriented_edges(left_state, left_body)?;
    let right_edges = oriented_edges(right_state, right_body)?;
    match contact.feature {
        ObbAxisFeature3d::LeftFace(axis) => face_tangents(left_edges, axis),
        ObbAxisFeature3d::RightFace(axis) => face_tangents(right_edges, axis),
        ObbAxisFeature3d::EdgeEdge {
            left_axis,
            right_axis,
        } => Ok([
            indexed_edge(left_edges, left_axis)?,
            indexed_edge(right_edges, right_axis)?,
        ]),
    }
}

fn oriented_edges(state: RigidBoxState3d, body: PhysicsBody3d) -> Result3d<[[i128; 3]; 3]> {
    let vertices =
        oriented_box_vertices(state.center, body.half_extents, state.angular.orientation)
            .map_err(|_| BoxBoxStabilizationError3d::ArithmeticOverflow)?;
    Ok([
        edge_delta(vertices[0], vertices[1])?,
        edge_delta(vertices[0], vertices[2])?,
        edge_delta(vertices[0], vertices[4])?,
    ])
}

fn edge_delta(left: Position, right: Position) -> Result3d<[i128; 3]> {
    Ok([
        i128::from(right.x)
            .checked_sub(i128::from(left.x))
            .ok_or(BoxBoxStabilizationError3d::ArithmeticOverflow)?,
        i128::from(right.y)
            .checked_sub(i128::from(left.y))
            .ok_or(BoxBoxStabilizationError3d::ArithmeticOverflow)?,
        i128::from(right.z)
            .checked_sub(i128::from(left.z))
            .ok_or(BoxBoxStabilizationError3d::ArithmeticOverflow)?,
    ])
}

fn face_tangents(edges: [[i128; 3]; 3], axis: u8) -> Result3d<[[i128; 3]; 2]> {
    match axis {
        0 => Ok([edges[1], edges[2]]),
        1 => Ok([edges[2], edges[0]]),
        2 => Ok([edges[0], edges[1]]),
        _ => Err(BoxBoxStabilizationError3d::ArithmeticOverflow),
    }
}

fn indexed_edge(edges: [[i128; 3]; 3], axis: u8) -> Result3d<[i128; 3]> {
    edges
        .get(usize::from(axis))
        .copied()
        .ok_or(BoxBoxStabilizationError3d::ArithmeticOverflow)
}

fn dominant_slip_tangent(
    relative_velocity: [i128; 3],
    tangents: [[i128; 3]; 2],
) -> Result3d<Option<[i128; 3]>> {
    let first =
        primitive_vector(tangents[0])?.ok_or(BoxBoxStabilizationError3d::ArithmeticOverflow)?;
    let second =
        primitive_vector(tangents[1])?.ok_or(BoxBoxStabilizationError3d::ArithmeticOverflow)?;
    let first_slip = checked_dot(relative_velocity, first)?;
    let second_slip = checked_dot(relative_velocity, second)?;
    if first_slip == 0 && second_slip == 0 {
        return Ok(None);
    }
    if first_slip.unsigned_abs() >= second_slip.unsigned_abs() {
        Ok(Some(first))
    } else {
        Ok(Some(second))
    }
}

fn apply_tangent_response(step: &mut BoxBoxStep3d, response: TangentResponse3d) -> Result3d<()> {
    let TangentResponse3d {
        left_body,
        right_body,
        left_offset,
        right_offset,
        tangent,
        contact,
        friction_milli,
    } = response;
    let tangent_length_squared = vector_length_squared(tangent)?;
    let relative_velocity =
        relative_contact_velocity(step.left, left_offset, step.right, right_offset)?;
    let tangent_velocity = checked_dot(relative_velocity, tangent)?;
    if tangent_velocity == 0 {
        return Ok(());
    }

    let effective_inverse_mass = checked_add(
        body_effective_inverse_mass_scaled(
            left_body,
            step.left.angular.orientation,
            left_offset,
            tangent,
            tangent_length_squared,
        )?,
        body_effective_inverse_mass_scaled(
            right_body,
            step.right.angular.orientation,
            right_offset,
            tangent,
            tangent_length_squared,
        )?,
    )?;
    if effective_inverse_mass <= 0 {
        return Ok(());
    }

    let opposing_velocity = tangent_velocity
        .checked_neg()
        .ok_or(BoxBoxStabilizationError3d::ArithmeticOverflow)?;
    let desired_impulse_units = div_round_nearest(
        checked_mul(opposing_velocity, RESPONSE_SCALE)?,
        effective_inverse_mass,
    )?;
    let tangent_impulse_units = coulomb_clamp(
        desired_impulse_units,
        tangent_length_squared,
        contact.normal_impulse_units,
        contact.axis_length_squared,
        friction_milli,
    )?;
    if tangent_impulse_units == 0 {
        return Ok(());
    }

    let left_impulse = scale_axis(tangent, checked_neg(tangent_impulse_units)?)?;
    let right_impulse = scale_axis(tangent, tangent_impulse_units)?;
    apply_body_impulse(&mut step.left, left_body, left_offset, left_impulse)?;
    apply_body_impulse(&mut step.right, right_body, right_offset, right_impulse)?;
    Ok(())
}

fn position_delta(center: Position, point: Position) -> Result3d<[i64; 3]> {
    Ok([
        point
            .x
            .checked_sub(center.x)
            .ok_or(BoxBoxStabilizationError3d::ArithmeticOverflow)?,
        point
            .y
            .checked_sub(center.y)
            .ok_or(BoxBoxStabilizationError3d::ArithmeticOverflow)?,
        point
            .z
            .checked_sub(center.z)
            .ok_or(BoxBoxStabilizationError3d::ArithmeticOverflow)?,
    ])
}

fn relative_contact_velocity(
    left: RigidBoxState3d,
    left_offset: [i64; 3],
    right: RigidBoxState3d,
    right_offset: [i64; 3],
) -> Result3d<[i128; 3]> {
    let left = contact_velocity(left, left_offset)?;
    let right = contact_velocity(right, right_offset)?;
    Ok([
        checked_sub(right[0], left[0])?,
        checked_sub(right[1], left[1])?,
        checked_sub(right[2], left[2])?,
    ])
}

fn contact_velocity(state: RigidBoxState3d, offset: [i64; 3]) -> Result3d<[i128; 3]> {
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
        checked_add(
            i128::from(state.linear_velocity.x),
            div_round_nearest(rotation_x, scale)?,
        )?,
        checked_add(
            i128::from(state.linear_velocity.y),
            div_round_nearest(rotation_y, scale)?,
        )?,
        checked_add(
            i128::from(state.linear_velocity.z),
            div_round_nearest(rotation_z, scale)?,
        )?,
    ])
}

fn primitive_vector(vector: [i128; 3]) -> Result3d<Option<[i128; 3]>> {
    let mut divisor = 0_u128;
    for component in vector {
        divisor = gcd(divisor, component.unsigned_abs());
    }
    if divisor == 0 {
        return Ok(None);
    }
    let divisor =
        i128::try_from(divisor).map_err(|_| BoxBoxStabilizationError3d::ArithmeticOverflow)?;
    Ok(Some([
        vector[0] / divisor,
        vector[1] / divisor,
        vector[2] / divisor,
    ]))
}

fn gcd(mut left: u128, mut right: u128) -> u128 {
    while right != 0 {
        let remainder = left % right;
        left = right;
        right = remainder;
    }
    left
}

fn vector_length_squared(vector: [i128; 3]) -> Result3d<u128> {
    vector.into_iter().try_fold(0_u128, |sum, component| {
        let magnitude = component.unsigned_abs();
        let square = magnitude
            .checked_mul(magnitude)
            .ok_or(BoxBoxStabilizationError3d::ArithmeticOverflow)?;
        sum.checked_add(square)
            .ok_or(BoxBoxStabilizationError3d::ArithmeticOverflow)
    })
}

fn coulomb_clamp(
    desired_impulse_units: i128,
    tangent_length_squared: u128,
    normal_impulse_units: i64,
    normal_length_squared: u128,
    friction_milli: u16,
) -> Result3d<i128> {
    if desired_impulse_units == 0 || normal_impulse_units <= 0 || friction_milli == 0 {
        return Ok(0);
    }
    let desired_magnitude = desired_impulse_units.unsigned_abs();
    let normal_magnitude = u128::try_from(normal_impulse_units)
        .map_err(|_| BoxBoxStabilizationError3d::ArithmeticOverflow)?;

    if within_coulomb_bound(
        desired_magnitude,
        tangent_length_squared,
        normal_magnitude,
        normal_length_squared,
        friction_milli,
    )? {
        return Ok(desired_impulse_units);
    }

    let mut low = 0_u128;
    let mut high = desired_magnitude;
    while low < high {
        let distance = high
            .checked_sub(low)
            .ok_or(BoxBoxStabilizationError3d::ArithmeticOverflow)?;
        let midpoint = low
            .checked_add(distance.div_ceil(2))
            .ok_or(BoxBoxStabilizationError3d::ArithmeticOverflow)?;
        if within_coulomb_bound(
            midpoint,
            tangent_length_squared,
            normal_magnitude,
            normal_length_squared,
            friction_milli,
        )? {
            low = midpoint;
        } else {
            high = midpoint - 1;
        }
    }

    let bounded =
        i128::try_from(low).map_err(|_| BoxBoxStabilizationError3d::ArithmeticOverflow)?;
    if desired_impulse_units < 0 {
        checked_neg(bounded)
    } else {
        Ok(bounded)
    }
}

fn within_coulomb_bound(
    tangent_impulse_units: u128,
    tangent_length_squared: u128,
    normal_impulse_units: u128,
    normal_length_squared: u128,
    friction_milli: u16,
) -> Result3d<bool> {
    let material_scale = u128::from(MATERIAL_SCALE);
    let friction = u128::from(friction_milli);
    let tangent_square = tangent_impulse_units
        .checked_mul(tangent_impulse_units)
        .ok_or(BoxBoxStabilizationError3d::ArithmeticOverflow)?;
    let left = tangent_square
        .checked_mul(tangent_length_squared)
        .and_then(|value| value.checked_mul(material_scale))
        .and_then(|value| value.checked_mul(material_scale))
        .ok_or(BoxBoxStabilizationError3d::ArithmeticOverflow)?;

    let normal_square = normal_impulse_units
        .checked_mul(normal_impulse_units)
        .ok_or(BoxBoxStabilizationError3d::ArithmeticOverflow)?;
    let right = normal_square
        .checked_mul(normal_length_squared)
        .and_then(|value| value.checked_mul(friction))
        .and_then(|value| value.checked_mul(friction))
        .ok_or(BoxBoxStabilizationError3d::ArithmeticOverflow)?;
    Ok(left <= right)
}

fn body_effective_inverse_mass_scaled(
    body: PhysicsBody3d,
    orientation: Orientation3d,
    contact_offset: [i64; 3],
    axis: [i128; 3],
    axis_length_squared: u128,
) -> Result3d<i128> {
    if body.kind == BodyKind::Fixed {
        return Ok(0);
    }

    let length_squared = i128::try_from(axis_length_squared)
        .map_err(|_| BoxBoxStabilizationError3d::ArithmeticOverflow)?;
    let translational = checked_mul(RESPONSE_SCALE, length_squared)? / i128::from(body.mass_units);
    let angular_impulse = cross_i64_i128(contact_offset, axis)?;
    let local = rotate_inverse(orientation, angular_impulse)?;
    let inertia = box_inertia(body).map_err(|_| BoxBoxStabilizationError3d::ArithmeticOverflow)?;
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
) -> Result3d<()> {
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
    let inertia = box_inertia(body).map_err(|_| BoxBoxStabilizationError3d::ArithmeticOverflow)?;
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

fn add_linear_impulse_axis(current: i32, impulse: i128, mass_units: u32) -> Result3d<i32> {
    let delta = div_round_nearest(impulse, i128::from(mass_units))?;
    let next = checked_add(i128::from(current), delta)?;
    i32::try_from(next).map_err(|_| BoxBoxStabilizationError3d::ArithmeticOverflow)
}

fn add_angular_axis(current: i32, delta: i128) -> Result3d<i32> {
    let next = checked_add(i128::from(current), delta)?;
    i32::try_from(next).map_err(|_| BoxBoxStabilizationError3d::ArithmeticOverflow)
}

fn inverse_inertia_scaled(principal_numerator: u128, denominator: u32) -> Result3d<i128> {
    if principal_numerator == 0 {
        return Err(BoxBoxStabilizationError3d::ArithmeticOverflow);
    }
    let principal = i128::try_from(principal_numerator)
        .map_err(|_| BoxBoxStabilizationError3d::ArithmeticOverflow)?;
    Ok(checked_mul(RESPONSE_SCALE, i128::from(denominator))? / principal)
}

fn cross_i64_i128(left: [i64; 3], right: [i128; 3]) -> Result3d<[i128; 3]> {
    let left = left.map(i128::from);
    Ok([
        cross_component(left[1], right[2], left[2], right[1])?,
        cross_component(left[2], right[0], left[0], right[2])?,
        cross_component(left[0], right[1], left[1], right[0])?,
    ])
}

fn cross_component(left_a: i128, right_a: i128, left_b: i128, right_b: i128) -> Result3d<i128> {
    checked_sub(checked_mul(left_a, right_a)?, checked_mul(left_b, right_b)?)
}

fn rotate_inverse(orientation: Orientation3d, vector: [i128; 3]) -> Result3d<[i128; 3]> {
    let matrix = rotation_matrix(
        orientation
            .normalized()
            .map_err(|_| BoxBoxStabilizationError3d::ArithmeticOverflow)?,
    )?;
    rotate_with_matrix(
        [
            [matrix[0][0], matrix[1][0], matrix[2][0]],
            [matrix[0][1], matrix[1][1], matrix[2][1]],
            [matrix[0][2], matrix[1][2], matrix[2][2]],
        ],
        vector,
    )
}

fn rotate_forward(orientation: Orientation3d, vector: [i128; 3]) -> Result3d<[i128; 3]> {
    let orientation = orientation
        .normalized()
        .map_err(|_| BoxBoxStabilizationError3d::ArithmeticOverflow)?;
    rotate_with_matrix(rotation_matrix(orientation)?, vector)
}

fn rotation_matrix(orientation: Orientation3d) -> Result3d<[[i128; 3]; 3]> {
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

fn rotate_with_matrix(matrix: [[i128; 3]; 3], vector: [i128; 3]) -> Result3d<[i128; 3]> {
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

fn scaled_twice(value: i128, scale: i128) -> Result3d<i128> {
    div_round_nearest(checked_mul(value, 2)?, scale)
}

fn checked_mul(left: i128, right: i128) -> Result3d<i128> {
    left.checked_mul(right)
        .ok_or(BoxBoxStabilizationError3d::ArithmeticOverflow)
}

fn checked_add(left: i128, right: i128) -> Result3d<i128> {
    left.checked_add(right)
        .ok_or(BoxBoxStabilizationError3d::ArithmeticOverflow)
}

fn checked_sub(left: i128, right: i128) -> Result3d<i128> {
    left.checked_sub(right)
        .ok_or(BoxBoxStabilizationError3d::ArithmeticOverflow)
}

fn checked_neg(value: i128) -> Result3d<i128> {
    value
        .checked_neg()
        .ok_or(BoxBoxStabilizationError3d::ArithmeticOverflow)
}

fn checked_dot(left: [i128; 3], right: [i128; 3]) -> Result3d<i128> {
    checked_add(
        checked_add(
            checked_mul(left[0], right[0])?,
            checked_mul(left[1], right[1])?,
        )?,
        checked_mul(left[2], right[2])?,
    )
}

fn scale_axis(axis: [i128; 3], scale: i128) -> Result3d<[i128; 3]> {
    Ok([
        checked_mul(axis[0], scale)?,
        checked_mul(axis[1], scale)?,
        checked_mul(axis[2], scale)?,
    ])
}

fn div_round_nearest(numerator: i128, denominator: i128) -> Result3d<i128> {
    if denominator <= 0 {
        return Err(BoxBoxStabilizationError3d::ArithmeticOverflow);
    }
    let half = denominator / 2;
    let adjusted = if numerator >= 0 {
        numerator.checked_add(half)
    } else {
        numerator.checked_sub(half)
    }
    .ok_or(BoxBoxStabilizationError3d::ArithmeticOverflow)?;
    Ok(adjusted / denominator)
}

#[cfg(test)]
mod tests {
    use ecs_physics::PhysicsMaterial;
    use ecs_workload::EntityId;

    use crate::angular::{AngularState3d, Orientation3d};

    use super::*;

    fn state(center: Position, velocity: Velocity) -> RigidBoxState3d {
        RigidBoxState3d::new(
            center,
            velocity,
            AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
        )
    }

    fn body(entity: u32, friction_milli: u16) -> PhysicsBody3d {
        PhysicsBody3d::dynamic(EntityId(entity), [10, 10, 10])
            .with_material(PhysicsMaterial::new(0, friction_milli))
    }

    #[test]
    fn zero_friction_preserves_normal_only_stabilization() {
        let left = state(Position::new3(0, 0, 0), Velocity::new3(60, 0, 40));
        let right = state(Position::new3(19, 0, 0), Velocity::new3(0, 0, 0));
        let expected = stabilize_without_friction(left, body(1, 0), right, body(2, 0))
            .expect("valid normal-only response");
        let actual = stabilize_box_box_contact(left, body(1, 0), right, body(2, 0))
            .expect("valid friction wrapper");

        assert_eq!(actual, expected);
    }

    #[test]
    fn sliding_face_impact_uses_friction_to_reduce_slip_and_generate_spin() {
        let left = state(Position::new3(0, 0, 0), Velocity::new3(60, 0, 40));
        let right = state(Position::new3(19, 0, 0), Velocity::new3(0, 0, 0));
        let step = stabilize_box_box_contact(left, body(1, MATERIAL_SCALE), right, body(2, 0))
            .expect("valid frictional response");
        let contact = step.contact.expect("overlap should contact");
        let left_offset = position_delta(left.center, contact.point).expect("valid offset");
        let right_offset = position_delta(right.center, contact.point).expect("valid offset");
        let relative = relative_contact_velocity(step.left, left_offset, step.right, right_offset)
            .expect("valid relative velocity");

        assert!(contact.normal_impulse_units > 0);
        assert!(relative[2].abs() < 40);
        assert_ne!(step.left.angular.angular_velocity.y, 0);
        assert_ne!(step.right.angular.angular_velocity.y, 0);
    }

    #[test]
    fn separating_overlap_does_not_inject_friction() {
        let left = state(Position::new3(0, 0, 0), Velocity::new3(-20, 0, 40));
        let right = state(Position::new3(19, 0, 0), Velocity::new3(20, 0, 0));
        let step = stabilize_box_box_contact(
            left,
            body(1, MATERIAL_SCALE),
            right,
            body(2, MATERIAL_SCALE),
        )
        .expect("valid separating overlap");

        assert_eq!(
            step.contact.expect("contact evidence").normal_impulse_units,
            0
        );
        assert_eq!(step.left.linear_velocity, left.linear_velocity);
        assert_eq!(step.right.linear_velocity, right.linear_velocity);
    }

    #[test]
    fn invalid_friction_fails_closed() {
        let invalid = PhysicsBody3d::dynamic(EntityId(1), [10, 10, 10])
            .with_material(PhysicsMaterial::new(0, MATERIAL_SCALE + 1));
        let error = stabilize_box_box_contact(
            state(Position::new3(0, 0, 0), Velocity::new3(60, 0, 0)),
            invalid,
            state(Position::new3(19, 0, 0), Velocity::new3(0, 0, 0)),
            body(2, 0),
        )
        .expect_err("invalid friction must fail closed");

        assert_eq!(error, BoxBoxStabilizationError3d::ArithmeticOverflow);
    }
}
