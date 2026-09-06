use std::fmt;

use ecs_physics::{BodyKind, MATERIAL_SCALE, PhysicsMaterial};
use ecs_workload::{Position, Velocity};

use crate::{
    ANGULAR_VELOCITY_SCALE, AngularError3d, AngularState3d, AngularVelocity3d, ORIENTATION_SCALE,
    Orientation3d, PhysicsBody3d, box_inertia, contact_angular_impulse, integrate_orientation,
};

const RESPONSE_SCALE: i128 = 1_i128 << 50;
const CORNER_SIGNS: [[i64; 3]; 8] = [
    [-1, -1, -1],
    [1, -1, -1],
    [-1, 1, -1],
    [1, 1, -1],
    [-1, -1, 1],
    [1, -1, 1],
    [-1, 1, 1],
    [1, 1, 1],
];

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct BoxPlaneState3d {
    pub center: Position,
    pub linear_velocity: Velocity,
    pub angular: AngularState3d,
}

impl BoxPlaneState3d {
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
pub struct BoxPlaneConfig3d {
    /// Linear acceleration in the coordinate units used by the box state.
    pub gravity: Velocity,
    pub plane_y: i64,
    pub timestep_numerator: i32,
    pub timestep_denominator: i32,
    pub plane_material: PhysicsMaterial,
    /// Angular velocity retained after each step, in thousandths.
    pub angular_damping_milli: u16,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct BoxPlaneContact3d {
    pub point: Position,
    pub manifold_vertices: u8,
    pub normal_impulse_units: i64,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct BoxPlaneStep3d {
    pub state: BoxPlaneState3d,
    pub contact: Option<BoxPlaneContact3d>,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BoxPlaneError3d {
    FixedBody,
    InvalidHalfExtents,
    ZeroMass,
    NegativeTimestepNumerator(i32),
    NonPositiveTimestepDenominator(i32),
    DampingOutOfRange(u16),
    Angular(AngularError3d),
    ArithmeticOverflow,
}

impl fmt::Display for BoxPlaneError3d {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::FixedBody => write!(
                formatter,
                "box-plane angular stepping requires a dynamic body"
            ),
            Self::InvalidHalfExtents => write!(
                formatter,
                "box-plane angular stepping requires positive half extents"
            ),
            Self::ZeroMass => write!(
                formatter,
                "box-plane angular stepping requires non-zero mass"
            ),
            Self::NegativeTimestepNumerator(value) => write!(
                formatter,
                "box-plane timestep numerator must be non-negative, got {value}"
            ),
            Self::NonPositiveTimestepDenominator(value) => write!(
                formatter,
                "box-plane timestep denominator must be positive, got {value}"
            ),
            Self::DampingOutOfRange(value) => write!(
                formatter,
                "box-plane angular damping must be 0..={MATERIAL_SCALE}, got {value}"
            ),
            Self::Angular(error) => write!(formatter, "box-plane angular state failed: {error}"),
            Self::ArithmeticOverflow => {
                write!(formatter, "box-plane angular calculation overflowed")
            }
        }
    }
}

impl std::error::Error for BoxPlaneError3d {}

impl From<AngularError3d> for BoxPlaneError3d {
    fn from(value: AngularError3d) -> Self {
        Self::Angular(value)
    }
}

/// Advances one oriented cuboid against a static horizontal plane.
///
/// Rust orientation determines all eight world-space corners. Tied lowest vertices become one stable
/// manifold. An off-center normal impulse changes both linear and angular velocity through cuboid
/// inertia. This does not rotate the existing AABB room solver; the orientation-aware plane fixture is
/// kept separate until OBB narrow phase is ready. Rotational CCD is also deferred, so this 60 Hz slice
/// corrects discrete penetration before response.
///
/// # Errors
///
/// Returns [`BoxPlaneError3d`] for malformed body/timestep configuration or arithmetic failure.
pub fn step_box_on_plane(
    state: BoxPlaneState3d,
    body: PhysicsBody3d,
    config: BoxPlaneConfig3d,
) -> Result<BoxPlaneStep3d, BoxPlaneError3d> {
    validate(body, config)?;

    let linear_velocity = integrate_linear_velocity(state.linear_velocity, config)?;
    let center = integrate_center(state.center, linear_velocity, config)?;
    let orientation = integrate_orientation(
        state.angular.orientation,
        state.angular.angular_velocity,
        config.timestep_numerator,
        config.timestep_denominator,
    )?;
    let mut next = BoxPlaneState3d::new(
        center,
        linear_velocity,
        AngularState3d::new(orientation, state.angular.angular_velocity),
    );

    let offsets = oriented_box_offsets(body.half_extents, orientation)?;
    let minimum_y = lowest_offset_y(&offsets);
    let lowest_world_y = next
        .center
        .y
        .checked_add(minimum_y)
        .ok_or(BoxPlaneError3d::ArithmeticOverflow)?;
    let contact = if lowest_world_y < config.plane_y {
        Some(resolve_plane_contact(
            &mut next,
            body,
            config,
            &offsets,
            minimum_y,
            lowest_world_y,
        )?)
    } else {
        None
    };

    next.angular.angular_velocity =
        damp_angular_velocity(next.angular.angular_velocity, config.angular_damping_milli)?;

    Ok(BoxPlaneStep3d {
        state: next,
        contact,
    })
}

/// Returns the eight world-space vertices of an oriented box in stable local-corner order.
///
/// # Errors
///
/// Returns [`BoxPlaneError3d`] when orientation normalization or coordinate arithmetic fails.
pub fn oriented_box_vertices(
    center: Position,
    half_extents: [i32; 3],
    orientation: Orientation3d,
) -> Result<[Position; 8], BoxPlaneError3d> {
    let offsets = oriented_box_offsets(half_extents, orientation)?;
    let mut vertices = [Position::new3(0, 0, 0); 8];
    for (vertex, offset) in vertices.iter_mut().zip(offsets) {
        *vertex = Position::new3(
            center
                .x
                .checked_add(offset[0])
                .ok_or(BoxPlaneError3d::ArithmeticOverflow)?,
            center
                .y
                .checked_add(offset[1])
                .ok_or(BoxPlaneError3d::ArithmeticOverflow)?,
            center
                .z
                .checked_add(offset[2])
                .ok_or(BoxPlaneError3d::ArithmeticOverflow)?,
        );
    }
    Ok(vertices)
}

fn validate(body: PhysicsBody3d, config: BoxPlaneConfig3d) -> Result<(), BoxPlaneError3d> {
    if body.kind != BodyKind::Dynamic {
        return Err(BoxPlaneError3d::FixedBody);
    }
    if body.half_extents.iter().any(|extent| *extent <= 0) {
        return Err(BoxPlaneError3d::InvalidHalfExtents);
    }
    if body.mass_units == 0 {
        return Err(BoxPlaneError3d::ZeroMass);
    }
    if config.timestep_numerator < 0 {
        return Err(BoxPlaneError3d::NegativeTimestepNumerator(
            config.timestep_numerator,
        ));
    }
    if config.timestep_denominator <= 0 {
        return Err(BoxPlaneError3d::NonPositiveTimestepDenominator(
            config.timestep_denominator,
        ));
    }
    if config.angular_damping_milli > MATERIAL_SCALE {
        return Err(BoxPlaneError3d::DampingOutOfRange(
            config.angular_damping_milli,
        ));
    }
    Ok(())
}

fn integrate_linear_velocity(
    velocity: Velocity,
    config: BoxPlaneConfig3d,
) -> Result<Velocity, BoxPlaneError3d> {
    Ok(Velocity::new3(
        integrate_velocity_axis(
            velocity.x,
            config.gravity.x,
            config.timestep_numerator,
            config.timestep_denominator,
        )?,
        integrate_velocity_axis(
            velocity.y,
            config.gravity.y,
            config.timestep_numerator,
            config.timestep_denominator,
        )?,
        integrate_velocity_axis(
            velocity.z,
            config.gravity.z,
            config.timestep_numerator,
            config.timestep_denominator,
        )?,
    ))
}

fn integrate_center(
    center: Position,
    velocity: Velocity,
    config: BoxPlaneConfig3d,
) -> Result<Position, BoxPlaneError3d> {
    Ok(Position::new3(
        integrate_position_axis(
            center.x,
            velocity.x,
            config.timestep_numerator,
            config.timestep_denominator,
        )?,
        integrate_position_axis(
            center.y,
            velocity.y,
            config.timestep_numerator,
            config.timestep_denominator,
        )?,
        integrate_position_axis(
            center.z,
            velocity.z,
            config.timestep_numerator,
            config.timestep_denominator,
        )?,
    ))
}

fn integrate_velocity_axis(
    velocity: i32,
    acceleration: i32,
    numerator: i32,
    denominator: i32,
) -> Result<i32, BoxPlaneError3d> {
    let acceleration_step = i128::from(acceleration)
        .checked_mul(i128::from(numerator))
        .ok_or(BoxPlaneError3d::ArithmeticOverflow)?;
    let delta = div_round_nearest(acceleration_step, i128::from(denominator))?;
    let next = i128::from(velocity)
        .checked_add(delta)
        .ok_or(BoxPlaneError3d::ArithmeticOverflow)?;
    i32::try_from(next).map_err(|_| BoxPlaneError3d::ArithmeticOverflow)
}

fn integrate_position_axis(
    position: i64,
    velocity: i32,
    numerator: i32,
    denominator: i32,
) -> Result<i64, BoxPlaneError3d> {
    let velocity_step = i128::from(velocity)
        .checked_mul(i128::from(numerator))
        .ok_or(BoxPlaneError3d::ArithmeticOverflow)?;
    let delta = div_round_nearest(velocity_step, i128::from(denominator))?;
    let next = i128::from(position)
        .checked_add(delta)
        .ok_or(BoxPlaneError3d::ArithmeticOverflow)?;
    i64::try_from(next).map_err(|_| BoxPlaneError3d::ArithmeticOverflow)
}

fn oriented_box_offsets(
    half_extents: [i32; 3],
    orientation: Orientation3d,
) -> Result<[[i64; 3]; 8], BoxPlaneError3d> {
    if half_extents.iter().any(|extent| *extent <= 0) {
        return Err(BoxPlaneError3d::InvalidHalfExtents);
    }
    let matrix = rotation_matrix(orientation.normalized()?)?;
    let mut offsets = [[0_i64; 3]; 8];
    for (offset, signs) in offsets.iter_mut().zip(CORNER_SIGNS) {
        let local = [
            signs[0] * i64::from(half_extents[0]),
            signs[1] * i64::from(half_extents[1]),
            signs[2] * i64::from(half_extents[2]),
        ];
        *offset = rotate_with_matrix(matrix, local)?;
    }
    Ok(offsets)
}

fn rotation_matrix(orientation: Orientation3d) -> Result<[[i128; 3]; 3], BoxPlaneError3d> {
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

fn scaled_twice(value: i128, scale: i128) -> Result<i128, BoxPlaneError3d> {
    div_round_nearest(checked_mul(value, 2)?, scale)
}

fn checked_mul(left: i128, right: i128) -> Result<i128, BoxPlaneError3d> {
    left.checked_mul(right)
        .ok_or(BoxPlaneError3d::ArithmeticOverflow)
}

fn checked_add(left: i128, right: i128) -> Result<i128, BoxPlaneError3d> {
    left.checked_add(right)
        .ok_or(BoxPlaneError3d::ArithmeticOverflow)
}

fn checked_sub(left: i128, right: i128) -> Result<i128, BoxPlaneError3d> {
    left.checked_sub(right)
        .ok_or(BoxPlaneError3d::ArithmeticOverflow)
}

fn rotate_with_matrix(
    matrix: [[i128; 3]; 3],
    vector: [i64; 3],
) -> Result<[i64; 3], BoxPlaneError3d> {
    let scale = i128::from(ORIENTATION_SCALE);
    let mut output = [0_i64; 3];
    for (target, row) in output.iter_mut().zip(matrix) {
        let first = checked_mul(row[0], i128::from(vector[0]))?;
        let second = checked_mul(row[1], i128::from(vector[1]))?;
        let third = checked_mul(row[2], i128::from(vector[2]))?;
        let sum = checked_add(checked_add(first, second)?, third)?;
        *target = i64::try_from(div_round_nearest(sum, scale)?)
            .map_err(|_| BoxPlaneError3d::ArithmeticOverflow)?;
    }
    Ok(output)
}

fn rotate_inverse(
    orientation: Orientation3d,
    vector: [i64; 3],
) -> Result<[i64; 3], BoxPlaneError3d> {
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

fn lowest_offset_y(offsets: &[[i64; 3]; 8]) -> i64 {
    offsets
        .iter()
        .map(|offset| offset[1])
        .min()
        .unwrap_or_default()
}

fn resolve_plane_contact(
    state: &mut BoxPlaneState3d,
    body: PhysicsBody3d,
    config: BoxPlaneConfig3d,
    offsets: &[[i64; 3]; 8],
    minimum_y: i64,
    lowest_world_y: i64,
) -> Result<BoxPlaneContact3d, BoxPlaneError3d> {
    let correction = config
        .plane_y
        .checked_sub(lowest_world_y)
        .ok_or(BoxPlaneError3d::ArithmeticOverflow)?;
    state.center.y = state
        .center
        .y
        .checked_add(correction)
        .ok_or(BoxPlaneError3d::ArithmeticOverflow)?;

    let (contact_offset, manifold_vertices) = lowest_manifold_offset(offsets, minimum_y)?;
    let point = Position::new3(
        state
            .center
            .x
            .checked_add(contact_offset[0])
            .ok_or(BoxPlaneError3d::ArithmeticOverflow)?,
        config.plane_y,
        state
            .center
            .z
            .checked_add(contact_offset[2])
            .ok_or(BoxPlaneError3d::ArithmeticOverflow)?,
    );
    let contact_y_velocity = contact_velocity(*state, contact_offset)?[1];
    let impulse = if contact_y_velocity < 0 {
        normal_impulse(
            body,
            state.angular.orientation,
            contact_offset,
            contact_y_velocity,
            config,
        )?
    } else {
        0
    };
    if impulse > 0 {
        apply_normal_impulse(state, body, contact_offset, impulse)?;
    }

    Ok(BoxPlaneContact3d {
        point,
        manifold_vertices,
        normal_impulse_units: impulse,
    })
}

fn lowest_manifold_offset(
    offsets: &[[i64; 3]; 8],
    minimum_y: i64,
) -> Result<([i64; 3], u8), BoxPlaneError3d> {
    let mut sum = [0_i128; 3];
    let mut count = 0_u8;
    for offset in offsets.iter().filter(|offset| offset[1] == minimum_y) {
        count = count
            .checked_add(1)
            .ok_or(BoxPlaneError3d::ArithmeticOverflow)?;
        for (total, component) in sum.iter_mut().zip(*offset) {
            *total = total
                .checked_add(i128::from(component))
                .ok_or(BoxPlaneError3d::ArithmeticOverflow)?;
        }
    }
    if count == 0 {
        return Err(BoxPlaneError3d::ArithmeticOverflow);
    }
    let divisor = i128::from(count);
    Ok((
        [
            i64::try_from(div_round_nearest(sum[0], divisor)?)
                .map_err(|_| BoxPlaneError3d::ArithmeticOverflow)?,
            i64::try_from(div_round_nearest(sum[1], divisor)?)
                .map_err(|_| BoxPlaneError3d::ArithmeticOverflow)?,
            i64::try_from(div_round_nearest(sum[2], divisor)?)
                .map_err(|_| BoxPlaneError3d::ArithmeticOverflow)?,
        ],
        count,
    ))
}

fn contact_velocity(
    state: BoxPlaneState3d,
    offset: [i64; 3],
) -> Result<[i64; 3], BoxPlaneError3d> {
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
) -> Result<i64, BoxPlaneError3d> {
    let rotational = i64::try_from(div_round_nearest(rotational_numerator, scale)?)
        .map_err(|_| BoxPlaneError3d::ArithmeticOverflow)?;
    i64::from(linear)
        .checked_add(rotational)
        .ok_or(BoxPlaneError3d::ArithmeticOverflow)
}

fn normal_impulse(
    body: PhysicsBody3d,
    orientation: Orientation3d,
    contact_offset: [i64; 3],
    normal_velocity: i64,
    config: BoxPlaneConfig3d,
) -> Result<i64, BoxPlaneError3d> {
    let inertia = box_inertia(body)?;
    let local = rotate_inverse(orientation, [-contact_offset[2], 0, contact_offset[0]])?;
    let mut rotational_term_scaled = 0_i128;
    for (axis, component) in local.into_iter().enumerate() {
        let inverse_inertia =
            inverse_inertia_scaled(inertia.principal_numerators[axis], inertia.denominator)?;
        let component = i128::from(component);
        rotational_term_scaled = checked_add(
            rotational_term_scaled,
            checked_mul(checked_mul(component, component)?, inverse_inertia)?,
        )?;
    }

    let inverse_mass_scaled = RESPONSE_SCALE / i128::from(body.mass_units);
    let effective_inverse_mass = checked_add(inverse_mass_scaled, rotational_term_scaled)?;
    if effective_inverse_mass <= 0 {
        return Err(BoxPlaneError3d::ArithmeticOverflow);
    }
    let restitution = body
        .material
        .restitution_milli
        .max(config.plane_material.restitution_milli);
    let closing_speed = i128::from(normal_velocity)
        .checked_neg()
        .ok_or(BoxPlaneError3d::ArithmeticOverflow)?;
    let bounce_scale = i128::from(MATERIAL_SCALE) + i128::from(restitution);
    let numerator = checked_mul(checked_mul(closing_speed, bounce_scale)?, RESPONSE_SCALE)?;
    let denominator = checked_mul(i128::from(MATERIAL_SCALE), effective_inverse_mass)?;
    i64::try_from(div_round_nearest(numerator, denominator)?)
        .map_err(|_| BoxPlaneError3d::ArithmeticOverflow)
}

fn inverse_inertia_scaled(
    principal_numerator: u128,
    denominator: u32,
) -> Result<i128, BoxPlaneError3d> {
    if principal_numerator == 0 {
        return Err(BoxPlaneError3d::ArithmeticOverflow);
    }
    let principal =
        i128::try_from(principal_numerator).map_err(|_| BoxPlaneError3d::ArithmeticOverflow)?;
    Ok(checked_mul(RESPONSE_SCALE, i128::from(denominator))? / principal)
}

fn apply_normal_impulse(
    state: &mut BoxPlaneState3d,
    body: PhysicsBody3d,
    contact_offset: [i64; 3],
    impulse: i64,
) -> Result<(), BoxPlaneError3d> {
    let linear_delta = div_round_nearest(i128::from(impulse), i128::from(body.mass_units))?;
    let next_linear_y = i128::from(state.linear_velocity.y)
        .checked_add(linear_delta)
        .ok_or(BoxPlaneError3d::ArithmeticOverflow)?;
    state.linear_velocity.y =
        i32::try_from(next_linear_y).map_err(|_| BoxPlaneError3d::ArithmeticOverflow)?;

    let angular_impulse = contact_angular_impulse(contact_offset, [0, impulse, 0])?;
    let angular_impulse_world = [
        i64::try_from(angular_impulse[0]).map_err(|_| BoxPlaneError3d::ArithmeticOverflow)?,
        i64::try_from(angular_impulse[1]).map_err(|_| BoxPlaneError3d::ArithmeticOverflow)?,
        i64::try_from(angular_impulse[2]).map_err(|_| BoxPlaneError3d::ArithmeticOverflow)?,
    ];
    let local_impulse = rotate_inverse(state.angular.orientation, angular_impulse_world)?;
    let inertia = box_inertia(body)?;
    let mut local_delta = [0_i64; 3];
    for (axis, (target, component)) in local_delta
        .iter_mut()
        .zip(local_impulse)
        .enumerate()
    {
        let inverse_inertia =
            inverse_inertia_scaled(inertia.principal_numerators[axis], inertia.denominator)?;
        let numerator = checked_mul(
            checked_mul(i128::from(component), inverse_inertia)?,
            i128::from(ANGULAR_VELOCITY_SCALE),
        )?;
        *target = i64::try_from(div_round_nearest(numerator, RESPONSE_SCALE)?)
            .map_err(|_| BoxPlaneError3d::ArithmeticOverflow)?;
    }
    let world_delta = rotate_with_matrix(rotation_matrix(state.angular.orientation)?, local_delta)?;
    state.angular.angular_velocity = AngularVelocity3d::new(
        add_angular_axis(state.angular.angular_velocity.x, world_delta[0])?,
        add_angular_axis(state.angular.angular_velocity.y, world_delta[1])?,
        add_angular_axis(state.angular.angular_velocity.z, world_delta[2])?,
    );
    Ok(())
}

fn add_angular_axis(current: i32, delta: i64) -> Result<i32, BoxPlaneError3d> {
    let next = i128::from(current)
        .checked_add(i128::from(delta))
        .ok_or(BoxPlaneError3d::ArithmeticOverflow)?;
    i32::try_from(next).map_err(|_| BoxPlaneError3d::ArithmeticOverflow)
}

fn damp_angular_velocity(
    velocity: AngularVelocity3d,
    damping_milli: u16,
) -> Result<AngularVelocity3d, BoxPlaneError3d> {
    let scale = i128::from(MATERIAL_SCALE);
    let damping = i128::from(damping_milli);
    Ok(AngularVelocity3d::new(
        damp_axis(velocity.x, damping, scale)?,
        damp_axis(velocity.y, damping, scale)?,
        damp_axis(velocity.z, damping, scale)?,
    ))
}

fn damp_axis(value: i32, damping: i128, scale: i128) -> Result<i32, BoxPlaneError3d> {
    let numerator = checked_mul(i128::from(value), damping)?;
    i32::try_from(div_round_nearest(numerator, scale)?)
        .map_err(|_| BoxPlaneError3d::ArithmeticOverflow)
}

fn div_round_nearest(numerator: i128, denominator: i128) -> Result<i128, BoxPlaneError3d> {
    if denominator <= 0 {
        return Err(BoxPlaneError3d::ArithmeticOverflow);
    }
    let half = denominator / 2;
    let adjusted = if numerator >= 0 {
        numerator.checked_add(half)
    } else {
        numerator.checked_sub(half)
    }
    .ok_or(BoxPlaneError3d::ArithmeticOverflow)?;
    Ok(adjusted / denominator)
}

#[cfg(test)]
mod tests {
    use ecs_physics::PhysicsMaterial;
    use ecs_workload::EntityId;

    use super::*;

    fn body() -> PhysicsBody3d {
        PhysicsBody3d::dynamic(EntityId(1), [10, 10, 10]).with_material(PhysicsMaterial::new(0, 0))
    }

    fn config() -> BoxPlaneConfig3d {
        BoxPlaneConfig3d {
            gravity: Velocity::new3(0, 0, 0),
            plane_y: 0,
            timestep_numerator: 1,
            timestep_denominator: 60,
            plane_material: PhysicsMaterial::new(0, 0),
            angular_damping_milli: 1_000,
        }
    }

    #[test]
    fn flat_face_impact_has_four_contact_vertices_and_no_torque() {
        let state = BoxPlaneState3d::new(
            Position::new3(0, 10, 0),
            Velocity::new3(0, -60, 0),
            AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
        );
        let step = step_box_on_plane(state, body(), config()).expect("valid plane impact");
        let contact = step.contact.expect("box should touch the plane");
        assert_eq!(contact.manifold_vertices, 4);
        assert!(contact.normal_impulse_units > 0);
        assert_eq!(
            step.state.angular.angular_velocity,
            AngularVelocity3d::default()
        );
        assert_eq!(step.state.center.y, 10);
    }

    #[test]
    fn tilted_corner_impact_generates_angular_velocity() {
        let orientation = Orientation3d::new(0, 0, 220_000_000, ORIENTATION_SCALE)
            .normalized()
            .expect("valid tilted orientation");
        let state = BoxPlaneState3d::new(
            Position::new3(0, 10, 0),
            Velocity::new3(0, -120, 0),
            AngularState3d::new(orientation, AngularVelocity3d::default()),
        );
        let step = step_box_on_plane(state, body(), config()).expect("valid tilted impact");
        let contact = step.contact.expect("tilted box should contact the plane");
        assert!(contact.manifold_vertices <= 2);
        assert!(contact.normal_impulse_units > 0);
        assert_ne!(step.state.angular.angular_velocity.z, 0);
    }

    #[test]
    fn identity_vertices_match_axis_aligned_box() {
        let vertices =
            oriented_box_vertices(Position::new3(2, 3, 4), [1, 2, 3], Orientation3d::IDENTITY)
                .expect("valid box");
        assert_eq!(vertices[0], Position::new3(1, 1, 1));
        assert_eq!(vertices[7], Position::new3(3, 5, 7));
    }

    #[test]
    fn zero_timestep_is_idempotent_when_clear_of_plane() {
        let state = BoxPlaneState3d::new(
            Position::new3(0, 100, 0),
            Velocity::new3(3, 4, 5),
            AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
        );
        let mut zero = config();
        zero.timestep_numerator = 0;
        assert_eq!(
            step_box_on_plane(state, body(), zero)
                .expect("zero time is valid")
                .state,
            state
        );
    }
}
