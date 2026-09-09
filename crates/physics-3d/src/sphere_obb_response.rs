use std::fmt;

use ecs_physics::{BodyKind, MATERIAL_SCALE, PhysicsMaterial};
use ecs_workload::{EntityId, Position, Velocity};

use crate::{
    ANGULAR_VELOCITY_SCALE, AngularError3d, AngularVelocity3d, ORIENTATION_SCALE, Orientation3d,
    OrientedBox3d, PhysicsBody3d, RigidBoxState3d, Sphere3d, SphereObbContact3d,
    SphereObbError3d, box_inertia, sphere_obb_contact,
};

const RESPONSE_SCALE: i128 = 1_i128 << 50;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct SphereBody3d {
    pub entity: EntityId,
    pub radius: i32,
    pub kind: BodyKind,
    pub mass_units: u32,
    pub material: PhysicsMaterial,
}

impl SphereBody3d {
    #[must_use]
    pub const fn dynamic(entity: EntityId, radius: i32) -> Self {
        Self {
            entity,
            radius,
            kind: BodyKind::Dynamic,
            mass_units: 1,
            material: PhysicsMaterial::new(0, 0),
        }
    }

    #[must_use]
    pub const fn fixed(entity: EntityId, radius: i32) -> Self {
        Self {
            entity,
            radius,
            kind: BodyKind::Fixed,
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

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct RigidSphereState3d {
    pub center: Position,
    pub linear_velocity: Velocity,
    /// Retained now so the later friction slice can add sphere spin without changing response state.
    /// A normal sphere contact never changes this value because its impulse passes through the center.
    pub angular_velocity: AngularVelocity3d,
}

impl RigidSphereState3d {
    #[must_use]
    pub const fn new(
        center: Position,
        linear_velocity: Velocity,
        angular_velocity: AngularVelocity3d,
    ) -> Self {
        Self {
            center,
            linear_velocity,
            angular_velocity,
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct SphereObbResponseContact3d {
    pub geometry: SphereObbContact3d,
    pub normal_length_squared: u128,
    /// Scalar multiplier applied to `geometry.normal` for the normal impulse.
    pub normal_impulse_units: i64,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct SphereObbStep3d {
    pub sphere: RigidSphereState3d,
    pub oriented_box: RigidBoxState3d,
    pub contact: Option<SphereObbResponseContact3d>,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SphereObbResponseError3d {
    SameEntity(EntityId),
    InvalidRadius(EntityId),
    InvalidHalfExtents(EntityId),
    ZeroMass(EntityId),
    RestitutionOutOfRange(EntityId, u16),
    Geometry(SphereObbError3d),
    Angular(AngularError3d),
    ArithmeticOverflow,
}

impl fmt::Display for SphereObbResponseError3d {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::SameEntity(entity) => write!(
                formatter,
                "sphere-OBB response requires two distinct bodies, got entity {} twice",
                entity.0
            ),
            Self::InvalidRadius(entity) => write!(
                formatter,
                "sphere-OBB response body {} requires a positive radius",
                entity.0
            ),
            Self::InvalidHalfExtents(entity) => write!(
                formatter,
                "sphere-OBB response body {} requires strictly positive half extents",
                entity.0
            ),
            Self::ZeroMass(entity) => write!(
                formatter,
                "dynamic sphere-OBB response body {} requires non-zero mass",
                entity.0
            ),
            Self::RestitutionOutOfRange(entity, value) => write!(
                formatter,
                "sphere-OBB response body {} has restitution {value}, expected 0..={MATERIAL_SCALE}",
                entity.0
            ),
            Self::Geometry(error) => write!(formatter, "sphere-OBB geometry failed: {error}"),
            Self::Angular(error) => write!(formatter, "sphere-OBB angular response failed: {error}"),
            Self::ArithmeticOverflow => {
                write!(formatter, "sphere-OBB response arithmetic overflowed")
            }
        }
    }
}

impl std::error::Error for SphereObbResponseError3d {}

impl From<SphereObbError3d> for SphereObbResponseError3d {
    fn from(value: SphereObbError3d) -> Self {
        Self::Geometry(value)
    }
}

impl From<AngularError3d> for SphereObbResponseError3d {
    fn from(value: AngularError3d) -> Self {
        Self::Angular(value)
    }
}

/// Resolves one overlapping sphere/oriented-box pair with a deterministic normal impulse.
///
/// The closest-feature query remains the single geometry authority. Its primitive normal points from the
/// box toward the sphere for exterior contacts and toward the selected escape face for an interior sphere
/// center. Relative contact velocity therefore uses `sphere - box`, applies the normal impulse toward the
/// sphere, and applies the equal-and-opposite impulse at the Rust-owned OBB contact point. Off-center
/// impacts can consequently change box angular velocity while the sphere receives no normal-contact spin.
///
/// This response seam does not project penetration, apply friction, integrate a sphere world, or claim
/// continuous sphere/OBB collision detection. Use [`stabilize_sphere_obb_contact`] when discrete position
/// projection is also required.
///
/// # Errors
///
/// Returns [`SphereObbResponseError3d`] for duplicate entities, malformed bodies/materials/orientation,
/// invalid geometry, or checked arithmetic overflow.
pub fn resolve_sphere_obb_contact(
    sphere_state: RigidSphereState3d,
    sphere_body: SphereBody3d,
    box_state: RigidBoxState3d,
    box_body: PhysicsBody3d,
) -> Result<SphereObbStep3d, SphereObbResponseError3d> {
    validate_pair(sphere_body, box_body)?;

    let geometry = sphere_obb_contact(
        Sphere3d::new(sphere_state.center, sphere_body.radius),
        OrientedBox3d::new(
            box_state.center,
            box_body.half_extents,
            box_state.angular.orientation,
        ),
    )?;
    if !geometry.overlaps() {
        return Ok(SphereObbStep3d {
            sphere: sphere_state,
            oriented_box: box_state,
            contact: None,
        });
    }

    let normal_length_squared = vector_length_squared(geometry.normal)?;
    let box_offset = position_delta(box_state.center, geometry.point)?;
    let box_velocity = box_contact_velocity(box_state, box_offset)?;
    let relative_velocity = [
        checked_sub(
            i128::from(sphere_state.linear_velocity.x),
            i128::from(box_velocity[0]),
        )?,
        checked_sub(
            i128::from(sphere_state.linear_velocity.y),
            i128::from(box_velocity[1]),
        )?,
        checked_sub(
            i128::from(sphere_state.linear_velocity.z),
            i128::from(box_velocity[2]),
        )?,
    ];
    let normal_velocity = checked_dot(relative_velocity, geometry.normal)?;

    let mut sphere = sphere_state;
    let mut oriented_box = box_state;
    let normal_impulse_units = if normal_velocity < 0
        && (sphere_body.kind == BodyKind::Dynamic || box_body.kind == BodyKind::Dynamic)
    {
        let impulse = normal_impulse(
            sphere_body,
            box_body,
            box_state.angular.orientation,
            box_offset,
            geometry.normal,
            normal_length_squared,
            normal_velocity,
        )?;
        if impulse > 0 {
            let sphere_impulse = scale_axis(geometry.normal, i128::from(impulse))?;
            let box_impulse = scale_axis(geometry.normal, -i128::from(impulse))?;
            apply_sphere_impulse(&mut sphere, sphere_body, sphere_impulse)?;
            apply_box_impulse(&mut oriented_box, box_body, box_offset, box_impulse)?;
        }
        impulse
    } else {
        0
    };

    Ok(SphereObbStep3d {
        sphere,
        oriented_box,
        contact: Some(SphereObbResponseContact3d {
            geometry,
            normal_length_squared,
            normal_impulse_units,
        }),
    })
}

/// Resolves a sphere/OBB normal contact and projects any remaining discrete penetration out of the pair.
///
/// Position correction uses the same geometry normal as the response impulse. Exterior penetration uses a
/// conservative integer upper bound for `radius - closest_distance`; an interior sphere uses
/// `radius + closest_surface_distance`. The required normal projection is converted to the nearest integer
/// vector and then closed exactly along the dominant normal axis, avoiding floating-point normalization.
///
/// Dynamic/dynamic correction is inverse-mass weighted. When an odd integer-grid correction cannot be
/// split exactly, ascending entity id is the stable tie-breaker: the lower id keeps the directly rounded
/// share and the other participant receives the residual needed to preserve the exact relative correction.
/// Fixed bodies never move.
///
/// # Errors
///
/// Returns [`SphereObbResponseError3d`] for response errors or checked stabilization overflow.
pub fn stabilize_sphere_obb_contact(
    sphere_state: RigidSphereState3d,
    sphere_body: SphereBody3d,
    box_state: RigidBoxState3d,
    box_body: PhysicsBody3d,
) -> Result<SphereObbStep3d, SphereObbResponseError3d> {
    let mut step = resolve_sphere_obb_contact(sphere_state, sphere_body, box_state, box_body)?;
    let Some(contact) = step.contact else {
        return Ok(step);
    };
    if !is_penetrating(contact.geometry)
        || (sphere_body.kind == BodyKind::Fixed && box_body.kind == BodyKind::Fixed)
    {
        return Ok(step);
    }

    let correction = penetration_correction(contact.geometry, sphere_body.radius)?;
    project_pair(
        &mut step.sphere,
        sphere_body,
        &mut step.oriented_box,
        box_body,
        correction,
    )?;
    Ok(step)
}

fn validate_pair(
    sphere_body: SphereBody3d,
    box_body: PhysicsBody3d,
) -> Result<(), SphereObbResponseError3d> {
    if sphere_body.entity == box_body.entity {
        return Err(SphereObbResponseError3d::SameEntity(sphere_body.entity));
    }
    if sphere_body.radius <= 0 {
        return Err(SphereObbResponseError3d::InvalidRadius(
            sphere_body.entity,
        ));
    }
    if box_body.half_extents.iter().any(|extent| *extent <= 0) {
        return Err(SphereObbResponseError3d::InvalidHalfExtents(
            box_body.entity,
        ));
    }
    if sphere_body.kind == BodyKind::Dynamic && sphere_body.mass_units == 0 {
        return Err(SphereObbResponseError3d::ZeroMass(sphere_body.entity));
    }
    if box_body.kind == BodyKind::Dynamic && box_body.mass_units == 0 {
        return Err(SphereObbResponseError3d::ZeroMass(box_body.entity));
    }
    if sphere_body.material.restitution_milli > MATERIAL_SCALE {
        return Err(SphereObbResponseError3d::RestitutionOutOfRange(
            sphere_body.entity,
            sphere_body.material.restitution_milli,
        ));
    }
    if box_body.material.restitution_milli > MATERIAL_SCALE {
        return Err(SphereObbResponseError3d::RestitutionOutOfRange(
            box_body.entity,
            box_body.material.restitution_milli,
        ));
    }
    Ok(())
}

const fn is_penetrating(contact: SphereObbContact3d) -> bool {
    contact.center_inside || contact.distance_squared_numerator < contact.threshold_squared_numerator
}

#[allow(clippy::too_many_arguments)]
fn normal_impulse(
    sphere_body: SphereBody3d,
    box_body: PhysicsBody3d,
    box_orientation: Orientation3d,
    box_offset: [i64; 3],
    axis: [i128; 3],
    axis_length_squared: u128,
    normal_velocity: i128,
) -> Result<i64, SphereObbResponseError3d> {
    let effective_inverse_mass = checked_add(
        sphere_effective_inverse_mass_scaled(sphere_body, axis_length_squared)?,
        box_effective_inverse_mass_scaled(
            box_body,
            box_orientation,
            box_offset,
            axis,
            axis_length_squared,
        )?,
    )?;
    if effective_inverse_mass <= 0 {
        return Ok(0);
    }

    let restitution = sphere_body
        .material
        .restitution_milli
        .max(box_body.material.restitution_milli);
    let closing_speed = normal_velocity
        .checked_neg()
        .ok_or(SphereObbResponseError3d::ArithmeticOverflow)?;
    let bounce_scale = checked_add(i128::from(MATERIAL_SCALE), i128::from(restitution))?;
    let numerator = checked_mul(checked_mul(closing_speed, bounce_scale)?, RESPONSE_SCALE)?;
    let denominator = checked_mul(i128::from(MATERIAL_SCALE), effective_inverse_mass)?;
    i64::try_from(div_round_nearest(numerator, denominator)?)
        .map_err(|_| SphereObbResponseError3d::ArithmeticOverflow)
}

fn sphere_effective_inverse_mass_scaled(
    body: SphereBody3d,
    axis_length_squared: u128,
) -> Result<i128, SphereObbResponseError3d> {
    if body.kind == BodyKind::Fixed {
        return Ok(0);
    }
    let length_squared = i128::try_from(axis_length_squared)
        .map_err(|_| SphereObbResponseError3d::ArithmeticOverflow)?;
    checked_mul(RESPONSE_SCALE, length_squared).map(|value| value / i128::from(body.mass_units))
}

fn box_effective_inverse_mass_scaled(
    body: PhysicsBody3d,
    orientation: Orientation3d,
    contact_offset: [i64; 3],
    axis: [i128; 3],
    axis_length_squared: u128,
) -> Result<i128, SphereObbResponseError3d> {
    if body.kind == BodyKind::Fixed {
        return Ok(0);
    }

    let length_squared = i128::try_from(axis_length_squared)
        .map_err(|_| SphereObbResponseError3d::ArithmeticOverflow)?;
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

fn apply_sphere_impulse(
    state: &mut RigidSphereState3d,
    body: SphereBody3d,
    impulse: [i128; 3],
) -> Result<(), SphereObbResponseError3d> {
    if body.kind == BodyKind::Fixed {
        return Ok(());
    }
    state.linear_velocity = Velocity::new3(
        add_linear_impulse_axis(state.linear_velocity.x, impulse[0], body.mass_units)?,
        add_linear_impulse_axis(state.linear_velocity.y, impulse[1], body.mass_units)?,
        add_linear_impulse_axis(state.linear_velocity.z, impulse[2], body.mass_units)?,
    );
    Ok(())
}

fn apply_box_impulse(
    state: &mut RigidBoxState3d,
    body: PhysicsBody3d,
    contact_offset: [i64; 3],
    impulse: [i128; 3],
) -> Result<(), SphereObbResponseError3d> {
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

fn box_contact_velocity(
    state: RigidBoxState3d,
    offset: [i64; 3],
) -> Result<[i64; 3], SphereObbResponseError3d> {
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

fn penetration_correction(
    contact: SphereObbContact3d,
    radius: i32,
) -> Result<[i64; 3], SphereObbResponseError3d> {
    let distance_floor = floor_sqrt_ratio(
        contact.distance_squared_numerator,
        contact.distance_squared_denominator,
    )?;
    let required_distance = if contact.center_inside {
        let distance_ceil = ceil_sqrt_ratio(
            contact.distance_squared_numerator,
            contact.distance_squared_denominator,
            distance_floor,
        )?;
        i128::from(radius)
            .checked_add(distance_ceil)
            .ok_or(SphereObbResponseError3d::ArithmeticOverflow)?
    } else {
        i128::from(radius)
            .checked_sub(distance_floor)
            .ok_or(SphereObbResponseError3d::ArithmeticOverflow)?
    };
    if required_distance <= 0 {
        return Ok([0, 0, 0]);
    }

    let axis_length_squared = vector_length_squared(contact.normal)?;
    let axis_length_ceil = ceil_sqrt_u128(axis_length_squared)?;
    let target_projection = checked_mul(
        required_distance,
        i128::try_from(axis_length_ceil)
            .map_err(|_| SphereObbResponseError3d::ArithmeticOverflow)?,
    )?;
    correction_for_projection(contact.normal, axis_length_squared, target_projection)
}

fn correction_for_projection(
    axis: [i128; 3],
    axis_length_squared: u128,
    target_projection: i128,
) -> Result<[i64; 3], SphereObbResponseError3d> {
    let length_squared = i128::try_from(axis_length_squared)
        .map_err(|_| SphereObbResponseError3d::ArithmeticOverflow)?;
    if length_squared <= 0 || target_projection <= 0 {
        return Err(SphereObbResponseError3d::ArithmeticOverflow);
    }

    let mut correction = [0_i128; 3];
    for (target, component) in correction.iter_mut().zip(axis) {
        *target = div_round_nearest(checked_mul(target_projection, component)?, length_squared)?;
    }

    let achieved = checked_dot(correction, axis)?;
    if achieved < target_projection {
        let dominant = dominant_axis(axis)?;
        let magnitude = checked_abs(axis[dominant])?;
        let residual = checked_sub(target_projection, achieved)?;
        let extra = div_ceil_positive(residual, magnitude)?;
        let signed_extra = if axis[dominant] < 0 { -extra } else { extra };
        correction[dominant] = checked_add(correction[dominant], signed_extra)?;
    }

    Ok([
        i64::try_from(correction[0])
            .map_err(|_| SphereObbResponseError3d::ArithmeticOverflow)?,
        i64::try_from(correction[1])
            .map_err(|_| SphereObbResponseError3d::ArithmeticOverflow)?,
        i64::try_from(correction[2])
            .map_err(|_| SphereObbResponseError3d::ArithmeticOverflow)?,
    ])
}

fn project_pair(
    sphere: &mut RigidSphereState3d,
    sphere_body: SphereBody3d,
    oriented_box: &mut RigidBoxState3d,
    box_body: PhysicsBody3d,
    correction: [i64; 3],
) -> Result<(), SphereObbResponseError3d> {
    match (sphere_body.kind, box_body.kind) {
        (BodyKind::Fixed, BodyKind::Fixed) => Ok(()),
        (BodyKind::Dynamic, BodyKind::Fixed) => {
            sphere.center = offset_position(sphere.center, correction)?;
            Ok(())
        }
        (BodyKind::Fixed, BodyKind::Dynamic) => {
            oriented_box.center = offset_position(oriented_box.center, negate_vector(correction)?)?;
            Ok(())
        }
        (BodyKind::Dynamic, BodyKind::Dynamic) => project_dynamic_pair(
            sphere,
            sphere_body,
            oriented_box,
            box_body,
            correction,
        ),
    }
}

fn project_dynamic_pair(
    sphere: &mut RigidSphereState3d,
    sphere_body: SphereBody3d,
    oriented_box: &mut RigidBoxState3d,
    box_body: PhysicsBody3d,
    correction: [i64; 3],
) -> Result<(), SphereObbResponseError3d> {
    let total_mass = u64::from(sphere_body.mass_units)
        .checked_add(u64::from(box_body.mass_units))
        .ok_or(SphereObbResponseError3d::ArithmeticOverflow)?;
    let mut sphere_move = [0_i64; 3];
    let mut box_move = [0_i64; 3];

    for axis in 0..3 {
        if sphere_body.entity < box_body.entity {
            let sphere_weighted = checked_mul(
                i128::from(correction[axis]),
                i128::from(box_body.mass_units),
            )?;
            sphere_move[axis] = i64::try_from(div_round_nearest(
                sphere_weighted,
                i128::from(total_mass),
            )?)
            .map_err(|_| SphereObbResponseError3d::ArithmeticOverflow)?;
            box_move[axis] = sphere_move[axis]
                .checked_sub(correction[axis])
                .ok_or(SphereObbResponseError3d::ArithmeticOverflow)?;
        } else {
            let box_weighted = checked_mul(
                -i128::from(correction[axis]),
                i128::from(sphere_body.mass_units),
            )?;
            box_move[axis] = i64::try_from(div_round_nearest(
                box_weighted,
                i128::from(total_mass),
            )?)
            .map_err(|_| SphereObbResponseError3d::ArithmeticOverflow)?;
            sphere_move[axis] = correction[axis]
                .checked_add(box_move[axis])
                .ok_or(SphereObbResponseError3d::ArithmeticOverflow)?;
        }
    }

    sphere.center = offset_position(sphere.center, sphere_move)?;
    oriented_box.center = offset_position(oriented_box.center, box_move)?;
    Ok(())
}

fn floor_sqrt_ratio(
    numerator: i128,
    denominator: i128,
) -> Result<i128, SphereObbResponseError3d> {
    if numerator < 0 || denominator <= 0 {
        return Err(SphereObbResponseError3d::ArithmeticOverflow);
    }
    let numerator = u128::try_from(numerator)
        .map_err(|_| SphereObbResponseError3d::ArithmeticOverflow)?;
    let denominator = u128::try_from(denominator)
        .map_err(|_| SphereObbResponseError3d::ArithmeticOverflow)?;
    let quotient = numerator / denominator;
    i128::try_from(integer_sqrt_u128(quotient))
        .map_err(|_| SphereObbResponseError3d::ArithmeticOverflow)
}

fn ceil_sqrt_ratio(
    numerator: i128,
    denominator: i128,
    floor: i128,
) -> Result<i128, SphereObbResponseError3d> {
    if numerator < 0 || denominator <= 0 || floor < 0 {
        return Err(SphereObbResponseError3d::ArithmeticOverflow);
    }
    let floor_squared = checked_mul(floor, floor)?;
    let represented = checked_mul(floor_squared, denominator)?;
    if represented == numerator {
        Ok(floor)
    } else {
        floor
            .checked_add(1)
            .ok_or(SphereObbResponseError3d::ArithmeticOverflow)
    }
}

fn ceil_sqrt_u128(value: u128) -> Result<u128, SphereObbResponseError3d> {
    let floor = integer_sqrt_u128(value);
    if floor
        .checked_mul(floor)
        .ok_or(SphereObbResponseError3d::ArithmeticOverflow)?
        == value
    {
        Ok(floor)
    } else {
        floor
            .checked_add(1)
            .ok_or(SphereObbResponseError3d::ArithmeticOverflow)
    }
}

fn integer_sqrt_u128(value: u128) -> u128 {
    if value < 2 {
        return value;
    }
    let mut result = 0_u128;
    let mut bit = 1_u128 << 126;
    while bit > value {
        bit >>= 2;
    }
    let mut remainder = value;
    while bit != 0 {
        if remainder >= result + bit {
            remainder -= result + bit;
            result = (result >> 1) + bit;
        } else {
            result >>= 1;
        }
        bit >>= 2;
    }
    result
}

fn dominant_axis(axis: [i128; 3]) -> Result<usize, SphereObbResponseError3d> {
    let mut best = 0_usize;
    let mut magnitude = checked_abs(axis[0])?;
    for (index, component) in axis.into_iter().enumerate().skip(1) {
        let candidate = checked_abs(component)?;
        if candidate > magnitude {
            best = index;
            magnitude = candidate;
        }
    }
    if magnitude == 0 {
        return Err(SphereObbResponseError3d::ArithmeticOverflow);
    }
    Ok(best)
}

fn vector_length_squared(vector: [i128; 3]) -> Result<u128, SphereObbResponseError3d> {
    vector.into_iter().try_fold(0_u128, |sum, component| {
        let magnitude = component.unsigned_abs();
        sum.checked_add(
            magnitude
                .checked_mul(magnitude)
                .ok_or(SphereObbResponseError3d::ArithmeticOverflow)?,
        )
        .ok_or(SphereObbResponseError3d::ArithmeticOverflow)
    })
}

fn position_delta(
    center: Position,
    point: Position,
) -> Result<[i64; 3], SphereObbResponseError3d> {
    Ok([
        point
            .x
            .checked_sub(center.x)
            .ok_or(SphereObbResponseError3d::ArithmeticOverflow)?,
        point
            .y
            .checked_sub(center.y)
            .ok_or(SphereObbResponseError3d::ArithmeticOverflow)?,
        point
            .z
            .checked_sub(center.z)
            .ok_or(SphereObbResponseError3d::ArithmeticOverflow)?,
    ])
}

fn offset_position(
    position: Position,
    delta: [i64; 3],
) -> Result<Position, SphereObbResponseError3d> {
    Ok(Position::new3(
        position
            .x
            .checked_add(delta[0])
            .ok_or(SphereObbResponseError3d::ArithmeticOverflow)?,
        position
            .y
            .checked_add(delta[1])
            .ok_or(SphereObbResponseError3d::ArithmeticOverflow)?,
        position
            .z
            .checked_add(delta[2])
            .ok_or(SphereObbResponseError3d::ArithmeticOverflow)?,
    ))
}

fn negate_vector(vector: [i64; 3]) -> Result<[i64; 3], SphereObbResponseError3d> {
    Ok([
        vector[0]
            .checked_neg()
            .ok_or(SphereObbResponseError3d::ArithmeticOverflow)?,
        vector[1]
            .checked_neg()
            .ok_or(SphereObbResponseError3d::ArithmeticOverflow)?,
        vector[2]
            .checked_neg()
            .ok_or(SphereObbResponseError3d::ArithmeticOverflow)?,
    ])
}

fn add_linear_rotation(
    linear: i32,
    rotational_numerator: i128,
    scale: i128,
) -> Result<i64, SphereObbResponseError3d> {
    let rotational = i64::try_from(div_round_nearest(rotational_numerator, scale)?)
        .map_err(|_| SphereObbResponseError3d::ArithmeticOverflow)?;
    i64::from(linear)
        .checked_add(rotational)
        .ok_or(SphereObbResponseError3d::ArithmeticOverflow)
}

fn add_linear_impulse_axis(
    current: i32,
    impulse: i128,
    mass_units: u32,
) -> Result<i32, SphereObbResponseError3d> {
    let delta = div_round_nearest(impulse, i128::from(mass_units))?;
    let next = checked_add(i128::from(current), delta)?;
    i32::try_from(next).map_err(|_| SphereObbResponseError3d::ArithmeticOverflow)
}

fn add_angular_axis(
    current: i32,
    delta: i128,
) -> Result<i32, SphereObbResponseError3d> {
    let next = checked_add(i128::from(current), delta)?;
    i32::try_from(next).map_err(|_| SphereObbResponseError3d::ArithmeticOverflow)
}

fn inverse_inertia_scaled(
    principal_numerator: u128,
    denominator: u32,
) -> Result<i128, SphereObbResponseError3d> {
    if principal_numerator == 0 {
        return Err(SphereObbResponseError3d::ArithmeticOverflow);
    }
    let principal = i128::try_from(principal_numerator)
        .map_err(|_| SphereObbResponseError3d::ArithmeticOverflow)?;
    Ok(checked_mul(RESPONSE_SCALE, i128::from(denominator))? / principal)
}

fn cross_i64_i128(
    left: [i64; 3],
    right: [i128; 3],
) -> Result<[i128; 3], SphereObbResponseError3d> {
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
) -> Result<i128, SphereObbResponseError3d> {
    checked_sub(checked_mul(left_a, right_a)?, checked_mul(left_b, right_b)?)
}

fn scale_axis(
    axis: [i128; 3],
    scale: i128,
) -> Result<[i128; 3], SphereObbResponseError3d> {
    Ok([
        checked_mul(axis[0], scale)?,
        checked_mul(axis[1], scale)?,
        checked_mul(axis[2], scale)?,
    ])
}

fn rotate_inverse(
    orientation: Orientation3d,
    vector: [i128; 3],
) -> Result<[i128; 3], SphereObbResponseError3d> {
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
) -> Result<[i128; 3], SphereObbResponseError3d> {
    rotate_with_matrix(rotation_matrix(orientation.normalized()?)?, vector)
}

fn rotation_matrix(
    orientation: Orientation3d,
) -> Result<[[i128; 3]; 3], SphereObbResponseError3d> {
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
) -> Result<[i128; 3], SphereObbResponseError3d> {
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

fn scaled_twice(
    value: i128,
    scale: i128,
) -> Result<i128, SphereObbResponseError3d> {
    div_round_nearest(checked_mul(value, 2)?, scale)
}

fn checked_abs(value: i128) -> Result<i128, SphereObbResponseError3d> {
    value
        .checked_abs()
        .ok_or(SphereObbResponseError3d::ArithmeticOverflow)
}

fn checked_mul(left: i128, right: i128) -> Result<i128, SphereObbResponseError3d> {
    left.checked_mul(right)
        .ok_or(SphereObbResponseError3d::ArithmeticOverflow)
}

fn checked_add(left: i128, right: i128) -> Result<i128, SphereObbResponseError3d> {
    left.checked_add(right)
        .ok_or(SphereObbResponseError3d::ArithmeticOverflow)
}

fn checked_sub(left: i128, right: i128) -> Result<i128, SphereObbResponseError3d> {
    left.checked_sub(right)
        .ok_or(SphereObbResponseError3d::ArithmeticOverflow)
}

fn checked_dot(
    left: [i128; 3],
    right: [i128; 3],
) -> Result<i128, SphereObbResponseError3d> {
    checked_add(
        checked_add(
            checked_mul(left[0], right[0])?,
            checked_mul(left[1], right[1])?,
        )?,
        checked_mul(left[2], right[2])?,
    )
}

fn div_round_nearest(
    numerator: i128,
    denominator: i128,
) -> Result<i128, SphereObbResponseError3d> {
    if denominator <= 0 {
        return Err(SphereObbResponseError3d::ArithmeticOverflow);
    }
    let half = denominator / 2;
    let adjusted = if numerator >= 0 {
        numerator.checked_add(half)
    } else {
        numerator.checked_sub(half)
    }
    .ok_or(SphereObbResponseError3d::ArithmeticOverflow)?;
    Ok(adjusted / denominator)
}

fn div_ceil_positive(
    numerator: i128,
    denominator: i128,
) -> Result<i128, SphereObbResponseError3d> {
    if numerator < 0 || denominator <= 0 {
        return Err(SphereObbResponseError3d::ArithmeticOverflow);
    }
    numerator
        .checked_add(denominator - 1)
        .map(|value| value / denominator)
        .ok_or(SphereObbResponseError3d::ArithmeticOverflow)
}

#[cfg(test)]
mod tests {
    use ecs_physics::PhysicsMaterial;

    use super::*;
    use crate::{AngularState3d, Orientation3d};

    fn sphere_state(center: Position, velocity: Velocity) -> RigidSphereState3d {
        RigidSphereState3d::new(center, velocity, AngularVelocity3d::default())
    }

    fn box_state(center: Position, velocity: Velocity) -> RigidBoxState3d {
        RigidBoxState3d::new(
            center,
            velocity,
            AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
        )
    }

    fn dynamic_sphere(entity: u32) -> SphereBody3d {
        SphereBody3d::dynamic(EntityId(entity), 2)
            .with_material(PhysicsMaterial::new(0, 0))
    }

    fn dynamic_box(entity: u32) -> PhysicsBody3d {
        PhysicsBody3d::dynamic(EntityId(entity), [10, 10, 10])
            .with_material(PhysicsMaterial::new(0, 0))
    }

    fn remaining_penetration(
        sphere: RigidSphereState3d,
        sphere_body: SphereBody3d,
        oriented_box: RigidBoxState3d,
        box_body: PhysicsBody3d,
    ) -> bool {
        let contact = sphere_obb_contact(
            Sphere3d::new(sphere.center, sphere_body.radius),
            OrientedBox3d::new(
                oriented_box.center,
                box_body.half_extents,
                oriented_box.angular.orientation,
            ),
        )
        .expect("valid geometry");
        is_penetrating(contact)
    }

    #[test]
    fn separated_pair_is_idempotently_unchanged() {
        let sphere = sphere_state(Position::new3(20, 0, 0), Velocity::new3(-20, 0, 0));
        let oriented_box = box_state(Position::new3(0, 0, 0), Velocity::new3(0, 0, 0));
        let step = stabilize_sphere_obb_contact(
            sphere,
            dynamic_sphere(1),
            oriented_box,
            dynamic_box(2),
        )
        .expect("valid separated pair");

        assert_eq!(step.contact, None);
        assert_eq!(step.sphere, sphere);
        assert_eq!(step.oriented_box, oriented_box);
    }

    #[test]
    fn centered_impact_changes_linear_velocity_without_spin() {
        let sphere = sphere_state(Position::new3(11, 0, 0), Velocity::new3(-60, 0, 0));
        let oriented_box = box_state(Position::new3(0, 0, 0), Velocity::new3(0, 0, 0));
        let step = resolve_sphere_obb_contact(
            sphere,
            dynamic_sphere(1),
            oriented_box,
            dynamic_box(2),
        )
        .expect("valid centered impact");
        let contact = step.contact.expect("overlapping pair should contact");

        assert_eq!(contact.geometry.normal, [1, 0, 0]);
        assert!(contact.normal_impulse_units > 0);
        assert!(step.sphere.linear_velocity.x > sphere.linear_velocity.x);
        assert!(step.oriented_box.linear_velocity.x < oriented_box.linear_velocity.x);
        assert_eq!(step.sphere.angular_velocity, sphere.angular_velocity);
        assert_eq!(
            step.oriented_box.angular.angular_velocity,
            AngularVelocity3d::default()
        );
    }

    #[test]
    fn glancing_sphere_impact_generates_box_spin() {
        let sphere = sphere_state(Position::new3(11, 5, 0), Velocity::new3(-90, 0, 0));
        let oriented_box = box_state(Position::new3(0, 0, 0), Velocity::new3(0, 0, 0));
        let step = resolve_sphere_obb_contact(
            sphere,
            dynamic_sphere(1),
            oriented_box,
            dynamic_box(2),
        )
        .expect("valid glancing impact");

        assert!(
            step.contact
                .expect("overlapping pair should contact")
                .normal_impulse_units
                > 0
        );
        assert_ne!(step.oriented_box.angular.angular_velocity.z, 0);
        assert_eq!(step.sphere.angular_velocity, sphere.angular_velocity);
    }

    #[test]
    fn fixed_box_remains_immutable() {
        let sphere = sphere_state(Position::new3(11, 0, 0), Velocity::new3(-60, 0, 0));
        let oriented_box = box_state(Position::new3(0, 0, 0), Velocity::new3(0, 0, 0));
        let box_body = PhysicsBody3d::fixed(EntityId(2), [10, 10, 10]);
        let step = stabilize_sphere_obb_contact(
            sphere,
            dynamic_sphere(1),
            oriented_box,
            box_body,
        )
        .expect("valid fixed-box contact");

        assert_eq!(step.oriented_box, oriented_box);
        assert!(!remaining_penetration(
            step.sphere,
            dynamic_sphere(1),
            step.oriented_box,
            box_body,
        ));
    }

    #[test]
    fn fixed_sphere_remains_immutable() {
        let sphere = sphere_state(Position::new3(11, 0, 0), Velocity::new3(0, 0, 0));
        let sphere_body = SphereBody3d::fixed(EntityId(1), 2);
        let oriented_box = box_state(Position::new3(0, 0, 0), Velocity::new3(60, 0, 0));
        let box_body = dynamic_box(2);
        let step = stabilize_sphere_obb_contact(sphere, sphere_body, oriented_box, box_body)
            .expect("valid fixed-sphere contact");

        assert_eq!(step.sphere, sphere);
        assert!(!remaining_penetration(
            step.sphere,
            sphere_body,
            step.oriented_box,
            box_body,
        ));
    }

    #[test]
    fn stabilization_closes_exterior_and_interior_penetration() {
        let box_body = PhysicsBody3d::fixed(EntityId(2), [10, 10, 10]);
        let oriented_box = box_state(Position::new3(0, 0, 0), Velocity::new3(0, 0, 0));

        for center in [Position::new3(11, 0, 0), Position::new3(0, 0, 0)] {
            let sphere = sphere_state(center, Velocity::new3(0, 0, 0));
            let sphere_body = dynamic_sphere(1);
            let step = stabilize_sphere_obb_contact(sphere, sphere_body, oriented_box, box_body)
                .expect("valid stabilization");

            assert!(!remaining_penetration(
                step.sphere,
                sphere_body,
                step.oriented_box,
                box_body,
            ));
        }
    }

    #[test]
    fn ascending_entity_id_breaks_odd_projection_ties_independent_of_shape_position() {
        let sphere = sphere_state(Position::new3(11, 0, 0), Velocity::new3(0, 0, 0));
        let oriented_box = box_state(Position::new3(0, 0, 0), Velocity::new3(0, 0, 0));

        let sphere_first = stabilize_sphere_obb_contact(
            sphere,
            dynamic_sphere(1),
            oriented_box,
            dynamic_box(2),
        )
        .expect("valid canonical tie");
        let box_first = stabilize_sphere_obb_contact(
            sphere,
            dynamic_sphere(2),
            oriented_box,
            dynamic_box(1),
        )
        .expect("valid reversed ids");

        assert_eq!(sphere_first.sphere.center.x, 12);
        assert_eq!(sphere_first.oriented_box.center.x, 0);
        assert_eq!(box_first.sphere.center.x, 11);
        assert_eq!(box_first.oriented_box.center.x, -1);
    }

    #[test]
    fn repeated_response_is_exactly_deterministic() {
        let sphere = sphere_state(Position::new3(11, 5, 0), Velocity::new3(-90, 0, 0));
        let oriented_box = box_state(Position::new3(0, 0, 0), Velocity::new3(0, 0, 0));
        let sphere_body = dynamic_sphere(1);
        let box_body = dynamic_box(2);

        let first = stabilize_sphere_obb_contact(sphere, sphere_body, oriented_box, box_body)
            .expect("valid first replay");
        let second = stabilize_sphere_obb_contact(sphere, sphere_body, oriented_box, box_body)
            .expect("valid second replay");

        assert_eq!(first, second);
    }

    #[test]
    fn duplicate_entity_and_invalid_radius_fail_closed() {
        let sphere = sphere_state(Position::new3(11, 0, 0), Velocity::new3(0, 0, 0));
        let oriented_box = box_state(Position::new3(0, 0, 0), Velocity::new3(0, 0, 0));

        assert_eq!(
            stabilize_sphere_obb_contact(
                sphere,
                dynamic_sphere(1),
                oriented_box,
                dynamic_box(1),
            ),
            Err(SphereObbResponseError3d::SameEntity(EntityId(1)))
        );
        assert_eq!(
            stabilize_sphere_obb_contact(
                sphere,
                SphereBody3d::dynamic(EntityId(1), 0),
                oriented_box,
                dynamic_box(2),
            ),
            Err(SphereObbResponseError3d::InvalidRadius(EntityId(1)))
        );
    }
}
