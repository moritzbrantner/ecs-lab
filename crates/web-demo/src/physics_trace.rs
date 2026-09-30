//! Full physical traces from the actual consumer loops; compiled only into native tests.
use ecs_physics::BodyKind;
use ecs_physics_3d::RigidBox3d;

/// Rows: entity, fixed flag, half extents XYZ, mass, restitution, friction, center XYZ,
/// linear velocity XYZ, quaternion XYZW, angular velocity XYZ.
pub(crate) fn record(
    scene: &str,
    step: u32,
    boxes: &[RigidBox3d],
    sampled_events: usize,
    tail_contacts: usize,
) {
    let rows = boxes
        .iter()
        .map(|rigid_box| {
            let body = rigid_box.body;
            let state = rigid_box.state;
            let q = state.angular.orientation;
            let omega = state.angular.angular_velocity;
            [
                i64::from(body.entity.0),
                i64::from(body.kind == BodyKind::Fixed),
                i64::from(body.half_extents[0]),
                i64::from(body.half_extents[1]),
                i64::from(body.half_extents[2]),
                i64::from(body.mass_units),
                i64::from(body.material.restitution_milli),
                i64::from(body.material.friction_milli),
                state.center.x,
                state.center.y,
                state.center.z,
                i64::from(state.linear_velocity.x),
                i64::from(state.linear_velocity.y),
                i64::from(state.linear_velocity.z),
                i64::from(q.x),
                i64::from(q.y),
                i64::from(q.z),
                i64::from(q.w),
                i64::from(omega.x),
                i64::from(omega.y),
                i64::from(omega.z),
            ]
        })
        .collect::<Vec<_>>();
    println!(
        "PHYSICAL_TRACE {{\"scene\":\"{scene}\",\"step\":{step},\"sampled_events\":{sampled_events},\"tail_contacts\":{tail_contacts},\"boxes\":{rows:?}}}"
    );
}
