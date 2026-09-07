use ecs_physics::PhysicsMaterial;
use ecs_physics_3d::{
    AngularState3d, AngularVelocity3d, Orientation3d, PhysicsBody3d, RigidBox3d, RigidBoxState3d,
    RigidBoxWorldConfig3d, stabilize_box_box_contact, step_rigid_box_world,
};
use ecs_workload::{EntityId, Position, Velocity};

const SCALE: i32 = 3_600;
const BLOCK_HALF_EXTENTS: [i32; 3] = [3_600, 2_700, 4_320];

fn config() -> RigidBoxWorldConfig3d {
    RigidBoxWorldConfig3d {
        gravity: Velocity::new3(0, -10 * SCALE, 0),
        timestep_numerator: 1,
        timestep_denominator: 60,
        angular_damping_milli: 996,
        solver_passes: 10,
    }
}

fn fixed_floor() -> RigidBox3d {
    RigidBox3d::new(
        PhysicsBody3d::fixed(EntityId(0), [30 * SCALE, SCALE, 12 * SCALE])
            .with_material(PhysicsMaterial::new(50, 850)),
        RigidBoxState3d::new(
            Position::new3(0, -i64::from(SCALE), 0),
            Velocity::new3(0, 0, 0),
            AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
        ),
    )
}

fn tower_block(entity: u32, layer: usize, row: usize, column: usize) -> RigidBox3d {
    let layer_x = i64::try_from(layer).expect("small fixture") * 2 * i64::from(SCALE);
    let row_y = 2_700 + i64::try_from(row).expect("small fixture") * 5_400;
    let column_z = (i64::try_from(column).expect("small fixture") - 1) * 8_640;
    RigidBox3d::new(
        PhysicsBody3d::dynamic(EntityId(entity), BLOCK_HALF_EXTENTS)
            .with_mass(2)
            .with_material(PhysicsMaterial::new(80, 760)),
        RigidBoxState3d::new(
            Position::new3(layer_x, row_y, column_z),
            Velocity::new3(0, 0, 0),
            AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
        ),
    )
}

fn scene(layers: usize, rows: usize, columns: usize) -> Vec<RigidBox3d> {
    let mut boxes = vec![fixed_floor()];
    let mut entity = 1_u32;
    for layer in 0..layers {
        for row in 0..rows {
            for column in 0..columns {
                boxes.push(tower_block(entity, layer, row, column));
                entity += 1;
            }
        }
    }
    boxes
}

fn spinning_after_one_step(boxes: &[RigidBox3d]) -> Vec<(u32, AngularVelocity3d)> {
    step_rigid_box_world(boxes, config())
        .expect("valid resting-contact fixture")
        .boxes
        .into_iter()
        .filter(|body| !body.state.angular.angular_velocity.is_zero())
        .map(|body| (body.body.entity.0, body.state.angular.angular_velocity))
        .collect()
}

fn integrate_gravity_for_fixture(boxes: &mut [RigidBox3d]) {
    for body in boxes.iter_mut().skip(1) {
        body.state.linear_velocity = Velocity::new3(0, -600, 0);
        body.state.center.y -= 10;
    }
}

fn resolve_phase(boxes: &mut [RigidBox3d], fixed_boundary: bool) -> Option<(u32, u32)> {
    for left_index in 0..boxes.len() {
        for right_index in (left_index + 1)..boxes.len() {
            let includes_floor = left_index == 0;
            if includes_floor != fixed_boundary {
                continue;
            }
            let (left_slice, right_slice) = boxes.split_at_mut(right_index);
            let left = &mut left_slice[left_index];
            let right = &mut right_slice[0];
            let before_left = left.state.angular.angular_velocity;
            let before_right = right.state.angular.angular_velocity;
            let resolved =
                stabilize_box_box_contact(left.state, left.body, right.state, right.body)
                    .expect("valid diagnostic contact pair");
            left.state = resolved.left;
            right.state = resolved.right;
            if left.state.angular.angular_velocity != before_left
                || right.state.angular.angular_velocity != before_right
            {
                return Some((left.body.entity.0, right.body.entity.0));
            }
        }
    }
    None
}

#[test]
fn one_block_on_wide_floor_stays_angularly_quiet() {
    let boxes = vec![fixed_floor(), tower_block(1, 0, 0, 0)];
    let spinning = spinning_after_one_step(&boxes);
    assert!(spinning.is_empty(), "unexpected spin: {spinning:?}");
}

#[test]
fn one_vertical_column_stays_angularly_quiet() {
    let mut boxes = vec![fixed_floor()];
    for row in 0..5 {
        boxes.push(tower_block(
            u32::try_from(row + 1).expect("small fixture"),
            0,
            row,
            0,
        ));
    }
    let spinning = spinning_after_one_step(&boxes);
    assert!(spinning.is_empty(), "unexpected spin: {spinning:?}");
}

#[test]
fn one_layer_grid_stays_angularly_quiet() {
    let boxes = scene(1, 5, 3);
    let spinning = spinning_after_one_step(&boxes);
    assert!(spinning.is_empty(), "unexpected spin: {spinning:?}");
}

#[test]
fn two_layer_grid_stays_angularly_quiet() {
    let boxes = scene(2, 5, 3);
    let spinning = spinning_after_one_step(&boxes);
    assert!(spinning.is_empty(), "unexpected spin: {spinning:?}");
}

#[test]
fn resting_grid_does_not_create_a_spin_contact_after_floor_support() {
    let mut boxes = scene(1, 5, 3);
    integrate_gravity_for_fixture(&mut boxes);
    assert_eq!(resolve_phase(&mut boxes, false), None);
    assert_eq!(resolve_phase(&mut boxes, true), None);
    let first_spin_pair = resolve_phase(&mut boxes, false);
    assert_eq!(
        first_spin_pair, None,
        "first spin pair: {first_spin_pair:?}"
    );
}
