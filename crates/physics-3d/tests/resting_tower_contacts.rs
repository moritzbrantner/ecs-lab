use ecs_physics::PhysicsMaterial;
use ecs_physics_3d::{
    AngularState3d, AngularVelocity3d, Orientation3d, PhysicsBody3d, RigidBox3d, RigidBoxState3d,
    RigidBoxWorldConfig3d, step_rigid_box_world,
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

fn spinning_after_one_step(boxes: Vec<RigidBox3d>) -> Vec<(u32, AngularVelocity3d)> {
    step_rigid_box_world(&boxes, config())
        .expect("valid resting-contact fixture")
        .boxes
        .into_iter()
        .filter(|body| !body.state.angular.angular_velocity.is_zero())
        .map(|body| (body.body.entity.0, body.state.angular.angular_velocity))
        .collect()
}

#[test]
fn one_block_on_wide_floor_stays_angularly_quiet() {
    let spinning = spinning_after_one_step(vec![fixed_floor(), tower_block(1, 0, 0, 0)]);
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
    let spinning = spinning_after_one_step(boxes);
    assert!(spinning.is_empty(), "unexpected spin: {spinning:?}");
}

#[test]
fn one_layer_grid_stays_angularly_quiet() {
    let spinning = spinning_after_one_step(scene(1, 5, 3));
    assert!(spinning.is_empty(), "unexpected spin: {spinning:?}");
}

#[test]
fn two_layer_grid_stays_angularly_quiet() {
    let spinning = spinning_after_one_step(scene(2, 5, 3));
    assert!(spinning.is_empty(), "unexpected spin: {spinning:?}");
}
