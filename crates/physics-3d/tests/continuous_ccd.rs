use ecs_physics_3d::{ContactNormal3d, PhysicsBody3d, PhysicsConfig3d, step_3d};
use ecs_workload::{EntityId, EntitySnapshot, Operation, Position, Velocity, WorldSnapshot};

const NO_GRAVITY: PhysicsConfig3d = PhysicsConfig3d {
    gravity: Velocity::new3(0, 0, 0),
};

#[test]
fn fast_dynamic_bodies_cannot_tunnel_along_z() {
    let near = EntityId(1);
    let far = EntityId(2);
    let snapshot = WorldSnapshot::new(vec![
        EntitySnapshot {
            id: near,
            position: Some(Position::new3(0, 0, -10)),
            velocity: Some(Velocity::new3(0, 0, 30)),
        },
        EntitySnapshot {
            id: far,
            position: Some(Position::new3(0, 0, 10)),
            velocity: Some(Velocity::new3(0, 0, -30)),
        },
    ]);
    let physics = step_3d(
        &snapshot,
        &[
            PhysicsBody3d::dynamic(near, [1, 1, 1]),
            PhysicsBody3d::dynamic(far, [1, 1, 1]),
        ],
        NO_GRAVITY,
        1,
    )
    .expect("relative-motion CCD should work identically along Z");

    assert_eq!(physics.stats().ccd_contacts, 1);
    assert_eq!(
        physics.contacts()[0].normal,
        ContactNormal3d { x: 0, y: 0, z: 1 }
    );
    assert!(
        physics
            .operations()
            .contains(&Operation::SetPosition(near, Position::new3(0, 0, -1)))
    );
    assert!(
        physics
            .operations()
            .contains(&Operation::SetPosition(far, Position::new3(0, 0, 1)))
    );
}

#[test]
fn equal_axis_toi_uses_xyz_order() {
    let left = EntityId(1);
    let right = EntityId(2);
    let snapshot = WorldSnapshot::new(vec![
        EntitySnapshot {
            id: left,
            position: Some(Position::new3(-10, -10, 0)),
            velocity: Some(Velocity::new3(30, 30, 0)),
        },
        EntitySnapshot {
            id: right,
            position: Some(Position::new3(10, 10, 0)),
            velocity: Some(Velocity::new3(-30, -30, 0)),
        },
    ]);
    let physics = step_3d(
        &snapshot,
        &[
            PhysicsBody3d::dynamic(left, [1, 1, 1]),
            PhysicsBody3d::dynamic(right, [1, 1, 1]),
        ],
        NO_GRAVITY,
        1,
    )
    .expect("equal-axis TOIs should remain deterministic");

    assert!(!physics.contacts().is_empty());
    assert_eq!(
        physics.contacts()[0].normal,
        ContactNormal3d { x: 1, y: 0, z: 0 }
    );
}

#[test]
fn wall_event_can_precede_later_dynamic_event() {
    let mover = EntityId(1);
    let follower = EntityId(2);
    let wall = EntityId(3);
    let snapshot = WorldSnapshot::new(vec![
        EntitySnapshot {
            id: mover,
            position: Some(Position::new3(0, 0, 0)),
            velocity: Some(Velocity::new3(30, 0, 0)),
        },
        EntitySnapshot {
            id: follower,
            position: Some(Position::new3(-10, 0, 0)),
            velocity: Some(Velocity::new3(30, 0, 0)),
        },
        EntitySnapshot {
            id: wall,
            position: Some(Position::new3(5, 0, 0)),
            velocity: None,
        },
    ]);
    let physics = step_3d(
        &snapshot,
        &[
            PhysicsBody3d::dynamic(mover, [1, 1, 1]),
            PhysicsBody3d::dynamic(follower, [1, 1, 1]),
            PhysicsBody3d::fixed(wall, [1, 10, 10]),
        ],
        NO_GRAVITY,
        1,
    )
    .expect("fixed and dynamic events should compete on one timeline");

    assert!(physics.contacts().len() >= 2);
    assert_eq!(
        (physics.contacts()[0].left, physics.contacts()[0].right),
        (mover, wall)
    );
    assert_eq!(
        (physics.contacts()[1].left, physics.contacts()[1].right),
        (mover, follower)
    );
}
