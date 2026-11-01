use ecs_physics::PhysicsMaterial;
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
fn equal_axis_toi_collects_xyz_ordered_contact_set() {
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
    .expect("equal-axis TOIs should resolve as one deterministic contact set");

    assert_eq!(physics.stats().ccd_contacts, 2);
    assert_eq!(physics.contacts().len(), 2);
    assert_eq!(
        physics.contacts()[0].normal,
        ContactNormal3d { x: 1, y: 0, z: 0 }
    );
    assert_eq!(
        physics.contacts()[1].normal,
        ContactNormal3d { x: 0, y: 1, z: 0 }
    );
    assert!(
        physics
            .operations()
            .contains(&Operation::SetVelocity(left, Velocity::new3(0, 0, 0)))
    );
    assert!(
        physics
            .operations()
            .contains(&Operation::SetVelocity(right, Velocity::new3(0, 0, 0)))
    );
}

#[test]
fn independent_equal_time_pairs_are_emitted_in_entity_order() {
    let snapshot = WorldSnapshot::new(vec![
        EntitySnapshot {
            id: EntityId(1),
            position: Some(Position::new3(-10, 0, 0)),
            velocity: Some(Velocity::new3(30, 0, 0)),
        },
        EntitySnapshot {
            id: EntityId(2),
            position: Some(Position::new3(10, 0, 0)),
            velocity: Some(Velocity::new3(-30, 0, 0)),
        },
        EntitySnapshot {
            id: EntityId(3),
            position: Some(Position::new3(-10, 10, 0)),
            velocity: Some(Velocity::new3(30, 0, 0)),
        },
        EntitySnapshot {
            id: EntityId(4),
            position: Some(Position::new3(10, 10, 0)),
            velocity: Some(Velocity::new3(-30, 0, 0)),
        },
    ]);
    let physics = step_3d(
        &snapshot,
        &[
            PhysicsBody3d::dynamic(EntityId(4), [1, 1, 1]),
            PhysicsBody3d::dynamic(EntityId(2), [1, 1, 1]),
            PhysicsBody3d::dynamic(EntityId(3), [1, 1, 1]),
            PhysicsBody3d::dynamic(EntityId(1), [1, 1, 1]),
        ],
        NO_GRAVITY,
        1,
    )
    .expect("independent equal-time contacts should share one ordered set");

    assert_eq!(physics.stats().ccd_contacts, 2);
    assert_eq!(
        (physics.contacts()[0].left, physics.contacts()[0].right),
        (EntityId(1), EntityId(2))
    );
    assert_eq!(
        (physics.contacts()[1].left, physics.contacts()[1].right),
        (EntityId(3), EntityId(4))
    );
}

#[test]
fn perpendicular_walls_resolve_in_one_contact_set() {
    let dynamic = EntityId(1);
    let x_wall = EntityId(2);
    let y_wall = EntityId(3);
    let snapshot = WorldSnapshot::new(vec![
        EntitySnapshot {
            id: dynamic,
            position: Some(Position::new3(0, 0, 0)),
            velocity: Some(Velocity::new3(30, 30, 0)),
        },
        EntitySnapshot {
            id: x_wall,
            position: Some(Position::new3(5, 0, 0)),
            velocity: None,
        },
        EntitySnapshot {
            id: y_wall,
            position: Some(Position::new3(0, 5, 0)),
            velocity: None,
        },
    ]);
    let physics = step_3d(
        &snapshot,
        &[
            PhysicsBody3d::dynamic(dynamic, [1, 1, 1]),
            PhysicsBody3d::fixed(x_wall, [1, 10, 10]),
            PhysicsBody3d::fixed(y_wall, [10, 1, 10]),
        ],
        NO_GRAVITY,
        1,
    )
    .expect("perpendicular equal-time wall contacts should resolve together");

    assert_eq!(physics.stats().ccd_contacts, 2);
    assert_eq!(
        (physics.contacts()[0].left, physics.contacts()[0].right),
        (dynamic, x_wall)
    );
    assert_eq!(
        (physics.contacts()[1].left, physics.contacts()[1].right),
        (dynamic, y_wall)
    );
    assert!(
        physics
            .operations()
            .contains(&Operation::SetPosition(dynamic, Position::new3(3, 3, 0)))
    );
    assert!(
        physics
            .operations()
            .contains(&Operation::SetVelocity(dynamic, Velocity::new3(0, 0, 0)))
    );
}

#[test]
fn three_body_equal_time_chain_converges_deterministically() {
    let left = EntityId(1);
    let middle = EntityId(2);
    let right = EntityId(3);
    let snapshot = WorldSnapshot::new(vec![
        EntitySnapshot {
            id: left,
            position: Some(Position::new3(-10, 0, 0)),
            velocity: Some(Velocity::new3(30, 0, 0)),
        },
        EntitySnapshot {
            id: middle,
            position: Some(Position::new3(0, 0, 0)),
            velocity: Some(Velocity::new3(0, 0, 0)),
        },
        EntitySnapshot {
            id: right,
            position: Some(Position::new3(10, 0, 0)),
            velocity: Some(Velocity::new3(-30, 0, 0)),
        },
    ]);
    let physics = step_3d(
        &snapshot,
        &[
            PhysicsBody3d::dynamic(right, [1, 1, 1]),
            PhysicsBody3d::dynamic(left, [1, 1, 1]),
            PhysicsBody3d::dynamic(middle, [1, 1, 1]),
        ],
        NO_GRAVITY,
        1,
    )
    .expect("a shared-body equal-time chain should converge under the bounded solver");

    assert_eq!(physics.stats().ccd_contacts, 2);
    assert_eq!(
        (physics.contacts()[0].left, physics.contacts()[0].right),
        (left, middle)
    );
    assert_eq!(
        (physics.contacts()[1].left, physics.contacts()[1].right),
        (middle, right)
    );
    assert!(
        physics
            .operations()
            .contains(&Operation::SetVelocity(left, Velocity::new3(0, 0, 0)))
    );
    assert!(
        physics
            .operations()
            .contains(&Operation::SetVelocity(right, Velocity::new3(0, 0, 0)))
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

#[test]
fn elastic_z_axis_pair_swaps_velocities() {
    let left = EntityId(1);
    let right = EntityId(2);
    let snapshot = WorldSnapshot::new(vec![
        EntitySnapshot {
            id: left,
            position: Some(Position::new3(0, 0, -3)),
            velocity: Some(Velocity::new3(0, 0, 2)),
        },
        EntitySnapshot {
            id: right,
            position: Some(Position::new3(0, 0, 3)),
            velocity: Some(Velocity::new3(0, 0, -2)),
        },
    ]);
    let material = PhysicsMaterial::new(1_000, 0);
    let physics = step_3d(
        &snapshot,
        &[
            PhysicsBody3d::dynamic(left, [1, 1, 1]).with_material(material),
            PhysicsBody3d::dynamic(right, [1, 1, 1]).with_material(material),
        ],
        NO_GRAVITY,
        1,
    )
    .expect("valid Z-axis collision should succeed");

    assert_eq!(physics.contacts().len(), 1);
    assert_eq!(
        physics.contacts()[0].normal,
        ContactNormal3d { x: 0, y: 0, z: 1 }
    );
    assert!(
        physics
            .operations()
            .contains(&Operation::SetVelocity(left, Velocity::new3(0, 0, -2)))
    );
    assert!(
        physics
            .operations()
            .contains(&Operation::SetVelocity(right, Velocity::new3(0, 0, 2)))
    );
}

#[test]
fn fast_dynamic_body_cannot_tunnel_through_fixed_wall() {
    let dynamic = EntityId(1);
    let wall = EntityId(2);
    let snapshot = WorldSnapshot::new(vec![
        EntitySnapshot {
            id: dynamic,
            position: Some(Position::new3(-10, 0, 0)),
            velocity: Some(Velocity::new3(30, 0, 0)),
        },
        EntitySnapshot {
            id: wall,
            position: Some(Position::new3(0, 0, 0)),
            velocity: None,
        },
    ]);
    let physics = step_3d(
        &snapshot,
        &[
            PhysicsBody3d::dynamic(dynamic, [1, 1, 1]),
            PhysicsBody3d::fixed(wall, [1, 20, 20]),
        ],
        NO_GRAVITY,
        1,
    )
    .expect("swept wall collision should succeed");

    assert!(
        physics
            .operations()
            .contains(&Operation::SetPosition(dynamic, Position::new3(-2, 0, 0)))
    );
    assert!(
        physics
            .operations()
            .contains(&Operation::SetVelocity(dynamic, Velocity::new3(0, 0, 0)))
    );
    assert_eq!(physics.stats().ccd_contacts, 1);
    assert_eq!(
        physics.contacts()[0].normal,
        ContactNormal3d { x: 1, y: 0, z: 0 }
    );
}

#[test]
fn bouncy_static_toi_consumes_the_remaining_fraction_of_the_step() {
    let dynamic = EntityId(1);
    let wall = EntityId(2);
    let snapshot = WorldSnapshot::new(vec![
        EntitySnapshot {
            id: dynamic,
            position: Some(Position::new3(-10, 0, 0)),
            velocity: Some(Velocity::new3(30, 0, 0)),
        },
        EntitySnapshot {
            id: wall,
            position: Some(Position::new3(0, 0, 0)),
            velocity: None,
        },
    ]);
    let bouncy = PhysicsMaterial::new(1_000, 0);
    let physics = step_3d(
        &snapshot,
        &[
            PhysicsBody3d::dynamic(dynamic, [1, 1, 1]).with_material(bouncy),
            PhysicsBody3d::fixed(wall, [1, 20, 20]),
        ],
        NO_GRAVITY,
        1,
    )
    .expect("bouncy swept wall collision should succeed");

    assert!(
        physics
            .operations()
            .contains(&Operation::SetPosition(dynamic, Position::new3(-24, 0, 0)))
    );
    assert!(
        physics
            .operations()
            .contains(&Operation::SetVelocity(dynamic, Velocity::new3(-30, 0, 0)))
    );
}

#[test]
fn swept_aabb_requires_tangent_overlap_at_time_of_impact() {
    let dynamic = EntityId(1);
    let wall = EntityId(2);
    let snapshot = WorldSnapshot::new(vec![
        EntitySnapshot {
            id: dynamic,
            position: Some(Position::new3(-10, 50, 0)),
            velocity: Some(Velocity::new3(30, 0, 0)),
        },
        EntitySnapshot {
            id: wall,
            position: Some(Position::new3(0, 0, 0)),
            velocity: None,
        },
    ]);
    let physics = step_3d(
        &snapshot,
        &[
            PhysicsBody3d::dynamic(dynamic, [1, 1, 1]),
            PhysicsBody3d::fixed(wall, [1, 5, 5]),
        ],
        NO_GRAVITY,
        1,
    )
    .expect("tangent miss should remain collision-free");

    assert!(
        physics
            .operations()
            .contains(&Operation::SetPosition(dynamic, Position::new3(20, 50, 0)))
    );
    assert_eq!(physics.stats().ccd_contacts, 0);
}

#[test]
fn bounded_toi_loop_handles_multiple_wall_impacts_in_one_step() {
    let dynamic = EntityId(1);
    let left_wall = EntityId(2);
    let right_wall = EntityId(3);
    let snapshot = WorldSnapshot::new(vec![
        EntitySnapshot {
            id: dynamic,
            position: Some(Position::new3(0, 0, 0)),
            velocity: Some(Velocity::new3(100, 0, 0)),
        },
        EntitySnapshot {
            id: left_wall,
            position: Some(Position::new3(-10, 0, 0)),
            velocity: None,
        },
        EntitySnapshot {
            id: right_wall,
            position: Some(Position::new3(10, 0, 0)),
            velocity: None,
        },
    ]);
    let bouncy = PhysicsMaterial::new(1_000, 0);
    let physics = step_3d(
        &snapshot,
        &[
            PhysicsBody3d::dynamic(dynamic, [1, 1, 1]).with_material(bouncy),
            PhysicsBody3d::fixed(left_wall, [1, 20, 20]),
            PhysicsBody3d::fixed(right_wall, [1, 20, 20]),
        ],
        NO_GRAVITY,
        1,
    )
    .expect("multiple swept wall impacts should succeed");

    assert!(
        physics
            .operations()
            .contains(&Operation::SetPosition(dynamic, Position::new3(4, 0, 0)))
    );
    assert!(
        !physics.operations().iter().any(
            |operation| matches!(operation, Operation::SetVelocity(entity, _) if *entity == dynamic)
        ),
        "six full-restitution impacts return to the input velocity, so no velocity write is emitted"
    );
    assert_eq!(physics.stats().ccd_contacts, 6);
}
