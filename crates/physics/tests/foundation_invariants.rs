use ecs_physics::{
    MATERIAL_SCALE, PhysicsBody, PhysicsConfig, PhysicsError, PhysicsMaterial, PhysicsStep, step,
};
use ecs_workload::{EntityId, EntitySnapshot, Operation, Position, Velocity, WorldSnapshot};

const NO_GRAVITY: PhysicsConfig = PhysicsConfig {
    gravity: Velocity::new(0, 0),
};
const MAX_EXACT_F32_INTEGER: i64 = 16_777_216;

fn resulting_velocity(original: Velocity, operations: &[Operation], entity: EntityId) -> Velocity {
    operations
        .iter()
        .fold(original, |current, operation| match operation {
            Operation::SetVelocity(id, velocity) if *id == entity => *velocity,
            _ => current,
        })
}

fn assert_only_ordered_component_writes(step: &PhysicsStep) {
    let mut previous = None;
    for operation in step.operations() {
        let entity = match operation {
            Operation::SetPosition(entity, _) | Operation::SetVelocity(entity, _) => *entity,
            other => panic!("physics emitted a non-component-write ECS operation: {other:?}"),
        };
        if let Some(previous) = previous {
            assert!(
                previous <= entity,
                "physics operations must remain entity ordered"
            );
        }
        previous = Some(entity);
    }
}

#[test]
fn material_matrix_is_body_and_snapshot_order_invariant() {
    let left = EntityId(11);
    let right = EntityId(29);
    let left_velocity = Velocity::new(2, 3);
    let right_velocity = Velocity::new(-2, -1);
    let left_snapshot = EntitySnapshot {
        id: left,
        position: Some(Position::new(-3, -3)),
        velocity: Some(left_velocity),
    };
    let right_snapshot = EntitySnapshot {
        id: right,
        position: Some(Position::new(3, 1)),
        velocity: Some(right_velocity),
    };

    for left_mass in [1, 2, 5] {
        for right_mass in [1, 3, 7] {
            for left_restitution in [0, 333, MATERIAL_SCALE] {
                for right_restitution in [0, 500, MATERIAL_SCALE] {
                    for left_friction in [0, 400, MATERIAL_SCALE] {
                        for right_friction in [0, 700, MATERIAL_SCALE] {
                            let left_body = PhysicsBody::dynamic(left, [1, 1])
                                .with_mass(left_mass)
                                .with_material(PhysicsMaterial::new(
                                    left_restitution,
                                    left_friction,
                                ));
                            let right_body = PhysicsBody::dynamic(right, [1, 1])
                                .with_mass(right_mass)
                                .with_material(PhysicsMaterial::new(
                                    right_restitution,
                                    right_friction,
                                ));

                            let forward = step(
                                &WorldSnapshot::new(vec![left_snapshot, right_snapshot]),
                                &[left_body, right_body],
                                NO_GRAVITY,
                                1,
                            );
                            let reversed = step(
                                &WorldSnapshot::new(vec![right_snapshot, left_snapshot]),
                                &[right_body, left_body],
                                NO_GRAVITY,
                                1,
                            );

                            assert_eq!(
                                forward, reversed,
                                "order changed material response for masses {left_mass}/{right_mass}, restitution {left_restitution}/{right_restitution}, friction {left_friction}/{right_friction}"
                            );
                        }
                    }
                }
            }
        }
    }
}

#[test]
fn dynamic_collision_rounding_keeps_momentum_error_bounded() {
    let left = EntityId(1);
    let right = EntityId(2);

    for left_mass in [1_u32, 2, 3, 7] {
        for right_mass in [1_u32, 2, 5, 8] {
            for restitution in [0_u16, 250, 500, 750, MATERIAL_SCALE] {
                for left_velocity_x in [1_i32, 3, 7, 11] {
                    for right_velocity_x in [-3_i32, -1, 0, 2] {
                        if left_velocity_x <= right_velocity_x {
                            continue;
                        }
                        let left_velocity = Velocity::new(left_velocity_x, 0);
                        let right_velocity = Velocity::new(right_velocity_x, 0);
                        let snapshot = WorldSnapshot::new(vec![
                            EntitySnapshot {
                                id: left,
                                position: Some(Position::new(-1 - i64::from(left_velocity_x), 0)),
                                velocity: Some(left_velocity),
                            },
                            EntitySnapshot {
                                id: right,
                                position: Some(Position::new(1 - i64::from(right_velocity_x), 0)),
                                velocity: Some(right_velocity),
                            },
                        ]);
                        let material = PhysicsMaterial::new(restitution, 0);
                        let physics = step(
                            &snapshot,
                            &[
                                PhysicsBody::dynamic(left, [1, 1])
                                    .with_mass(left_mass)
                                    .with_material(material),
                                PhysicsBody::dynamic(right, [1, 1])
                                    .with_mass(right_mass)
                                    .with_material(material),
                            ],
                            NO_GRAVITY,
                            1,
                        )
                        .expect("valid dynamic collision should succeed");

                        let left_after =
                            resulting_velocity(left_velocity, physics.operations(), left).x;
                        let right_after =
                            resulting_velocity(right_velocity, physics.operations(), right).x;
                        let momentum_before = i64::from(left_mass) * i64::from(left_velocity_x)
                            + i64::from(right_mass) * i64::from(right_velocity_x);
                        let momentum_after = i64::from(left_mass) * i64::from(left_after)
                            + i64::from(right_mass) * i64::from(right_after);
                        let rounding_bound = i64::from(left_mass) + i64::from(right_mass);

                        assert!(
                            (momentum_after - momentum_before).abs() <= rounding_bound,
                            "integer response exceeded its rounding bound: masses {left_mass}/{right_mass}, restitution {restitution}, velocities {left_velocity_x}/{right_velocity_x}, before {momentum_before}, after {momentum_after}"
                        );

                        let closing_speed = i64::from(left_velocity_x - right_velocity_x);
                        let separating_speed = i64::from(right_after) - i64::from(left_after);
                        assert!(
                            separating_speed <= closing_speed + 2,
                            "restitution amplified relative speed beyond integer rounding tolerance"
                        );
                        assert!(
                            separating_speed >= -2,
                            "collision remained materially closing after response"
                        );
                    }
                }
            }
        }
    }
}

#[test]
fn friction_never_materially_increases_tangent_relative_speed() {
    let left = EntityId(3);
    let right = EntityId(4);

    for left_mass in [1_u32, 2, 5] {
        for right_mass in [1_u32, 3, 7] {
            for friction in [0_u16, 250, 500, 750, MATERIAL_SCALE] {
                for left_tangent in [-9_i32, -3, 0, 4, 11] {
                    for right_tangent in [-8_i32, -1, 2, 7] {
                        let left_velocity = Velocity::new(2, left_tangent);
                        let right_velocity = Velocity::new(-2, right_tangent);
                        let snapshot = WorldSnapshot::new(vec![
                            EntitySnapshot {
                                id: left,
                                position: Some(Position::new(-3, -i64::from(left_tangent))),
                                velocity: Some(left_velocity),
                            },
                            EntitySnapshot {
                                id: right,
                                position: Some(Position::new(3, -i64::from(right_tangent))),
                                velocity: Some(right_velocity),
                            },
                        ]);
                        let material = PhysicsMaterial::new(0, friction);
                        let physics = step(
                            &snapshot,
                            &[
                                PhysicsBody::dynamic(left, [1, 1])
                                    .with_mass(left_mass)
                                    .with_material(material),
                                PhysicsBody::dynamic(right, [1, 1])
                                    .with_mass(right_mass)
                                    .with_material(material),
                            ],
                            NO_GRAVITY,
                            1,
                        )
                        .expect("valid friction contact should succeed");

                        let left_after =
                            resulting_velocity(left_velocity, physics.operations(), left).y;
                        let right_after =
                            resulting_velocity(right_velocity, physics.operations(), right).y;
                        let before_delta = i64::from(left_tangent - right_tangent).abs();
                        let after_delta = i64::from(left_after - right_after).abs();
                        assert!(
                            after_delta <= before_delta + 2,
                            "friction increased tangent relative speed beyond rounding tolerance"
                        );
                        if friction == MATERIAL_SCALE {
                            assert_eq!(
                                left_after, right_after,
                                "maximum dynamic friction must collapse tangent relative velocity"
                            );
                        }
                    }
                }
            }
        }
    }
}

#[test]
fn full_restitution_against_fixed_surface_preserves_normal_speed() {
    let dynamic = EntityId(7);
    let floor = EntityId(8);
    let material = PhysicsMaterial::new(MATERIAL_SCALE, 0);

    for speed in 1_i32..=32 {
        let velocity = Velocity::new(0, -speed);
        let snapshot = WorldSnapshot::new(vec![
            EntitySnapshot {
                id: dynamic,
                position: Some(Position::new(0, 2 + i64::from(speed))),
                velocity: Some(velocity),
            },
            EntitySnapshot {
                id: floor,
                position: Some(Position::new(0, 0)),
                velocity: None,
            },
        ]);
        let physics = step(
            &snapshot,
            &[
                PhysicsBody::dynamic(dynamic, [1, 1]).with_material(material),
                PhysicsBody::fixed(floor, [64, 1]),
            ],
            NO_GRAVITY,
            1,
        )
        .expect("fixed-surface restitution fixture should succeed");

        assert_eq!(
            resulting_velocity(velocity, physics.operations(), dynamic).y,
            speed
        );
        assert!(physics.is_supported(dynamic));
    }
}

#[test]
fn maximum_fixed_surface_friction_zeroes_tangent_velocity() {
    let dynamic = EntityId(9);
    let floor = EntityId(10);
    let material = PhysicsMaterial::new(0, MATERIAL_SCALE);

    for tangent in -32_i32..=32 {
        let velocity = Velocity::new(tangent, -2);
        let snapshot = WorldSnapshot::new(vec![
            EntitySnapshot {
                id: dynamic,
                position: Some(Position::new(0, 4)),
                velocity: Some(velocity),
            },
            EntitySnapshot {
                id: floor,
                position: Some(Position::new(0, 0)),
                velocity: None,
            },
        ]);
        let physics = step(
            &snapshot,
            &[
                PhysicsBody::dynamic(dynamic, [1, 1]).with_material(material),
                PhysicsBody::fixed(floor, [128, 1]),
            ],
            NO_GRAVITY,
            1,
        )
        .expect("fixed-surface friction fixture should succeed");
        let after = resulting_velocity(velocity, physics.operations(), dynamic);

        assert_eq!(after.x, 0, "maximum friction must remove tangent motion");
        assert_eq!(after.y, 0, "zero restitution must remove normal motion");
    }
}

#[test]
fn fixed_bodies_never_receive_generated_mutations() {
    let floor = EntityId(1);
    let dynamic = EntityId(2);
    let snapshot = WorldSnapshot::new(vec![
        EntitySnapshot {
            id: floor,
            position: Some(Position::new(0, 0)),
            velocity: Some(Velocity::new(99, 99)),
        },
        EntitySnapshot {
            id: dynamic,
            position: Some(Position::new(0, 4)),
            velocity: Some(Velocity::new(3, -2)),
        },
    ]);
    let physics = step(
        &snapshot,
        &[
            PhysicsBody::fixed(floor, [16, 1]),
            PhysicsBody::dynamic(dynamic, [1, 1]).with_material(PhysicsMaterial::new(500, 500)),
        ],
        NO_GRAVITY,
        1,
    )
    .expect("fixed-body fixture should succeed");

    for operation in physics.operations() {
        let entity = match operation {
            Operation::SetPosition(entity, _) | Operation::SetVelocity(entity, _) => *entity,
            other => panic!("unexpected physics operation: {other:?}"),
        };
        assert_eq!(entity, dynamic, "fixed entities must remain immutable");
    }
}

#[test]
fn support_evidence_is_sorted_unique_across_multiple_floor_contacts() {
    let left_floor = EntityId(1);
    let right_floor = EntityId(2);
    let dynamic = EntityId(9);
    let snapshot = WorldSnapshot::new(vec![
        EntitySnapshot {
            id: dynamic,
            position: Some(Position::new(0, 2)),
            velocity: Some(Velocity::new(0, 0)),
        },
        EntitySnapshot {
            id: right_floor,
            position: Some(Position::new(2, 0)),
            velocity: None,
        },
        EntitySnapshot {
            id: left_floor,
            position: Some(Position::new(-2, 0)),
            velocity: None,
        },
    ]);
    let physics = step(
        &snapshot,
        &[
            PhysicsBody::dynamic(dynamic, [3, 1]),
            PhysicsBody::fixed(right_floor, [1, 1]),
            PhysicsBody::fixed(left_floor, [1, 1]),
        ],
        NO_GRAVITY,
        1,
    )
    .expect("multi-support fixture should succeed");

    assert!(physics.operations().is_empty());
    assert_eq!(physics.supporting_entities(), [dynamic]);
    assert_eq!(
        physics
            .contacts()
            .iter()
            .filter(|contact| contact.left == dynamic || contact.right == dynamic)
            .count(),
        2
    );
}

#[test]
fn generated_operations_stay_within_the_ordinary_ecs_mutation_boundary() {
    let entities = [EntityId(30), EntityId(10), EntityId(20)];
    let snapshot = WorldSnapshot::new(vec![
        EntitySnapshot {
            id: entities[0],
            position: Some(Position::new(30, 0)),
            velocity: Some(Velocity::new(1, 1)),
        },
        EntitySnapshot {
            id: entities[1],
            position: Some(Position::new(-30, 0)),
            velocity: Some(Velocity::new(2, -1)),
        },
        EntitySnapshot {
            id: entities[2],
            position: Some(Position::new(0, 30)),
            velocity: Some(Velocity::new(-1, 2)),
        },
    ]);
    let bodies = [
        PhysicsBody::dynamic(entities[0], [1, 1]),
        PhysicsBody::dynamic(entities[1], [1, 1]),
        PhysicsBody::dynamic(entities[2], [1, 1]),
    ];
    let physics = step(&snapshot, &bodies, NO_GRAVITY, 1)
        .expect("ordinary mutation-boundary fixture should succeed");

    assert_only_ordered_component_writes(&physics);
    assert_eq!(physics.operations().len(), 3);
}

#[test]
fn all_three_body_input_permutations_produce_the_same_step() {
    let floor = EntityId(5);
    let left = EntityId(10);
    let right = EntityId(20);
    let snapshot = WorldSnapshot::new(vec![
        EntitySnapshot {
            id: right,
            position: Some(Position::new(2, 4)),
            velocity: Some(Velocity::new(-1, -2)),
        },
        EntitySnapshot {
            id: floor,
            position: Some(Position::new(0, 0)),
            velocity: None,
        },
        EntitySnapshot {
            id: left,
            position: Some(Position::new(-2, 4)),
            velocity: Some(Velocity::new(1, -2)),
        },
    ]);
    let floor_body = PhysicsBody::fixed(floor, [10, 1]);
    let left_body = PhysicsBody::dynamic(left, [1, 1])
        .with_mass(2)
        .with_material(PhysicsMaterial::new(750, 250));
    let right_body = PhysicsBody::dynamic(right, [1, 1])
        .with_mass(5)
        .with_material(PhysicsMaterial::new(250, 750));
    let permutations = [
        [floor_body, left_body, right_body],
        [floor_body, right_body, left_body],
        [left_body, floor_body, right_body],
        [left_body, right_body, floor_body],
        [right_body, floor_body, left_body],
        [right_body, left_body, floor_body],
    ];
    let expected = step(&snapshot, &permutations[0], NO_GRAVITY, 1)
        .expect("three-body baseline should succeed");

    for bodies in &permutations[1..] {
        assert_eq!(
            step(&snapshot, bodies, NO_GRAVITY, 1),
            Ok(expected.clone()),
            "body permutation changed deterministic response"
        );
    }
}

#[test]
fn geometry_range_crossing_fails_closed_before_collision_evidence() {
    let entity = EntityId(77);
    let snapshot = WorldSnapshot::new(vec![EntitySnapshot {
        id: entity,
        position: Some(Position::new(MAX_EXACT_F32_INTEGER - 1, 0)),
        velocity: Some(Velocity::new(1, 0)),
    }]);

    assert_eq!(
        step(
            &snapshot,
            &[PhysicsBody::dynamic(entity, [1, 1])],
            NO_GRAVITY,
            1,
        ),
        Err(PhysicsError::CoordinateOutOfRange(entity))
    );
}

#[test]
fn repeated_steps_from_identical_input_are_bit_for_bit_deterministic() {
    let floor = EntityId(1);
    let left = EntityId(2);
    let right = EntityId(3);
    let snapshot = WorldSnapshot::new(vec![
        EntitySnapshot {
            id: floor,
            position: Some(Position::new(0, 0)),
            velocity: None,
        },
        EntitySnapshot {
            id: left,
            position: Some(Position::new(-2, 4)),
            velocity: Some(Velocity::new(1, -2)),
        },
        EntitySnapshot {
            id: right,
            position: Some(Position::new(2, 4)),
            velocity: Some(Velocity::new(-1, -2)),
        },
    ]);
    let bodies = [
        PhysicsBody::fixed(floor, [12, 1]),
        PhysicsBody::dynamic(left, [1, 1])
            .with_mass(3)
            .with_material(PhysicsMaterial::new(1_000, 250)),
        PhysicsBody::dynamic(right, [1, 1])
            .with_mass(7)
            .with_material(PhysicsMaterial::new(500, 1_000)),
    ];
    let expected =
        step(&snapshot, &bodies, NO_GRAVITY, 1).expect("determinism baseline should succeed");

    for repetition in 0..256 {
        assert_eq!(
            step(&snapshot, &bodies, NO_GRAVITY, 1),
            Ok(expected.clone()),
            "identical input diverged on repetition {repetition}"
        );
    }
}
