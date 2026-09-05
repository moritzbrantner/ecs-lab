use ecs_physics_scenarios::{BouncingRoomScenario, FallingBoxesScenario};
use ecs_reference::ReferenceWorld;
use ecs_sparse_set::SparseWorld;
use ecs_workload::Operation;

#[test]
fn bouncing_room_keeps_reference_and_sparse_storage_in_lockstep_long_term() {
    let scenario = BouncingRoomScenario::new();
    let mut reference = ReferenceWorld::new();
    let mut sparse = SparseWorld::new();
    reference
        .replay(scenario.setup())
        .expect("bouncing-room setup must replay in reference storage");
    sparse
        .replay(scenario.setup())
        .expect("bouncing-room setup must replay in sparse storage");

    for frame in 0..512 {
        assert_eq!(
            sparse.snapshot(),
            reference.snapshot(),
            "storage snapshots diverged before frame {frame}"
        );
        let reference_step = scenario
            .step(&reference.snapshot())
            .expect("reference bouncing-room step must succeed");
        let sparse_step = scenario
            .step(&sparse.snapshot())
            .expect("sparse bouncing-room step must succeed");
        assert_eq!(
            sparse_step, reference_step,
            "physics evidence diverged at frame {frame}"
        );

        for operation in reference_step.operations() {
            assert!(
                matches!(
                    operation,
                    Operation::SetPosition(_, _) | Operation::SetVelocity(_, _)
                ),
                "scenario physics escaped the ordinary ECS mutation boundary: {operation:?}"
            );
            reference
                .apply(*operation)
                .expect("reference storage must accept physics operation");
            sparse
                .apply(*operation)
                .expect("sparse storage must accept physics operation");
        }
    }

    assert_eq!(sparse.snapshot(), reference.snapshot());
}

#[test]
fn falling_boxes_keeps_storage_parity_across_sizes_and_horizons() {
    for dynamic_count in [1_u32, 2, 16, 48, 96] {
        let scenario = FallingBoxesScenario::new(dynamic_count);
        let mut reference = ReferenceWorld::new();
        let mut sparse = SparseWorld::new();
        reference
            .replay(scenario.setup())
            .expect("falling-box setup must replay in reference storage");
        sparse
            .replay(scenario.setup())
            .expect("falling-box setup must replay in sparse storage");

        for frame in 0..24 {
            assert_eq!(
                sparse.snapshot(),
                reference.snapshot(),
                "storage snapshots diverged for {dynamic_count} boxes before frame {frame}"
            );
            let reference_step = scenario
                .step(&reference.snapshot())
                .expect("reference falling-box step must succeed");
            let sparse_step = scenario
                .step(&sparse.snapshot())
                .expect("sparse falling-box step must succeed");
            assert_eq!(
                sparse_step, reference_step,
                "physics evidence diverged for {dynamic_count} boxes at frame {frame}"
            );

            for operation in reference_step.operations() {
                reference
                    .apply(*operation)
                    .expect("reference storage must accept physics operation");
                sparse
                    .apply(*operation)
                    .expect("sparse storage must accept physics operation");
            }
        }

        assert_eq!(
            sparse.snapshot(),
            reference.snapshot(),
            "final storage snapshots diverged for {dynamic_count} boxes"
        );
    }
}

#[test]
fn named_scenarios_replay_deterministically_at_multiple_horizons() {
    let bouncing = BouncingRoomScenario::new();
    for frames in [0_u32, 1, 3, 16, 64, 256] {
        let first = bouncing
            .reference_after(frames)
            .expect("first bouncing-room replay must succeed");
        let second = bouncing
            .reference_after(frames)
            .expect("second bouncing-room replay must succeed");
        assert_eq!(first, second, "bouncing-room replay diverged at {frames} frames");
    }

    let falling = FallingBoxesScenario::new(32);
    for frames in [0_u32, 1, 4, 12, 32] {
        let first = falling
            .reference_after(frames)
            .expect("first falling-box replay must succeed");
        let second = falling
            .reference_after(frames)
            .expect("second falling-box replay must succeed");
        assert_eq!(first, second, "falling-box replay diverged at {frames} frames");
    }
}

#[test]
fn scenario_step_evidence_is_repeatable_without_mutating_the_snapshot() {
    let scenario = BouncingRoomScenario::new();
    let mut reference = ReferenceWorld::new();
    reference
        .replay(scenario.setup())
        .expect("bouncing-room setup must replay");
    let snapshot = reference.snapshot();
    let expected = scenario
        .step(&snapshot)
        .expect("baseline bouncing-room step must succeed");

    for repetition in 0..256 {
        assert_eq!(
            scenario.step(&snapshot),
            Ok(expected.clone()),
            "scenario step changed identical input on repetition {repetition}"
        );
    }
    assert_eq!(
        reference.snapshot(),
        snapshot,
        "physics evidence generation must not mutate ECS storage"
    );
}
