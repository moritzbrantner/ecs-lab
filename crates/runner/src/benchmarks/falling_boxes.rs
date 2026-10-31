use ecs_archetype::ArchetypeWorld;
use ecs_cached_sparse::CachedSparseWorld;
use ecs_physics_scenarios::FallingBoxesScenario;
use ecs_reference::ReferenceWorld;
use ecs_sparse_set::SparseWorld;
use ecs_workload::WorldSnapshot;

use super::{benchmark, must};

const FALLING_BOX_SEED: u32 = 0;

pub(super) fn run_falling_box_benchmarks(smoke: bool, fingerprint: &str) {
    let (dynamic_count, frames, repetitions) = if smoke { (96, 12, 2) } else { (512, 40, 3) };
    let scenario = FallingBoxesScenario::new(dynamic_count);
    let body_count = dynamic_count.saturating_add(1);
    let reference_expected = reference_falling_box_snapshot(&scenario, frames);
    let sparse_expected = sparse_falling_box_snapshot(&scenario, frames);
    let cached_expected = cached_sparse_falling_box_snapshot(&scenario, frames);
    let archetype_expected = archetype_falling_box_snapshot(&scenario, frames);
    assert_eq!(
        sparse_expected, reference_expected,
        "falling-box benchmark fixture must prove sparse/reference parity before timing"
    );
    assert_eq!(
        cached_expected, reference_expected,
        "falling-box benchmark fixture must prove cached-sparse/reference parity before timing"
    );
    assert_eq!(
        archetype_expected, reference_expected,
        "falling-box benchmark fixture must prove archetype/reference parity before timing"
    );

    benchmark(
        "falling-boxes",
        "reference",
        body_count,
        frames,
        FALLING_BOX_SEED,
        repetitions,
        fingerprint,
        || reference_falling_box_snapshot(&scenario, frames),
    );
    benchmark(
        "falling-boxes",
        "sparse-set",
        body_count,
        frames,
        FALLING_BOX_SEED,
        repetitions,
        fingerprint,
        || sparse_falling_box_snapshot(&scenario, frames),
    );
    benchmark(
        "falling-boxes",
        "cached-sparse",
        body_count,
        frames,
        FALLING_BOX_SEED,
        repetitions,
        fingerprint,
        || cached_sparse_falling_box_snapshot(&scenario, frames),
    );
    benchmark(
        "falling-boxes",
        "archetype-table",
        body_count,
        frames,
        FALLING_BOX_SEED,
        repetitions,
        fingerprint,
        || archetype_falling_box_snapshot(&scenario, frames),
    );
}

fn reference_falling_box_snapshot(scenario: &FallingBoxesScenario, frames: u32) -> WorldSnapshot {
    let mut world = ReferenceWorld::new();
    must(
        world.replay(scenario.setup()),
        "validated reference falling-box setup must replay",
    );
    for _ in 0..frames {
        let physics = must(
            scenario.step(&world.snapshot()),
            "validated reference falling-box physics step must succeed",
        );
        for operation in physics.operations() {
            must(
                world.apply(*operation),
                "reference storage must accept generated physics operation",
            );
        }
    }
    world.snapshot()
}

fn sparse_falling_box_snapshot(scenario: &FallingBoxesScenario, frames: u32) -> WorldSnapshot {
    let mut world = SparseWorld::new();
    must(
        world.replay(scenario.setup()),
        "validated sparse falling-box setup must replay",
    );
    for _ in 0..frames {
        let physics = must(
            scenario.step(&world.snapshot()),
            "validated sparse falling-box physics step must succeed",
        );
        for operation in physics.operations() {
            must(
                world.apply(*operation),
                "sparse storage must accept generated physics operation",
            );
        }
    }
    world.snapshot()
}

fn cached_sparse_falling_box_snapshot(
    scenario: &FallingBoxesScenario,
    frames: u32,
) -> WorldSnapshot {
    let mut world = CachedSparseWorld::new();
    must(
        world.replay(scenario.setup()),
        "validated cached-sparse falling-box setup must replay",
    );
    for _ in 0..frames {
        let physics = must(
            scenario.step(&world.snapshot()),
            "validated cached-sparse falling-box physics step must succeed",
        );
        for operation in physics.operations() {
            must(
                world.apply(*operation),
                "cached-sparse storage must accept generated physics operation",
            );
        }
    }
    world.snapshot()
}

fn archetype_falling_box_snapshot(scenario: &FallingBoxesScenario, frames: u32) -> WorldSnapshot {
    let mut world = ArchetypeWorld::new();
    must(
        world.replay(scenario.setup()),
        "validated archetype falling-box setup must replay",
    );
    for _ in 0..frames {
        let physics = must(
            scenario.step(&world.snapshot()),
            "validated archetype falling-box physics step must succeed",
        );
        for operation in physics.operations() {
            must(
                world.apply(*operation),
                "archetype storage must accept generated physics operation",
            );
        }
    }
    world.snapshot()
}
