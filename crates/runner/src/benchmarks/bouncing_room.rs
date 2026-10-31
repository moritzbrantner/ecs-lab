use ecs_archetype::ArchetypeWorld;
use ecs_cached_sparse::CachedSparseWorld;
use ecs_physics_scenarios::BouncingRoomScenario;
use ecs_reference::ReferenceWorld;
use ecs_sparse_set::SparseWorld;
use ecs_workload::WorldSnapshot;

use super::{benchmark, must};

const BOUNCING_ROOM_SEED: u32 = 0xB00C_E001;

pub(super) fn run_bouncing_room_benchmarks(smoke: bool, fingerprint: &str) {
    let (frames, repetitions) = if smoke { (128, 3) } else { (4_096, 5) };
    let scenario = BouncingRoomScenario::new();
    let body_count = must(
        u32::try_from(scenario.bodies().len()),
        "scenario body count must fit u32",
    );
    let reference_expected = reference_bouncing_room_snapshot(&scenario, frames);
    let sparse_expected = sparse_bouncing_room_snapshot(&scenario, frames);
    let cached_expected = cached_sparse_bouncing_room_snapshot(&scenario, frames);
    let archetype_expected = archetype_bouncing_room_snapshot(&scenario, frames);
    assert_eq!(
        sparse_expected, reference_expected,
        "bouncing-room benchmark fixture must prove sparse/reference parity before timing"
    );
    assert_eq!(
        cached_expected, reference_expected,
        "bouncing-room benchmark fixture must prove cached-sparse/reference parity before timing"
    );
    assert_eq!(
        archetype_expected, reference_expected,
        "bouncing-room benchmark fixture must prove archetype/reference parity before timing"
    );

    benchmark(
        "bouncing-room",
        "reference",
        body_count,
        frames,
        BOUNCING_ROOM_SEED,
        repetitions,
        fingerprint,
        || reference_bouncing_room_snapshot(&scenario, frames),
    );
    benchmark(
        "bouncing-room",
        "sparse-set",
        body_count,
        frames,
        BOUNCING_ROOM_SEED,
        repetitions,
        fingerprint,
        || sparse_bouncing_room_snapshot(&scenario, frames),
    );
    benchmark(
        "bouncing-room",
        "cached-sparse",
        body_count,
        frames,
        BOUNCING_ROOM_SEED,
        repetitions,
        fingerprint,
        || cached_sparse_bouncing_room_snapshot(&scenario, frames),
    );
    benchmark(
        "bouncing-room",
        "archetype-table",
        body_count,
        frames,
        BOUNCING_ROOM_SEED,
        repetitions,
        fingerprint,
        || archetype_bouncing_room_snapshot(&scenario, frames),
    );
}

fn reference_bouncing_room_snapshot(scenario: &BouncingRoomScenario, frames: u32) -> WorldSnapshot {
    let mut world = ReferenceWorld::new();
    must(
        world.replay(scenario.setup()),
        "validated reference bouncing-room setup must replay",
    );
    for _ in 0..frames {
        let physics = must(
            scenario.step(&world.snapshot()),
            "validated reference bouncing-room physics step must succeed",
        );
        for operation in physics.operations() {
            must(
                world.apply(*operation),
                "reference storage must accept bouncing-room operation",
            );
        }
    }
    world.snapshot()
}

fn sparse_bouncing_room_snapshot(scenario: &BouncingRoomScenario, frames: u32) -> WorldSnapshot {
    let mut world = SparseWorld::new();
    must(
        world.replay(scenario.setup()),
        "validated sparse bouncing-room setup must replay",
    );
    for _ in 0..frames {
        let physics = must(
            scenario.step(&world.snapshot()),
            "validated sparse bouncing-room physics step must succeed",
        );
        for operation in physics.operations() {
            must(
                world.apply(*operation),
                "sparse storage must accept bouncing-room operation",
            );
        }
    }
    world.snapshot()
}

fn cached_sparse_bouncing_room_snapshot(
    scenario: &BouncingRoomScenario,
    frames: u32,
) -> WorldSnapshot {
    let mut world = CachedSparseWorld::new();
    must(
        world.replay(scenario.setup()),
        "validated cached-sparse bouncing-room setup must replay",
    );
    for _ in 0..frames {
        let physics = must(
            scenario.step(&world.snapshot()),
            "validated cached-sparse bouncing-room physics step must succeed",
        );
        for operation in physics.operations() {
            must(
                world.apply(*operation),
                "cached-sparse storage must accept bouncing-room operation",
            );
        }
    }
    world.snapshot()
}

fn archetype_bouncing_room_snapshot(scenario: &BouncingRoomScenario, frames: u32) -> WorldSnapshot {
    let mut world = ArchetypeWorld::new();
    must(
        world.replay(scenario.setup()),
        "validated archetype bouncing-room setup must replay",
    );
    for _ in 0..frames {
        let physics = must(
            scenario.step(&world.snapshot()),
            "validated archetype bouncing-room physics step must succeed",
        );
        for operation in physics.operations() {
            must(
                world.apply(*operation),
                "archetype storage must accept bouncing-room operation",
            );
        }
    }
    world.snapshot()
}
