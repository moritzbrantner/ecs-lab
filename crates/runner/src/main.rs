use std::{fmt::Display, hint::black_box, time::Instant};

use ecs_physics::{PhysicsBody, PhysicsConfig, PhysicsMaterial, step};
use ecs_physics_scenarios::{BouncingRoomScenario, FallingBoxesScenario};
use ecs_reference::ReferenceWorld;
use ecs_sparse_set::SparseWorld;
use ecs_workload::{
    EntityId, EntitySnapshot, Operation, Position, Velocity, Workload, WorldSnapshot,
};

const BENCHMARK_SEED: u32 = 0x5EED_CAFE;
const FALLING_BOX_SEED: u32 = 0;
const MATERIAL_FIXTURE_SEED: u32 = 0x0BAD_5EED;
const BOUNCING_ROOM_SEED: u32 = 0xB00C_E001;
const MATERIAL_COLUMNS: u32 = 16;
const NO_GRAVITY: PhysicsConfig = PhysicsConfig {
    gravity: Velocity::new(0, 0),
};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut arguments = std::env::args().skip(1);
    match arguments.next().as_deref() {
        Some("benchmark") => {
            let fingerprint = benchmark_fingerprint(arguments.next());
            run_benchmarks(false, &fingerprint);
        }
        Some("benchmark-smoke") => {
            let fingerprint = benchmark_fingerprint(arguments.next());
            run_benchmarks(true, &fingerprint);
        }
        _ => run_demo()?,
    }
    Ok(())
}

fn must<T, E: Display>(result: Result<T, E>, context: &str) -> T {
    match result {
        Ok(value) => value,
        Err(error) => panic!("{context}: {error}"),
    }
}

fn benchmark_fingerprint(argument: Option<String>) -> String {
    argument.unwrap_or_else(|| "unverified".to_owned())
}

fn run_demo() -> Result<(), Box<dyn std::error::Error>> {
    let entity = EntityId(1);
    let workload = Workload::new(vec![
        Operation::Spawn(entity),
        Operation::SetPosition(entity, Position::new(0, 0)),
        Operation::SetVelocity(entity, Velocity::new(2, 1)),
        Operation::Integrate { ticks: 3 },
    ]);

    let mut world = SparseWorld::new();
    world.replay(&workload)?;
    println!("{:#?}", world.snapshot());
    Ok(())
}

fn run_benchmarks(smoke: bool, fingerprint: &str) {
    run_motion_benchmarks(smoke, fingerprint);
    run_falling_box_benchmarks(smoke, fingerprint);
    run_material_step_benchmarks(smoke, fingerprint);
    run_bouncing_room_benchmarks(smoke, fingerprint);
}

fn run_motion_benchmarks(smoke: bool, fingerprint: &str) {
    let (entity_count, rounds, repetitions) = if smoke {
        (1_000, 10, 2)
    } else {
        (50_000, 50, 5)
    };
    let workload = Workload::motion_scenario(BENCHMARK_SEED, entity_count, rounds);
    let reference_expected = reference_motion_snapshot(&workload);
    let sparse_expected = sparse_motion_snapshot(&workload);
    assert_eq!(
        sparse_expected, reference_expected,
        "motion benchmark fixture must prove storage parity before timing"
    );

    benchmark(
        "motion",
        "reference",
        entity_count,
        rounds,
        BENCHMARK_SEED,
        repetitions,
        fingerprint,
        || reference_motion_snapshot(black_box(&workload)),
    );
    benchmark(
        "motion",
        "sparse-set",
        entity_count,
        rounds,
        BENCHMARK_SEED,
        repetitions,
        fingerprint,
        || sparse_motion_snapshot(black_box(&workload)),
    );
}

fn reference_motion_snapshot(workload: &Workload) -> WorldSnapshot {
    let mut world = ReferenceWorld::new();
    must(
        world.replay(workload),
        "validated reference motion benchmark replay must succeed",
    );
    world.snapshot()
}

fn sparse_motion_snapshot(workload: &Workload) -> WorldSnapshot {
    let mut world = SparseWorld::new();
    must(
        world.replay(workload),
        "validated sparse motion benchmark replay must succeed",
    );
    world.snapshot()
}

fn run_falling_box_benchmarks(smoke: bool, fingerprint: &str) {
    let (dynamic_count, frames, repetitions) = if smoke { (96, 12, 2) } else { (512, 40, 3) };
    let scenario = FallingBoxesScenario::new(dynamic_count);
    let body_count = dynamic_count.saturating_add(1);
    let reference_expected = reference_falling_box_snapshot(&scenario, frames);
    let sparse_expected = sparse_falling_box_snapshot(&scenario, frames);
    assert_eq!(
        sparse_expected, reference_expected,
        "falling-box benchmark fixture must prove storage parity before timing"
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

fn run_material_step_benchmarks(smoke: bool, fingerprint: &str) {
    let (sparse_count, dense_count, repetitions) = if smoke { (96, 64, 3) } else { (512, 192, 5) };
    let (sparse_snapshot, sparse_bodies) = material_step_fixture(sparse_count, 8);
    let (dense_snapshot, dense_bodies) = material_step_fixture(dense_count, 1);

    let sparse_preflight = must(
        step(&sparse_snapshot, &sparse_bodies, NO_GRAVITY, 1),
        "sparse material benchmark preflight must succeed",
    );
    assert_eq!(
        sparse_preflight.stats().contacts,
        0,
        "sparse material benchmark must isolate candidate-pair traversal"
    );
    let dense_preflight = must(
        step(&dense_snapshot, &dense_bodies, NO_GRAVITY, 1),
        "dense material benchmark preflight must succeed",
    );
    assert!(
        dense_preflight.stats().contacts > 0,
        "dense material benchmark must exercise contact/material response"
    );

    benchmark(
        "material-step-sparse",
        "rust-step",
        sparse_count,
        1,
        MATERIAL_FIXTURE_SEED,
        repetitions,
        fingerprint,
        || {
            must(
                step(
                    black_box(&sparse_snapshot),
                    black_box(&sparse_bodies),
                    NO_GRAVITY,
                    1,
                ),
                "validated sparse material benchmark step must succeed",
            )
        },
    );
    benchmark(
        "material-step-dense",
        "rust-step",
        dense_count,
        1,
        MATERIAL_FIXTURE_SEED,
        repetitions,
        fingerprint,
        || {
            must(
                step(
                    black_box(&dense_snapshot),
                    black_box(&dense_bodies),
                    NO_GRAVITY,
                    1,
                ),
                "validated dense material benchmark step must succeed",
            )
        },
    );
}

fn material_step_fixture(body_count: u32, spacing: i64) -> (WorldSnapshot, Vec<PhysicsBody>) {
    let mut entities = Vec::new();
    let mut bodies = Vec::new();

    for raw_id in 0..body_count {
        let entity = EntityId(raw_id);
        let column = i64::from(raw_id % MATERIAL_COLUMNS);
        let row = i64::from(raw_id / MATERIAL_COLUMNS);
        let velocity_x = must(
            i32::try_from(raw_id % 5),
            "benchmark velocity residue must fit i32",
        ) - 2;
        let velocity_y = must(
            i32::try_from((raw_id / 5) % 5),
            "benchmark velocity residue must fit i32",
        ) - 2;
        let restitution = must(
            u16::try_from((raw_id % 5) * 250),
            "benchmark restitution must fit u16",
        );
        let friction = must(
            u16::try_from(((raw_id + 2) % 5) * 250),
            "benchmark friction must fit u16",
        );

        entities.push(EntitySnapshot {
            id: entity,
            position: Some(Position::new(column * spacing, row * spacing)),
            velocity: Some(Velocity::new(velocity_x, velocity_y)),
        });
        bodies.push(
            PhysicsBody::dynamic(entity, [1, 1])
                .with_mass(1 + raw_id % 7)
                .with_material(PhysicsMaterial::new(restitution, friction)),
        );
    }

    (WorldSnapshot::new(entities), bodies)
}

fn run_bouncing_room_benchmarks(smoke: bool, fingerprint: &str) {
    let (frames, repetitions) = if smoke { (128, 3) } else { (4_096, 5) };
    let scenario = BouncingRoomScenario::new();
    let body_count = must(
        u32::try_from(scenario.bodies().len()),
        "scenario body count must fit u32",
    );
    let reference_expected = reference_bouncing_room_snapshot(&scenario, frames);
    let sparse_expected = sparse_bouncing_room_snapshot(&scenario, frames);
    assert_eq!(
        sparse_expected, reference_expected,
        "bouncing-room benchmark fixture must prove storage parity before timing"
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

#[allow(clippy::too_many_arguments)]
fn benchmark<T>(
    scenario: &str,
    implementation: &str,
    entity_count: u32,
    rounds: u32,
    seed: u32,
    repetitions: u32,
    fingerprint: &str,
    mut run: impl FnMut() -> T,
) {
    let started = Instant::now();
    for _ in 0..repetitions {
        black_box(run());
    }
    let elapsed = started.elapsed();
    println!(
        "scenario={scenario} implementation={implementation} entities={entity_count} rounds={rounds} seed={seed} repetitions={repetitions} elapsed_ns={} environment_fingerprint={fingerprint}",
        elapsed.as_nanos()
    );
}
