use std::{hint::black_box, time::Instant};

use ecs_reference::ReferenceWorld;
use ecs_sparse_set::SparseWorld;
use ecs_workload::{EntityId, Operation, Position, Velocity, Workload, WorldSnapshot};

const BENCHMARK_SEED: u32 = 0x5EED_CAFE;

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
    let (entity_count, rounds, repetitions) = if smoke {
        (1_000, 10, 2)
    } else {
        (50_000, 50, 5)
    };
    let workload = Workload::motion_scenario(BENCHMARK_SEED, entity_count, rounds);

    benchmark(
        "motion",
        "reference",
        entity_count,
        rounds,
        repetitions,
        fingerprint,
        || {
            let mut world = ReferenceWorld::new();
            if world.replay(black_box(&workload)).is_err() {
                return WorldSnapshot::default();
            }
            world.snapshot()
        },
    );
    benchmark(
        "motion",
        "sparse-set",
        entity_count,
        rounds,
        repetitions,
        fingerprint,
        || {
            let mut world = SparseWorld::new();
            if world.replay(black_box(&workload)).is_err() {
                return WorldSnapshot::default();
            }
            world.snapshot()
        },
    );
}

#[allow(clippy::too_many_arguments)]
fn benchmark(
    scenario: &str,
    implementation: &str,
    entity_count: u32,
    rounds: u32,
    repetitions: u32,
    fingerprint: &str,
    mut run: impl FnMut() -> WorldSnapshot,
) {
    let started = Instant::now();
    for _ in 0..repetitions {
        black_box(run());
    }
    let elapsed = started.elapsed();
    println!(
        "scenario={scenario} implementation={implementation} entities={entity_count} rounds={rounds} seed={BENCHMARK_SEED} repetitions={repetitions} elapsed_ns={} environment_fingerprint={fingerprint}",
        elapsed.as_nanos()
    );
}
