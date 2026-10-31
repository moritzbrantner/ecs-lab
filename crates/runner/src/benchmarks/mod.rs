mod bouncing_room;
mod falling_boxes;
mod material_step;
mod storage_workloads;
mod workload_evidence;

use std::{fmt::Display, hint::black_box, time::Instant};

pub(crate) fn run_benchmarks(smoke: bool, fingerprint: &str) {
    storage_workloads::run_motion_benchmarks(smoke, fingerprint);
    storage_workloads::run_mixed_motion_benchmarks(smoke, fingerprint);
    storage_workloads::run_component_churn_benchmarks(smoke, fingerprint);
    falling_boxes::run_falling_box_benchmarks(smoke, fingerprint);
    material_step::run_material_step_benchmarks(smoke, fingerprint);
    bouncing_room::run_bouncing_room_benchmarks(smoke, fingerprint);
}

fn must<T, E: Display>(result: Result<T, E>, context: &str) -> T {
    match result {
        Ok(value) => value,
        Err(error) => panic!("{context}: {error}"),
    }
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
