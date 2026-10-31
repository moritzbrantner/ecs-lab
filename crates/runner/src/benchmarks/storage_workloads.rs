use std::hint::black_box;

use ecs_workload::Workload;

use super::{
    benchmark,
    workload_evidence::{
        archetype_motion_snapshot, archetype_workload_evidence, cached_sparse_motion_snapshot,
        cached_sparse_workload_evidence, print_storage_work_evidence, reference_motion_snapshot,
        reference_workload_evidence, sparse_motion_snapshot, sparse_workload_evidence,
    },
};

const BENCHMARK_SEED: u32 = 0x5EED_CAFE;
const MIXED_MOTION_SEED: u32 = 0xC0DE_4D1D;
const MIXED_MOTION_VELOCITY_STRIDE: u32 = 4;
const COMPONENT_CHURN_SEED: u32 = 0xC11A_7E55;

pub(super) fn run_motion_benchmarks(smoke: bool, fingerprint: &str) {
    let (entity_count, rounds, repetitions) = if smoke {
        (1_000, 10, 2)
    } else {
        (50_000, 50, 5)
    };
    let workload = Workload::motion_scenario(BENCHMARK_SEED, entity_count, rounds);
    let (reference_expected, reference_work, reference_snapshot_work, reference_index) =
        reference_workload_evidence(&workload);
    let (sparse_expected, sparse_work, sparse_snapshot_work, sparse_index) =
        sparse_workload_evidence(&workload);
    let (cached_expected, cached_work, cached_snapshot_work, cached_index) =
        cached_sparse_workload_evidence(&workload);
    let (archetype_expected, archetype_work, archetype_snapshot_work, archetype_index) =
        archetype_workload_evidence(&workload);
    assert_eq!(
        sparse_expected, reference_expected,
        "motion benchmark fixture must prove sparse/reference parity before timing"
    );
    assert_eq!(
        cached_expected, reference_expected,
        "motion benchmark fixture must prove cached-sparse/reference parity before timing"
    );
    assert_eq!(
        archetype_expected, reference_expected,
        "motion benchmark fixture must prove archetype/reference parity before timing"
    );
    print_storage_work_evidence(
        "motion",
        "reference",
        reference_work,
        reference_snapshot_work,
        reference_index,
    );
    print_storage_work_evidence(
        "motion",
        "sparse-set",
        sparse_work,
        sparse_snapshot_work,
        sparse_index,
    );
    print_storage_work_evidence(
        "motion",
        "cached-sparse",
        cached_work,
        cached_snapshot_work,
        cached_index,
    );
    print_storage_work_evidence(
        "motion",
        "archetype-table",
        archetype_work,
        archetype_snapshot_work,
        archetype_index,
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
    benchmark(
        "motion",
        "cached-sparse",
        entity_count,
        rounds,
        BENCHMARK_SEED,
        repetitions,
        fingerprint,
        || cached_sparse_motion_snapshot(black_box(&workload)),
    );
    benchmark(
        "motion",
        "archetype-table",
        entity_count,
        rounds,
        BENCHMARK_SEED,
        repetitions,
        fingerprint,
        || archetype_motion_snapshot(black_box(&workload)),
    );
}

pub(super) fn run_mixed_motion_benchmarks(smoke: bool, fingerprint: &str) {
    let (entity_count, rounds, repetitions) = if smoke {
        (1_000, 20, 3)
    } else {
        (50_000, 50, 5)
    };
    let workload = Workload::mixed_motion_scenario(
        MIXED_MOTION_SEED,
        entity_count,
        rounds,
        MIXED_MOTION_VELOCITY_STRIDE,
    );
    verify_mixed_motion_evidence(&workload);

    benchmark(
        "mixed-motion",
        "reference",
        entity_count,
        rounds,
        MIXED_MOTION_SEED,
        repetitions,
        fingerprint,
        || reference_motion_snapshot(black_box(&workload)),
    );
    benchmark(
        "mixed-motion",
        "sparse-set",
        entity_count,
        rounds,
        MIXED_MOTION_SEED,
        repetitions,
        fingerprint,
        || sparse_motion_snapshot(black_box(&workload)),
    );
    benchmark(
        "mixed-motion",
        "cached-sparse",
        entity_count,
        rounds,
        MIXED_MOTION_SEED,
        repetitions,
        fingerprint,
        || cached_sparse_motion_snapshot(black_box(&workload)),
    );
    benchmark(
        "mixed-motion",
        "archetype-table",
        entity_count,
        rounds,
        MIXED_MOTION_SEED,
        repetitions,
        fingerprint,
        || archetype_motion_snapshot(black_box(&workload)),
    );
}

fn verify_mixed_motion_evidence(workload: &Workload) {
    let (reference_expected, reference_work, reference_snapshot_work, reference_index) =
        reference_workload_evidence(workload);
    let (sparse_expected, sparse_work, sparse_snapshot_work, sparse_index) =
        sparse_workload_evidence(workload);
    let (cached_expected, cached_work, cached_snapshot_work, cached_index) =
        cached_sparse_workload_evidence(workload);
    let (archetype_expected, archetype_work, archetype_snapshot_work, archetype_index) =
        archetype_workload_evidence(workload);

    assert_eq!(
        sparse_expected, reference_expected,
        "mixed-motion fixture must prove sparse/reference parity before timing"
    );
    assert_eq!(
        cached_expected, reference_expected,
        "mixed-motion fixture must prove cached-sparse/reference parity before timing"
    );
    assert_eq!(
        archetype_expected, reference_expected,
        "mixed-motion fixture must prove archetype/reference parity before timing"
    );
    assert_eq!(
        sparse_work.integrated_entities, reference_work.integrated_entities,
        "mixed-motion sparse useful integration work must match reference"
    );
    assert_eq!(
        cached_work.integrated_entities, reference_work.integrated_entities,
        "mixed-motion cached useful integration work must match reference"
    );
    assert_eq!(
        archetype_work.integrated_entities, reference_work.integrated_entities,
        "mixed-motion archetype useful integration work must match reference"
    );
    assert_eq!(
        cached_work.integration_rows_scanned, archetype_work.integration_rows_scanned,
        "mixed-motion cached query should scan the same participating rows as archetype storage"
    );
    assert_eq!(
        cached_work.component_lookups, 0,
        "mixed-motion cached query should avoid per-row component lookups"
    );
    assert_eq!(
        sparse_work.integration_rows_scanned, archetype_work.integration_rows_scanned,
        "mixed-motion sparse query must scan exactly the participating rows after selectivity-aware driving"
    );
    assert!(
        sparse_work.component_lookups > 0 && archetype_work.component_lookups == 0,
        "mixed-motion fixture must expose sparse lookup work versus archetype columns"
    );

    print_storage_work_evidence(
        "mixed-motion",
        "reference",
        reference_work,
        reference_snapshot_work,
        reference_index,
    );
    print_storage_work_evidence(
        "mixed-motion",
        "sparse-set",
        sparse_work,
        sparse_snapshot_work,
        sparse_index,
    );
    print_storage_work_evidence(
        "mixed-motion",
        "cached-sparse",
        cached_work,
        cached_snapshot_work,
        cached_index,
    );
    print_storage_work_evidence(
        "mixed-motion",
        "archetype-table",
        archetype_work,
        archetype_snapshot_work,
        archetype_index,
    );
}

pub(super) fn run_component_churn_benchmarks(smoke: bool, fingerprint: &str) {
    let (entity_count, rounds, repetitions) = if smoke { (512, 8, 3) } else { (20_000, 20, 5) };
    let workload = Workload::component_churn_scenario(COMPONENT_CHURN_SEED, entity_count, rounds);
    verify_component_churn_evidence(&workload);

    benchmark(
        "component-churn",
        "reference",
        entity_count,
        rounds,
        COMPONENT_CHURN_SEED,
        repetitions,
        fingerprint,
        || reference_motion_snapshot(black_box(&workload)),
    );
    benchmark(
        "component-churn",
        "sparse-set",
        entity_count,
        rounds,
        COMPONENT_CHURN_SEED,
        repetitions,
        fingerprint,
        || sparse_motion_snapshot(black_box(&workload)),
    );
    benchmark(
        "component-churn",
        "cached-sparse",
        entity_count,
        rounds,
        COMPONENT_CHURN_SEED,
        repetitions,
        fingerprint,
        || cached_sparse_motion_snapshot(black_box(&workload)),
    );
    benchmark(
        "component-churn",
        "archetype-table",
        entity_count,
        rounds,
        COMPONENT_CHURN_SEED,
        repetitions,
        fingerprint,
        || archetype_motion_snapshot(black_box(&workload)),
    );
}

fn verify_component_churn_evidence(workload: &Workload) {
    let (reference_expected, reference_work, reference_snapshot_work, reference_index) =
        reference_workload_evidence(workload);
    let (sparse_expected, sparse_work, sparse_snapshot_work, sparse_index) =
        sparse_workload_evidence(workload);
    let (cached_expected, cached_work, cached_snapshot_work, cached_index) =
        cached_sparse_workload_evidence(workload);
    let (archetype_expected, archetype_work, archetype_snapshot_work, archetype_index) =
        archetype_workload_evidence(workload);

    assert_eq!(
        sparse_expected, reference_expected,
        "component-churn fixture must prove sparse/reference parity before timing"
    );
    assert_eq!(
        cached_expected, reference_expected,
        "component-churn fixture must prove cached-sparse/reference parity before timing"
    );
    assert_eq!(
        archetype_expected, reference_expected,
        "component-churn fixture must prove archetype/reference parity before timing"
    );
    assert_eq!(
        sparse_work.integrated_entities, reference_work.integrated_entities,
        "component-churn sparse useful integration work must match reference"
    );
    assert_eq!(
        cached_work.integrated_entities, reference_work.integrated_entities,
        "component-churn cached useful integration work must match reference"
    );
    assert_eq!(
        archetype_work.integrated_entities, reference_work.integrated_entities,
        "component-churn archetype useful integration work must match reference"
    );
    assert!(
        archetype_work.structural_table_transitions > 0,
        "component-churn fixture must exercise archetype migrations"
    );
    assert!(
        cached_work.query_cache_updates > 0,
        "component-churn fixture must exercise cached-query maintenance"
    );

    print_storage_work_evidence(
        "component-churn",
        "reference",
        reference_work,
        reference_snapshot_work,
        reference_index,
    );
    print_storage_work_evidence(
        "component-churn",
        "sparse-set",
        sparse_work,
        sparse_snapshot_work,
        sparse_index,
    );
    print_storage_work_evidence(
        "component-churn",
        "cached-sparse",
        cached_work,
        cached_snapshot_work,
        cached_index,
    );
    print_storage_work_evidence(
        "component-churn",
        "archetype-table",
        archetype_work,
        archetype_snapshot_work,
        archetype_index,
    );
}
