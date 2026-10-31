use ecs_archetype::ArchetypeWorld;
use ecs_cached_sparse::CachedSparseWorld;
use ecs_reference::ReferenceWorld;
use ecs_sparse_set::SparseWorld;
use ecs_workload::{
    SnapshotWorkStats, StorageIndexStats, StorageWorkStats, Workload, WorldSnapshot,
};

use super::must;

pub(super) fn print_storage_work_evidence(
    scenario: &str,
    implementation: &str,
    work: StorageWorkStats,
    snapshot: SnapshotWorkStats,
    index: StorageIndexStats,
) {
    println!(
        "storage_work scenario={scenario} implementation={implementation} integration_rows_scanned={} integrated_entities={} component_lookups={} structural_table_transitions={} query_cache_updates={} snapshot_slots_scanned={} snapshot_entities_materialized={} entity_index_entries={} component_index_slots={} query_index_slots={} query_rows={}",
        work.integration_rows_scanned,
        work.integrated_entities,
        work.component_lookups,
        work.structural_table_transitions,
        work.query_cache_updates,
        snapshot.slots_scanned,
        snapshot.entities_materialized,
        index.entity_index_entries,
        index.component_index_slots,
        index.query_index_slots,
        index.query_rows,
    );
}

pub(super) fn reference_workload_evidence(
    workload: &Workload,
) -> (
    WorldSnapshot,
    StorageWorkStats,
    SnapshotWorkStats,
    StorageIndexStats,
) {
    let mut world = ReferenceWorld::new();
    let mut work = StorageWorkStats::default();
    for operation in workload.operations() {
        work.accumulate(world.operation_work(*operation));
        must(
            world.apply(*operation),
            "validated reference motion evidence replay must succeed",
        );
    }
    let (snapshot, snapshot_work) = world.snapshot_with_stats();
    let index = world.index_stats();
    (snapshot, work, snapshot_work, index)
}

pub(super) fn sparse_workload_evidence(
    workload: &Workload,
) -> (
    WorldSnapshot,
    StorageWorkStats,
    SnapshotWorkStats,
    StorageIndexStats,
) {
    let mut world = SparseWorld::new();
    let mut work = StorageWorkStats::default();
    for operation in workload.operations() {
        work.accumulate(world.operation_work(*operation));
        must(
            world.apply(*operation),
            "validated sparse motion evidence replay must succeed",
        );
    }
    let (snapshot, snapshot_work) = world.snapshot_with_stats();
    let index = world.index_stats();
    (snapshot, work, snapshot_work, index)
}

pub(super) fn cached_sparse_workload_evidence(
    workload: &Workload,
) -> (
    WorldSnapshot,
    StorageWorkStats,
    SnapshotWorkStats,
    StorageIndexStats,
) {
    let mut world = CachedSparseWorld::new();
    let mut work = StorageWorkStats::default();
    for operation in workload.operations() {
        work.accumulate(world.operation_work(*operation));
        must(
            world.apply(*operation),
            "validated cached-sparse motion evidence replay must succeed",
        );
    }
    let (snapshot, snapshot_work) = world.snapshot_with_stats();
    let index = world.index_stats();
    (snapshot, work, snapshot_work, index)
}

pub(super) fn archetype_workload_evidence(
    workload: &Workload,
) -> (
    WorldSnapshot,
    StorageWorkStats,
    SnapshotWorkStats,
    StorageIndexStats,
) {
    let mut world = ArchetypeWorld::new();
    let mut work = StorageWorkStats::default();
    for operation in workload.operations() {
        work.accumulate(world.operation_work(*operation));
        must(
            world.apply(*operation),
            "validated archetype motion evidence replay must succeed",
        );
    }
    let (snapshot, snapshot_work) = world.snapshot_with_stats();
    let index = world.index_stats();
    (snapshot, work, snapshot_work, index)
}

pub(super) fn reference_motion_snapshot(workload: &Workload) -> WorldSnapshot {
    let mut world = ReferenceWorld::new();
    must(
        world.replay(workload),
        "validated reference motion benchmark replay must succeed",
    );
    world.snapshot()
}

pub(super) fn sparse_motion_snapshot(workload: &Workload) -> WorldSnapshot {
    let mut world = SparseWorld::new();
    must(
        world.replay(workload),
        "validated sparse motion benchmark replay must succeed",
    );
    world.snapshot()
}

pub(super) fn cached_sparse_motion_snapshot(workload: &Workload) -> WorldSnapshot {
    let mut world = CachedSparseWorld::new();
    must(
        world.replay(workload),
        "validated cached-sparse motion benchmark replay must succeed",
    );
    world.snapshot()
}

pub(super) fn archetype_motion_snapshot(workload: &Workload) -> WorldSnapshot {
    let mut world = ArchetypeWorld::new();
    must(
        world.replay(workload),
        "validated archetype motion benchmark replay must succeed",
    );
    world.snapshot()
}
