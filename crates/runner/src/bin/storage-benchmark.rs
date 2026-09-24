//! Fixed storage experiments. Probes and correctness checks never enter timed repetitions.
use std::{hint::black_box, time::Instant};

use ecs_archetype::ArchetypeWorld;
use ecs_cached_sparse::CachedSparseWorld;
use ecs_reference::ReferenceWorld;
use ecs_sparse_set::SparseWorld;
use ecs_workload::{
    EntityId, Operation, Position, SnapshotWorkStats, StorageIndexStats, StorageWorkStats,
    Velocity, Workload, WorldSnapshot,
};

const SEED: u32 = 0xC0DE_4D1D;
const ROUNDS: u32 = 8;

struct Fixture {
    name: &'static str,
    workload: Workload,
    useful: u64,
    live: usize,
}

#[derive(Debug, PartialEq)]
struct Evidence {
    snapshot: WorldSnapshot,
    work: StorageWorkStats,
    projection: SnapshotWorkStats,
    index: StorageIndexStats,
    peak_component_slots: u64,
    peak_query_slots: u64,
}

// Concrete expansion deliberately avoids imposing a common dispatch trait on storage hot paths.
macro_rules! backend {
    ($capture:ident, $replay:ident, $world:ty) => {
        fn $capture(workload: &Workload) -> Evidence {
            let mut world = <$world>::new();
            let mut work = StorageWorkStats::default();
            let mut peak_component_slots = 0;
            let mut peak_query_slots = 0;
            for &operation in workload.operations() {
                work.accumulate(world.operation_work(operation));
                assert_eq!(world.apply(operation), Ok(()));
                let index = world.index_stats();
                peak_component_slots = peak_component_slots.max(index.component_index_slots);
                peak_query_slots = peak_query_slots.max(index.query_index_slots);
            }
            let (snapshot, projection) = world.snapshot_with_stats();
            Evidence {
                snapshot,
                work,
                projection,
                index: world.index_stats(),
                peak_component_slots,
                peak_query_slots,
            }
        }

        fn $replay(workload: &Workload) -> WorldSnapshot {
            let mut world = <$world>::new();
            assert_eq!(world.replay(workload), Ok(()));
            world.snapshot()
        }
    };
}

backend!(reference_evidence, reference_replay, ReferenceWorld);
backend!(sparse_evidence, sparse_replay, SparseWorld);
backend!(cached_evidence, cached_replay, CachedSparseWorld);
backend!(archetype_evidence, archetype_replay, ArchetypeWorld);

fn fixtures() -> Vec<Fixture> {
    let mut fixtures = Vec::new();
    for (name, count, stride) in [
        ("dense-128", 128_u32, 1_u32),
        ("dense-1024", 1_024, 1),
        ("quarter-1024", 1_024, 4),
        ("sparse-1024", 1_024, 64),
        ("no-match-1024", 1_024, 0),
        ("empty", 0, 1),
    ] {
        let participants = if stride == 0 {
            0
        } else {
            count.div_ceil(stride)
        };
        fixtures.push(Fixture {
            name,
            workload: Workload::mixed_motion_scenario(SEED, count, ROUNDS, stride),
            useful: u64::from(participants) * u64::from(ROUNDS),
            live: usize::try_from(count).unwrap_or(usize::MAX),
        });
    }
    fixtures.push(Fixture {
        name: "churn-128",
        workload: Workload::component_churn_scenario(SEED, 128, ROUNDS),
        useful: 128 * u64::from(ROUNDS),
        live: 128,
    });
    fixtures.push(Fixture {
        name: "distant-live",
        workload: distant_workload(false),
        useful: 12,
        live: 6,
    });
    fixtures.push(Fixture {
        name: "distant-reclaimed",
        workload: distant_workload(true),
        useful: 12,
        live: 0,
    });
    fixtures
}

fn distant_workload(reclaim: bool) -> Workload {
    // Four pages, including adjacent entries at both a page boundary and the u32 ceiling.
    let ids = [0, 255, 256, 65_536, u32::MAX - 1, u32::MAX].map(EntityId);
    let mut operations = Vec::new();
    for id in ids {
        operations.extend([
            Operation::Spawn(id),
            Operation::SetPosition(id, Position::new3(i64::from(id.0), 2, -3)),
            Operation::SetVelocity(id, Velocity::new3(1, -2, 3)),
        ]);
    }
    operations.push(Operation::Integrate { ticks: 2 });
    // Non-tail component removals force dense-index and retained-query repairs.
    operations.extend([
        Operation::RemovePosition(ids[1]),
        Operation::SetPosition(ids[1], Position::new3(4, 5, 6)),
        Operation::RemoveVelocity(ids[2]),
        Operation::SetVelocity(ids[2], Velocity::new3(3, 2, 1)),
        Operation::Integrate { ticks: 3 },
    ]);
    if reclaim {
        for index in [1, 4, 0, 3, 2, 5] {
            operations.push(Operation::Despawn(ids[index]));
        }
        operations.push(Operation::Integrate { ticks: 1 });
    }
    Workload::new(operations)
}

fn emit(fixture: &Fixture, implementation: &str, evidence: &Evidence) {
    let work = evidence.work;
    let projection = evidence.projection;
    let index = evidence.index;
    println!(
        "storage_work scenario={} implementation={implementation} operations={} integrated_entities={} live_entities={} integration_rows_scanned={} component_lookups={} structural_table_transitions={} query_cache_updates={} snapshot_slots_scanned={} snapshot_entities_materialized={} entity_index_entries={} component_index_slots={} query_index_slots={} query_rows={} peak_component_index_slots={} peak_query_index_slots={}",
        fixture.name,
        fixture.workload.operations().len(),
        work.integrated_entities,
        evidence.snapshot.entities().len(),
        work.integration_rows_scanned,
        work.component_lookups,
        work.structural_table_transitions,
        work.query_cache_updates,
        projection.slots_scanned,
        projection.entities_materialized,
        index.entity_index_entries,
        index.component_index_slots,
        index.query_index_slots,
        index.query_rows,
        evidence.peak_component_slots,
        evidence.peak_query_slots,
    );
}

fn verify(fixture: &Fixture) -> [Evidence; 4] {
    let evidence = [
        reference_evidence(&fixture.workload),
        sparse_evidence(&fixture.workload),
        cached_evidence(&fixture.workload),
        archetype_evidence(&fixture.workload),
    ];
    for result in &evidence {
        assert_eq!(
            result.snapshot, evidence[0].snapshot,
            "{} parity",
            fixture.name
        );
        assert_eq!(
            result.work.integrated_entities, fixture.useful,
            "{} useful work",
            fixture.name
        );
        assert_eq!(
            result.snapshot.entities().len(),
            fixture.live,
            "{} live entities",
            fixture.name
        );
    }
    evidence
}

fn measure(
    fixture: &Fixture,
    implementation: &str,
    replay: fn(&Workload) -> WorldSnapshot,
    environment_fingerprint: &str,
) {
    // Workload construction, evidence, and parity are excluded. World construction, replay,
    // final canonical snapshot and teardown are included consistently for every backend.
    drop(black_box(replay(black_box(&fixture.workload))));
    let mut samples = [0_u128; 7];
    for sample in &mut samples {
        let start = Instant::now();
        drop(black_box(replay(black_box(&fixture.workload))));
        *sample = start.elapsed().as_nanos();
    }
    samples.sort_unstable();
    println!(
        "storage_timing scenario={} implementation={implementation} samples=7 median_ns={} min_ns={} max_ns={} advisory=true environment_fingerprint={environment_fingerprint}",
        fixture.name, samples[3], samples[0], samples[6],
    );
}

fn parse_args(args: &[String]) -> Result<(bool, &str), String> {
    match args {
        [] => Ok((false, "unverified")),
        [mode] if mode == "--bench" => Ok((true, "unverified")),
        [mode, environment_fingerprint] if mode == "--bench" => {
            Ok((true, environment_fingerprint.as_str()))
        }
        _ => Err("usage: storage-benchmark [--bench [environment-fingerprint]]".to_owned()),
    }
}

fn main() -> Result<(), String> {
    let args: Vec<_> = std::env::args().skip(1).collect();
    let (timed, environment_fingerprint) = parse_args(&args)?;
    for fixture in fixtures() {
        let evidence = verify(&fixture);
        for (implementation, result) in [
            "reference",
            "sparse-set",
            "cached-sparse",
            "archetype-table",
        ]
        .into_iter()
        .zip(&evidence)
        {
            emit(&fixture, implementation, result);
        }
        if timed {
            for (implementation, replay) in [
                (
                    "reference",
                    reference_replay as fn(&Workload) -> WorldSnapshot,
                ),
                ("sparse-set", sparse_replay),
                ("cached-sparse", cached_replay),
                ("archetype-table", archetype_replay),
            ] {
                measure(&fixture, implementation, replay, environment_fingerprint);
            }
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn benchmark_arguments_preserve_environment_identity_and_reject_trailing_values() {
        assert_eq!(parse_args(&[]), Ok((false, "unverified")));
        assert_eq!(
            parse_args(&["--bench".to_owned()]),
            Ok((true, "unverified"))
        );
        assert_eq!(
            parse_args(&["--bench".to_owned(), "env-v1:sha256:test".to_owned()]),
            Ok((true, "env-v1:sha256:test"))
        );
        assert!(parse_args(&["--bench".to_owned(), "env".to_owned(), "extra".to_owned()]).is_err());
    }

    #[test]
    fn fixture_matrix_is_deterministic_and_proves_parity() {
        let cases = fixtures();
        assert_eq!(cases.len(), 9);
        for case in cases {
            assert_eq!(verify(&case), verify(&case), "{} repeatability", case.name);
        }
    }

    #[test]
    fn distant_lifecycle_preserves_each_intermediate_snapshot() {
        let mut reference = ReferenceWorld::new();
        let mut sparse = SparseWorld::new();
        let mut cached = CachedSparseWorld::new();
        let mut archetype = ArchetypeWorld::new();
        // Replay twice in the same worlds: released pages and recycled IDs must remain usable.
        for _ in 0..2 {
            for &operation in distant_workload(true).operations() {
                assert_eq!(reference.apply(operation), Ok(()));
                assert_eq!(sparse.apply(operation), Ok(()));
                assert_eq!(cached.apply(operation), Ok(()));
                assert_eq!(archetype.apply(operation), Ok(()));
                let expected = reference.snapshot();
                assert_eq!(sparse.snapshot(), expected, "{operation:?}");
                assert_eq!(cached.snapshot(), expected, "{operation:?}");
                assert_eq!(archetype.snapshot(), expected, "{operation:?}");
            }
            assert_eq!(sparse.index_stats().component_index_slots, 0);
            assert_eq!(cached.index_stats().component_index_slots, 0);
            assert_eq!(cached.index_stats().query_index_slots, 0);
            assert_eq!(cached.index_stats().query_rows, 0);
        }
    }
}
