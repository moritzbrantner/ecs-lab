//! Multi-scale operation work evidence for the `operations` in `.performance/contract.json`.
//!
//! Every counter here comes from `apply_measured` or `snapshot_with_stats`, which count inside the
//! loops and branches the storages actually execute. They are not the state-based predictions of
//! `operation_work`, so a cached path that falls back to scanning every entity changes the evidence.
//!
//! Each test first proves reference/sparse-set/cached-sparse/archetype snapshot equality, then
//! records its samples. With `ECS_LAB_WORK_EVIDENCE_DIR` set (by `scripts/collect-work-evidence.sh`)
//! the samples are written as `<operation>.json` and `coding-tooling performance work` judges them
//! against the contract. Without it, the test asserts the same budgets and growth bounds itself, so
//! plain `cargo test` fails on the same regressions.

use std::collections::BTreeMap;

use ecs_archetype::ArchetypeWorld;
use ecs_cached_sparse::CachedSparseWorld;
use ecs_reference::ReferenceWorld;
use ecs_sparse_set::SparseWorld;
use ecs_workload::{
    EntityId, Operation, Position, SnapshotWorkStats, StorageWorkStats, Velocity, Workload,
    WorldSnapshot,
};

const EVIDENCE_DIR_ENV: &str = "ECS_LAB_WORK_EVIDENCE_DIR";
const SEED: u32 = 0x00EC_5138;

/// One scale point: dimension values and metric values, in declaration order.
struct Sample {
    dimensions: Vec<(&'static str, u64)>,
    metrics: Vec<(&'static str, u64)>,
}

/// The values of every dimension except the bounded one.
type GroupKey = Vec<(&'static str, u64)>;

#[derive(Clone, Copy, Debug)]
enum Growth {
    Constant,
    Linear,
}

/// One metric's declared bounds, mirroring the contract entry with the same name.
struct Bound {
    metric: &'static str,
    budget_max: Option<u64>,
    growth: &'static [(&'static str, Growth)],
}

/// The four storages driven in lockstep so every sample is gated on snapshot parity.
#[derive(Default)]
struct Worlds {
    reference: ReferenceWorld,
    sparse: SparseWorld,
    cached: CachedSparseWorld,
    archetype: ArchetypeWorld,
}

/// Measured work per storage for one or more applied operations.
#[derive(Default)]
struct Work {
    reference: StorageWorkStats,
    sparse: StorageWorkStats,
    cached: StorageWorkStats,
    archetype: StorageWorkStats,
}

impl Worlds {
    fn apply(&mut self, operation: Operation, work: &mut Work) {
        work.reference
            .accumulate(self.reference.apply_measured(operation).expect("reference"));
        work.sparse
            .accumulate(self.sparse.apply_measured(operation).expect("sparse-set"));
        work.cached.accumulate(
            self.cached
                .apply_measured(operation)
                .expect("cached-sparse"),
        );
        work.archetype
            .accumulate(self.archetype.apply_measured(operation).expect("archetype"));
    }

    fn replay(&mut self, workload: &Workload) {
        let mut ignored = Work::default();
        for &operation in workload.operations() {
            self.apply(operation, &mut ignored);
        }
    }

    /// Correctness prerequisite: every storage projects the reference snapshot.
    fn assert_snapshot_parity(&self, context: &str) -> WorldSnapshot {
        let expected = self.reference.snapshot();
        assert_eq!(self.sparse.snapshot(), expected, "sparse-set {context}");
        assert_eq!(self.cached.snapshot(), expected, "cached-sparse {context}");
        assert_eq!(self.archetype.snapshot(), expected, "archetype {context}");
        expected
    }
}

impl Work {
    /// Useful work must not depend on the storage strategy.
    fn assert_equal_useful_work(&self, context: &str) -> u64 {
        let integrated = self.reference.integrated_entities;
        assert_eq!(self.sparse.integrated_entities, integrated, "{context}");
        assert_eq!(self.cached.integrated_entities, integrated, "{context}");
        assert_eq!(self.archetype.integrated_entities, integrated, "{context}");
        integrated
    }
}

/// Positioned entities where every `total / moving`-th entity also owns Velocity, followed by one
/// integration. This is the existing mixed-motion fixture at a chosen selectivity.
fn sparse_motion_worlds(total: u32, moving: u32) -> Worlds {
    assert!(
        moving > 0 && total.is_multiple_of(moving),
        "{total} / {moving}"
    );
    let workload = Workload::mixed_motion_scenario(SEED, total, 1, total / moving);
    let mut worlds = Worlds::default();
    worlds.replay(&workload);
    worlds
}

fn evidence_dir() -> Option<std::path::PathBuf> {
    std::env::var_os(EVIDENCE_DIR_ENV).map(Into::into)
}

fn fields(pairs: &[(&str, u64)]) -> String {
    pairs
        .iter()
        .map(|(name, value)| format!("\"{name}\":{value}"))
        .collect::<Vec<_>>()
        .join(",")
}

fn write_evidence(dir: &std::path::Path, operation: &str, samples: &[Sample]) {
    let samples = samples
        .iter()
        .map(|sample| {
            format!(
                "{{\"dimensions\":{{{}}},\"metrics\":{{{}}}}}",
                fields(&sample.dimensions),
                fields(&sample.metrics)
            )
        })
        .collect::<Vec<_>>()
        .join(",");
    std::fs::write(
        dir.join(format!("{operation}.json")),
        format!("{{\"id\":\"{operation}\",\"samples\":[{samples}]}}\n"),
    )
    .expect("work evidence directory must be writable");
}

fn value(pairs: &[(&str, u64)], name: &str) -> u64 {
    pairs
        .iter()
        .find(|(key, _)| *key == name)
        .map_or_else(|| panic!("missing {name}"), |(_, value)| *value)
}

/// Applies the same budget and growth semantics as `coding-tooling performance work`.
fn assert_bounds(operation: &str, samples: &[Sample], bounds: &[Bound]) {
    for bound in bounds {
        let metric = bound.metric;
        for sample in samples {
            let observed = value(&sample.metrics, metric);
            if let Some(max) = bound.budget_max {
                assert!(
                    observed <= max,
                    "{operation}.{metric} = {observed} exceeds budget {max} at {:?}",
                    sample.dimensions
                );
            }
        }
        for &(dimension, growth) in bound.growth {
            // Group by every other dimension, then order each group by the bounded one.
            let mut groups: BTreeMap<GroupKey, Vec<(u64, u64)>> = BTreeMap::new();
            for sample in samples {
                let key = sample
                    .dimensions
                    .iter()
                    .copied()
                    .filter(|(name, _)| *name != dimension)
                    .collect();
                groups.entry(key).or_default().push((
                    value(&sample.dimensions, dimension),
                    value(&sample.metrics, metric),
                ));
            }
            for (key, mut points) in groups {
                points.sort_unstable();
                let context = format!("{operation}.{metric} {growth:?} in {dimension} at {key:?}");
                check_growth(&context, growth, &points);
            }
        }
    }
}

fn check_growth(context: &str, growth: Growth, points: &[(u64, u64)]) {
    let points: Vec<(i128, i128)> = points
        .iter()
        .map(|&(x, v)| (i128::from(x), i128::from(v)))
        .collect();
    // A single point carries no growth information.
    let [(x0, v0), ..] = points[..] else {
        return;
    };
    if points.len() < 2 {
        return;
    }
    match growth {
        Growth::Constant => {
            for &(x, v) in &points[1..] {
                assert!(v <= v0, "{context}: {v} at {x} > {v0} at {x0}");
            }
        }
        Growth::Linear if points.len() == 2 => {
            let (x1, v1) = points[1];
            assert!(
                v1 * x0 <= v0 * x1,
                "{context}: {v0} at {x0} -> {v1} at {x1}"
            );
        }
        Growth::Linear => {
            let (x1, v1) = points[1];
            let rate = (v1 - v0).max(0);
            for window in points[1..].windows(2) {
                let [(xa, va), (xb, vb)] = [window[0], window[1]];
                assert!(
                    (vb - va) * (x1 - x0) <= rate * (xb - xa),
                    "{context}: marginal work {va} at {xa} -> {vb} at {xb} exceeds first rate {v0} at {x0} -> {v1} at {x1}"
                );
            }
        }
    }
}

fn finish(operation: &str, samples: &[Sample], bounds: &[Bound]) {
    for sample in samples {
        let mut declared: Vec<&str> = sample.metrics.iter().map(|(name, _)| *name).collect();
        let mut bounded: Vec<&str> = bounds.iter().map(|bound| bound.metric).collect();
        declared.sort_unstable();
        bounded.sort_unstable();
        assert_eq!(declared, bounded, "{operation} metrics and bounds diverge");
    }
    if let Some(dir) = evidence_dir() {
        write_evidence(&dir, operation, samples);
    } else {
        assert_bounds(operation, samples, bounds);
    }
}

const CONSTANT_N_LINEAR_M: &[(&str, Growth)] = &[
    ("total_entities", Growth::Constant),
    ("moving_entities", Growth::Linear),
];

/// `(total_entities, moving_entities)`: N varies with M fixed, then M varies with N fixed.
const SPARSE_SUBSET_POINTS: [(u32, u32); 5] = [
    (1_000, 1_000),
    (10_000, 1_000),
    (100_000, 1_000),
    (100_000, 10_000),
    (100_000, 100_000),
];

#[test]
fn integrate_sparse_subset_scales_with_moving_entities() {
    let operation = Operation::Integrate { ticks: 3 };
    let samples: Vec<Sample> = SPARSE_SUBSET_POINTS
        .into_iter()
        .map(|(total, moving)| {
            let mut worlds = sparse_motion_worlds(total, moving);
            let mut work = Work::default();
            worlds.apply(operation, &mut work);
            let context = format!("total={total} moving={moving}");
            worlds.assert_snapshot_parity(&context);
            let integrated = work.assert_equal_useful_work(&context);
            assert_eq!(integrated, u64::from(moving), "{context}");
            Sample {
                dimensions: vec![
                    ("total_entities", u64::from(total)),
                    ("moving_entities", u64::from(moving)),
                ],
                metrics: vec![
                    ("integrated_entities", integrated),
                    (
                        "reference_integration_rows_scanned",
                        work.reference.integration_rows_scanned,
                    ),
                    (
                        "sparse_set_integration_rows_scanned",
                        work.sparse.integration_rows_scanned,
                    ),
                    (
                        "sparse_set_component_lookups",
                        work.sparse.component_lookups,
                    ),
                    (
                        "cached_sparse_integration_rows_scanned",
                        work.cached.integration_rows_scanned,
                    ),
                    (
                        "cached_sparse_component_lookups",
                        work.cached.component_lookups,
                    ),
                    (
                        "archetype_integration_rows_scanned",
                        work.archetype.integration_rows_scanned,
                    ),
                    (
                        "archetype_component_lookups",
                        work.archetype.component_lookups,
                    ),
                ],
            }
        })
        .collect();

    finish(
        "integrate-sparse-subset",
        &samples,
        &[
            Bound {
                metric: "integrated_entities",
                budget_max: None,
                growth: CONSTANT_N_LINEAR_M,
            },
            Bound {
                metric: "reference_integration_rows_scanned",
                budget_max: None,
                growth: &[
                    ("total_entities", Growth::Linear),
                    ("moving_entities", Growth::Constant),
                ],
            },
            Bound {
                metric: "sparse_set_integration_rows_scanned",
                budget_max: None,
                growth: CONSTANT_N_LINEAR_M,
            },
            Bound {
                metric: "sparse_set_component_lookups",
                budget_max: None,
                growth: CONSTANT_N_LINEAR_M,
            },
            Bound {
                metric: "cached_sparse_integration_rows_scanned",
                budget_max: None,
                growth: CONSTANT_N_LINEAR_M,
            },
            Bound {
                metric: "cached_sparse_component_lookups",
                budget_max: Some(0),
                growth: &[],
            },
            Bound {
                metric: "archetype_integration_rows_scanned",
                budget_max: None,
                growth: CONSTANT_N_LINEAR_M,
            },
            Bound {
                metric: "archetype_component_lookups",
                budget_max: Some(0),
                growth: &[],
            },
        ],
    );
}

/// `(total_entities, matching_entities, repetitions)`.
const REPEAT_QUERY_POINTS: [(u32, u32, u32); 7] = [
    (1_000, 1_000, 4),
    (10_000, 1_000, 4),
    (100_000, 1_000, 4),
    (100_000, 10_000, 4),
    (100_000, 100_000, 4),
    (100_000, 1_000, 1),
    (100_000, 1_000, 16),
];

fn zero_constant_in_n(metrics: &[&'static str]) -> Vec<Bound> {
    metrics
        .iter()
        .map(|&metric| Bound {
            metric,
            budget_max: Some(0),
            growth: &[("total_entities", Growth::Constant)],
        })
        .collect()
}

const REPEAT_ROW_GROWTH: &[(&str, Growth)] = &[
    ("total_entities", Growth::Constant),
    ("matching_entities", Growth::Linear),
    ("repetitions", Growth::Linear),
];

#[test]
fn repeated_cached_query_scales_with_returned_rows() {
    let samples: Vec<Sample> = REPEAT_QUERY_POINTS
        .into_iter()
        .map(|(total, matching, repetitions)| {
            // The fixture's own integration is the warm-up; only the repeats are measured.
            let mut worlds = sparse_motion_worlds(total, matching);
            let mut work = Work::default();
            for round in 0..repetitions {
                let ticks = i32::try_from(round % 5 + 1).expect("small tick count");
                worlds.apply(Operation::Integrate { ticks }, &mut work);
            }
            let context = format!("total={total} matching={matching} repetitions={repetitions}");
            worlds.assert_snapshot_parity(&context);
            let rows_returned = work.assert_equal_useful_work(&context);
            assert_eq!(
                rows_returned,
                u64::from(matching) * u64::from(repetitions),
                "{context}"
            );
            Sample {
                dimensions: vec![
                    ("total_entities", u64::from(total)),
                    ("matching_entities", u64::from(matching)),
                    ("repetitions", u64::from(repetitions)),
                ],
                metrics: vec![
                    ("rows_returned", rows_returned),
                    (
                        "cached_sparse_integration_rows_scanned",
                        work.cached.integration_rows_scanned,
                    ),
                    (
                        "cached_sparse_query_cache_updates",
                        work.cached.query_cache_updates,
                    ),
                    (
                        "archetype_integration_rows_scanned",
                        work.archetype.integration_rows_scanned,
                    ),
                    (
                        "archetype_structural_table_transitions",
                        work.archetype.structural_table_transitions,
                    ),
                    (
                        "cached_sparse_query_rows_inspected",
                        work.cached.query_rows_inspected,
                    ),
                    (
                        "archetype_entity_locations_inspected",
                        work.archetype.entity_locations_inspected,
                    ),
                    (
                        "cached_sparse_component_slots_touched",
                        work.cached.component_slots_touched,
                    ),
                    (
                        "cached_sparse_entity_entries_touched",
                        work.cached.entity_locations_inspected,
                    ),
                    (
                        "archetype_table_rows_touched",
                        work.archetype.table_rows_touched,
                    ),
                ],
            }
        })
        .collect();

    let mut bounds = vec![
        Bound {
            metric: "rows_returned",
            budget_max: None,
            growth: REPEAT_ROW_GROWTH,
        },
        Bound {
            metric: "cached_sparse_integration_rows_scanned",
            budget_max: None,
            growth: REPEAT_ROW_GROWTH,
        },
        Bound {
            metric: "archetype_integration_rows_scanned",
            budget_max: None,
            growth: REPEAT_ROW_GROWTH,
        },
    ];
    // No structural maintenance at all while the query only repeats.
    bounds.extend(zero_constant_in_n(&[
        "cached_sparse_query_cache_updates",
        "archetype_structural_table_transitions",
        "cached_sparse_query_rows_inspected",
        "archetype_entity_locations_inspected",
        "cached_sparse_component_slots_touched",
        "cached_sparse_entity_entries_touched",
        "archetype_table_rows_touched",
    ]));
    finish("repeat-cached-query", &samples, &bounds);
}

/// `(total_entities, changed_entities)`: unrelated population varies with one changed entity,
/// then the number of changed entities varies with the population fixed.
const CHURN_POINTS: [(u32, u32); 5] = [
    (1_000, 1),
    (10_000, 1),
    (100_000, 1),
    (100_000, 16),
    (100_000, 256),
];

const CONSTANT_N_LINEAR_C: &[(&str, Growth)] = &[
    ("total_entities", Growth::Constant),
    ("changed_entities", Growth::Linear),
];

/// Every structural-path counter, per storage, with its analytic per-changed-entity ceiling.
///
/// The ceilings follow from the instrumented primitives: a sparse-set insert touches at most
/// three slots and a removal at most four, a query refresh at most five sparse-index entries and
/// rows, an archetype move one location read plus at most two writes and at most three table rows.
fn structural_metrics(work: &Work) -> Vec<(&'static str, u64)> {
    vec![
        (
            "reference_entity_entries_touched",
            work.reference.entity_locations_inspected,
        ),
        (
            "sparse_set_entity_entries_touched",
            work.sparse.entity_locations_inspected,
        ),
        (
            "sparse_set_component_slots_touched",
            work.sparse.component_slots_touched,
        ),
        (
            "cached_sparse_entity_entries_touched",
            work.cached.entity_locations_inspected,
        ),
        (
            "cached_sparse_component_slots_touched",
            work.cached.component_slots_touched,
        ),
        (
            "cached_sparse_query_rows_inspected",
            work.cached.query_rows_inspected,
        ),
        (
            "cached_sparse_query_cache_updates",
            work.cached.query_cache_updates,
        ),
        (
            "archetype_entity_locations_inspected",
            work.archetype.entity_locations_inspected,
        ),
        (
            "archetype_table_rows_touched",
            work.archetype.table_rows_touched,
        ),
        (
            "archetype_structural_table_transitions",
            work.archetype.structural_table_transitions,
        ),
    ]
}

/// Bounds for the raw counters (constant in N, linear in C) and for their per-change
/// normalization `ceil(observed / C)`, whose budget is the analytic per-change ceiling. Because
/// `ceil(observed / C) <= ceiling` exactly when `observed <= ceiling * C`, the contract enforces
/// the per-change ceiling at every sample, not only at the largest C.
fn structural_bounds(per_change: [u64; 10]) -> Vec<Bound> {
    let names = structural_metrics(&Work::default());
    let raw = names.iter().map(|&(metric, _)| Bound {
        metric,
        budget_max: None,
        growth: CONSTANT_N_LINEAR_C,
    });
    let normalized = names
        .iter()
        .zip(per_change)
        .map(|(&(metric, _), ceiling)| Bound {
            metric: per_change_name(metric),
            budget_max: Some(ceiling),
            growth: &[("total_entities", Growth::Constant)],
        });
    raw.chain(normalized).collect()
}

/// Raw structural metric names paired with their per-change normalization, in
/// `structural_metrics` order.
const PER_CHANGE_NAMES: [(&str, &str); 10] = [
    (
        "reference_entity_entries_touched",
        "reference_entity_entries_touched_per_change",
    ),
    (
        "sparse_set_entity_entries_touched",
        "sparse_set_entity_entries_touched_per_change",
    ),
    (
        "sparse_set_component_slots_touched",
        "sparse_set_component_slots_touched_per_change",
    ),
    (
        "cached_sparse_entity_entries_touched",
        "cached_sparse_entity_entries_touched_per_change",
    ),
    (
        "cached_sparse_component_slots_touched",
        "cached_sparse_component_slots_touched_per_change",
    ),
    (
        "cached_sparse_query_rows_inspected",
        "cached_sparse_query_rows_inspected_per_change",
    ),
    (
        "cached_sparse_query_cache_updates",
        "cached_sparse_query_cache_updates_per_change",
    ),
    (
        "archetype_entity_locations_inspected",
        "archetype_entity_locations_inspected_per_change",
    ),
    (
        "archetype_table_rows_touched",
        "archetype_table_rows_touched_per_change",
    ),
    (
        "archetype_structural_table_transitions",
        "archetype_structural_table_transitions_per_change",
    ),
];

fn per_change_name(metric: &str) -> &'static str {
    PER_CHANGE_NAMES
        .iter()
        .find(|(raw, _)| *raw == metric)
        .map_or_else(
            || panic!("no per-change name for {metric}"),
            |(_, name)| *name,
        )
}

/// Asserts `observed <= ceiling * changed_entities` for every raw structural counter and sample.
fn assert_per_change_ceilings(operation: &str, samples: &[Sample], per_change: [u64; 10]) {
    for sample in samples {
        let changed = value(&sample.dimensions, "changed_entities");
        for (&(metric, observed), ceiling) in sample.metrics.iter().zip(per_change) {
            assert!(
                observed <= ceiling * changed,
                "{operation}.{metric} = {observed} exceeds {ceiling} per change at {:?}",
                sample.dimensions
            );
        }
    }
}

/// Runs one structural operation and records raw and per-change counters.
fn finish_structural(operation: &str, samples: &[Sample], per_change: [u64; 10]) {
    if evidence_dir().is_none() {
        assert_per_change_ceilings(operation, samples, per_change);
    }
    let samples: Vec<Sample> = samples
        .iter()
        .map(|sample| {
            let changed = value(&sample.dimensions, "changed_entities");
            let normalized: Vec<(&'static str, u64)> = sample
                .metrics
                .iter()
                .map(|&(metric, observed)| (per_change_name(metric), observed.div_ceil(changed)))
                .collect();
            Sample {
                dimensions: sample.dimensions.clone(),
                metrics: sample.metrics.iter().copied().chain(normalized).collect(),
            }
        })
        .collect();
    finish(operation, &samples, &structural_bounds(per_change));
}

/// Applies `operations(entity)` to C evenly spaced entities in a world where all N entities match
/// Position+Velocity, so a rescan of the unrelated population would be paid in full.
fn structural_samples(operations: impl Fn(EntityId) -> Vec<Operation>) -> Vec<Sample> {
    CHURN_POINTS
        .into_iter()
        .map(|(total, changed)| {
            let mut worlds = sparse_motion_worlds(total, total);
            let stride = total / changed;
            let mut work = Work::default();
            for index in 0..changed {
                for operation in operations(EntityId(index * stride + stride / 2)) {
                    worlds.apply(operation, &mut work);
                }
            }
            worlds.assert_snapshot_parity(&format!("total={total} changed={changed}"));
            Sample {
                dimensions: vec![
                    ("total_entities", u64::from(total)),
                    ("changed_entities", u64::from(changed)),
                ],
                metrics: structural_metrics(&work),
            }
        })
        .collect()
}

#[test]
fn single_entity_component_churn_does_not_scale_with_population() {
    let samples = structural_samples(|entity| {
        vec![
            Operation::RemoveVelocity(entity),
            Operation::SetVelocity(entity, Velocity::new(-3, 2)),
        ]
    });
    // Per changed entity: two alive/entry checks (reference, sparse, cached); a removal (4) and an
    // insert (3) of component slots, plus two sparse reads per query refresh (up to 3 refreshes);
    // query refreshes 5 (slot read, sparse remove, row swap_remove, moved row read and sparse
    // repair) + 2 (moved entity's slot and row) + 3 (slot read, row push, sparse write);
    // up to 3 row changes; two archetype moves of 3 locations and 3 table rows each.
    finish_structural(
        "add-remove-component",
        &samples,
        [2, 2, 7, 2, 13, 10, 3, 6, 6, 2],
    );
}

#[test]
fn single_entity_respawn_does_not_scale_with_population() {
    let samples = structural_samples(|entity| {
        vec![
            Operation::Despawn(entity),
            Operation::Spawn(entity),
            Operation::SetPosition(entity, Position::new(4, -4)),
            Operation::SetVelocity(entity, Velocity::new(1, 1)),
        ]
    });
    // Per changed entity: despawn removes two components (4 slots each) and refreshes the leaving
    // row (5) and up to two swap-moved rows (2 each); spawn and two inserts follow, refreshing a
    // non-matching slot (1) and then pushing the returning row (3). Archetype: despawn (3 locations, 2 rows), spawn
    // (2 locations, 1 row) and two moves (3 locations, 3 rows each).
    finish_structural(
        "respawn-entity",
        &samples,
        [5, 5, 14, 5, 24, 13, 4, 11, 9, 2],
    );
}

const SNAPSHOT_POINTS: [u32; 3] = [1_000, 10_000, 100_000];

#[test]
fn full_snapshot_is_declared_linear_in_entities() {
    let linear_n: &'static [(&str, Growth)] = &[("total_entities", Growth::Linear)];
    let samples: Vec<Sample> = SNAPSHOT_POINTS
        .into_iter()
        .map(|total| {
            let worlds = sparse_motion_worlds(total, total / 4);
            let expected = worlds.assert_snapshot_parity(&format!("total={total}"));
            let stats: [(&str, SnapshotWorkStats); 4] = [
                ("reference", worlds.reference.snapshot_with_stats().1),
                ("sparse_set", worlds.sparse.snapshot_with_stats().1),
                ("cached_sparse", worlds.cached.snapshot_with_stats().1),
                ("archetype", worlds.archetype.snapshot_with_stats().1),
            ];
            let entities = u64::try_from(expected.entities().len()).expect("entity count");
            for (storage, work) in stats {
                assert_eq!(work.entities_materialized, entities, "{storage} {total}");
            }
            let [reference, sparse, cached, archetype] = stats.map(|(_, work)| work);
            Sample {
                dimensions: vec![("total_entities", u64::from(total))],
                metrics: vec![
                    ("reference_snapshot_slots_scanned", reference.slots_scanned),
                    ("sparse_set_snapshot_slots_scanned", sparse.slots_scanned),
                    ("cached_sparse_snapshot_slots_scanned", cached.slots_scanned),
                    ("archetype_snapshot_slots_scanned", archetype.slots_scanned),
                    ("snapshot_entities_materialized", entities),
                ],
            }
        })
        .collect();

    finish(
        "full-snapshot",
        &samples,
        &[
            Bound {
                metric: "reference_snapshot_slots_scanned",
                budget_max: None,
                growth: linear_n,
            },
            Bound {
                metric: "sparse_set_snapshot_slots_scanned",
                budget_max: None,
                growth: linear_n,
            },
            Bound {
                metric: "cached_sparse_snapshot_slots_scanned",
                budget_max: None,
                growth: linear_n,
            },
            Bound {
                metric: "archetype_snapshot_slots_scanned",
                budget_max: None,
                growth: linear_n,
            },
            Bound {
                metric: "snapshot_entities_materialized",
                budget_max: None,
                growth: linear_n,
            },
        ],
    );
}

/// The v1 ratchet still reads the state-based `operation_work` predictions. On the existing
/// fixtures they must agree with the measured executed path, operation by operation.
#[test]
fn predicted_operation_work_matches_measured_work_on_v1_fixtures() {
    let workloads = [
        ("motion", Workload::motion_scenario(17, 256, 10)),
        (
            "mixed-motion",
            Workload::mixed_motion_scenario(17, 256, 8, 4),
        ),
        (
            "component-churn",
            Workload::component_churn_scenario(17, 128, 8),
        ),
    ];
    for (name, workload) in workloads {
        let mut worlds = Worlds::default();
        for (index, &operation) in workload.operations().iter().enumerate() {
            let context = format!("{name} operation {index}: {operation:?}");
            let predicted = [
                worlds.reference.operation_work(operation),
                worlds.sparse.operation_work(operation),
                worlds.cached.operation_work(operation),
                worlds.archetype.operation_work(operation),
            ];
            let measured = [
                worlds
                    .reference
                    .apply_measured(operation)
                    .expect("reference"),
                worlds.sparse.apply_measured(operation).expect("sparse-set"),
                worlds
                    .cached
                    .apply_measured(operation)
                    .expect("cached-sparse"),
                worlds
                    .archetype
                    .apply_measured(operation)
                    .expect("archetype"),
            ];
            // Inspection counters exist only on the measured path; predictions leave them 0.
            let measured = measured.map(|work| StorageWorkStats {
                query_rows_inspected: 0,
                entity_locations_inspected: 0,
                component_slots_touched: 0,
                table_rows_touched: 0,
                ..work
            });
            assert_eq!(measured, predicted, "{context}");
        }
        worlds.assert_snapshot_parity(name);
    }
}
