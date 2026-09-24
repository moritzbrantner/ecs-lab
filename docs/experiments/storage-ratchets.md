# Storage benchmark and ratchet contract

The fixed `storage-benchmark` matrix extends the existing motion/physics benchmark suite;
it does not replace its scenarios, physics authority, or observable-state contract.
It compares reference, sparse-set, cached-sparse and archetype-table storage through
concrete replay functions, not a shared hot-path dispatch trait.

## Fixtures

The version-one matrix contains nine fixtures and 36 implementation rows:

- Dense motion at 128 and 1,024 entities, each with eight integration rounds.
- 1,024 entities with one quarter, one sixty-fourth, or none participating in motion.
- An empty world with eight integration operations.
- 128 entities with eight rounds of component removal/reinsertion.
- Six distant IDs across four 256-slot pages, including `u32::MAX`, with non-tail
  component removals that force dense-index and query-row repairs.
- The same distant-ID workload followed by non-tail despawns and an empty integration.

Each fixture proves canonical final-snapshot parity against the reference model and
independently asserts its useful integration count and final live-entity count before
publishing evidence or timings. Native tests repeat the matrix to check deterministic
results and compare every intermediate distant-ID snapshot, including two complete
reclaim/reuse cycles in the same worlds.

## Blocking evidence

```sh
bash scripts/storage-ratchet.sh
```

The gate runs the Python checker tests, then one untimed native evidence pass. It checks
all 36 rows against `.performance/storage-ratchet.tsv`. Useful work, operation count,
live entities and materialized snapshot entities must match exactly. Other columns are
upper bounds: integration scans, integration component lookups, table transitions,
query-cache updates, snapshot scans, entity index entries, component/query index slots,
query rows and peak component/query slots sampled after each operation.

Counters use the existing storage probes. They describe logical data-structure work and
allocated page slots, **not allocator bytes, retained Vec capacity, CPU instructions or
measured allocation counts**. Changes to storage paths must keep their probes truthful.
These ratchets cannot detect unreported implementation work. Timing is complementary
evidence, never a substitute for parity or a deterministic PR blocker.

Empty and no-match cases preserve zero-work ceilings where the backend can skip work.
Distant-ID peak budgets protect against numeric-ID high-water indexing; terminal zero
budgets protect page reclamation. The matrix exposes the cached-query maintenance and
index footprint trade-off rather than treating low iteration work as a universal win.

Malformed counters, unknown/missing fields, duplicate/missing/extra rows, and empty or
partial output fail closed. The shell pipeline also fails when the native process fails.
Advisory timing records are excluded from comparison, including arbitrarily slow ones.

## One-way budget changes

CI passes its base revision through `BASE_SHA`. The gate compares the committed budget
with that revision: existing rows cannot disappear, exact fixture invariants cannot
change, and ceilings cannot increase. New cases may be added. An absent budget at the
base revision is reported as an explicit first introduction, not fabricated evidence.
An invalid/missing base commit is an error. With no `BASE_SHA`, local runs still enforce
the current committed budgets, but do not check their history.

To retain a proven improvement explicitly:

```sh
cargo run --locked --release -p ecs-runner --bin storage-benchmark > /tmp/storage-evidence.txt
python3 scripts/storage_ratchet.py \
  --baseline .performance/storage-ratchet.tsv \
  --evidence /tmp/storage-evidence.txt --tighten
```

`--tighten` first requires complete passing evidence, then atomically writes lower
observed ceilings. It never learns a regression or weakens useful work. Review and
commit the resulting diff; ordinary CI never rewrites the budget. A deliberate change
of fixture semantics or counter meaning needs an explicit contract migration, not a
larger number smuggled into an optimization PR.

## Advisory timing

```sh
bash scripts/benchmark.sh storage
```

The storage command performs the same preflight, then one warm-up and seven samples per
fixture/backend. It reports median, minimum and maximum nanoseconds. Fixture generation,
work probes and parity assertions are outside timing; world construction, replay, final
canonical snapshot, and teardown are included consistently. These are end-to-end storage
journeys, not isolated integration-loop timings. Compare runs in the same declared
environment; do not infer portable speedups from a single hosted runner.

The existing `smoke` and `full` benchmark commands remain unchanged. The blocking ratchet
uses the existing Validate job and Cargo build cache; it adds no profiler/environment
verification lane and no hardware-dependent threshold.
