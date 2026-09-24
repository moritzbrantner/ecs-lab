# Enableable component experiment

This experiment keeps Position and Velocity semantically present while changing whether an
entity participates in motion. It compares two representations:

- structural enabled/disabled buckets, where a toggle migrates the full row;
- a packed 64-bit enabled mask over stable component rows.

Disabling is intentionally not component deletion. Component values remain observable and
are preserved across disable/enable cycles.

The fixed 1,024-entity ratchet requires the mask path to scan exactly 16 words per
integration round, perform the same useful integration count as the structural path, and
perform zero structural migrations. The structural path records one row migration for each
effective toggle. Dense, sparse, empty-active-set and toggle-heavy regimes are covered.

This makes the trade-off explicit: masks add predictable filtering work even with no active
entities, while structural buckets keep the hot loop compact but pay data movement whenever
participation changes.

Optional timings are advisory:

```sh
cargo run --locked --release -p ecs-enableable --bin enableable-benchmark -- --bench
```
