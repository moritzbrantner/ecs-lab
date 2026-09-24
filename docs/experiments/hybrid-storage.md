# Hybrid per-component storage experiment

This experiment adds a frequently toggled auxiliary status component to otherwise stable
Position+Velocity motion data and compares three representations:

- pure archetype: status membership changes move the full motion row between tables;
- hybrid: motion rows remain contiguous and stable while status uses a paged sparse index;
- pure sparse: Position, Velocity, and status all use independent sparse membership.

The observable state includes the canonical Position/Velocity world snapshot plus sorted
status membership. All three representations must match after every churn/integration round.

The principal deterministic ratchet uses 1,024 entities, eight rounds, and 256 status
changes per round. It requires 2,048 full motion-row moves for the pure-archetype model and
zero motion-row moves for hybrid and sparse models. The hybrid retains contiguous,
lookup-free motion iteration; the pure sparse comparison exposes its per-row component
lookup cost. Sparse status footprint remains bounded by allocated 256-slot pages.

Optional timings are advisory:

```sh
cargo run --locked --release -p ecs-hybrid-storage --bin hybrid-storage-benchmark -- --bench
```
