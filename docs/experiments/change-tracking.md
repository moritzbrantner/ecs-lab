# Change tracking experiment

This experiment compares a complete Position-derived projection rebuild with an explicit
dirty-entity workset. The derived value depends only on Position; advancing simulation time
is deliberately absent so an unchanged component is genuinely sufficient to skip work.

The fixed ratchet covers 1,024 entities, eight mutation/checkpoint rounds, and changed
populations of 0, 1, 16, 256, and 1,024 entities per round. Every checkpoint rebuilds the
full reference projection and incrementally updates only dirty entity IDs, then requires
exact projection equality.

For a fixed scenario the deterministic work contract is:

- full scan rows = entities × (rounds + 1);
- incremental rows = initial entities + changed-per-round × rounds;
- equal-value writes do not dirty a component;
- removal visits one dirty ID and removes exactly the corresponding derived row.

The counters describe logical ECS work, not allocator bytes or CPU instructions. Timing is
optional and advisory:

```sh
cargo run --locked --release -p ecs-change-tracking --bin change-tracking-benchmark
cargo run --locked --release -p ecs-change-tracking --bin change-tracking-benchmark -- --bench
```
