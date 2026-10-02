# Generational entity handle experiment

The workload's `EntityId` remains logical identity. Physical storage addressing is modeled by
`EntityHandle { slot, generation }`, so recycling a slot never silently redefines the
logical entity contract.

Free slots are kept in an ordered set and the lowest slot is reused deterministically. A
despawn increments the slot generation before reuse. Old handles therefore fail resolution
after a slot is recycled.

Generation overflow is fail-closed: a slot already at `u32::MAX` is retired on despawn and
is never returned to the free set. The allocator uses a new slot instead of wrapping the
generation and risking stale-handle aliasing.

The reuse ratchet requires peak physical slots to remain equal to batch size across repeated
spawn/despawn cycles, with every post-first-cycle spawn coming from the free set and every
previous-cycle handle rejected as stale.

Optional timings are advisory:

```sh
cargo run --locked --release -p ecs-generational --bin generational-benchmark -- --bench
```
