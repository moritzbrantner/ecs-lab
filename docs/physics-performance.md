# Physics performance techniques

ECS Lab treats performance work as an implementation concern, not permission to change physics semantics. The Rust solver remains authoritative, body pairs are still resolved in deterministic entity-id order, and faster paths must preserve the same contacts, support evidence, and ordinary ECS operations.

## 1. Cache derived collision geometry

The ECS stores integer positions and collider half extents because those values are deterministic and exact. The reusable geometry kernel consumes `f32` AABBs. Building that AABB is derived work: range validation, integer-to-float conversion, and min/max construction do not need to be repeated for every pair involving the same body.

The solver therefore builds each body's geometry once after motion and keeps it beside the mutable body state. A cached AABB is rebuilt only when positional collision correction actually moves the body.

Why this helps: an all-pairs loop touches the same body many times. With `n` bodies, rebuilding geometry inside every pair repeats equivalent work roughly `O(n²)` times. Caching changes that derived work toward `O(n + corrections)` while leaving pair resolution itself unchanged.

Why it is safe: the cache is private derived state. ECS position remains the source of truth, and every position mutation that can affect later collision tests refreshes the cached AABB before the next pair is examined. Range validation still fails closed when refreshed geometry cannot be represented exactly by the reusable collision kernel.

## 2. Reject separated axes before the general kernel

AABBs cannot overlap if their intervals are separated on even one axis. ECS Lab already has exact integer positions and half extents, so it can prove obvious separation cheaply before doing floating-point collision work.

For each deterministic pair, the solver now checks X first. If X is separated, it returns immediately. Only X survivors check Y. Only pairs that survive both integer axes call `geometry-kernels::aabb_aabb`.

Why this helps: sparse scenes usually contain many distant pairs. The cheapest possible outcome for those pairs is a couple of integer min/max operations and an early branch instead of conversions, AABB construction, and the reusable three-axis relation calculation.

Why it is safe: the integer test is only a conservative rejection. A negative interval overlap proves the boxes cannot touch. Zero overlap is not rejected, so touching pairs still reach `geometry-kernels`. Possible contacts therefore keep the canonical kernel's touching/overlap semantics.

## 3. Write back only actual ECS changes

The physics step emits ordinary ECS operations only when a dynamic body's position or velocity differs from its input snapshot. Fixed bodies never receive generated mutations.

Why this helps: collision detection can inspect many bodies without forcing equivalent component writes, storage churn, or browser-visible state updates afterward.

Why it is safe: this is an output compression technique. The resulting world state is identical to writing unchanged values back, while the mutation boundary stays the same ordinary `SetPosition` and `SetVelocity` operations used everywhere else.

## 4. Keep deterministic order while optimizing the hot path

The current solver deliberately keeps its ascending entity-id all-pairs order. Collision response mutates body positions as pairs are resolved, so a later pair can observe a correction made by an earlier pair.

This matters for broad-phase optimization. A sweep-and-prune or uniform-grid candidate set computed once before response could miss a contact that is created by an earlier correction in the same step. That would be faster but semantically different.

The next scalable broad-phase slice should therefore be parity-gated: candidate generation must be proven conservative for the solver's actual update model before it is allowed to prune authoritative pair work. The existing `rust-kernels` sweep/grid implementations are useful building blocks, but they do not become physics authority merely because they are faster.

## 5. Measure sparse and dense workloads separately

The benchmark suite intentionally contains both a sparse material fixture with zero contacts and a dense material fixture with real material response. These answer different questions:

- The sparse fixture exposes the cost of candidate-pair traversal and should benefit strongly from cheap axis rejection and cached derived geometry.
- The dense fixture exercises restitution, friction, mass, penetration correction, and cache refreshes, showing whether the optimization remains worthwhile when many pairs become contacts.

Benchmark timing is descriptive rather than a correctness gate. Fixture validity, deterministic replay, storage parity, and physics invariants remain blocking. Performance numbers only become useful after those conditions are satisfied.

## Techniques deliberately not added yet

**Sleeping** can skip large amounts of work, but it introduces new semantic state: sleep thresholds, wake propagation, and interactions with impulses or moving supports. It belongs in its own tested physics feature rather than being smuggled in as an optimization.

**Approximate or GPU-owned collision pruning** is also deferred. WebGPU remains post-physics evidence and must match the Rust-owned collision representation exactly before its results are trusted. An optimization is not allowed to become a second source of physics truth.
