# AABB physics workload

## Purpose

Use a small physics loop as a realistic deterministic ECS workload without turning `ecs-lab` into a game engine or physics framework.

The core remains deliberately 2D and AABB-only:

1. dynamic bodies receive integer gravity;
2. semi-implicit Euler integration advances velocity and position;
3. body pairs are visited in ascending `EntityId` order;
4. `geometry-kernels::aabb_aabb` owns the contact decision;
5. ECS Lab applies minimum-axis positional correction and a fully inelastic equal-mass normal impulse;
6. the step returns ordinary `ecs-workload::Operation` values.

This keeps the physics result storage-independent. Both the reference ECS and sparse-set candidate consume the same generated operations without a physics-specific storage trait.

## Cross-repository boundary

Collision Lab consumes reusable collision mechanisms from `rust-kernels`. ECS Lab follows that same ownership boundary rather than depending on the `collision-lab` application crate.

The current pin is:

- `geometry-kernels` / `spatial-kernels`
- `rust-kernels` revision `986bff4dc8d13a64b90fab3a9f7f02bb8d1aa35e`

The pin matches the collision-kernel revision used by Collision Lab for its analytical AABB/OBB teaching paths.

## Determinism contract

- Body configuration order does not affect the result; bodies are sorted by entity id.
- Pair resolution is single-pass and ordered, so solver order is explicit rather than scheduler-dependent.
- ECS positions and velocities remain integer values.
- Geometry conversion is allowed only while AABB bounds remain exactly representable as `f32` integers.
- Touching counts as contact because that is the reusable geometry-kernel contract.
- Fixed bodies never receive generated component operations.

## Falling-box scenario

`ecs-physics-scenarios` owns the named `falling-boxes` fixture rather than adding scenario policy to the solver crate. It creates a deterministic 16-column field of dynamic boxes above one fixed floor and exposes the setup workload, body configuration, and one-step physics operation.

A differential test replays the setup and every generated physics step through `ReferenceWorld` and `SparseWorld`. Their observable snapshots and generated `PhysicsStep` values must remain identical frame by frame.

The benchmark runner now measures both storage implementations under the same named physics scenario:

- smoke: 96 dynamic boxes + floor, 12 frames, 2 repetitions;
- full: 512 dynamic boxes + floor, 40 frames, 3 repetitions.

As with the existing `motion` benchmark, elapsed time is descriptive evidence rather than a merge threshold and is useful for comparison only with a verified environment fingerprint.

## Optional WebGPU compute

The scenario crate can advance the Rust reference world to a named frame and then produce a canonical AABB list plus triangular collision-pair bitset using `geometry-kernels::aabb_aabb`.

The Pages demo exposes one cached 96-box, six-frame fixture through WebAssembly. When WebGPU is enabled, JavaScript packs those Rust-returned AABBs into the same eight-float layout used by Collision Lab's naive WebGPU path and runs an all-pairs WGSL compute shader. The GPU writes the triangular pair set into an atomic `u32` bitset.

The browser then compares every GPU bitset word with the Rust/CPU bitset. Only an exact match makes the GPU timing eligible for display. A mismatch, unavailable adapter, shader failure, or disabled WebGPU leaves Rust/CPU authoritative and suppresses performance evidence.

This is intentionally a compute-evidence seam, not a second physics solver. Contact response and ECS mutations remain on the deterministic Rust path. Feeding a GPU broad-phase candidate set into the solver should be a later experiment only after a conservative candidate contract can preserve exact solver semantics.

## Evidence

`PhysicsStepStats` exposes body count, naive candidate-pair count, contacts, and resolved contacts. `BroadPhaseFrame` separately exposes the post-physics AABB set, exact overlap count, and canonical pair words used for CPU↔WebGPU parity.

## Follow-up horizon

Useful next slices remain separate:

- compare the falling-box benchmark against an archetype/table candidate;
- introduce a pinned spatial broad-phase kernel and require exact pair parity before benchmarking it;
- evaluate whether a conservative WebGPU candidate list can safely feed deterministic CPU response;
- add configurable restitution/friction and solver iterations only after the current baseline is measured;
- consider circles and then 3D only after the AABB workload has useful ECS evidence.
