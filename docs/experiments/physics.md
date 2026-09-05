# AABB physics workload

## Purpose

Use a small physics loop as a realistic deterministic ECS workload without turning `ecs-lab` into a game engine or physics framework.

The core remains deliberately 2D and AABB-only. The current solver:

1. applies integer gravity to dynamic bodies;
2. advances velocity and position with semi-implicit Euler integration;
3. visits body pairs in ascending `EntityId` order;
4. delegates AABB contact decisions to `geometry-kernels::aabb_aabb`;
5. applies deterministic minimum-axis positional correction;
6. applies integer mass, restitution, and contact-friction response;
7. exposes ordered contact normals, penetration, and supporting-body evidence; and
8. returns ordinary `ecs-workload::Operation` values for state mutation.

This keeps the physics result storage-independent. Both the reference ECS and sparse-set candidate consume the same generated operations without a physics-specific storage trait.

## Cross-repository boundary

Collision Lab consumes reusable collision mechanisms from `rust-kernels`. ECS Lab follows that same ownership boundary rather than depending on the `collision-lab` application crate.

The current pin is:

- `geometry-kernels` / `spatial-kernels`
- `rust-kernels` revision `986bff4dc8d13a64b90fab3a9f7f02bb8d1aa35e`

The pin matches the collision-kernel revision used by Collision Lab for its analytical AABB/OBB teaching paths.

## Deterministic material contract

`PhysicsBody::dynamic` still defaults to mass `1`, restitution `0`, and friction `0`. `PhysicsBody::fixed` remains immovable. Those defaults preserve the original equal-mass, fully inelastic, frictionless behavior for existing workloads.

Material coefficients are integer thousandths:

- `0` means no restitution/friction;
- `1000` means full restitution/friction;
- values outside `0..=1000` are rejected before simulation.

Contact restitution uses the lower of the two body coefficients. Contact friction uses the higher coefficient. Dynamic-vs-dynamic normal response uses the ordinary one-dimensional mass/restitution equation evaluated with integer rational arithmetic; division truncation is explicit and deterministic. Dynamic-vs-fixed response reflects the normal velocity by restitution. Tangential contact friction deterministically damps velocity toward the contact pair's mass-weighted common tangential velocity.

Dynamic bodies must have positive integer `mass_units`. Mass also controls dynamic-vs-dynamic positional correction: the lighter body receives the larger correction share. Fixed-body mass is ignored.

This is a controlled deterministic contact model for ECS experiments, not a claim of production-grade rigid-body fidelity.

## Contact and support evidence

Each `PhysicsStep` exposes its ordered `PhysicsContact` values in the same stable pair order used by the solver. A contact records:

- left and right entity ids;
- an axis-aligned normal pointing from the left body toward the right body; and
- integer penetration, where touching is `0` because touching is part of the reusable geometry-kernel contact contract.

The step also exposes a sorted set of supported dynamic entities. A dynamic body is supported when a vertical contact places the other body below it. This evidence is intentionally solver output rather than a `Grounded` ECS component: later character-controller experiments can consume it without teaching the collision solver what a character or jump is.

## Determinism contract

- Body configuration order does not affect the result; bodies are sorted by entity id.
- Pair resolution is single-pass and ordered, so solver order is explicit rather than scheduler-dependent.
- ECS positions and velocities remain integer values.
- Mass and material coefficients are integer values with explicit rational division semantics.
- Geometry conversion is allowed only while AABB bounds remain exactly representable as `f32` integers.
- Touching counts as contact because that is the reusable geometry-kernel contract.
- Fixed bodies never receive generated component operations.
- Contact and support evidence is derived only from the canonical Rust solver path.

## Falling-box scenario

`ecs-physics-scenarios` owns the named `falling-boxes` fixture rather than adding scenario policy to the solver crate. It creates a deterministic 16-column field of dynamic boxes above one fixed floor and exposes the setup workload, body configuration, and one-step physics operation.

The defaults mean this existing scenario retains the original inelastic/frictionless response while the material/contact foundation is introduced.

A differential test replays the setup and every generated physics step through `ReferenceWorld` and `SparseWorld`. Their observable snapshots and generated `PhysicsStep` values must remain identical frame by frame.

The benchmark runner measures both storage implementations under the same named physics scenario:

- smoke: 96 dynamic boxes + floor, 12 frames, 2 repetitions;
- full: 512 dynamic boxes + floor, 40 frames, 3 repetitions.

As with the existing `motion` benchmark, elapsed time is descriptive evidence rather than a merge threshold and is useful for comparison only with a verified environment fingerprint.

## Optional WebGPU compute

The scenario crate can advance the Rust reference world to a named frame and then produce a canonical AABB list plus triangular collision-pair bitset using `geometry-kernels::aabb_aabb`.

The Pages demo exposes one cached 96-box, six-frame fixture through WebAssembly. When WebGPU is enabled, JavaScript packs those Rust-returned AABBs into the same eight-float layout used by Collision Lab's naive WebGPU path and runs an all-pairs WGSL compute shader. The GPU writes the triangular pair set into an atomic `u32` bitset.

The browser then compares every GPU bitset word with the Rust/CPU bitset. Only an exact match makes the GPU timing eligible for display. A mismatch, unavailable adapter, shader failure, or disabled WebGPU leaves Rust/CPU authoritative and suppresses performance evidence.

This remains a compute-evidence seam, not a second physics solver. Contact response, material behavior, contact/support evidence, and ECS mutations remain on the deterministic Rust path.

## Evidence

`PhysicsStepStats` exposes body count, naive candidate-pair count, contacts, and resolved contacts. `PhysicsStep` additionally exposes ordered contacts and supported entities. `BroadPhaseFrame` separately exposes the post-physics AABB set, exact overlap count, and canonical pair words used for CPU↔WebGPU parity.

The material/contact foundation must retain regression coverage for:

- legacy default response;
- restitution against fixed geometry;
- contact friction;
- mass-weighted dynamic response;
- touching/contact normal and support semantics;
- malformed mass/material rejection; and
- ordinary ECS-operation replay.

## Epic #10 follow-up

The solver foundation deliberately lands before the interactive material scenario. The remaining work in Epic #10 is:

- add the named `bouncing-room` scenario with contrasting restitution/friction bodies;
- replay that scenario through both `ReferenceWorld` and `SparseWorld`;
- expose editable material parameters through the Rust/Wasm Pages boundary; and
- add benchmark/evidence only where it helps compare ECS/storage behavior.

Solver iterations are not being added pre-emptively. Add a bounded explicit iteration count only if the measured `bouncing-room` scenario demonstrates a correctness/stability need.

## Later horizon

Keep later roadmap slices separate:

- character controller and jumping consume support evidence without entering the core solver;
- broad-phase and collider-family expansion must preserve exact canonical narrow-phase evidence;
- destruction consumes contact/impact evidence as ECS lifecycle policy;
- liquid interaction remains separate from rigid contact semantics;
- advanced fluid state should use solver-owned packed buffers rather than ordinary ECS entities per particle/cell.
