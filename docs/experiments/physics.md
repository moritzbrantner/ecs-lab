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

Contact restitution uses the higher of the two body coefficients so a bouncy body can bounce against an ordinary non-bouncy surface. Contact friction currently uses the higher coefficient. Dynamic-vs-dynamic normal response uses the ordinary one-dimensional mass/restitution equation evaluated with integer rational arithmetic; division truncation is explicit and deterministic. Dynamic-vs-fixed response reflects the normal velocity by restitution. Tangential contact friction deterministically damps velocity toward the contact pair's mass-weighted common tangential velocity.

Dynamic bodies must have positive integer `mass_units`. Mass also controls dynamic-vs-dynamic positional correction: the lighter body receives the larger correction share. When integer division leaves a one-unit remainder, that remainder is assigned to the lighter body; equal masses retain the stable entity-order bias of the original solver. Fixed-body mass is ignored.

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

## Bouncing-room material scenario

`BouncingRoomScenario` adds a deliberately small material fixture above one fixed floor. Three dynamic bodies use contrasting mass, restitution, and friction configurations: a light fully bouncy/frictionless body, a medium mixed-material body, and a heavier fully inelastic/high-friction body.

The scenario keeps material behavior outside storage policy. The same setup and every generated `PhysicsStep` are replayed through `ReferenceWorld` and `SparseWorld`; their snapshots and step evidence must remain identical frame by frame. Focused regression evidence also verifies that the bouncy body rebounds after floor impact while the maximum-friction inelastic body loses its tangential motion.

The interactive Pages workbench uses the same Rust physics path rather than reimplementing response in JavaScript. Each editable dynamic entity can set:

- mass units;
- restitution from `0` to `1000`;
- friction from `0` to `1000`;
- position, velocity, and AABB half extent.

JavaScript transports those inputs into WebAssembly and draws the resulting positions. Rust rebuilds the `ReferenceWorld`, advances deterministic physics steps, and returns the canonical final frame. The workbench currently uses zero gravity so side-by-side material collisions remain easy to inspect; the named `bouncing-room` scenario retains gravity and a floor as the canonical falling/bouncing fixture.

No new benchmark was added for this small material scenario because it would not yet provide useful storage evidence beyond the existing falling-box benchmark. Performance remains descriptive, and solver iterations are still deferred until a measured scenario demonstrates a stability need.

## Optional WebGPU compute

The scenario crate can advance the Rust reference world to a named frame and then produce a canonical AABB list plus triangular collision-pair bitset using `geometry-kernels::aabb_aabb`.

The Pages demo exposes one cached 96-box, six-frame fixture through WebAssembly. When WebGPU is enabled, JavaScript packs those Rust-returned AABBs into the same eight-float layout used by Collision Lab's naive WebGPU path and runs an all-pairs WGSL compute shader. The GPU writes the triangular pair set into an atomic `u32` bitset.

The browser then compares every GPU bitset word with the Rust/CPU bitset. Only an exact match makes the GPU timing eligible for display. A mismatch, unavailable adapter, shader failure, or disabled WebGPU leaves Rust/CPU authoritative and suppresses performance evidence.

This remains a compute-evidence seam, not a second physics solver. Contact response, material behavior, contact/support evidence, and ECS mutations remain on the deterministic Rust path. In the interactive material workbench WebGPU receives only the Rust-evaluated final AABBs and verifies their pair bitset; it never participates in contact response.

## Evidence

`PhysicsStepStats` exposes body count, naive candidate-pair count, contacts, and resolved contacts. `PhysicsStep` additionally exposes ordered contacts and supported entities. `BroadPhaseFrame` separately exposes the post-physics AABB set, exact overlap count, and canonical pair words used for CPU↔WebGPU parity.

The material/contact foundation retains regression coverage for:

- legacy default response;
- restitution against ordinary fixed geometry;
- contact friction;
- mass-weighted dynamic response and penetration correction;
- touching/contact normal and support semantics;
- duplicate/malformed body configuration rejection;
- ordinary ECS-operation replay;
- `bouncing-room` replay parity across both ECS storage implementations; and
- Rust-owned interactive material response through the Wasm boundary.

## Epic #10 completion boundary

Epic #10's intended implementation is split into two reviewable slices: the deterministic solver/material/contact foundation and the stacked `bouncing-room` + interactive material demonstration. The epic is ready for integration once both slices are green on their exact heads and the stacked slice remains clean after rebasing onto `main` following the foundation merge.

Do not pull character-controller behavior into this boundary. Ground/support evidence exists specifically so Epic #11 can implement jumping and movement above the solver rather than adding character semantics to collision response.

## Later horizon

Keep later roadmap slices separate:

- character controller and jumping consume support evidence without entering the core solver;
- broad-phase and collider-family expansion must preserve exact canonical narrow-phase evidence;
- destruction consumes contact/impact evidence as ECS lifecycle policy;
- liquid interaction remains separate from rigid contact semantics;
- advanced fluid state should use solver-owned packed buffers rather than ordinary ECS entities per particle/cell.
