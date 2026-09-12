# Deterministic physics workloads

## Purpose

Use physics as a realistic deterministic ECS workload without turning `ecs-lab` into a general-purpose game engine. The repository keeps three deliberately distinct surfaces:

- `ecs-physics`: the existing 2D AABB solver retained as the compatibility, regression, and benchmark foundation;
- the continuous AABB work in `ecs-physics-3d`: an ECS Lab reference/experiment for three-axis translational CCD and simultaneous AABB contacts;
- the pinned standalone `physics-engine`: the reusable authority for rotating rigid-box world stepping used by the committed 3D playground and trebuchet tower.

All ECS Lab surfaces consume observable ECS-shaped state rather than owning storage. Rendering and browser interaction remain consumer concerns.

## Three-axis ECS state

`ecs-workload::Position` and `Velocity` carry X, Y, and Z. The existing two-argument constructors remain source-compatible and set Z to zero, while `Position::new3(...)` and `Velocity::new3(...)` construct explicit three-dimensional values.

`ReferenceWorld` and `SparseWorld` both integrate all three axes. This keeps storage parity as the same fundamental contract whether a workload stays on the legacy Z=0 plane or moves through depth.

## Cross-repository physics boundaries

Reusable low-level AABB and spatial decisions remain in pinned `rust-kernels` crates:

- `geometry-kernels` / `spatial-kernels`
- `rust-kernels` revision `986bff4dc8d13a64b90fab3a9f7f02bb8d1aa35e`

Reusable rotating rigid-body semantics belong to the separately pinned `physics-engine`. That engine owns rotating-cuboid integration, sampled collision/re-contact discovery, OBB contact response, restitution, friction, and persistent-contact tail handling. ECS Lab owns scenario composition, ECS-shaped body/state conversion, bounded frame policy, deterministic evidence projection, and browser adaptation around that engine.

Application and teaching repositories such as `collision-lab` remain independent consumers and are not implementation dependencies.

## Legacy 2D solver

`ecs-physics` remains intentionally stable. Existing constructors, deterministic pair ordering, mass/restitution/friction behavior, contact/support evidence, regression matrices, `falling-boxes`, `BouncingRoomScenario`, and benchmark fixtures continue to operate on the Z=0 plane.

Keeping this surface intact provides historical correctness and performance evidence without forcing the newer 3D experiments into the same implementation.

## Continuous AABB reference experiment

The translational continuous solver in `ecs-physics-3d` remains an independent ECS Lab experiment. It runs the supported AABB collision set on one deterministic continuous timeline:

1. dynamic bodies receive integer X/Y/Z gravity;
2. every pair containing at least one dynamic body is considered across the remaining `(x, y, z, t)` interval rather than only at the end position;
3. pair-relative motion turns AABB-vs-AABB into a swept point against a Minkowski-expanded relative AABB;
4. a conservative swept broad phase reduces candidates without becoming simulation authority;
5. X/Y/Z slab entry and exit intervals produce exact candidate times of impact;
6. dynamic↔fixed and dynamic↔dynamic candidates compete for the same globally earliest exact TOI;
7. every pair at that TOI and every tied collision axis within those pairs is collected into one deterministic contact set;
8. all dynamic bodies advance once to that shared time using deterministic Q32.32 subticks;
9. contact constraints are projected and resolved through bounded stable-order passes;
10. material restitution is admitted on the first response pass and later passes are non-restorative correction;
11. deterministic tangent friction follows normal convergence;
12. the remaining timestep is searched again under the same event-set policy; and
13. final private fixed-point positions are converted deterministically back to integer ECS coordinates.

The same material scale is reused: `0..=1000` thousandths for restitution and friction. Dynamic bodies require positive integer mass units; fixed bodies remain immovable.

This solver is retained as inspectable translational AABB evidence. It is not the implementation used by the committed rotating OBB worlds and should not grow into a second rotating rigid-body authority.

## Standalone rotating rigid-box authority

The browser playground and trebuchet tower convert ECS Lab `RigidBox3d` state through `step_rigid_box_world_with_physics_engine`. The adapter deliberately stays thin:

- ECS entity identity maps to stable engine-local body identity;
- ECS Lab retains scenario/body metadata and integer export contracts;
- frame-level angular damping and bounded angular substep selection remain explicit consumer policy;
- every selected substep is executed by `physics-engine`;
- sampled rotating collision-event search, OBB response, restitution, friction, and persistent-tail stabilization stay inside the engine;
- engine results are mapped back to the existing ECS-facing state without introducing a second solver path.

Rotational collision discovery remains explicitly sampled rather than analytic rotational CCD. The browser must not describe the current solver as analytic rotational CCD or as owning guarantees beyond the engine's bounded sampled model.

## Bouncing room 3D scenario

`BouncingRoom3dScenario` is the canonical dense 3D browser fixture. It contains 48 dynamic boxes arranged across three depth layers and four height rows. Their footprints, mass, restitution, friction, and X/Y/Z velocities vary deterministically. They move inside six fixed slabs:

- floor and ceiling;
- left and right X walls;
- back and front Z walls.

The scenario itself remains owned by ECS Lab. Its committed rotating playback is advanced through the pinned `physics-engine` adapter, while the exported broad-phase evidence and browser representation remain ECS Lab concerns.

The scenario can also replay through `ReferenceWorld` and `SparseWorld` where storage differential evidence is required. Those storage comparisons remain separate from engine ownership.

## Trebuchet tower scenario

The tower is another engine consumer, not a local solver. ECS Lab defines the floor, projectile, block arrangement, material/mass values, frame rate, damping, bounded response-pass policy, and exported OBB vertices. `physics-engine` advances the rotating world.

Acceptance is deliberately behavioral: the tower stays quiescent before impact, the projectile produces multi-block angular response, and exported dynamic geometry must not finish below the floor surface. Engine-native sampled-event and persistent-tail counts are evidence only; they are not reinterpreted by the browser as a second contact solver.

## Browser and WebGPU ownership

The dedicated `/physics/` Pages demo reads authoritative X/Y/Z positions, 3D half extents, body/material metadata, orientation, and pair evidence from Wasm.

Smooth motion is presentation-only. The browser preloads discrete deterministic Rust frames and interpolates displayed positions between consecutive frames. At each integer physics step, the displayed state snaps exactly to the Rust result; JavaScript never integrates velocity or resolves contacts.

Camera state and the optional timeline matrix are presentation-only. Pointer drag, pan, zoom, and reset controls never write ECS or physics state.

WebGPU has two independent roles:

- **3D rendering** — WebGPU or the Canvas fallback displays already-authoritative Rust state;
- **broad-phase evidence** — the existing compute path may check exported AABB pair evidence, but it is accepted only after equality with the authoritative Rust result.

Neither GPU path feeds impulses, gravity, friction, contact iteration, collision timing, or ECS mutation back into the simulation.

## Determinism contract

Across ECS Lab physics workloads:

- body configuration order cannot change canonical observable ordering;
- ECS positions, velocities, masses, and material coefficients remain integer-valued;
- fixed bodies never receive generated ECS motion writes;
- browser rendering and WebGPU evidence cannot become simulation authority;
- unsupported bounded work fails closed rather than silently dropping collision work.

For the local continuous AABB experiment specifically:

- fixed and dynamic pair candidates compete on one timeline;
- TOI comparisons use exact integer fractions;
- all pair/axis contacts at the globally earliest exact TOI form one set;
- temporal integration uses private fixed-point state;
- final positions use deterministic integer quantization;
- bounded projection/normal/stabilization work fails closed.

For rotating worlds specifically:

- `physics-engine` is the sole collision/integration authority;
- ECS Lab may select frame/search/substep budgets but does not duplicate their algorithms;
- the pinned engine revision is immutable and part of the repository environment fingerprint;
- rotational collision discovery is sampled and bounded, not analytic CCD.

## Performance evidence

Existing 2D benchmark fixtures remain the historical performance baseline:

- motion replay;
- falling boxes through reference and sparse-set storage;
- sparse and dense material-step fixtures;
- long-running 2D bouncing-room replay.

Timing remains descriptive rather than a correctness threshold. Performance experiments must not fork reusable rotating collision semantics back into `ecs-lab`; reusable engine improvements belong upstream in `physics-engine` and can then be consumed through a new immutable pin.

## Later horizons

Keep later work separated by ownership:

- ECS/storage experiments, scenario composition, differential evidence, browser presentation, and independent reference algorithms may live here;
- reusable rotating rigid-body features such as constraints/joints, sleeping/islands, particles, fluids, and destructible-body simulation belong on the `physics-engine` roadmap;
- reusable low-level geometry/spatial primitives belong in `rust-kernels` when they are independent of engine orchestration;
- GPU acceleration must retain a deterministic CPU/reference boundary before it can influence authoritative results.
