# Deterministic physics workloads

## Purpose

Use physics as a realistic deterministic ECS workload without turning `ecs-lab` into a general-purpose game engine. The repository has two deliberately separate solver surfaces:

- `ecs-physics`: the existing 2D AABB solver retained as the compatibility, regression, and benchmark foundation;
- `ecs-physics-3d`: the three-dimensional continuous AABB solver used by the dedicated browser physics world.

Both consume observable ECS snapshots and return ordinary `ecs-workload::Operation` mutations rather than owning storage.

## Three-axis ECS state

`ecs-workload::Position` and `Velocity` carry X, Y, and Z. The existing two-argument constructors remain source-compatible and set Z to zero, while `Position::new3(...)` and `Velocity::new3(...)` construct explicit three-dimensional values.

`ReferenceWorld` and `SparseWorld` both integrate all three axes. This keeps storage parity as the same fundamental contract whether a workload stays on the legacy Z=0 plane or moves through depth.

## Cross-repository collision boundary

Reusable AABB decisions continue to live in pinned `rust-kernels` crates rather than in application/lab repositories:

- `geometry-kernels` / `spatial-kernels`
- `rust-kernels` revision `986bff4dc8d13a64b90fab3a9f7f02bb8d1aa35e`

`geometry-kernels::aabb_aabb` operates on three-dimensional `spatial_kernels::Aabb` values. ECS Lab owns integration, deterministic pair ordering, material response, positional correction, continuous-time orchestration, scenarios, and browser adaptation around that reusable overlap decision.

## Legacy 2D solver

`ecs-physics` remains intentionally stable. Existing constructors, deterministic pair ordering, mass/restitution/friction behavior, contact/support evidence, regression matrices, `falling-boxes`, `BouncingRoomScenario`, and benchmark fixtures continue to operate on the Z=0 plane.

Keeping this surface intact gives the 3D path a proven reference boundary and avoids converting performance evidence merely to make the demo look three-dimensional.

## True 3D solver

`ecs-physics-3d` now runs the complete supported AABB collision set on one deterministic continuous timeline:

1. dynamic bodies receive integer X/Y/Z gravity;
2. every pair containing at least one dynamic body is considered across the full remaining `(x, y, z, t)` interval rather than only at the end position;
3. pair-relative motion turns AABB-vs-AABB into a swept point against a Minkowski-expanded relative AABB;
4. a temporal broad phase rejects pairs whose complete relative swept segment cannot reach those expanded bounds;
5. X/Y/Z slab entry and exit intervals produce an exact candidate time of impact;
6. dynamic↔fixed and dynamic↔dynamic candidates compete for the same globally earliest event;
7. candidate TOIs are compared as exact integer fractions, then equal times are ordered by stable entity pair and X→Y→Z axis order;
8. temporal advancement uses deterministic Q32.32 subticks while public ECS state remains integer-valued;
9. the selected pair is snapped to its exact normal-axis contact relation, deterministic restitution/friction response is applied, and the remaining timestep is searched again;
10. the event loop and penetration stabilization are explicitly bounded and fail closed rather than silently accepting unsupported collision churn; and
11. only ordinary `SetPosition` / `SetVelocity` ECS operations mutate observable ECS state.

The same material scale is reused: `0..=1000` thousandths for restitution and friction. Dynamic bodies require positive integer mass units; fixed bodies remain immovable.

This is a controlled deterministic 3D contact model for ECS experiments, not a claim of production rigid-body fidelity. The current path remains AABB-only and does not add rotation, angular velocity, torque, OBBs, meshes, or a full simultaneous-contact manifold solver.

## Unified space-time CCD

The first continuous slice prevented fast dynamic boxes from tunneling through fixed walls, floors, ceilings, and fixed obstacles. The next slice removes the remaining frame-end-only body-body gap by putting moving pairs on the same event timeline.

For a pair of bodies, the solver subtracts the right body's position and velocity from the left body's. Fixed bodies naturally have zero velocity, so fixed and moving pairs share the same relative-motion calculation. The two collider half extents expand the relative origin into one AABB slab problem.

For each moving relative axis, the solver computes exact entry and exit fractions. The latest axis entry is the pair TOI and the earliest exit bounds validity. The three intervals must overlap, the entry must be non-negative, and the candidate must fall inside the remaining Q32.32 timestep interval.

Every surviving pair candidate is compared against the current global event using integer cross-products instead of floating-point epsilon rules. Equal fractions use the canonical sorted entity pair and then X→Y→Z axis order. That ordering is explicit solver policy rather than an incidental consequence of container traversal.

Once an event is chosen, all dynamic bodies advance by the same consumed subtick interval. The colliding pair is snapped only along the collision normal to remove Q32.32 flooring error while preserving tangent-axis precision. Dynamic↔dynamic snap correction is split deterministically by mass; fixed bodies never move. The normal response then uses the existing integer mass/restitution equations and both tangent axes use deterministic friction. The solver subtracts the consumed time and performs another global search.

This global ordering matters. A moving body may hit another moving body before either reaches a wall, or a wall impact may occur first and change the later body-body trajectory. These events can no longer be resolved in separate static and dynamic phases without changing the physical history.

The public ECS position remains integer-valued. Q32.32 is private solver state used only while traversing one timestep; the final positions are deterministically quantized back to the existing ECS contract.

Focused continuous-collision evidence now includes:

- a fast body moving farther than a fixed wall's thickness in one step;
- multiple fixed-wall impacts inside one timestep;
- two fast dynamic bodies whose frame-end positions would cross completely;
- a fully bouncy dynamic pair that must travel through the remaining fraction after its TOI;
- tangent-separated moving bodies that must remain collision-free;
- independent equal-TOI pairs ordered by entity identity; and
- a body-body event competing with a later wall event on the same timeline.

Simultaneous contacts that share bodies are intentionally still processed through the explicit deterministic event order. A stronger equal-time contact-set/manifold iteration is the next robustness horizon rather than hidden inside this slice.

## Bouncing room 3D scenario

`BouncingRoom3dScenario` is the canonical 3D browser fixture. It contains 48 dynamic boxes arranged across three depth layers and four height rows. Their footprints, mass, restitution, friction, and X/Y/Z velocities vary deterministically so the same scene acts as both a dense visualization and a stronger contact workload. They move inside six fixed AABB slabs:

- floor and ceiling;
- left and right X walls;
- back and front Z walls.

Gravity acts along negative Y. Both room-boundary impacts and dynamic-body impacts now participate in the Rust solver's continuous event timeline, so neither thin fixed slabs nor other fast AABBs can be skipped merely because a frame displacement is large.

The scenario can replay through both `ReferenceWorld` and `SparseWorld`. Differential tests require the storage snapshots to remain identical while the same `PhysicsStep3d` operations are applied. Focused tests also exercise direct Z-axis response and repeatable 3D broad-phase evidence.

## Browser and WebGPU ownership

The dedicated `/physics/` Pages demo reads authoritative X/Y/Z positions, 3D half extents, body/material metadata, and canonical pair words from the Wasm adapter over `BouncingRoom3dScenario`.

Smooth motion is presentation-only. The browser preloads discrete deterministic Rust frames and interpolates displayed positions between consecutive frames. At each integer physics step, the displayed state snaps exactly to the Rust result; JavaScript never integrates velocity or resolves contacts.

The optional timeline matrix is also presentation-only. It samples already-authoritative Rust frames and must not be confused with the solver's continuous `(x, y, z, t)` collision calculations.

Camera state is presentation-only. Pointer drag orbits, Shift-drag/right-drag pans, the wheel changes camera radius, and double-click resets the view. Those controls change only the view/projection inputs used by the renderer and never write ECS or physics state.

WebGPU has two independent roles:

- **3D renderer** — a raw browser WebGPU render pipeline draws instanced boxes with depth testing and the mouse-controlled camera. If no usable WebGPU renderer exists, a projected Canvas wireframe keeps the same Rust simulation visible.
- **collision evidence** — the existing all-pairs compute shader receives the exact Rust-produced final-frame min/max XYZ AABBs. Its triangular pair bitset is accepted only after word-for-word equality with the Rust broad-phase evidence.

Neither GPU path feeds impulses, gravity, friction, TOI decisions, or ECS mutation back into the solver.

## Determinism contract

For both solver surfaces:

- body configuration order cannot change canonical pair order;
- ECS positions, velocities, masses, and material coefficients remain integer-valued;
- conversion to the reusable f32 AABB kernel is allowed only while every bound is exactly representable;
- touching remains part of the reusable AABB contact contract;
- fixed bodies never receive generated component writes;
- browser rendering and WebGPU evidence cannot become simulation authority.

For continuous 3D CCD specifically:

- all bodies are canonicalized by entity id before pair traversal;
- fixed and dynamic pair candidates compete on one timeline;
- TOI comparisons use exact integer fractions;
- equal TOIs use entity-pair order, followed by X→Y→Z axis order;
- temporal integration uses a fixed Q32.32 scale rather than floating-point wall-clock time;
- the fixed-point representation is private intra-step state rather than a new public ECS representation;
- the bounded event/stabilization loops fail closed if the supported impact budget is exhausted.

## Performance evidence

Existing 2D benchmark fixtures remain the historical performance baseline:

- motion replay;
- falling boxes through reference and sparse-set storage;
- sparse and dense material-step solver fixtures;
- long-running 2D bouncing-room replay.

Timing remains descriptive rather than a correctness threshold. The dense 3D browser fixture now exercises global pair-relative temporal broad-phase rejection and repeated earliest-event searches. Dedicated 3D solver benchmarks and a scalable conservative 3D broad phase are therefore the next performance-oriented work rather than making browser presentation cost a solver threshold.

## Later 3D horizons

Keep these as separate reviewable vertical slices after unified AABB CCD:

- deterministic simultaneous/equal-time contact sets and stronger bounded manifold/contact iteration for dense stacks and corners;
- conservative scalable 3D and swept space-time broad-phase candidates with exact narrow-phase parity;
- spheres, then sphere↔AABB collision, followed later by oriented boxes only after the continuous foundation is stable;
- collision layers, masks, and sensors when they fit the same deterministic collider contract;
- character movement/jumping consuming Rust-owned support evidence;
- impact evidence consumed by deterministic destructible ECS behavior;
- liquid volumes with buoyancy and drag before any full fluid solver;
- one bounded solver-owned 3D fluid experiment with a deterministic CPU reference before optional verified GPU acceleration;
- angular state, rotations, torque, joints, arbitrary mesh collision, and production-engine API work only after measured need.
