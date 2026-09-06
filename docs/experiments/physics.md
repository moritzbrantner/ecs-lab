# Deterministic physics workloads

## Purpose

Use physics as a realistic deterministic ECS workload without turning `ecs-lab` into a general-purpose game engine. The repository has two deliberately separate solver surfaces:

- `ecs-physics`: the existing 2D AABB solver retained as the compatibility, regression, and benchmark foundation;
- `ecs-physics-3d`: the three-dimensional AABB solver used by the dedicated browser physics world.

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

`ecs-physics-3d` extends the deterministic response model to three axes and now has a first continuous-collision horizon:

1. dynamic bodies receive integer X/Y/Z gravity;
2. dynamic-vs-fixed collision detection spans the full `(x, y, z, t)` interval instead of testing only the end position;
3. a swept point against the fixed body's Minkowski-expanded AABB yields slab entry/exit times on X, Y, and Z;
4. time-of-impact ordering is compared as exact integer fractions with stable X→Y→Z and entity-id tie ordering;
5. temporal advancement uses deterministic Q32.32 subticks, while the normal-axis contact coordinate is snapped exactly to the integer AABB boundary;
6. the earliest static impact is resolved first, then the remaining fraction of the timestep is simulated with the updated velocity;
7. the TOI loop is bounded and fails closed rather than silently allowing unbounded collision churn;
8. final dynamic-vs-fixed penetrations receive a bounded correction pass after dynamic-body response;
9. dynamic-vs-dynamic response remains the existing deterministic discrete path in this horizon;
10. normal response uses deterministic integer mass/restitution arithmetic and friction is applied along both tangent axes; and
11. only ordinary `SetPosition` / `SetVelocity` ECS operations mutate observable ECS state.

The same material scale is reused: `0..=1000` thousandths for restitution and friction. Dynamic bodies require positive integer mass units; fixed bodies remain immovable.

This is a controlled deterministic 3D contact model for ECS experiments, not a claim of production rigid-body fidelity. The current path remains AABB-only and does not add rotation, angular velocity, torque, OBBs, meshes, or iterative manifolds.

## First space-time CCD horizon

The purpose of the first CCD slice is narrow: fast dynamic boxes must not tunnel through fixed walls, floors, ceilings, or other fixed AABBs merely because their start and end positions lie on opposite sides of the obstacle.

For each dynamic body, the solver first applies the existing semi-implicit gravity update. It then keeps the body's position in deterministic Q32.32 subunits for the duration of the step. Every fixed body is expanded by the moving body's half extents, turning swept AABB-vs-AABB into a moving-point slab test. A cheap temporal broad phase rejects fixed bodies whose expanded box does not intersect the complete swept segment.

For surviving candidates, each moving axis produces entry and exit times. The three axis intervals are intersected and the latest entry time is the time of impact. Candidate TOIs are compared with exact integer cross-products rather than floating-point epsilon rules. The selected impact advances every axis to that time, snaps the collision axis to the exact expanded boundary, applies restitution and tangent friction, subtracts the consumed time, and continues through the remaining interval.

This means a fully bouncy box can hit a wall early in a step and travel away from it for the remaining fraction rather than simply being clamped at the wall until the next frame. Multiple wall impacts inside one timestep are also supported up to the explicit bounded event limit.

The public ECS position remains integer-valued. Q32.32 is private solver state used only while traversing one timestep; the final position is deterministically quantized back to the existing ECS contract. The collision normal coordinate itself remains exact because static AABB boundaries and collider half extents are integer-valued.

Focused regression coverage includes:

- a body moving farther than a wall's thickness in one step;
- a fully bouncy wall hit that must consume the remainder of the timestep;
- a swept path that crosses the wall's X range but misses its tangent ranges;
- multiple wall impacts inside one timestep; and
- the pre-existing direct Z-axis dynamic collision regression.

Dynamic-vs-dynamic CCD is deliberately not hidden inside this slice. Two fast moving bodies can still require a relative-motion TOI path; that is the next continuous-collision horizon.

## Bouncing room 3D scenario

`BouncingRoom3dScenario` is the canonical 3D browser fixture. It contains 48 dynamic boxes arranged across three depth layers and four height rows. Their footprints, mass, restitution, friction, and X/Y/Z velocities vary deterministically so the same scene acts as both a dense visualization and a stronger contact workload. They move inside six fixed AABB slabs:

- floor and ceiling;
- left and right X walls;
- back and front Z walls.

Gravity acts along negative Y. The six room slabs are now traversed by the static space-time CCD path, so a body cannot cross a room boundary merely because its integer frame-to-frame displacement exceeds the slab thickness.

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

For static 3D CCD specifically:

- swept candidate ordering is stable by entity id;
- axis ties remain X→Y→Z;
- TOI comparisons use exact integer fractions;
- temporal integration uses a fixed Q32.32 scale rather than floating-point wall-clock time;
- the bounded event loop fails closed if a body exceeds its supported impact budget.

## Performance evidence

Existing 2D benchmark fixtures remain the current performance baseline:

- motion replay;
- falling boxes through reference and sparse-set storage;
- sparse and dense material-step solver fixtures;
- long-running 2D bouncing-room replay.

Timing remains descriptive rather than a correctness threshold. The dense 3D browser fixture now also exercises temporal broad-phase rejection and swept fixed-body tests, but dedicated 3D benchmark fixtures should still be added separately so browser presentation cost is not confused with solver performance.

## Later 3D horizons

Keep these as separate reviewable slices after the fixed-geometry CCD boundary is stable:

- dynamic-vs-dynamic CCD using relative space-time motion and earliest-pair TOI ordering;
- stronger manifold/contact iteration for dense body stacks and simultaneous contacts;
- conservative scalable broad-phase candidates with exact pair parity;
- character movement/jumping consuming support evidence;
- circles/spheres and then oriented boxes;
- angular state, rotations, and torque;
- destruction consuming impact evidence;
- liquids/fluids as a distinct solver-owned state model rather than ordinary rigid-body entities.
