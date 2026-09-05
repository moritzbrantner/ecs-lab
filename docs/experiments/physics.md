# Deterministic physics workloads

## Purpose

Use physics as a realistic deterministic ECS workload without turning `ecs-lab` into a general-purpose game engine. The repository now has two deliberately separate solver surfaces:

- `ecs-physics`: the existing 2D AABB solver retained as the compatibility, regression, and benchmark foundation;
- `ecs-physics-3d`: the true three-dimensional AABB solver used by the dedicated browser physics world.

Both consume observable ECS snapshots and return ordinary `ecs-workload::Operation` mutations rather than owning storage.

## Three-axis ECS state

`ecs-workload::Position` and `Velocity` carry X, Y, and Z. The existing two-argument constructors remain source-compatible and set Z to zero, while `Position::new3(...)` and `Velocity::new3(...)` construct explicit three-dimensional values.

`ReferenceWorld` and `SparseWorld` both integrate all three axes. This keeps storage parity as the same fundamental contract whether a workload stays on the legacy Z=0 plane or moves through depth.

## Cross-repository collision boundary

Reusable AABB decisions continue to live in pinned `rust-kernels` crates rather than in application/lab repositories:

- `geometry-kernels` / `spatial-kernels`
- `rust-kernels` revision `986bff4dc8d13a64b90fab3a9f7f02bb8d1aa35e`

`geometry-kernels::aabb_aabb` already operates on three-dimensional `spatial_kernels::Aabb` values. ECS Lab owns integration, deterministic pair ordering, material response, positional correction, scenarios, and browser adaptation around that reusable overlap decision.

## Legacy 2D solver

`ecs-physics` remains intentionally stable. Existing constructors, deterministic pair ordering, mass/restitution/friction behavior, contact/support evidence, regression matrices, `falling-boxes`, `BouncingRoomScenario`, and benchmark fixtures continue to operate on the Z=0 plane.

Keeping this surface intact gives the new 3D path a proven reference boundary and avoids converting performance evidence merely to make the demo look three-dimensional.

## True 3D solver

`ecs-physics-3d` extends the same deterministic response model to three axes:

1. dynamic bodies receive integer X/Y/Z gravity;
2. semi-implicit integration advances all three velocity and position components;
3. body pairs are visited in ascending `EntityId` order;
4. the reusable geometry kernel decides exact 3D AABB overlap/touching;
5. penetration is measured on X, Y, and Z and resolved along the minimum-overlap axis with stable X→Y→Z tie ordering;
6. normal response uses deterministic integer mass/restitution arithmetic;
7. friction is applied along both tangent axes of the selected contact normal;
8. vertical support evidence remains a Y-axis concept; and
9. only ordinary `SetPosition` / `SetVelocity` ECS operations mutate state.

The same material scale is reused: `0..=1000` thousandths for restitution and friction. Dynamic bodies require positive integer mass units; fixed bodies remain immovable.

This is a controlled deterministic 3D contact model for ECS experiments, not a claim of production rigid-body fidelity. The first 3D slice remains AABB-only and does not add rotation, angular velocity, torque, OBBs, meshes, or iterative manifolds.

## Bouncing room 3D scenario

`BouncingRoom3dScenario` is the canonical 3D browser fixture. It now contains 48 dynamic boxes arranged across three depth layers and four height rows. Their footprints, mass, restitution, friction, and X/Y/Z velocities vary deterministically so the same scene acts as both a much denser visualization and a stronger contact workload. They move inside six fixed AABB slabs:

- floor and ceiling;
- left and right X walls;
- back and front Z walls.

Gravity acts along negative Y. The scenario therefore demonstrates depth motion, body-body interaction, material variation, and Z-axis collisions as physics behavior rather than as a rendering trick.

The scenario can replay through both `ReferenceWorld` and `SparseWorld`. Differential tests require the storage snapshots to remain identical while the same `PhysicsStep3d` operations are applied. Focused tests also exercise a direct Z-axis collision and repeatable 3D broad-phase evidence.

## Browser and WebGPU ownership

The dedicated `/physics/` Pages demo reads authoritative X/Y/Z positions, 3D half extents, body/material metadata, and canonical pair words from the Wasm adapter over `BouncingRoom3dScenario`.

Smooth motion is presentation-only. The browser preloads discrete deterministic Rust frames and interpolates displayed positions between consecutive frames. At each integer physics step, the displayed state snaps exactly to the Rust result; JavaScript never integrates velocity or resolves contacts.

Camera state is also presentation-only. Pointer drag orbits, Shift-drag/right-drag pans, the wheel changes camera radius, and double-click resets the view. Those controls change only the view/projection inputs used by the renderer and never write ECS or physics state.

WebGPU has two independent roles:

- **3D renderer** — a raw browser WebGPU render pipeline draws instanced boxes with depth testing and the mouse-controlled camera. If no usable WebGPU renderer exists, a projected Canvas wireframe keeps the same Rust simulation visible.
- **collision evidence** — the existing all-pairs compute shader receives the exact Rust-produced min/max XYZ AABBs. Its triangular pair bitset is accepted only after word-for-word equality with the Rust broad-phase evidence.

Neither GPU path feeds impulses, gravity, friction, or ECS mutation back into the solver.

## Determinism contract

For both solver surfaces:

- body configuration order cannot change canonical pair order;
- ECS positions, velocities, masses, and material coefficients remain integer-valued;
- conversion to the reusable f32 AABB kernel is allowed only while every bound is exactly representable;
- touching remains part of the reusable AABB contact contract;
- fixed bodies never receive generated component writes;
- browser rendering and WebGPU evidence cannot become simulation authority.

For the 3D solver specifically, all three axes participate in collision decisions, and both tangent axes participate in contact friction.

## Performance evidence

Existing 2D benchmark fixtures remain the current performance baseline:

- motion replay;
- falling boxes through reference and sparse-set storage;
- sparse and dense material-step solver fixtures;
- long-running 2D bouncing-room replay.

Timing remains descriptive rather than a correctness threshold. The denser 3D browser fixture gives the solver and GPU parity path more realistic pressure, but dedicated 3D benchmark fixtures should still be added separately so browser presentation cost is not confused with solver performance.

## Later 3D horizons

Keep these as separate reviewable slices after true 3D AABB semantics are stable:

- conservative scalable broad-phase candidates with exact pair parity;
- character movement/jumping consuming support evidence;
- circles/spheres and then oriented boxes;
- angular state, rotations, and torque;
- destruction consuming impact evidence;
- liquids/fluids as a distinct solver-owned state model rather than ordinary rigid-body entities.