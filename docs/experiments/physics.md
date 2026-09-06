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

`ecs-physics-3d` runs the supported AABB collision set on one deterministic continuous timeline:

1. dynamic bodies receive integer X/Y/Z gravity;
2. every pair containing at least one dynamic body is considered across the full remaining `(x, y, z, t)` interval rather than only at the end position;
3. pair-relative motion turns AABB-vs-AABB into a swept point against a Minkowski-expanded relative AABB;
4. a temporal broad phase rejects pairs whose complete relative swept segment cannot reach those expanded bounds;
5. X/Y/Z slab entry and exit intervals produce exact candidate times of impact;
6. dynamic↔fixed and dynamic↔dynamic candidates compete for the same globally earliest exact TOI;
7. every pair at that TOI and every tied collision axis within those pairs is collected into one deterministic contact set, ordered by entity pair and then X→Y→Z;
8. all dynamic bodies advance once to that shared time using deterministic Q32.32 subticks;
9. the contact set is projected to exact normal-axis contact relations through a bounded stable-order pass;
10. coupled contact normals are resolved through bounded stable-order iteration, with material restitution allowed on the first pass and later passes acting only as non-restorative constraint correction;
11. deterministic tangent friction is applied after normal convergence, with one further bounded inelastic normal solve if friction makes another simultaneous constraint approaching;
12. the remaining timestep is searched again under the same global event-set policy;
13. private Q32.32 positions are rounded deterministically to the integer ECS grid at the step boundary; and
14. only ordinary `SetPosition` / `SetVelocity` ECS operations mutate observable ECS state.

The same material scale is reused: `0..=1000` thousandths for restitution and friction. Dynamic bodies require positive integer mass units; fixed bodies remain immovable.

This is a controlled deterministic 3D contact model for ECS experiments, not a claim of production rigid-body fidelity. The current path remains AABB-only and does not add rotation, angular velocity, torque, OBBs, meshes, or a general rigid-body manifold framework.

## Unified space-time CCD

The first continuous slice prevented fast dynamic boxes from tunneling through fixed walls, floors, ceilings, and fixed obstacles. The next slice removed the remaining frame-end-only body-body gap by putting moving pairs on the same event timeline.

For a pair of bodies, the solver subtracts the right body's position and velocity from the left body's. Fixed bodies naturally have zero velocity, so fixed and moving pairs share the same relative-motion calculation. The two collider half extents expand the relative origin into one AABB slab problem.

For each moving relative axis, the solver computes exact entry and exit fractions. The latest axis entry is the pair TOI and the earliest exit bounds validity. The three intervals must overlap, the entry must be non-negative, and the candidate must fall inside the remaining Q32.32 timestep interval.

Every surviving pair candidate is compared against the current global event using integer cross-products instead of floating-point epsilon rules. Equal fractions are not discarded: every pair at the earliest time is retained, and each pair retains every X/Y/Z axis whose exact entry fraction equals the pair's latest entry. The resulting set is sorted by canonical entity pair and then X→Y→Z.

This global ordering matters. A moving body may hit another moving body before either reaches a wall, or a wall impact may occur first and change the later body-body trajectory. Equal-time contacts also matter: a body can reach two perpendicular walls simultaneously, or one body can participate in a short contact chain at the same TOI. Those cases cannot be represented faithfully by advancing time between otherwise simultaneous pair responses.

## Deterministic simultaneous contact sets

At the selected event-set TOI, every dynamic body advances by the same consumed Q32.32 interval exactly once. The solver then projects the equal-time constraints in stable order so flooring the exact fraction to a representable subtick cannot leave one selected contact fractionally short of its exact normal boundary. Dynamic↔dynamic projection remains mass-aware; fixed bodies remain immovable.

The coupled normal solve then iterates the selected contacts in stable entity-pair/X→Y→Z order. On the first pass only, each still-approaching contact may use its configured restitution. Later passes use zero restitution and exist only to restore the non-approaching contact constraints after another pair in the same set changes a shared body's velocity. This avoids repeatedly injecting bounce energy merely because a contact belongs to a coupled set.

After normal convergence, tangent friction is applied deterministically once per selected contact. If that tangent response makes another simultaneous normal approaching again, the solver runs a final bounded inelastic normal correction. Position projection and normal iteration each have explicit fail-closed budgets.

The private fixed-point representation also has an explicit step-boundary conversion rule. Q32.32 positions are rounded to the nearest integer ECS coordinate, with exact half cases rounded away from zero. Earlier truncation toward zero could turn a perfectly valid fractional touching chain into an integer overlap on one side; symmetric nearest-integer quantization avoids that directional artifact while keeping the public ECS contract integer-valued.

Focused continuous-collision/contact-set evidence includes:

- a fast body moving farther than a fixed wall's thickness in one step;
- multiple fixed-wall impacts inside one timestep;
- two fast dynamic bodies whose frame-end positions would cross completely;
- a fully bouncy dynamic pair that must travel through the remaining fraction after its TOI;
- tangent-separated moving bodies that must remain collision-free;
- two independent pairs at one exact TOI emitted in stable entity order;
- a diagonal pair whose X and Y slab entries tie exactly, emitted X before Y and resolved on both axes;
- a dynamic body reaching perpendicular fixed walls at one exact TOI;
- a three-body shared-contact chain with a non-dyadic TOI that converges under the bounded solver;
- Z-axis dynamic tunneling regression; and
- both body-before-wall and wall-before-body event ordering on the unified timeline.

## Bouncing room 3D scenario

`BouncingRoom3dScenario` is the canonical 3D browser fixture. It contains 48 dynamic boxes arranged across three depth layers and four height rows. Their footprints, mass, restitution, friction, and X/Y/Z velocities vary deterministically so the same scene acts as both a dense visualization and a stronger contact workload. They move inside six fixed AABB slabs:

- floor and ceiling;
- left and right X walls;
- back and front Z walls.

Gravity acts along negative Y. Both room-boundary impacts and dynamic-body impacts participate in the Rust solver's continuous event timeline, and equal-time pair/axis contacts are resolved as bounded deterministic sets. Thin fixed slabs, other fast AABBs, and simple simultaneous corners therefore no longer depend on frame-end overlap or incidental pair traversal.

The scenario can replay through both `ReferenceWorld` and `SparseWorld`. Differential tests require the storage snapshots to remain identical while the same `PhysicsStep3d` operations are applied. Focused tests also exercise direct Z-axis response and repeatable 3D broad-phase evidence.

## Browser and WebGPU ownership

The dedicated `/physics/` Pages demo reads authoritative X/Y/Z positions, 3D half extents, body/material metadata, and canonical pair words from the Wasm adapter over `BouncingRoom3dScenario`.

Smooth motion is presentation-only. The browser preloads discrete deterministic Rust frames and interpolates displayed positions between consecutive frames. At each integer physics step, the displayed state snaps exactly to the Rust result; JavaScript never integrates velocity or resolves contacts.

The optional timeline matrix is also presentation-only. It samples already-authoritative Rust frames and must not be confused with the solver's continuous `(x, y, z, t)` collision calculations or simultaneous-contact iteration.

Camera state is presentation-only. Pointer drag orbits, Shift-drag/right-drag pans, the wheel changes camera radius, and double-click resets the view. Those controls change only the view/projection inputs used by the renderer and never write ECS or physics state.

WebGPU has two independent roles:

- **3D renderer** — a raw browser WebGPU render pipeline draws instanced boxes with depth testing and the mouse-controlled camera. If no usable WebGPU renderer exists, a projected Canvas wireframe keeps the same Rust simulation visible.
- **collision evidence** — the existing all-pairs compute shader receives the exact Rust-produced final-frame min/max XYZ AABBs. Its triangular pair bitset is accepted only after word-for-word equality with the Rust broad-phase evidence.

Neither GPU path feeds impulses, gravity, friction, contact-set iteration, TOI decisions, or ECS mutation back into the solver.

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
- all pair/axis contacts at the globally earliest exact TOI form one set;
- contact-set order is entity pair followed by X→Y→Z axis order;
- temporal integration uses a fixed Q32.32 scale rather than floating-point wall-clock time;
- Q32.32 is private intra-step state rather than a new public ECS representation;
- final fixed-point positions use deterministic nearest-integer quantization with half cases away from zero;
- restitution is restricted to the first coupled normal pass and later passes are non-restorative;
- event, contact-projection, normal-iteration, and penetration-stabilization bounds fail closed when exhausted.

## Performance evidence

Existing 2D benchmark fixtures remain the historical performance baseline:

- motion replay;
- falling boxes through reference and sparse-set storage;
- sparse and dense material-step solver fixtures;
- long-running 2D bouncing-room replay.

Timing remains descriptive rather than a correctness threshold. The dense 3D browser fixture now exercises repeated global pair-relative swept searches plus equal-time contact-set solving. The next performance-oriented horizon is therefore a scalable conservative 3D/swept broad phase that reduces candidates while retaining the naive all-pairs path as exact correctness evidence.

## Later 3D horizons

Keep these as separate reviewable vertical slices after unified AABB CCD and simultaneous contact sets:

- conservative scalable 3D and swept space-time broad-phase candidates with exact narrow-phase/final-state parity against the naive reference path;
- spheres, then sphere↔AABB collision, followed later by oriented boxes only after the continuous foundation is stable;
- collision layers, masks, and sensors when they fit the same deterministic collider contract;
- character movement/jumping consuming Rust-owned support evidence;
- impact evidence consumed by deterministic destructible ECS behavior;
- liquid volumes with buoyancy and drag before any full fluid solver;
- one bounded solver-owned 3D fluid experiment with a deterministic CPU reference before optional verified GPU acceleration;
- angular state, rotations, torque, joints, arbitrary mesh collision, and production-engine API work only after measured need.
