# Physics workloads

## Purpose

Use physics as a realistic ECS workload without turning `ecs-lab` into a game engine. The repository keeps two distinct surfaces:

- `ecs-physics`: the 2D integer AABB solver retained as the deterministic workload generator for the storage benchmarks and invariant suite;
- `ecs-physics-3d`: the 3D rigid-box scenarios (room, tower, dice, material traces), with Rapier (`rapier3d =0.36.1`, 3D f32, default features, matching ARPG) as the rigid-body authority.

All ECS Lab surfaces consume or produce observable ECS-shaped state rather than owning storage. Rendering and browser interaction remain consumer concerns.

## Three-axis ECS state

`ecs-workload::Position` and `Velocity` carry X, Y, and Z. The two-argument constructors set Z to zero, while `Position::new3(...)` and `Velocity::new3(...)` construct explicit three-dimensional values. Every storage candidate integrates all three axes and is parity-checked against `ReferenceWorld`.

## 2D workload generator

`ecs-physics` stays intentionally stable. Its constructors, deterministic pair ordering, mass/restitution/friction behavior, contact/support evidence, regression matrices, `falling-boxes`, `BouncingRoomScenario`, and benchmark fixtures operate on the Z=0 plane. The storage benchmarks compute each frame's physics from the storage's own snapshot inside the timed loop and assert exact snapshot parity across storages before timing, so this generator must be a pure function of the snapshot. It is kept until a replacement workload exists (#154).

Reusable low-level AABB and spatial decisions for it remain in the pinned `rust-kernels` crates (`geometry-kernels`, `spatial-kernels`).

## Rapier rigid-box authority

`RapierWorld3d` builds one persistent Rapier world from ECS Lab `RigidBox3d` values (entity, kind, half extents, mass units, material, integer pose, velocity and spin) and steps it once per frame. Rapier owns integration, collision detection, CCD, contact response, restitution, friction and sleeping. ECS Lab owns scenario composition and the quantization seam:

- one Rapier metre is `spatial_scale` integer units; linear velocity is units per second; quaternions are scaled by `ORIENTATION_SCALE` (2³⁰) and angular velocity by `ANGULAR_VELOCITY_SCALE` (10⁶ per rad/s);
- every frame quantizes Rapier's state (round to nearest) into `RigidBox3d` views and integer `SetPosition` / `SetVelocity` operations for each dynamic body;
- the integer view never feeds back into Rapier, so quantization error does not accumulate;
- fixed bodies keep their exact integer pose and never receive operations;
- restitution and friction (`0..=1000` thousandths) combine with Rapier's `Max` rule, matching the 2D solver.

Following the owner's oracle strategy, a scenario moves to `physics-engine` only once physics-engine's Rapier oracle shows it matches.

## Scenarios

- **Bouncing room** (`BouncingRoom3dScenario`): 48 spinning dynamic boxes with varied footprints, masses and materials inside six fixed slabs, 60 Hz, 3600 units per room unit, gravity 1 unit/s².
- **Trebuchet tower**: a 24-mass projectile at 24 m/s through a two-layer, 30-block tower on a 120 × 60 m floor, gravity 10 m/s².
- **Dice**: one tilted die dropped on a fixed slab, gravity 10 m/s².
- **Material traces**: six rotation-locked cubes that differ only in restitution and friction.

Tuning defaults, chosen to match the retired integer solver's per-frame behavior:

| Scenario | Angular damping (1/s) | Solver iterations |
| --- | --- | --- |
| Room | 0.12 (≈ 0.998 per frame) | 6 |
| Tower | 0.24 (≈ 0.996 per frame) | 10 |
| Dice | 0.3 (≈ 0.995 per frame) | 4 |
| Material traces | 0, rotation locked | 4 |

A tower body counts as spinning above 0.05 rad/s.

## Evidence contract

Rapier with default features is f32 and not cross-platform deterministic. Evidence is therefore tolerance-based physical quality plus same-machine replay:

- **Same-machine replay**: two fresh worlds produce identical quantized frames (room, tower over 480 frames, dice). The browser projection check (`scripts/record-rotating-consumers-wasm.mjs`) also asserts reset/dispose replay inside the actual Wasm build.
- **Room**: every dynamic center stays inside the walls (0.25 unit slack) and collider overlap stays below 0.2 units for 240 frames.
- **Tower**: blocks tilt less than 0.01 rad and do not spin before impact; the projectile passes the front face; at least four blocks spin; overlap stays below 0.5 m in every frame (impact transient ≈ 0.42 m, below the 0.75 m smallest half extent); no corner sinks more than 0.2 m into the floor; at rest the overlap is below 0.01 m and nothing spins.
- **Dice**: the off-center impact spins the die; no corner sinks more than 0.1 m in any frame or 0.01 m at rest.
- **Storage seam**: replaying Rapier-generated operations keeps `ReferenceWorld` and `SparseWorld` in exact lockstep, with snapshots equal to the quantized boxes.

The long tower run (`tower_long_horizon_regression`) runs in the Runtime evidence workflow.

## Browser and WebGPU ownership

The `/physics/` Pages demo reads quantized X/Y/Z positions, half extents, body/material metadata, orientation and pair evidence from Wasm. Smooth motion is presentation-only: displayed state snaps exactly to the Rust frame at each step, and JavaScript never integrates velocity or resolves contacts. Camera state is presentation-only.

WebGPU either renders already-quantized state or checks exported AABB pair evidence, accepted only after equality with the CPU result. Neither path feeds back into the simulation.
