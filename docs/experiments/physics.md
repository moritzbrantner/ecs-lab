# AABB physics workload

## Purpose

Use a small physics loop as a realistic deterministic ECS workload without turning `ecs-lab` into a game engine or physics framework.

The first slice is deliberately 2D and AABB-only:

1. dynamic bodies receive integer gravity;
2. semi-implicit Euler integration advances velocity and position;
3. body pairs are visited in ascending `EntityId` order;
4. `geometry-kernels::aabb_aabb` owns the contact decision;
5. ECS Lab applies minimum-axis positional correction and a fully inelastic equal-mass normal impulse;
6. the step returns ordinary `ecs-workload::Operation` values.

This makes the physics result storage-independent. The reference ECS can replay the generated operations now, and future sparse-set/archetype candidates can consume the same step without a physics-specific storage trait.

## Cross-repository boundary

Collision Lab already consumes reusable collision mechanisms from `rust-kernels`. ECS Lab follows that same ownership boundary rather than depending on the `collision-lab` application crate.

The initial pin is:

- `geometry-kernels` / `spatial-kernels`
- `rust-kernels` revision `986bff4dc8d13a64b90fab3a9f7f02bb8d1aa35e`

The pin matches the collision-kernel revision used by Collision Lab when its analytical AABB/OBB lessons were established.

## Determinism contract

- Body configuration order does not affect the result; bodies are sorted by entity id.
- Pair resolution is single-pass and ordered, so solver order is explicit rather than scheduler-dependent.
- ECS positions and velocities remain integer values.
- Geometry conversion is allowed only while AABB bounds remain exactly representable as `f32` integers.
- Touching counts as contact because that is the reusable geometry-kernel contract.
- Fixed bodies never receive generated component operations.

## Evidence

`PhysicsStepStats` exposes body count, naive candidate-pair count, contacts, and resolved contacts. These are descriptive evidence, not merge thresholds.

## Follow-up horizon

The useful next slices are intentionally separate:

- feed the same physics workload through the sparse-set candidate after PR #3 settles;
- add a named many-body falling-box benchmark and compare reference vs candidate storage costs;
- replace naive all-pairs candidate generation with a pinned broad-phase kernel while preserving exact contact parity;
- add configurable restitution/friction and solver iterations only after the deterministic baseline is measured;
- consider circles and then 3D only after the AABB workload has useful ECS evidence.
