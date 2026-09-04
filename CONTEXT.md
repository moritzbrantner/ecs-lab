# ecs-lab context

## Goal

Measure how different ECS storage strategies behave under identical deterministic workloads while keeping correctness independently checkable.

## Vocabulary

**Workload** — A deterministic sequence of entity/component operations independent of a storage implementation.

**Observable snapshot** — Canonically ordered entity/component state used for differential comparison.

**Reference model** — Deliberately simple implementation optimized for clarity and correctness, not speed.

**Candidate** — An implementation under comparison, such as sparse-set or archetype storage.

**Parity** — Equivalent observable result for the same workload, including equivalent failure behavior where the contract exposes failure.

**Benchmark scenario** — A named, versionable workload configuration with explicit entity count, operation mix/seed, measured unit, and environment identity.

**Physics workload** — A storage-independent transformation from an observable snapshot plus body configuration into ordinary ECS workload operations and descriptive contact evidence.

**Collision-kernel boundary** — Reusable collision decisions live in pinned `rust-kernels` crates. ECS Lab owns ECS-facing scheduling/integration/response experiments and does not depend on the Collision Lab application.

## Current boundary

The storage experiments remain Rust and CPU-local. Rendering, networking, persistence, Worldgen, and application-level dependencies remain outside the graph.

Cross-repository reuse is allowed only where a task explicitly establishes a reusable-kernel boundary. The first such boundary is the AABB physics workload, which pins `geometry-kernels` and `spatial-kernels` from `rust-kernels` while preserving `collision-lab` as an independent teaching/visualization consumer.
