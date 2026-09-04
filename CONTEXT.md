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

**Compute evidence** — Optional accelerated work whose output is compared with canonical CPU evidence before timing or performance claims are accepted.

## Current boundary

Storage and physics semantics remain Rust/CPU-owned. Rendering, networking, persistence, Worldgen, and application-level dependencies remain outside the core graph.

Cross-repository reuse is allowed only where a task explicitly establishes a reusable-kernel boundary. The AABB physics workload pins `geometry-kernels` and `spatial-kernels` from `rust-kernels` while preserving `collision-lab` as an independent teaching/visualization consumer.

The browser demo may use WebGPU as optional compute evidence. WebGPU availability is never required for repository correctness or headless CI, and GPU timing is invalid unless the produced collision-pair bitset exactly matches the Rust/CPU bitset for the same Rust-owned frame.
