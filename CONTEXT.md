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

## Initial boundary

The first horizon is entirely Rust and CPU-local. Rendering, networking, persistence, `rust-kernels`, Worldgen, and other repositories are outside the dependency graph.
