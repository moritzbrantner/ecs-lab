# ecs-lab

A focused Rust laboratory for comparing entity-component-system storage models with deterministic workloads, differential correctness tests, and controlled benchmarks.

## Scope

`ecs-lab` is an experiment harness, not an ECS framework. Implementations are compared through shared workloads and observable state rather than forced behind a single performance-sensitive trait.

The initial storage horizon covers a reference model, a sparse-set world, and an archetype/table world. That horizon intentionally started without cross-repository dependencies. The physics workload is an explicitly approved reuse boundary: ECS Lab may pin reusable kernel crates from `rust-kernels`, but it does not depend on application repositories such as `collision-lab`.

## Physics workload

`ecs-physics` adds a deterministic 2D AABB physics step over the existing observable snapshot contract. It applies integer gravity and integration, delegates AABB contact decisions to the same reusable `rust-kernels` geometry seam used by Collision Lab, resolves contacts in stable entity-id order, and emits ordinary ECS workload operations.

This keeps physics useful as an ECS workload instead of growing a second physics framework. See `docs/experiments/physics.md` for the solver and ownership contract.

## Development

```sh
bash scripts/codex-environment.sh setup
cargo test --workspace
cargo clippy --workspace --all-targets -- -D warnings
cargo fmt --all -- --check
```

`coding-tooling` is the canonical semantic validation interface in CI. See `AGENTS.md` and `CONTEXT.md` for repository-specific boundaries.
