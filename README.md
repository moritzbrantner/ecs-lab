# ecs-lab

A focused Rust laboratory for comparing entity-component-system storage models with deterministic workloads, differential correctness tests, and controlled benchmarks.

## Scope

`ecs-lab` is an experiment harness, not an ECS framework. Implementations are compared through shared workloads and observable state rather than forced behind a single performance-sensitive trait.

The storage horizon covers a reference model, a sparse-set world, and later archetype/table experiments. Cross-repository reuse is deliberately narrow: ECS Lab may pin reusable kernel crates from `rust-kernels`, but it does not depend on application repositories such as `collision-lab`.

## Physics workload

`ecs-physics` adds a deterministic 2D AABB physics step over the observable snapshot contract. It applies integer gravity and integration, delegates AABB contact decisions to the same reusable `rust-kernels` geometry seam used by Collision Lab, resolves contacts in stable entity-id order, and emits ordinary ECS workload operations.

`ecs-physics-scenarios` owns named experiment fixtures around that core. The first `falling-boxes` scenario is replayed through both `ReferenceWorld` and `SparseWorld`, has a benchmark profile, and can produce an exact Rust/CPU AABB pair bitset for browser compute experiments.

## Optional WebGPU

The Pages demo keeps CPU/Rust semantics authoritative. When WebGPU is enabled, the browser sends the Rust-generated falling-box AABBs through an all-pairs compute shader adapted from Collision Lab's proven WebGPU collision path. The GPU result is compared word-for-word with the Rust pair bitset. GPU timings are displayed only after exact parity succeeds; browsers without WebGPU continue on the CPU path without changing behavior.

See `docs/experiments/physics.md` for the solver, benchmark, and compute ownership contract.

## Development

```sh
bash scripts/codex-environment.sh setup
cargo test --workspace
cargo clippy --workspace --all-targets -- -D warnings
cargo fmt --all -- --check
```

Run the named benchmark suite with:

```sh
bash scripts/benchmark.sh smoke
bash scripts/benchmark.sh full
```

`coding-tooling` is the canonical semantic validation interface in CI. See `AGENTS.md` and `CONTEXT.md` for repository-specific boundaries.
