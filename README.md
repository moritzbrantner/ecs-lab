# ecs-lab

A focused Rust laboratory for comparing entity-component-system storage models with deterministic workloads, differential correctness tests, and controlled benchmarks.

## Scope

`ecs-lab` is an experiment harness, not an ECS framework. Implementations are compared through shared workloads and observable state rather than forced behind a single performance-sensitive trait.

The storage horizon covers a reference model, a sparse-set world, and later archetype/table experiments. Cross-repository reuse is deliberately narrow: ECS Lab may pin reusable kernel crates from `rust-kernels`, but it does not depend on application repositories such as `collision-lab`.

## Physics workload

`ecs-physics` adds a deterministic 2D AABB physics step over the observable snapshot contract. It applies integer gravity and integration, delegates AABB contact decisions to the same reusable `rust-kernels` geometry seam used by Collision Lab, resolves contacts in stable entity-id order, and emits ordinary ECS workload operations.

`ecs-physics-scenarios` owns named experiment fixtures around that core. The first `falling-boxes` scenario is replayed through both `ReferenceWorld` and `SparseWorld`, has a benchmark profile, and can produce an exact Rust/CPU AABB pair bitset for browser compute experiments.

## Interactive Pages demo and WebGPU

The Pages workbench lets you add entities, select them, and edit Position, Velocity, and Collider components. JavaScript owns only interaction state and visualization: the browser synchronizes those experiment inputs into WebAssembly, `ReferenceWorld` evaluates the frame, and the Rust physics path produces the canonical interactive collision-pair bitset.

WebGPU is optional. When enabled, the browser sends the current Rust-evaluated frame's AABBs through the all-pairs compute shader adapted from Collision Lab's proven WebGPU collision path. The GPU result is accepted only when its pair bitset matches the Rust CPU evidence word-for-word. Browsers without WebGPU continue on the Rust path without changing behavior.

The deterministic `falling-boxes` WebGPU fixture remains available as benchmark evidence and regression coverage. See `docs/experiments/physics.md` for the solver, benchmark, and compute ownership contract.

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
