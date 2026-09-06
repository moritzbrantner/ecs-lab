# ecs-lab

A focused Rust laboratory for comparing entity-component-system storage models with deterministic workloads, differential correctness tests, and controlled benchmarks.

## Scope

`ecs-lab` is an experiment harness, not an ECS framework. Implementations are compared through shared workloads and observable state rather than forced behind a single performance-sensitive trait.

The storage horizon covers a reference model, a sparse-set world, and later archetype/table experiments. Cross-repository reuse is deliberately narrow: ECS Lab may pin reusable kernel crates from `rust-kernels`, but it does not depend on application repositories such as `collision-lab`.

## Physics workloads

`ecs-workload::Position` and `Velocity` are three-axis ECS components. Their existing two-argument constructors remain source-compatible and place values on the `z = 0` plane; `new3(...)` opts into depth. Both `ReferenceWorld` and `SparseWorld` integrate X/Y/Z identically.

`ecs-physics` remains the deterministic 2D compatibility/reference solver used by the existing benchmark and invariant foundation. `ecs-physics-3d` is the true 3D solver used by the dedicated physics demo. The 3D solver now runs fixed and moving AABBs on one continuous `(x, y, z, t)` timeline: relative-motion swept AABB tests find the earliest pair impact, exact integer-fraction TOIs are ordered by entity pair and X→Y→Z axis order, deterministic Q32.32 subticks consume the remaining timestep after each response, and the bounded event loop fails closed if its supported collision budget is exhausted.

The 3D crate owns `BouncingRoom3dScenario`: 48 dynamic bodies with varied box footprints, masses, materials, and three-axis velocities move inside six fixed AABB slabs (floor, ceiling, ±X and ±Z walls). The scenario can be replayed through both storage implementations and produces exact Rust final-frame 3D AABB pair evidence.

## Interactive Pages demo and WebGPU

The main Pages workbench remains useful for small editable ECS/material experiments. JavaScript owns only interaction state and visualization: the browser synchronizes experiment inputs into WebAssembly and Rust produces the canonical frame.

The dedicated physics playground at [moritzbrantner.github.io/ecs-lab/physics/](https://moritzbrantner.github.io/ecs-lab/physics/) runs the dense `BouncingRoom3dScenario`. Rust/Wasm supplies X/Y/Z positions, 3D collider extents, material metadata, and exact collision-pair words. The browser interpolates only between consecutive authoritative Rust frames for smoother motion.

The camera is presentation state only. Drag inside the scene to orbit, Shift-drag or right-drag to pan, use the mouse wheel to zoom, and double-click or use the in-stage reset control to restore the canonical view. Camera input never feeds the physics solver. The optional timeline matrix also remains presentation-only and is separate from the Rust solver's continuous space-time collision calculations.

WebGPU has two deliberately separate roles in the 3D demo. A raw browser WebGPU render pipeline draws the cutaway room when an adapter is available, with a Canvas 3D fallback otherwise. Separately, the existing WebGPU all-pairs compute path receives the exact Rust-produced final-frame 3D AABBs and is accepted only when its pair bitset matches Rust word-for-word. Neither GPU path owns physics response or time-of-impact decisions.

The deterministic `falling-boxes` fixture remains available as 2D benchmark/regression evidence. See `docs/experiments/physics.md` for the solver, compatibility, continuous-collision, and compute ownership contracts.

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
