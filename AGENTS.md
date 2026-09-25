# AGENTS.md

## Repository purpose

`ecs-lab` is a scientific/engineering laboratory for ECS storage and query strategies. Optimize for reproducible experiments, correctness parity, and inspectable trade-offs rather than framework ergonomics or public API stability.

## Boundaries

- Do not turn this repository into a general-purpose ECS framework.
- Do not add rendering, windowing, game-engine, or UI dependencies unless a later task explicitly establishes that boundary.
- Cross-repository dependencies require an explicit reusable-kernel boundary and an immutable revision pin. Prefer the owning low-level crate; do not depend on application/lab repositories merely to reuse their transitive implementation.
- The approved physics boundary reuses `geometry-kernels` / `spatial-kernels` from `rust-kernels`; `collision-lab` remains an independent consumer and visualization/teaching repository.
- Shared workloads and observable snapshots are the primary comparison seam. Do not force implementations through a common trait when doing so would distort the implementation being measured.
- A candidate optimization must prove parity with the reference model before benchmark results are treated as meaningful.

## Visualization boundary

- The current repository remains a headless ECS experiment lab; do not add UI dependencies merely to satisfy interface conventions.
- If a browser visualization or interactive experiment surface is introduced, explicitly adopt the current shared `ui` conventions from `moritzbrantner/coding-agent-conventions`, especially `PRINCIPLE-009`, `UI-008`, `UI-012`, and `UI-013`.
- Such a surface should manipulate and inspect the existing authoritative workload/snapshot model rather than create a second ECS state model in presentation code.
- Keep exact experiment parameters available, give each browser gesture one owner, and protect browser-specific interaction geometry when it becomes part of the experiment.

## Validation

Run the narrowest affected checks first, then the repository gate:

```sh
cargo test --workspace
cargo clippy --workspace --all-targets -- -D warnings
cargo fmt --all -- --check
```

Before classifying a failure as a source regression, verify the declared environment with `coding-tooling environment verify --json` when the tooling checkout is available.

Deterministic gates do not retry until green. Hardware/profiler-dependent evidence belongs in an explicitly non-blocking performance/canary path.

## Repository knowledge

- Keep the concise domain vocabulary in `CONTEXT.md`.
- Put durable experiment notes under `docs/experiments/` only when an experiment exists.
- Use `docs/adr/` only for consequential decisions that are expensive to reverse.
- Keep TODOs actionable: `TODO: <action>` or `TODO(#123): <action>`.
