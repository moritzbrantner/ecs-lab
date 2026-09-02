# ecs-lab

A focused Rust laboratory for comparing entity-component-system storage models with deterministic workloads, differential correctness tests, and controlled benchmarks.

## Scope

`ecs-lab` is an experiment harness, not an ECS framework. Implementations are compared through shared workloads and observable state rather than forced behind a single performance-sensitive trait.

The first implementation horizon covers a reference model, a sparse-set world, and an archetype/table world. The repository intentionally has no dependency on other `moritzbrantner/*` repositories during that horizon.

## Development

```sh
bash scripts/codex-environment.sh setup
cargo test --workspace
cargo clippy --workspace --all-targets -- -D warnings
cargo fmt --all -- --check
```

`coding-tooling` is the canonical semantic validation interface in CI. See `AGENTS.md` and `CONTEXT.md` for repository-specific boundaries.
