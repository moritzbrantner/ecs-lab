# Physics invariant tests

These tests intentionally exercise the public `ecs-physics` surface. Keep private arithmetic helpers free to evolve as long as the observable deterministic contracts remain unchanged.

When adding a new solver feature, prefer extending the table-driven invariant matrices here before adding a one-off regression test. Named scenarios and cross-storage replay belong in `ecs-physics-scenarios` so storage parity remains a separate concern from solver arithmetic.
