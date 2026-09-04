# Sparse-set experiment

## Question

Can dense component iteration and O(1)-style entity lookup preserve the exact reference-world semantics while avoiding a hash/table lookup for every component access?

## Representation

Each component type owns:

- a sparse entity-ID → dense-index table;
- a dense entity-ID vector;
- a dense component-value vector.

Removal uses `swap_remove` and immediately repairs the sparse index for the moved dense entry. Entity liveness remains a separate canonically ordered set so snapshots are deterministic and independent of dense storage order.

## Correctness oracle

The same seeded `motion_scenario` workloads are replayed through `ReferenceWorld` and `SparseWorld`. Their canonical snapshots must be identical. Targeted tests also cover dense-slot repair and invalid lifecycle behavior.

## Initial benchmark scenario

`motion` spawns N entities with deterministic position/velocity data, performs repeated integration passes, and churns one velocity component per round. The runner reports scenario name, implementation, entity count, rounds, seed, repetitions, elapsed nanoseconds, and the supplied semantic environment fingerprint.

Benchmark timing is evidence, not a PR pass/fail threshold. A result with `environment_fingerprint=unverified` is not suitable for baseline comparison.
