# Reference model experiment

## Question

What is the smallest explicit behavior contract that future ECS storage candidates can be compared against without forcing their internal layout behind a shared ECS abstraction?

## Contract

A workload is an ordered list of entity/component operations. The observable result is a canonically ordered `WorldSnapshot`. Invalid entity lifecycle operations produce explicit `WorkloadError` values.

The reference implementation uses `BTreeMap` intentionally. Its purpose is clarity and deterministic ordering, not performance.

## Initial semantics

- Spawning an already-live entity is an error.
- Despawning or updating a missing entity is an error.
- Removing a missing component from a live entity is a successful no-op.
- Integration updates only entities that currently have both position and velocity.
- Position integration uses saturating integer arithmetic so the experiment contract is deterministic at numeric boundaries.

Future candidates must match these observable semantics before their performance evidence is considered meaningful.
