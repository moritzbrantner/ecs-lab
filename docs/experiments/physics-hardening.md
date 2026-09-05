# Physics foundation hardening

The physics experiment is only useful if deterministic response semantics remain trustworthy while the workloads grow. This hardening layer therefore treats correctness evidence as a prerequisite for performance evidence.

## Blocking invariants

The integration suite exercises the public `ecs-physics` boundary rather than private solver helpers. It verifies:

- body and snapshot input ordering cannot change a material response;
- unequal-mass integer collision response keeps momentum error within the explicit one-velocity-quantum-per-body rounding bound;
- friction does not materially increase tangent relative speed, and maximum friction collapses it;
- maximum restitution against a fixed surface preserves normal speed magnitude;
- fixed bodies never receive generated mutations;
- support evidence stays sorted and unique across multiple supporting contacts;
- physics emits only ordinary `SetPosition` / `SetVelocity` ECS operations, in entity order;
- all permutations of a multi-contact fixture produce the same step;
- exact-f32 geometry limits fail closed before collision evidence is produced;
- identical snapshots produce bit-for-bit identical physics evidence across repeated runs.

The scenario suite separately runs the named material and falling-box scenarios through both `ReferenceWorld` and `SparseWorld`, comparing snapshots and `PhysicsStep` evidence frame by frame over longer horizons.

## Performance evidence

Benchmarks remain descriptive evidence, never correctness thresholds. A benchmark fixture must first prove that its reference and candidate results match. Environment fingerprints remain part of every timing record so results from different machines or toolchains are not silently compared as equivalent.

Dense-contact and sparse-contact solver fixtures should be kept separate: candidate-pair traversal and actual contact/material response are different costs. End-to-end scenario benchmarks should continue to include both reference and sparse storage so storage effects are not confused with solver effects.
