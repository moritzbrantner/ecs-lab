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

Benchmarks remain descriptive evidence, never correctness thresholds. Every storage benchmark computes both implementations before timing and aborts if their snapshots differ. Benchmark helpers also fail closed on setup, physics, or ECS-application errors rather than substituting an empty/default snapshot.

The runner keeps separate workloads for different costs:

- `motion`: baseline ECS replay in `ReferenceWorld` and `SparseWorld`;
- `falling-boxes`: larger end-to-end gravity/contact replay in both storage implementations;
- `material-step-sparse`: Rust solver candidate-pair traversal with deliberately separated bodies and zero contacts;
- `material-step-dense`: Rust solver traversal with overlapping bodies and varied mass/restitution/friction so contact/material response is exercised;
- `bouncing-room`: long-running material scenario replay in both storage implementations.

Smoke mode keeps the fixtures small enough for hosted validation. Full mode increases entity counts, frame horizons, and repetitions for local fingerprinted measurements. Environment fingerprints remain part of every timing record so results from different machines or toolchains are not silently compared as equivalent.

Hosted `Validate` executes the smoke benchmark suite after deterministic format/lint/build/test validation. The job does not compare elapsed time to a threshold; it only requires that every benchmark fixture constructs, proves its parity preconditions, and executes successfully. Hardware-dependent timing remains evidence rather than a merge gate.

WebGPU is intentionally absent from these physics-response benchmarks. Its existing role remains post-physics AABB pair-bitset verification against Rust evidence, not collision response or material simulation.
