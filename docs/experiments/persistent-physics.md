# Persistent rotating physics adapter

Issue [#133](https://github.com/moritzbrantner/ecs-lab/issues/133) changes the lifetime of the existing rotating-box consumer. The engine pin is `46d4eed291762815becde11f79c97019857533d9`; this remains the integer compatibility API with floating-point CPU math. The preceding [engine adoption experiment](rotating-engine-adoption.md) isolates the earlier pin change. Floating-state solver migration remains separate.

## Ownership and commands

`PersistentPhysicsWorld3d` owns one `physics-engine` world. Physics owns pose, linear/angular velocity, sleep, parked-body support and solver history. ECS owns component-shaped body metadata and stable EntityId-to-BodyId mappings. Converted output is an observable view. Neither actual playground nor tower reads cached frames back into setters. Rewind reads the cache without advancing physics.

Initial engine IDs preserve entity ordering. Remapping preserves engine identity; removal followed by entity reuse receives a fresh monotonically allocated identity. Output remains in EntityId order after edits. This deliberately separates consumer identity from physics pair traversal order after topology changes.

Insertion/removal, explicitly authored replacement, shape/material/mass/kind updates, teleport, motion authority and intended motion have individual commands. Shape/material updates borrow current simulated motion directly from physics. Unchanged quaternion/motion authority edits preserve exact simulated quaternion state. Invalid edits do not change mappings or physical state. There is no per-frame descriptor or motion replay.

Construction owns the lifetime; `reset` builds replacement state before discarding the old world; dropping the owner disposes it. The actual WASM playground and tower additionally export independent reset/dispose functions. Reset clears physics and the rewind cache. Disposal drops both; the next frame read creates a fresh scenario. The static mutex itself remains allocated.

## Complete frames

The same angular substep policy scans current physics angular velocities. The engine advances the entire rational frame through its atomic interval API. Angular damping changes authoritative engine velocity once after all substeps, including zero-duration frames. Late errors restore physical, sleep and parked support state; intended commands made before the frame remain applied. Discarded engine work remains counted.

Event sums are checked inside the engine transaction. Projection after commit only widens integer coordinates and reads internally maintained metadata, so no fallible ECS conversion remains after physics commits. No ordinary full-world rollback snapshot is made by the adapter. Engine journals retain only touched before-images, although policy, damping and complete output still scan bodies.

The old rebuild function is available only to native unit tests or the explicit `rebuild-reference` feature. Production demo builds omit that feature. The separate WASM contract example opts in for comparison and is not copied into Pages assets.

## Acceptance and measurement

Eleven shared native/WASM fixtures cover stationary/sparse parity, actual contacts, near misses, support removal, angular substeps, authoritative damping, output-view isolation, stable identity, reset, independent worlds, authored mutations, invalid edits and late-frame restoration. The actual playground runs frames 0–600; tower runs 0–480 with the original impact, spin and floor assertions. WASM verifies these same endpoints, fixed geometry, and reset/dispose independence.

The development benchmark times complete adapter calls, including scans, physics, conversion and output allocation/writeback. Each route runs in its own native process. Parity precedes timing; only stationary and sparse nonrotating workloads are compared for speed. Counts distinguish one construction/insertion from warm maintenance and unavoidable O(N) output. Engine contact-work counters are a diagnostic subset, not a count of all geometry work.

Retained evidence separates engine body/mapping/metadata/report entries from process maximum resident set size. RSS includes the executable, allocator and temporary peak allocations; it cannot attribute exact retained engine bytes. Native demo cache payload counts include vector capacities and element sizes, excluding allocator bookkeeping. Native trace tests retain full tower body views for inspection; production tower caches only vertices and statistics. Frame caching is presentation/rewind storage, not authoritative physics state.

Measurement results, producer revisions, full trace comparison and their limitations are recorded with this experiment. No timing comparison between physically different rotating trajectories establishes a speedup.
