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

Twelve shared native/WASM fixtures cover stationary/sparse parity, actual contacts, near misses, support removal, angular substeps, authoritative damping, output-view isolation, stable identity, reset, independent worlds, authored mutations, invalid edits and late-frame restoration. The actual playground runs frames 0–600; tower runs 0–480 with the original impact, spin and floor assertions. WASM verifies these same endpoints, fixed geometry, and reset/dispose independence.

The development benchmark times complete adapter calls, including scans, physics, conversion and output allocation/writeback. Each route runs in its own native process. Parity precedes timing; only stationary and sparse nonrotating workloads are compared for speed. Counts distinguish one construction/insertion from warm maintenance and unavoidable O(N) output. Engine contact-work counters are a diagnostic subset, not a count of all geometry work.

Retained evidence separates engine body/mapping/metadata/report entries from process maximum resident set size. RSS includes the executable, allocator and temporary peak allocations; it cannot attribute exact retained engine bytes. Native demo cache payload counts include vector capacities and element sizes, excluding allocator bookkeeping. Native trace tests retain full tower body views for inspection; production tower caches only vertices and statistics. Frame caching is presentation/rewind storage, not authoritative physics state.

Measurement results, producer revisions, full trace comparison and their limitations are recorded with this experiment. No timing comparison between physically different rotating trajectories establishes a speedup.

## Results, 2026-09-30

Clean producer `5daec5c8195af097f48e79e297d6a85f10ba7bd4` uses Rust 1.98.0, release builds on x86_64 Linux (Ryzen 7 5700X), and conventions source revision `e6acb5310afaf15c0cba24f87108f5f4ad1bedc3`. [Raw timing/work/memory trials](persistent-physics-measurements-2026-09-30.json) and [full trace classification](persistent-physics-traces-2026-09-30.json) record the producers and scope. The development rebuild consumer is pinned to the identical engine revision at producer `5f85291a3caaf24df224e555e3d64c309641901b` (only its dependency pin differs from #135).

Each timing case proves complete physical/event parity through 40 warm plus 120 measured frames before five interleaved trials. Times include the entire adapter call and output replacement; process RSS comes from separate `/usr/bin/time` invocations.

| Bodies / scene | Rebuild µs/frame | Persistent µs/frame | Rebuild peak RSS KiB | Persistent peak RSS KiB |
| --- | ---: | ---: | ---: | ---: |
| 128 stationary | 273.224 | 3.520 | 3452 | 3504 |
| 128 one moving body | 270.934 | 52.364 | 3328 | 3556 |
| 512 stationary | 1522.103 | 16.536 | 4384 | 4632 |
| 512 one moving body | 1535.268 | 414.128 | 4396 | 4964 |

These are medians on this machine, not performance floors or general contact-scene speedups. Retained physics uses additional memory. Peak RSS cannot establish exact retained bytes or a memory reduction. In each 120-frame measured interval, persistent construction, insertion, input conversion, descriptor/motion commands and output sorting are all zero; angular scans and complete output conversions each visit `120*N` bodies. Stationary frames journal no changed bodies; sparse frames journal one body's motion and sleep state per frame. The reference constructs 120 worlds and source maps, inserts/converts `120*N` inputs, scans its angular policy and converts `120*N` outputs. It retains the current exported body vector between calls, not engine history.

The explicit mutation sample operates on 128 warmed bodies: 64 material edits, 64 teleports, 64 motion commands, 64 remaps, one insertion and one removal. It completes in 4.853 ms on this machine. Counters show 128 changed descriptors, 64 metadata changes, 64 changed motion commands, no reconstruction, and only the new member's full input conversion plus the removed member's output conversion. This is a single measured sample with no speedup claim. Descriptor preparation borrows existing physics motion; these counters do not count every internal contact-admission test or allocation.

The actual playground's 600 advances construct once, insert/convert 54 inputs once, and scan/convert 32,400 outputs. Tower's 480 advances construct once, insert/convert 32 inputs once, and scan/convert 15,360 outputs across 639 angular substeps. Neither makes any descriptor/motion command in ordinary playback. Rewind leaves all work counters unchanged. Final engine/mapping/metadata counts remain 54/54/54 and 32/32/32; report capacity is four. Native trace cache capacity payloads are 4,220,628 bytes (playground) and 4,469,760 bytes (tower including native-only full body views), separate from the retained physics owner.

## Physical classification

Both reference and persistent actual loops finish every requested frame with unchanged metadata and fixed geometry. Native debug/release physical traces agree. Persistent native/WASM comparison matches 32,454 playground body projections and 123,136 tower vertex projections exactly after the documented f32 display conversion; tower event/tail/spin reports agree for all 481 frames (frame zero intentionally has default statistics). Playground exports do not expose linear velocity or per-step event/tail counts; no parity claim is made for those WASM fields or for native-versus-WASM playground bounds beyond the driver's finite/fixed-wall checks.

Playground first differs from the rebuild reference at frame 91, in two quaternion components by one integer unit. Tower first differs at frame 61, in one quaternion component by one unit. The isolated shared quaternion regression proves that reconstruction from an exported integer quaternion can re-normalize it again; the retained owner avoids that operation. Retained sleep is also observably different: playground entity 3 has zero linear/angular motion at frame 139, while the rebuild reference still has small residual motion. Both are expected lifetime effects covered by the stationary/parked/wake and quaternion fixtures. Later contact trajectories amplify these small differences: 510/601 playground frames and 420/481 tower frames differ; tower event/tail counts differ in 378 frames. The full component counts and first differences are preserved in the JSON. This classifies the combined lifetime effects rather than claiming one isolated cause for every later trajectory change. The solver, numerical representation, angular schedule, materials, tolerances and event limits remain unchanged by this adapter migration.

Original tower pre-impact quiescence, projectile impact, induced spin and floor assertions pass through frame 480. Contact parity is checked on short nonrotating fixtures before retained sleep history differs; near misses remain parked, admitted contacts restore real dynamic mass, and support removal wakes dependents without joining independent floor islands. The actual WASM driver separately validates full endpoints, fixed geometry and independent reset/dispose replay.

Reproduce from the producer revision:

```sh
cargo test --locked --workspace
cargo clippy --locked --workspace --all-targets -- -D warnings
cargo fmt --all -- --check
cargo test --locked --release -p ecs-web-demo playground_physical_trace_acceptance -- --ignored --nocapture
cargo test --locked --release -p ecs-web-demo tower_long_horizon_regression -- --ignored --nocapture
bash scripts/build-pages.sh
cargo build --locked --release -p ecs-physics-3d --example persistent-benchmark --features rebuild-reference
target/release/examples/persistent-benchmark compare 512 sparse
/usr/bin/time -v target/release/examples/persistent-benchmark persistent 512 sparse
/usr/bin/time -v target/release/examples/persistent-benchmark rebuild 512 sparse
target/release/examples/persistent-benchmark mutations 128 quiet
```

The Pages build first runs the separate reference-enabled contract example, then builds production WASM with default features. WASM artifact SHA-256: `e40a3980092fb475dea9ba9decfecc5fa601759eb006847897514ccf486b2d91`. No workflow, runner, event-limit or ratchet-epoch changes were added.

Local validation completes with 184 workspace tests passing, two explicitly exercised ignored traces, workspace Clippy (default and all features) and formatting passing, plus the Pages/WASM acceptance above. The declared environment fingerprint verifies. Chromium loads the actual Pages artifact, renders both consumers at 600/480, and returns both to frame zero through the existing rewind controls without page errors. This browser run uses Canvas fallback because the headless machine has no suitable graphics adapter; it does not establish GPU rendering coverage. The reset/dispose API itself is exercised directly by the actual production WASM driver.
