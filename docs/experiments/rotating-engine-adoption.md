# Rotating-engine adoption, 2026-09-30

## Boundary and control

The playground and tower adopt `physics-engine` revision `221bf2e08cca0955b3c34b780b89ec12f191f070`, replacing `9635eb79cd72c3308f26a5d9840e88fc34fb7535`. The adapter still constructs a fresh legacy rotating-box world for each frame. Scenario populations, inputs, material values, angular substep selection, solver budgets and once-per-frame angular damping are unchanged. This is a prerequisite for [persistent lifetime #133](https://github.com/moritzbrantner/ecs-lab/issues/133), whose acceptance remains outstanding.

The new pin includes floating-point CPU math by default and intervening contact, wake and response fixes. ECS positions, velocities, orientations and metadata retain their existing integer compatibility representation. This change does not migrate the consumer to `approximate::World`. The upstream atomic interval API is available but is not called by this adapter yet.

The direct AABB kernel pin remains `986bff4dc8d13a64b90fab3a9f7f02bb8d1aa35e`. The new engine brings its own geometry/spatial kernels at `92c1e17b7922e2199c408c8dbe7bb2e3df802065`; Cargo resolves both revisions. No unrelated existing package version was upgraded.

## Physical comparison

Native tests record every body's identity, kind, extents, mass, material, position, linear velocity, orientation and angular velocity. Old-pin traces use producer `490583f716c2ac1f90608a2eeb7a5fee5a6e92f0`; matched new-pin traces use `3b8799d02ea4fa6d750cf85cdd9ee3ee0a9a35a7`. Both run the actual consumer loops. The matched interval is frames 0–600 for the playground and 0–240 for the tower, including all 54 and 32 bodies respectively.

Body metadata and initial state are identical. Playground physical states agree through frame 26; the first difference is body 0's position at frame 27: `[-60481, 8804, -29519]` becomes `[-60480, 8803, -29520]`. Tower states agree through frame 37; body 1 first differs at frame 38: `[-10196, 21528, 6]` becomes `[-10198, 21526, 9]`. Later positions, velocities, orientations and angular velocities differ. Tower sampled-event/tail-count pairs differ in 203 matched frames.

These are engine-adoption effects. The comparison does not isolate individual numerical or contact changes, and it cannot support a persistence or speedup claim. Future lifetime comparisons must use the fresh-world adapter on this new pin as their control. Raw early playground traces used zero placeholders for event counters that the playground frame does not retain; those fields are unobserved, not measured zero. The recorder now emits `null`, and playground event parity is not claimed.

## Native, WASM and browser acceptance

The native suite passes 172 default workspace tests with two diagnostic/long tests ignored. Workspace Clippy and formatting checks pass. Both ignored consumer tests pass separately. The tower retains its original quiescence, multi-block spin and floor-penetration assertions; its acceptance now covers the full advertised 480 frames. That additional new-pin coverage uses producer `1010b0d9b822755ac35d2f9a957885948232dafe` and has no old-pin 480-frame comparison.

The actual production WASM exports pass 1,082 frames: playground 0–600 and tower 0–480. The driver checks population, finite projections, positive extents, unchanged fixed walls and a nonzero, constant tower-floor sentinel that catches silent getter defaults after a failed frame. Native floor-quality assertions supply the stronger contact check. Within each pin, all 32,454 playground body projections match native and WASM exactly after the public `f32` conversions, including positions, orientations, angular velocities, extents and material metadata. This is evidence for the recorded targets, not a universal cross-platform replay guarantee.

The Pages build passes. Chromium reaches playground frame 600 and tower frame 480 through the real page controls with no page errors and renders the toppled tower. This headless environment has no WebGPU adapter, so the verified renderer is the Canvas fallback; GPU rendering coverage is not claimed.

## Reproduction

Run native captures with `cargo test --locked --release -p ecs-web-demo playground_physical_trace_acceptance -- --ignored --nocapture` and `cargo test --locked --release -p ecs-web-demo tower_long_horizon_regression -- --ignored --nocapture`. Parse lines beginning `PHYSICAL_TRACE ` as JSON. Production WASM acceptance runs inside the existing `scripts/build-pages.sh`; the same driver can be called directly with a WASM path and output JSON path. No additional workflow was added.

[Machine-readable evidence](rotating-engine-adoption-2026-09-30.json) records producers, artifact hashes, comparison counts and target metadata. Validation elapsed times were collected during concurrent checks and are not benchmarks. Shared conventions resolved at source revision `e6acb5310afaf15c0cba24f87108f5f4ad1bedc3`; the declared environment verification passed without changing declared versions.
