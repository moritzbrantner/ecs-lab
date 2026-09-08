# Tower Rust/Wasm `wgpu` renderer experiment

The tower-destruction scene now has an explicit renderer experiment rather than extending the physics crate with browser GPU types.

## Boundary

- `ecs-web-demo` remains the authoritative source of tower frame state and exact oriented-box collision vertices.
- `experiments/tower-wgpu-renderer` is a standalone WebAssembly adapter pinned to `wgpu 30.0.1` and `wasm-bindgen 0.2.127`.
- JavaScript owns browser input, playback, fallback selection, and transfer of one immutable frame into the renderer.
- Rust/Wasm owns the WebGPU surface, camera/view-projection matrix, GPU buffers, shaders, depth target, command encoding, and draw submission.
- The 48-box room already used raw JavaScript WebGPU, so this experiment compares the tower's old Canvas2D projection with a Rust/Wasm `wgpu` path; it is not a Three.js-to-`wgpu` comparison inside this repository.

## Ball projectile scope

The trebuchet projectile is presented as a shaded sphere. Its center and radius are derived from the Rust-owned projectile frame vertices, so browser code does not invent a trajectory or scale.

The current tower world solver is still `RigidBox3d`-only. The sphere is therefore a presentation shape over the existing equal-extents OBB collision proxy. Do not describe this slice as sphere↔OBB collision response. A true physical ball requires a reusable mixed sphere↔oriented-box response path in `physics-3d`, not a tower-only special case.

## Camera

The tower stage follows the room-camera interaction model:

- drag to orbit;
- Shift-drag or right-drag to pan;
- wheel to zoom;
- arrow keys to orbit;
- `+` / `-` to zoom;
- `R`, double-click, or the reset-camera button to restore the default view.

Camera changes affect presentation only. They never step or mutate physics.

## Evidence

`moonlight.eval.toml` includes the Pages/Wasm build so baseline and candidate behavior evaluation covers the new renderer's compilation and packaging seam. The existing runtime-profiler PR canary still compares the deterministic ECS benchmark before and after this change, which is the appropriate guard against accidentally moving physics work into the renderer or slowing the authoritative workload.

The tower page reports CPU-side draw/submission duration using `performance.now()` for the active renderer. That number includes JavaScript↔Wasm transfer and CPU submission work; it is not GPU-completion time and must not be used as a GPU throughput claim.

`runtime-profiler` currently has a process-command collector rather than a browser/WebGPU frame collector, so this experiment deliberately does not produce a synthetic renderer-retention score. A later profiler slice can add a browser collector with reproducible adapter metadata and, where supported, GPU timestamp-query evidence.
