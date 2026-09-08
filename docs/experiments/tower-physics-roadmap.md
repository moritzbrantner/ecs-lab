# Tower destruction physics roadmap

The tower fixture is now useful enough that its next work should deepen physical truth instead of adding presentation-only effects.

## Integrated baseline

The current Pages experiment already has:

- a 30-block Rust-owned masonry fixture on the angular rigid-box world;
- gravity, mass, restitution, friction, OBB collision response, stabilization, and angular velocity;
- deterministic broad-phase acceleration with exact OBB narrow-phase truth;
- bounded sampled rotating-contact handling rather than an analytic rotational-CCD claim;
- a Rust/Wasm `wgpu` renderer with orbit/pan/zoom controls and Canvas fallback;
- a visually spherical trebuchet projectile whose center and radius come from Rust-owned frame state.

The important remaining mismatch is that the projectile is still an equal-extents `RigidBox3d` collision proxy. Rendering a sphere does not make the physical collider spherical.

## Next vertical slices

### 1. Sphere <-> OBB contact geometry -- active

Add a reusable deterministic fixed-point closest-feature query between a sphere and an oriented box. Keep this as geometry evidence only: no tower-only response branch and no continuous-collision claim.

Acceptance:

- face, edge/corner, rotated-box, interior-center, and invalid-input regressions;
- stable nearest-face selection when a sphere center is inside an OBB;
- integer/fixed-point arithmetic only.

### 2. Mixed-shape normal response and stabilization

Add a reusable sphere <-> OBB normal impulse path that can transfer off-center impact torque into the box. Add deterministic penetration correction while keeping fixed bodies immovable and preserving canonical pair ordering.

Do not add friction in the same slice unless the normal-response contract is already stable.

### 3. Replace the tower projectile proxy with a true sphere

Introduce a solver-owned rigid sphere state and run the trebuchet projectile through the mixed-shape world. The renderer should then consume a real physical radius rather than deriving a display sphere from box vertices.

Acceptance should keep the current tower guarantees and add:

- the projectile contacts the tower through sphere <-> OBB geometry only;
- no pre-impact tower motion;
- a glancing sphere hit produces block angular motion;
- exact deterministic replay;
- no browser-owned trajectory, radius, collision point, or impulse.

### 4. Sphere <-> OBB friction and projectile spin

Add tangential response after the normal mixed-shape path is stable. A stone ball should be able to pick up spin from ground/tower contact without changing collision authority or repeatedly injecting energy across solver passes.

### 5. Better masonry contact manifolds

The current OBB response intentionally reduces contact to bounded single-point evidence in several rotated cases. Add deterministic polygon clipping / multi-point reduction for arbitrary rotated face contacts so stacked blocks settle and transfer load more faithfully.

This is more valuable for the tower than adding more renderer effects.

### 6. Richer tower construction

Once contact quality is strong enough, make the fixture structurally more interesting using the same solver primitives:

- alternating bonded masonry courses rather than a simple repeated grid;
- corners, buttresses, parapets, and bounded openings/arches built from rigid pieces;
- heterogeneous block masses/materials where they represent a physical distinction rather than visual variety;
- multiple deterministic launch positions/velocities as scenario inputs rather than browser-authored physics.

Keep each construction change paired with physical acceptance evidence; do not grow the scene only for object count.

### 7. Breakable mortar / structural constraints

After ordinary contact stacking is trustworthy, add a small deterministic breakable-constraint model for mortar or ties between selected masonry pieces. Break decisions should consume Rust-owned force/impulse evidence and have explicit thresholds and stable ordering.

This is the point where tower destruction can model structural cohesion instead of only loose-block collapse.

### 8. Resting islands and sleeping

If richer masonry makes long settled simulations expensive or noisy, add deterministic resting-island/sleeping semantics with explicit wake conditions. Sleeping must be an optimization over equivalent physical state, not a browser-side animation shortcut.

### 9. Continuous mixed-shape and rotational collision handling

Extend the bounded rotating-contact work to sphere <-> OBB motion so a fast ball or fast rotating block cannot tunnel between coarse samples. Preserve explicit budgets and fail-closed behavior; do not relabel sampled search as analytic CCD.

### 10. Performance evidence after realism

Only after the mixed-shape and masonry contracts are stable, measure the larger tower through the existing deterministic profiler/broad-phase evidence. Keep hardware-dependent renderer timing separate from simulation correctness and avoid turning software-GPU CI measurements into hardware-GPU claims.

## Boundary

This roadmap keeps ECS Lab a deterministic physics laboratory rather than a general game engine. Mesh fracture, arbitrary soft bodies, production networking/rollback, and a broad engine API remain out of scope until multiple experiments prove a reusable need.
