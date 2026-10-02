# Deferred command buffer experiment

The command buffer separates structural mutation from the code that records it. Exact
`Operation` commands are committed in FIFO order and preserve the existing archetype
world's failure and transition semantics. A failed command stays buffered together with
all later commands.

Deferral alone is not treated as an optimization: constructing an entity as
Spawn → Position → Velocity still performs two archetype-table transitions whether those
three commands execute immediately or at a later commit boundary.

The separate `SpawnBundle` command is valid only when intermediate construction states
are not observable. It inserts directly into the final component-shape table. The fixed
ratchet therefore requires, for N Position+Velocity entities:

- immediate commands: 3N, transitions: 2N;
- deferred exact commands: 3N, transitions: 2N;
- bundled commands: N, transitions: 0;
- all three final snapshots exactly equal.

Optional advisory timings are available with:

```sh
cargo run --locked --release -p ecs-command-buffer --bin command-buffer-benchmark -- --bench
```
