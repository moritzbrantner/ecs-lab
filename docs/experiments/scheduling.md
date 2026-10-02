# Access-aware scheduling experiment

Systems declare explicit component read/write sets. A deterministic first-fit planner sorts by
stable system ID before constructing batches, so caller order and thread timing cannot alter
the schedule.

The reference scenario has three systems:

1. Integrate reads Velocity and writes Position.
2. RegenerateEnergy reads/writes Energy and can run in the same batch as Integrate.
3. Accelerate reads Position and writes Velocity, so it runs after the first batch.

The experiment executes this schedule three ways: serially, with independent systems in the
first batch on scoped threads, and with each system partitioned over configurable worker
counts. Exact snapshots must match for 1, 2, 4, and 7 partition workers.

The deterministic ratchet requires two batches per round, one parallel-capable batch, one
schedule build, and the expected useful entity-system runs. Wall-clock scaling is advisory
because hosted-runner thread scheduling is not a correctness signal.

```sh
cargo run --locked --release -p ecs-scheduler --bin scheduler-benchmark -- --bench
```
