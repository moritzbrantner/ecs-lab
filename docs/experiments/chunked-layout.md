# Chunked SoA layout experiment

This experiment compares the existing scalar Position/Velocity representation with explicit
fixed-capacity chunks whose x/y/z and vx/vy/vz fields are stored as separate columns.

The chunked loop is deliberately scalar. Its SoA columns are suitable for compiler
vectorization and future explicit SIMD experiments, while the flat scalar representation
remains the arithmetic oracle. No SIMD speedup is claimed here.

Chunk allocation is explicit: a new block pre-reserves every column to the configured chunk
size, so appending later chunks never copies rows from earlier chunks. Deterministic evidence
records chunk count, partially occupied rows, blocks visited, rows processed, logical chunk
allocations, and row-copy work caused by growth.

The ratchet matrix crosses chunk boundaries at 16, 64, and 256 row block sizes and requires
exact scalar snapshot parity. Optional wall-clock measurements are advisory:

```sh
cargo run --locked --release -p ecs-chunked --bin chunked-benchmark -- --bench
```
