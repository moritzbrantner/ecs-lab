use ecs_reference::ReferenceWorld;
use ecs_workload::{
    EntityId, EntitySnapshot, Operation, Position, Velocity, Workload, WorldSnapshot,
};

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct Row {
    entity: EntityId,
    position: Position,
    velocity: Velocity,
}

#[derive(Clone, Debug, Default, Eq, PartialEq)]
struct FlatMotionWorld {
    entities: Vec<EntityId>,
    positions: Vec<Position>,
    velocities: Vec<Velocity>,
}

impl FlatMotionWorld {
    fn with_capacity(capacity: usize) -> Self {
        Self {
            entities: Vec::with_capacity(capacity),
            positions: Vec::with_capacity(capacity),
            velocities: Vec::with_capacity(capacity),
        }
    }

    fn push(&mut self, entity: EntityId, position: Position, velocity: Velocity) {
        self.entities.push(entity);
        self.positions.push(position);
        self.velocities.push(velocity);
    }

    fn integrate(&mut self, ticks: i32) -> u64 {
        let ticks = i64::from(ticks);
        for (position, velocity) in self
            .positions
            .iter_mut()
            .zip(self.velocities.iter().copied())
        {
            integrate(position, velocity, ticks);
        }
        as_u64(self.entities.len())
    }

    fn snapshot(&self) -> WorldSnapshot {
        WorldSnapshot::new(
            self.entities
                .iter()
                .copied()
                .zip(self.positions.iter().copied())
                .zip(self.velocities.iter().copied())
                .map(|((id, position), velocity)| EntitySnapshot {
                    id,
                    position: Some(position),
                    velocity: Some(velocity),
                })
                .collect(),
        )
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
struct Chunk {
    entities: Vec<EntityId>,
    x: Vec<i64>,
    y: Vec<i64>,
    z: Vec<i64>,
    vx: Vec<i32>,
    vy: Vec<i32>,
    vz: Vec<i32>,
}

impl Chunk {
    fn with_capacity(capacity: usize) -> Self {
        Self {
            entities: Vec::with_capacity(capacity),
            x: Vec::with_capacity(capacity),
            y: Vec::with_capacity(capacity),
            z: Vec::with_capacity(capacity),
            vx: Vec::with_capacity(capacity),
            vy: Vec::with_capacity(capacity),
            vz: Vec::with_capacity(capacity),
        }
    }

    fn push(&mut self, entity: EntityId, position: Position, velocity: Velocity) {
        self.entities.push(entity);
        self.x.push(position.x);
        self.y.push(position.y);
        self.z.push(position.z);
        self.vx.push(velocity.x);
        self.vy.push(velocity.y);
        self.vz.push(velocity.z);
    }

    fn integrate(&mut self, ticks: i32) -> u64 {
        let ticks = i64::from(ticks);
        for index in 0..self.entities.len() {
            self.x[index] = self.x[index]
                .saturating_add(i64::from(self.vx[index]).saturating_mul(ticks));
            self.y[index] = self.y[index]
                .saturating_add(i64::from(self.vy[index]).saturating_mul(ticks));
            self.z[index] = self.z[index]
                .saturating_add(i64::from(self.vz[index]).saturating_mul(ticks));
        }
        as_u64(self.entities.len())
    }

    fn append_snapshot(&self, output: &mut Vec<EntitySnapshot>) {
        for index in 0..self.entities.len() {
            output.push(EntitySnapshot {
                id: self.entities[index],
                position: Some(Position::new3(self.x[index], self.y[index], self.z[index])),
                velocity: Some(Velocity::new3(
                    self.vx[index],
                    self.vy[index],
                    self.vz[index],
                )),
            });
        }
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
struct ChunkedMotionWorld {
    chunk_size: usize,
    chunks: Vec<Chunk>,
}

impl ChunkedMotionWorld {
    fn new(chunk_size: usize) -> Self {
        assert!(chunk_size > 0, "chunk_size must be non-zero");
        Self {
            chunk_size,
            chunks: Vec::new(),
        }
    }

    fn push(&mut self, entity: EntityId, position: Position, velocity: Velocity) {
        let needs_chunk = self
            .chunks
            .last()
            .is_none_or(|chunk| chunk.entities.len() == self.chunk_size);
        if needs_chunk {
            self.chunks.push(Chunk::with_capacity(self.chunk_size));
        }
        let Some(chunk) = self.chunks.last_mut() else {
            panic!("a chunk must exist after capacity growth");
        };
        chunk.push(entity, position, velocity);
    }

    fn integrate(&mut self, ticks: i32) -> (u64, u64) {
        let mut rows = 0_u64;
        for chunk in &mut self.chunks {
            rows = rows.saturating_add(chunk.integrate(ticks));
        }
        (as_u64(self.chunks.len()), rows)
    }

    fn snapshot(&self) -> WorldSnapshot {
        let entity_count = self
            .chunks
            .iter()
            .map(|chunk| chunk.entities.len())
            .sum();
        let mut entities = Vec::with_capacity(entity_count);
        for chunk in &self.chunks {
            chunk.append_snapshot(&mut entities);
        }
        WorldSnapshot::new(entities)
    }

    fn partial_chunk_rows(&self) -> usize {
        self.chunks
            .last()
            .map_or(0, |chunk| {
                if chunk.entities.len() == self.chunk_size {
                    0
                } else {
                    chunk.entities.len()
                }
            })
    }
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct ChunkedStats {
    pub flat_rows_processed: u64,
    pub chunk_rows_processed: u64,
    pub chunk_blocks_visited: u64,
    pub chunk_count: u64,
    pub partial_chunk_rows: u64,
    pub logical_chunk_allocations: u64,
    pub rows_copied_during_growth: u64,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ChunkedEvidence {
    pub snapshot: WorldSnapshot,
    pub stats: ChunkedStats,
}

/// Runs the flat/chunked layout experiment and proves both against the canonical reference world.
///
/// # Panics
///
/// Panics when `chunk_size` is zero or when either candidate diverges from the shared-workload
/// reference snapshot.
#[must_use]
pub fn run_chunked_scenario(
    entity_count: u32,
    rounds: u32,
    chunk_size: usize,
) -> ChunkedEvidence {
    assert!(chunk_size > 0, "chunk_size must be non-zero");

    let rows = fixture_rows(entity_count);
    let workload = canonical_workload(&rows, rounds);
    let mut reference = ReferenceWorld::new();
    assert_eq!(reference.replay(&workload), Ok(()));
    let expected = reference.snapshot();

    let mut flat = FlatMotionWorld::with_capacity(entity_count as usize);
    let mut chunked = ChunkedMotionWorld::new(chunk_size);
    for row in &rows {
        flat.push(row.entity, row.position, row.velocity);
        chunked.push(row.entity, row.position, row.velocity);
    }
    assert_eq!(flat.snapshot(), chunked.snapshot());

    let mut stats = ChunkedStats {
        chunk_count: as_u64(chunked.chunks.len()),
        partial_chunk_rows: as_u64(chunked.partial_chunk_rows()),
        logical_chunk_allocations: as_u64(chunked.chunks.len()),
        // Each chunk pre-reserves every SoA column to its final fixed capacity, so row data never
        // moves because another chunk is appended.
        rows_copied_during_growth: 0,
        ..ChunkedStats::default()
    };

    for _ in 0..rounds {
        stats.flat_rows_processed = stats
            .flat_rows_processed
            .saturating_add(flat.integrate(1));
        let (blocks, rows) = chunked.integrate(1);
        stats.chunk_blocks_visited = stats.chunk_blocks_visited.saturating_add(blocks);
        stats.chunk_rows_processed = stats.chunk_rows_processed.saturating_add(rows);
    }

    let flat_snapshot = flat.snapshot();
    let actual = chunked.snapshot();
    assert_eq!(
        flat_snapshot, expected,
        "flat layout must match the canonical reference world"
    );
    assert_eq!(
        actual, expected,
        "chunked layout must match the canonical reference world"
    );
    assert_eq!(stats.flat_rows_processed, stats.chunk_rows_processed);

    ChunkedEvidence {
        snapshot: actual,
        stats,
    }
}

/// Replays only the flat scalar layout used as the comparison candidate.
#[must_use]
pub fn replay_flat_scenario(entity_count: u32, rounds: u32) -> WorldSnapshot {
    let rows = fixture_rows(entity_count);
    let mut flat = FlatMotionWorld::with_capacity(rows.len());
    for row in rows {
        flat.push(row.entity, row.position, row.velocity);
    }
    for _ in 0..rounds {
        flat.integrate(1);
    }
    flat.snapshot()
}

/// Replays only the chunked SoA layout.
///
/// # Panics
///
/// Panics when `chunk_size` is zero.
#[must_use]
pub fn replay_chunked_scenario(
    entity_count: u32,
    rounds: u32,
    chunk_size: usize,
) -> WorldSnapshot {
    assert!(chunk_size > 0, "chunk_size must be non-zero");
    let rows = fixture_rows(entity_count);
    let mut chunked = ChunkedMotionWorld::new(chunk_size);
    for row in rows {
        chunked.push(row.entity, row.position, row.velocity);
    }
    for _ in 0..rounds {
        chunked.integrate(1);
    }
    chunked.snapshot()
}

fn fixture_rows(entity_count: u32) -> Vec<Row> {
    (0..entity_count).map(initial_row).collect()
}

fn initial_row(raw_id: u32) -> Row {
    let value = i64::from(raw_id);
    Row {
        entity: EntityId(raw_id),
        position: Position::new3(value, value.saturating_mul(2), -value),
        velocity: Velocity::new3(1, -2, 3),
    }
}

fn canonical_workload(rows: &[Row], rounds: u32) -> Workload {
    let mut operations = Vec::new();
    for row in rows {
        operations.extend([
            Operation::Spawn(row.entity),
            Operation::SetPosition(row.entity, row.position),
            Operation::SetVelocity(row.entity, row.velocity),
        ]);
    }
    for _ in 0..rounds {
        operations.push(Operation::Integrate { ticks: 1 });
    }
    Workload::new(operations)
}

fn integrate(position: &mut Position, velocity: Velocity, ticks: i64) {
    position.x = position
        .x
        .saturating_add(i64::from(velocity.x).saturating_mul(ticks));
    position.y = position
        .y
        .saturating_add(i64::from(velocity.y).saturating_mul(ticks));
    position.z = position
        .z
        .saturating_add(i64::from(velocity.z).saturating_mul(ticks));
}

fn as_u64(value: usize) -> u64 {
    u64::try_from(value).unwrap_or(u64::MAX)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn chunk_boundary_matrix_preserves_scalar_parity() {
        for chunk_size in [16_usize, 64, 256] {
            for entity_count in [0_u32, 1, 15, 16, 17, 63, 64, 65, 1_023, 1_024, 1_025] {
                let evidence = run_chunked_scenario(entity_count, 8, chunk_size);
                let expected_chunks =
                    u64::try_from((entity_count as usize).div_ceil(chunk_size)).unwrap_or(u64::MAX);
                let expected_rows = u64::from(entity_count) * 8;
                assert_eq!(evidence.stats.chunk_count, expected_chunks);
                assert_eq!(evidence.stats.logical_chunk_allocations, expected_chunks);
                assert_eq!(evidence.stats.chunk_blocks_visited, expected_chunks * 8);
                assert_eq!(evidence.stats.chunk_rows_processed, expected_rows);
                assert_eq!(evidence.stats.flat_rows_processed, expected_rows);
                assert_eq!(evidence.stats.rows_copied_during_growth, 0);

                let remainder = entity_count as usize % chunk_size;
                let expected_partial = if remainder == 0 {
                    0
                } else {
                    remainder as u64
                };
                assert_eq!(evidence.stats.partial_chunk_rows, expected_partial);
                assert_eq!(evidence.snapshot.entities().len(), entity_count as usize);
            }
        }
    }

    #[test]
    fn dedicated_layout_replays_match_reference_checked_evidence() {
        for chunk_size in [16_usize, 64, 256] {
            let expected = run_chunked_scenario(129, 4, chunk_size).snapshot;
            assert_eq!(replay_flat_scenario(129, 4), expected);
            assert_eq!(replay_chunked_scenario(129, 4, chunk_size), expected);
        }
    }

    #[test]
    fn partially_filled_chunk_is_not_materialized_as_full_rows() {
        let evidence = run_chunked_scenario(65, 3, 64);
        assert_eq!(evidence.stats.chunk_count, 2);
        assert_eq!(evidence.stats.partial_chunk_rows, 1);
        assert_eq!(evidence.stats.chunk_rows_processed, 195);
        assert_eq!(evidence.snapshot.entities().len(), 65);
    }
}
