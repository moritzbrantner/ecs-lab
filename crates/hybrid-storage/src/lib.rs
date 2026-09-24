use ecs_sparse_index::PagedSparseIndex;
use ecs_sparse_set::SparseWorld;
use ecs_workload::{EntityId, EntitySnapshot, Operation, Position, Velocity, WorldSnapshot};

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct MotionRow {
    entity: EntityId,
    position: Position,
    velocity: Velocity,
}

impl MotionRow {
    fn integrate(&mut self, ticks: i64) {
        self.position.x = self
            .position
            .x
            .saturating_add(i64::from(self.velocity.x).saturating_mul(ticks));
        self.position.y = self
            .position
            .y
            .saturating_add(i64::from(self.velocity.y).saturating_mul(ticks));
        self.position.z = self
            .position
            .z
            .saturating_add(i64::from(self.velocity.z).saturating_mul(ticks));
    }

    fn snapshot(self) -> EntitySnapshot {
        EntitySnapshot {
            id: self.entity,
            position: Some(self.position),
            velocity: Some(self.velocity),
        }
    }
}

#[derive(Clone, Debug, Default, Eq, PartialEq)]
struct SparseStatus {
    index: PagedSparseIndex,
    entities: Vec<EntityId>,
}

impl SparseStatus {
    fn contains(&self, entity: EntityId) -> bool {
        self.index.get(entity.0).is_some()
    }

    fn insert(&mut self, entity: EntityId) -> bool {
        if self.contains(entity) {
            return false;
        }
        let dense = self.entities.len();
        self.entities.push(entity);
        self.index.set(entity.0, dense);
        true
    }

    fn remove(&mut self, entity: EntityId) -> bool {
        let Some(dense) = self.index.remove(entity.0) else {
            return false;
        };
        self.entities.swap_remove(dense);
        if let Some(moved) = self.entities.get(dense).copied() {
            self.index.set(moved.0, dense);
        }
        true
    }

    fn sorted_entities(&self) -> Vec<EntityId> {
        let mut entities = self.entities.clone();
        entities.sort_unstable();
        entities
    }

    fn allocated_slots(&self) -> u64 {
        self.index.allocated_slots()
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum StatusBucket {
    Plain,
    WithStatus,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct Location {
    bucket: StatusBucket,
    index: usize,
}

#[derive(Clone, Debug, Eq, PartialEq)]
struct ArchetypeStatusWorld {
    locations: Vec<Location>,
    plain: Vec<MotionRow>,
    with_status: Vec<MotionRow>,
}

impl ArchetypeStatusWorld {
    fn new(entity_count: u32, status_stride: u32) -> Self {
        let mut world = Self {
            locations: Vec::with_capacity(entity_count as usize),
            plain: Vec::new(),
            with_status: Vec::new(),
        };
        for raw_id in 0..entity_count {
            let row = initial_row(raw_id);
            let has_status = status_stride != 0 && raw_id % status_stride == 0;
            let (bucket, index) = if has_status {
                let index = world.with_status.len();
                world.with_status.push(row);
                (StatusBucket::WithStatus, index)
            } else {
                let index = world.plain.len();
                world.plain.push(row);
                (StatusBucket::Plain, index)
            };
            world.locations.push(Location { bucket, index });
        }
        world
    }

    fn has_status(&self, entity: EntityId) -> bool {
        self.locations
            .get(entity.0 as usize)
            .is_some_and(|location| location.bucket == StatusBucket::WithStatus)
    }

    fn set_status(&mut self, entity: EntityId, enabled: bool) -> bool {
        let entity_index = entity.0 as usize;
        let location = self.locations[entity_index];
        let target = if enabled {
            StatusBucket::WithStatus
        } else {
            StatusBucket::Plain
        };
        if location.bucket == target {
            return false;
        }

        let row = match location.bucket {
            StatusBucket::Plain => {
                let row = self.plain.swap_remove(location.index);
                if let Some(moved) = self.plain.get(location.index) {
                    self.locations[moved.entity.0 as usize] = Location {
                        bucket: StatusBucket::Plain,
                        index: location.index,
                    };
                }
                row
            }
            StatusBucket::WithStatus => {
                let row = self.with_status.swap_remove(location.index);
                if let Some(moved) = self.with_status.get(location.index) {
                    self.locations[moved.entity.0 as usize] = Location {
                        bucket: StatusBucket::WithStatus,
                        index: location.index,
                    };
                }
                row
            }
        };

        let target_index = match target {
            StatusBucket::Plain => {
                let index = self.plain.len();
                self.plain.push(row);
                index
            }
            StatusBucket::WithStatus => {
                let index = self.with_status.len();
                self.with_status.push(row);
                index
            }
        };
        self.locations[entity_index] = Location {
            bucket: target,
            index: target_index,
        };
        true
    }

    fn integrate(&mut self, ticks: i32) -> u64 {
        let ticks = i64::from(ticks);
        for row in self.plain.iter_mut().chain(self.with_status.iter_mut()) {
            row.integrate(ticks);
        }
        as_u64(self.locations.len())
    }

    fn snapshot(&self) -> HybridSnapshot {
        let mut entities = Vec::with_capacity(self.locations.len());
        entities.extend(self.plain.iter().copied().map(MotionRow::snapshot));
        entities.extend(self.with_status.iter().copied().map(MotionRow::snapshot));
        let mut status_entities: Vec<_> =
            self.with_status.iter().map(|row| row.entity).collect();
        status_entities.sort_unstable();
        HybridSnapshot {
            world: WorldSnapshot::new(entities),
            status_entities,
        }
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
struct HybridWorld {
    rows: Vec<MotionRow>,
    status: SparseStatus,
}

impl HybridWorld {
    fn new(entity_count: u32, status_stride: u32) -> Self {
        let rows: Vec<_> = (0..entity_count).map(initial_row).collect();
        let mut status = SparseStatus::default();
        for raw_id in 0..entity_count {
            if status_stride != 0 && raw_id % status_stride == 0 {
                status.insert(EntityId(raw_id));
            }
        }
        Self { rows, status }
    }

    fn has_status(&self, entity: EntityId) -> bool {
        self.status.contains(entity)
    }

    fn set_status(&mut self, entity: EntityId, enabled: bool) -> bool {
        if enabled {
            self.status.insert(entity)
        } else {
            self.status.remove(entity)
        }
    }

    fn integrate(&mut self, ticks: i32) -> u64 {
        let ticks = i64::from(ticks);
        for row in &mut self.rows {
            row.integrate(ticks);
        }
        as_u64(self.rows.len())
    }

    fn snapshot(&self) -> HybridSnapshot {
        HybridSnapshot {
            world: WorldSnapshot::new(self.rows.iter().copied().map(MotionRow::snapshot).collect()),
            status_entities: self.status.sorted_entities(),
        }
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
struct AllSparseWorld {
    motion: SparseWorld,
    status: SparseStatus,
}

impl AllSparseWorld {
    fn new(entity_count: u32, status_stride: u32) -> Self {
        let mut motion = SparseWorld::new();
        let mut status = SparseStatus::default();
        for raw_id in 0..entity_count {
            let row = initial_row(raw_id);
            for operation in [
                Operation::Spawn(row.entity),
                Operation::SetPosition(row.entity, row.position),
                Operation::SetVelocity(row.entity, row.velocity),
            ] {
                assert_eq!(motion.apply(operation), Ok(()));
            }
            if status_stride != 0 && raw_id % status_stride == 0 {
                status.insert(row.entity);
            }
        }
        Self { motion, status }
    }

    fn has_status(&self, entity: EntityId) -> bool {
        self.status.contains(entity)
    }

    fn set_status(&mut self, entity: EntityId, enabled: bool) -> bool {
        if enabled {
            self.status.insert(entity)
        } else {
            self.status.remove(entity)
        }
    }

    fn integrate(&mut self, ticks: i32) -> (u64, u64) {
        let operation = Operation::Integrate { ticks };
        let work = self.motion.operation_work(operation);
        assert_eq!(self.motion.apply(operation), Ok(()));
        (work.integration_rows_scanned, work.component_lookups)
    }

    fn snapshot(&self) -> HybridSnapshot {
        HybridSnapshot {
            world: self.motion.snapshot(),
            status_entities: self.status.sorted_entities(),
        }
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct HybridSnapshot {
    pub world: WorldSnapshot,
    pub status_entities: Vec<EntityId>,
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct HybridStats {
    pub status_changes: u64,
    pub archetype_motion_row_moves: u64,
    pub hybrid_motion_row_moves: u64,
    pub sparse_motion_row_moves: u64,
    pub archetype_rows_scanned: u64,
    pub hybrid_rows_scanned: u64,
    pub sparse_rows_scanned: u64,
    pub sparse_component_lookups: u64,
    pub hybrid_status_peak_slots: u64,
    pub sparse_status_peak_slots: u64,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct HybridEvidence {
    pub snapshot: HybridSnapshot,
    pub stats: HybridStats,
}

/// Runs archetype, hybrid, and all-sparse status storage in lockstep.
///
/// # Panics
///
/// Panics when `toggles_per_round` exceeds `entity_count`, or if the representations
/// diverge in status membership, mutation behavior, or canonical snapshots.
#[must_use]
pub fn run_status_churn_scenario(
    entity_count: u32,
    rounds: u32,
    status_stride: u32,
    toggles_per_round: u32,
) -> HybridEvidence {
    assert!(toggles_per_round <= entity_count);
    let mut archetype = ArchetypeStatusWorld::new(entity_count, status_stride);
    let mut hybrid = HybridWorld::new(entity_count, status_stride);
    let mut sparse = AllSparseWorld::new(entity_count, status_stride);
    let mut stats = HybridStats {
        hybrid_status_peak_slots: hybrid.status.allocated_slots(),
        sparse_status_peak_slots: sparse.status.allocated_slots(),
        ..HybridStats::default()
    };

    let initial = archetype.snapshot();
    assert_eq!(hybrid.snapshot(), initial);
    assert_eq!(sparse.snapshot(), initial);

    if entity_count == 0 {
        return HybridEvidence {
            snapshot: initial,
            stats,
        };
    }

    for round in 0..rounds {
        for offset in 0..toggles_per_round {
            let entity = EntityId(offset.wrapping_add(round) % entity_count);
            let before = hybrid.has_status(entity);
            assert_eq!(archetype.has_status(entity), before);
            assert_eq!(sparse.has_status(entity), before);
            let target = !before;
            let a = archetype.set_status(entity, target);
            let h = hybrid.set_status(entity, target);
            let s = sparse.set_status(entity, target);
            assert_eq!((a, h, s), (true, true, true));
            stats.status_changes = stats.status_changes.saturating_add(1);
            stats.archetype_motion_row_moves =
                stats.archetype_motion_row_moves.saturating_add(1);
            stats.hybrid_status_peak_slots = stats
                .hybrid_status_peak_slots
                .max(hybrid.status.allocated_slots());
            stats.sparse_status_peak_slots = stats
                .sparse_status_peak_slots
                .max(sparse.status.allocated_slots());
        }

        stats.archetype_rows_scanned = stats
            .archetype_rows_scanned
            .saturating_add(archetype.integrate(1));
        stats.hybrid_rows_scanned = stats
            .hybrid_rows_scanned
            .saturating_add(hybrid.integrate(1));
        let (rows, lookups) = sparse.integrate(1);
        stats.sparse_rows_scanned = stats.sparse_rows_scanned.saturating_add(rows);
        stats.sparse_component_lookups =
            stats.sparse_component_lookups.saturating_add(lookups);

        let expected = archetype.snapshot();
        assert_eq!(hybrid.snapshot(), expected);
        assert_eq!(sparse.snapshot(), expected);
    }

    HybridEvidence {
        snapshot: hybrid.snapshot(),
        stats,
    }
}

/// Replays only the archetype status representation.
///
/// # Panics
///
/// Panics when `toggles_per_round` exceeds `entity_count`.
#[must_use]
pub fn replay_archetype_status_scenario(
    entity_count: u32,
    rounds: u32,
    status_stride: u32,
    toggles_per_round: u32,
) -> HybridSnapshot {
    assert!(toggles_per_round <= entity_count);
    let mut world = ArchetypeStatusWorld::new(entity_count, status_stride);
    if entity_count == 0 {
        return world.snapshot();
    }
    for round in 0..rounds {
        for offset in 0..toggles_per_round {
            let entity = EntityId(offset.wrapping_add(round) % entity_count);
            let target = !world.has_status(entity);
            world.set_status(entity, target);
        }
        world.integrate(1);
    }
    world.snapshot()
}

/// Replays only the hybrid table-plus-sparse-status representation.
///
/// # Panics
///
/// Panics when `toggles_per_round` exceeds `entity_count`.
#[must_use]
pub fn replay_hybrid_status_scenario(
    entity_count: u32,
    rounds: u32,
    status_stride: u32,
    toggles_per_round: u32,
) -> HybridSnapshot {
    assert!(toggles_per_round <= entity_count);
    let mut world = HybridWorld::new(entity_count, status_stride);
    if entity_count == 0 {
        return world.snapshot();
    }
    for round in 0..rounds {
        for offset in 0..toggles_per_round {
            let entity = EntityId(offset.wrapping_add(round) % entity_count);
            let target = !world.has_status(entity);
            world.set_status(entity, target);
        }
        world.integrate(1);
    }
    world.snapshot()
}

/// Replays only the all-sparse motion/status representation.
///
/// # Panics
///
/// Panics when `toggles_per_round` exceeds `entity_count`.
#[must_use]
pub fn replay_sparse_status_scenario(
    entity_count: u32,
    rounds: u32,
    status_stride: u32,
    toggles_per_round: u32,
) -> HybridSnapshot {
    assert!(toggles_per_round <= entity_count);
    let mut world = AllSparseWorld::new(entity_count, status_stride);
    if entity_count == 0 {
        return world.snapshot();
    }
    for round in 0..rounds {
        for offset in 0..toggles_per_round {
            let entity = EntityId(offset.wrapping_add(round) % entity_count);
            let target = !world.has_status(entity);
            world.set_status(entity, target);
        }
        world.integrate(1);
    }
    world.snapshot()
}

fn initial_row(raw_id: u32) -> MotionRow {
    let value = i64::from(raw_id);
    MotionRow {
        entity: EntityId(raw_id),
        position: Position::new3(value, value.saturating_mul(2), -value),
        velocity: Velocity::new3(1, -2, 3),
    }
}

fn as_u64(value: usize) -> u64 {
    u64::try_from(value).unwrap_or(u64::MAX)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn hybrid_storage_ratchet_keeps_stable_motion_rows_stationary() {
        let evidence = run_status_churn_scenario(1_024, 8, 4, 256);
        let stats = evidence.stats;
        assert_eq!(stats.status_changes, 2_048);
        assert_eq!(stats.archetype_motion_row_moves, 2_048);
        assert_eq!(stats.hybrid_motion_row_moves, 0);
        assert_eq!(stats.sparse_motion_row_moves, 0);
        assert_eq!(stats.archetype_rows_scanned, 8_192);
        assert_eq!(stats.hybrid_rows_scanned, 8_192);
        assert_eq!(stats.sparse_rows_scanned, 8_192);
        assert_eq!(stats.sparse_component_lookups, 8_192);
        assert!(stats.hybrid_status_peak_slots <= 1_024);
        assert!(stats.sparse_status_peak_slots <= 1_024);
    }

    #[test]
    fn no_status_churn_moves_no_motion_rows() {
        let evidence = run_status_churn_scenario(1_024, 8, 4, 0);
        assert_eq!(evidence.stats.status_changes, 0);
        assert_eq!(evidence.stats.archetype_motion_row_moves, 0);
        assert_eq!(evidence.stats.hybrid_motion_row_moves, 0);
        assert_eq!(evidence.stats.sparse_motion_row_moves, 0);
        assert_eq!(evidence.stats.hybrid_status_peak_slots, 1_024);
        assert_eq!(evidence.stats.sparse_status_peak_slots, 1_024);
    }

    #[test]
    fn dedicated_storage_replays_match_lockstep_evidence() {
        for (stride, toggles) in [(4, 0), (4, 32), (16, 64)] {
            let expected = run_status_churn_scenario(128, 4, stride, toggles).snapshot;
            assert_eq!(
                replay_archetype_status_scenario(128, 4, stride, toggles),
                expected
            );
            assert_eq!(
                replay_hybrid_status_scenario(128, 4, stride, toggles),
                expected
            );
            assert_eq!(
                replay_sparse_status_scenario(128, 4, stride, toggles),
                expected
            );
        }
    }

    #[test]
    fn empty_world_is_supported() {
        let evidence = run_status_churn_scenario(0, 8, 0, 0);
        assert!(evidence.snapshot.world.entities().is_empty());
        assert!(evidence.snapshot.status_entities.is_empty());
        assert_eq!(evidence.stats, HybridStats::default());
    }
}
