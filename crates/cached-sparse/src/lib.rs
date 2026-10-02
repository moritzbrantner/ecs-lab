use std::collections::BTreeSet;

use ecs_sparse_index::PagedSparseIndex;
use ecs_workload::{
    EntityId, EntitySnapshot, Operation, Position, SnapshotWorkStats, StorageIndexStats,
    StorageWorkStats, Velocity, Workload, WorkloadError, WorldSnapshot,
};

#[derive(Clone, Debug, Eq, PartialEq)]
struct SparseSet<T> {
    sparse: PagedSparseIndex,
    dense_entities: Vec<EntityId>,
    dense_values: Vec<T>,
}

impl<T> Default for SparseSet<T> {
    fn default() -> Self {
        Self {
            sparse: PagedSparseIndex::new(),
            dense_entities: Vec::new(),
            dense_values: Vec::new(),
        }
    }
}

struct Removed {
    moved: Option<(EntityId, usize)>,
}

impl<T> SparseSet<T> {
    fn insert(&mut self, entity: EntityId, value: T) -> usize {
        if let Some(index) = self.sparse.get(entity.0) {
            self.dense_values[index] = value;
            return index;
        }

        let index = self.dense_values.len();
        self.sparse.set(entity.0, index);
        self.dense_entities.push(entity);
        self.dense_values.push(value);
        index
    }

    fn index(&self, entity: EntityId) -> Option<usize> {
        self.sparse.get(entity.0)
    }

    fn get(&self, entity: EntityId) -> Option<&T> {
        self.index(entity)
            .and_then(|index| self.dense_values.get(index))
    }

    fn moved_entity_if_removed(&self, entity: EntityId) -> Option<EntityId> {
        let index = self.index(entity)?;
        let last = self.dense_entities.len().checked_sub(1)?;
        if index == last {
            None
        } else {
            self.dense_entities.get(last).copied()
        }
    }

    fn remove(&mut self, entity: EntityId) -> Option<Removed> {
        let index = self.sparse.remove(entity.0)?;

        self.dense_entities.swap_remove(index);
        self.dense_values.swap_remove(index);
        let moved = if index < self.dense_entities.len() {
            let moved_entity = self.dense_entities[index];
            self.sparse.set(moved_entity.0, index);
            Some((moved_entity, index))
        } else {
            None
        };

        Some(Removed { moved })
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct MotionQueryRow {
    entity: EntityId,
    position_index: usize,
    velocity_index: usize,
}

#[derive(Clone, Debug, Default, Eq, PartialEq)]
struct MotionQuery {
    sparse: PagedSparseIndex,
    rows: Vec<MotionQueryRow>,
}

impl MotionQuery {
    fn row_index(&self, entity: EntityId) -> Option<usize> {
        self.sparse.get(entity.0)
    }

    fn refresh(
        &mut self,
        entity: EntityId,
        position_index: Option<usize>,
        velocity_index: Option<usize>,
    ) -> u64 {
        match (position_index, velocity_index, self.row_index(entity)) {
            (Some(position_index), Some(velocity_index), Some(row_index)) => {
                let row = &mut self.rows[row_index];
                if row.position_index == position_index && row.velocity_index == velocity_index {
                    0
                } else {
                    row.position_index = position_index;
                    row.velocity_index = velocity_index;
                    1
                }
            }
            (Some(position_index), Some(velocity_index), None) => {
                let row_index = self.rows.len();
                self.rows.push(MotionQueryRow {
                    entity,
                    position_index,
                    velocity_index,
                });
                self.sparse.set(entity.0, row_index);
                1
            }
            (_, _, Some(row_index)) => {
                self.sparse.remove(entity.0);
                self.rows.swap_remove(row_index);
                if let Some(moved) = self.rows.get(row_index).copied() {
                    self.sparse.set(moved.entity.0, row_index);
                }
                1
            }
            _ => 0,
        }
    }
}

/// Sparse component storage with a retained Position+Velocity query.
///
/// Components remain independently owned sparse sets. The query cache stores only entity membership
/// plus the current dense indices needed by the motion system. Stable component shapes therefore avoid
/// per-row sparse lookups without turning component storage into archetype tables; structural changes
/// pay explicit cache maintenance instead.
#[derive(Clone, Debug, Default, Eq, PartialEq)]
pub struct CachedSparseWorld {
    alive: BTreeSet<EntityId>,
    positions: SparseSet<Position>,
    velocities: SparseSet<Velocity>,
    motion_query: MotionQuery,
}

impl CachedSparseWorld {
    #[must_use]
    pub const fn new() -> Self {
        Self {
            alive: BTreeSet::new(),
            positions: SparseSet {
                sparse: PagedSparseIndex::new(),
                dense_entities: Vec::new(),
                dense_values: Vec::new(),
            },
            velocities: SparseSet {
                sparse: PagedSparseIndex::new(),
                dense_entities: Vec::new(),
                dense_values: Vec::new(),
            },
            motion_query: MotionQuery {
                sparse: PagedSparseIndex::new(),
                rows: Vec::new(),
            },
        }
    }

    /// Applies one operation while maintaining the cached motion query.
    ///
    /// # Errors
    ///
    /// Returns [`WorkloadError`] for invalid entity lifecycle operations.
    pub fn apply(&mut self, operation: Operation) -> Result<(), WorkloadError> {
        match operation {
            Operation::Spawn(entity) => {
                if !self.alive.insert(entity) {
                    return Err(WorkloadError::EntityAlreadyExists(entity));
                }
                Ok(())
            }
            Operation::Despawn(entity) => {
                self.require_alive(entity)?;
                let position_removed = self.positions.remove(entity);
                let velocity_removed = self.velocities.remove(entity);
                self.refresh_motion(entity);
                if let Some(removed) = position_removed
                    && let Some((moved, _)) = removed.moved
                {
                    self.refresh_motion(moved);
                }
                if let Some(removed) = velocity_removed
                    && let Some((moved, _)) = removed.moved
                {
                    self.refresh_motion(moved);
                }
                self.alive.remove(&entity);
                Ok(())
            }
            Operation::SetPosition(entity, position) => {
                self.require_alive(entity)?;
                self.positions.insert(entity, position);
                self.refresh_motion(entity);
                Ok(())
            }
            Operation::RemovePosition(entity) => {
                self.require_alive(entity)?;
                let removed = self.positions.remove(entity);
                self.refresh_motion(entity);
                if let Some(removed) = removed
                    && let Some((moved, _)) = removed.moved
                {
                    self.refresh_motion(moved);
                }
                Ok(())
            }
            Operation::SetVelocity(entity, velocity) => {
                self.require_alive(entity)?;
                self.velocities.insert(entity, velocity);
                self.refresh_motion(entity);
                Ok(())
            }
            Operation::RemoveVelocity(entity) => {
                self.require_alive(entity)?;
                let removed = self.velocities.remove(entity);
                self.refresh_motion(entity);
                if let Some(removed) = removed
                    && let Some((moved, _)) = removed.moved
                {
                    self.refresh_motion(moved);
                }
                Ok(())
            }
            Operation::Integrate { ticks } => {
                self.integrate(ticks);
                Ok(())
            }
        }
    }

    /// Replays every operation in order.
    ///
    /// # Errors
    ///
    /// Returns the first [`WorkloadError`] produced by the workload.
    pub fn replay(&mut self, workload: &Workload) -> Result<(), WorkloadError> {
        for operation in workload.operations() {
            self.apply(*operation)?;
        }
        Ok(())
    }

    #[must_use]
    pub fn snapshot(&self) -> WorldSnapshot {
        WorldSnapshot::new(
            self.alive
                .iter()
                .map(|&id| EntitySnapshot {
                    id,
                    position: self.positions.get(id).copied(),
                    velocity: self.velocities.get(id).copied(),
                })
                .collect(),
        )
    }

    #[must_use]
    pub fn snapshot_with_stats(&self) -> (WorldSnapshot, SnapshotWorkStats) {
        let entity_count = u64::try_from(self.alive.len()).unwrap_or(u64::MAX);
        (
            self.snapshot(),
            SnapshotWorkStats {
                slots_scanned: entity_count,
                entities_materialized: entity_count,
            },
        )
    }

    #[must_use]
    pub fn index_stats(&self) -> StorageIndexStats {
        StorageIndexStats {
            entity_index_entries: u64::try_from(self.alive.len()).unwrap_or(u64::MAX),
            component_index_slots: self
                .positions
                .sparse
                .allocated_slots()
                .saturating_add(self.velocities.sparse.allocated_slots()),
            query_index_slots: self.motion_query.sparse.allocated_slots(),
            query_rows: u64::try_from(self.motion_query.rows.len()).unwrap_or(u64::MAX),
        }
    }

    #[must_use]
    pub fn operation_work(&self, operation: Operation) -> StorageWorkStats {
        match operation {
            Operation::Integrate { .. } => {
                let rows = u64::try_from(self.motion_query.rows.len()).unwrap_or(u64::MAX);
                StorageWorkStats {
                    integration_rows_scanned: rows,
                    integrated_entities: rows,
                    ..StorageWorkStats::default()
                }
            }
            Operation::SetPosition(entity, _) => StorageWorkStats {
                query_cache_updates: u64::from(
                    self.positions.index(entity).is_none()
                        && self.velocities.index(entity).is_some(),
                ),
                ..StorageWorkStats::default()
            },
            Operation::SetVelocity(entity, _) => StorageWorkStats {
                query_cache_updates: u64::from(
                    self.velocities.index(entity).is_none()
                        && self.positions.index(entity).is_some(),
                ),
                ..StorageWorkStats::default()
            },
            Operation::RemovePosition(entity) => StorageWorkStats {
                query_cache_updates: self.position_removal_query_updates(entity),
                ..StorageWorkStats::default()
            },
            Operation::RemoveVelocity(entity) => StorageWorkStats {
                query_cache_updates: self.velocity_removal_query_updates(entity),
                ..StorageWorkStats::default()
            },
            Operation::Despawn(entity) => StorageWorkStats {
                query_cache_updates: self.despawn_query_updates(entity),
                ..StorageWorkStats::default()
            },
            Operation::Spawn(_) => StorageWorkStats::default(),
        }
    }

    fn position_removal_query_updates(&self, entity: EntityId) -> u64 {
        let target = u64::from(
            self.positions.index(entity).is_some() && self.velocities.index(entity).is_some(),
        );
        let moved = self
            .positions
            .moved_entity_if_removed(entity)
            .is_some_and(|moved| self.velocities.index(moved).is_some());
        target.saturating_add(u64::from(moved))
    }

    fn velocity_removal_query_updates(&self, entity: EntityId) -> u64 {
        let target = u64::from(
            self.positions.index(entity).is_some() && self.velocities.index(entity).is_some(),
        );
        let moved = self
            .velocities
            .moved_entity_if_removed(entity)
            .is_some_and(|moved| self.positions.index(moved).is_some());
        target.saturating_add(u64::from(moved))
    }

    fn despawn_query_updates(&self, entity: EntityId) -> u64 {
        let target = u64::from(
            self.positions.index(entity).is_some() && self.velocities.index(entity).is_some(),
        );
        let moved_position = self
            .positions
            .moved_entity_if_removed(entity)
            .filter(|moved| self.velocities.index(*moved).is_some());
        let moved_velocity = self
            .velocities
            .moved_entity_if_removed(entity)
            .filter(|moved| self.positions.index(*moved).is_some());

        let moved = match (moved_position, moved_velocity) {
            (Some(left), Some(right)) if left == right => 1,
            (Some(_), Some(_)) => 2,
            (Some(_), None) | (None, Some(_)) => 1,
            (None, None) => 0,
        };
        target.saturating_add(moved)
    }

    fn refresh_motion(&mut self, entity: EntityId) {
        let position_index = self.positions.index(entity);
        let velocity_index = self.velocities.index(entity);
        let _ = self
            .motion_query
            .refresh(entity, position_index, velocity_index);
    }

    fn require_alive(&self, entity: EntityId) -> Result<(), WorkloadError> {
        if self.alive.contains(&entity) {
            Ok(())
        } else {
            Err(WorkloadError::MissingEntity(entity))
        }
    }

    fn integrate(&mut self, ticks: i32) {
        let ticks = i64::from(ticks);
        for row in &self.motion_query.rows {
            debug_assert_eq!(
                self.positions.dense_entities[row.position_index],
                row.entity
            );
            debug_assert_eq!(
                self.velocities.dense_entities[row.velocity_index],
                row.entity
            );
            let velocity = self.velocities.dense_values[row.velocity_index];
            let position = &mut self.positions.dense_values[row.position_index];
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
    }
}

#[cfg(test)]
mod tests {
    use ecs_reference::ReferenceWorld;
    use ecs_workload::{EntityId, Operation, Position, Velocity, Workload, WorkloadError};

    use super::CachedSparseWorld;

    #[test]
    fn deterministic_motion_matches_reference_world() {
        for seed in [0, 1, 17, 99, u32::MAX] {
            let workload = Workload::mixed_motion_scenario(seed, 128, 32, 4);
            let mut reference = ReferenceWorld::new();
            let mut cached = CachedSparseWorld::new();

            assert_eq!(reference.replay(&workload), Ok(()));
            assert_eq!(cached.replay(&workload), Ok(()));
            assert_eq!(cached.snapshot(), reference.snapshot(), "seed {seed}");
        }
    }

    #[test]
    fn query_indices_repair_after_component_swap_remove() {
        let mut reference = ReferenceWorld::new();
        let mut cached = CachedSparseWorld::new();

        for raw_id in 0..3 {
            let entity = EntityId(raw_id);
            for operation in [
                Operation::Spawn(entity),
                Operation::SetPosition(entity, Position::new(i64::from(raw_id), 0)),
                Operation::SetVelocity(entity, Velocity::new(1, 2)),
            ] {
                assert_eq!(cached.apply(operation), reference.apply(operation));
            }
        }

        for operation in [
            Operation::RemovePosition(EntityId(1)),
            Operation::SetPosition(EntityId(1), Position::new(10, 20)),
            Operation::RemoveVelocity(EntityId(0)),
            Operation::SetVelocity(EntityId(0), Velocity::new(3, 4)),
            Operation::Integrate { ticks: 2 },
        ] {
            assert_eq!(cached.apply(operation), reference.apply(operation));
        }

        assert_eq!(cached.snapshot(), reference.snapshot());
    }

    #[test]
    fn component_churn_matches_reference_across_seeds() {
        for seed in [0, 1, 17, 99, u32::MAX] {
            let workload = Workload::component_churn_scenario(seed, 64, 8);
            let mut reference = ReferenceWorld::new();
            let mut cached = CachedSparseWorld::new();

            assert_eq!(reference.replay(&workload), Ok(()));
            assert_eq!(cached.replay(&workload), Ok(()));
            assert_eq!(cached.snapshot(), reference.snapshot(), "seed {seed}");
        }
    }

    #[test]
    fn despawn_repairs_component_and_query_swap_indices() {
        let mut reference = ReferenceWorld::new();
        let mut cached = CachedSparseWorld::new();

        for raw_id in 0..4 {
            let entity = EntityId(raw_id);
            for operation in [
                Operation::Spawn(entity),
                Operation::SetPosition(entity, Position::new(i64::from(raw_id), 0)),
                Operation::SetVelocity(entity, Velocity::new(1, 1)),
            ] {
                assert_eq!(cached.apply(operation), reference.apply(operation));
            }
        }

        for operation in [
            Operation::Despawn(EntityId(1)),
            Operation::Integrate { ticks: 3 },
            Operation::RemoveVelocity(EntityId(2)),
            Operation::SetVelocity(EntityId(2), Velocity::new(4, -1)),
            Operation::Integrate { ticks: 2 },
        ] {
            assert_eq!(cached.apply(operation), reference.apply(operation));
        }

        assert_eq!(cached.snapshot(), reference.snapshot());
    }

    #[test]
    fn work_evidence_exposes_cached_query_tradeoff() {
        let mut cached = CachedSparseWorld::new();
        let first = EntityId(0);
        let second = EntityId(1);

        for operation in [
            Operation::Spawn(first),
            Operation::SetPosition(first, Position::new(1, 2)),
            Operation::SetVelocity(first, Velocity::new(3, 4)),
            Operation::Spawn(second),
            Operation::SetPosition(second, Position::new(5, 6)),
        ] {
            let work = cached.operation_work(operation);
            if matches!(operation, Operation::SetVelocity(_, _)) {
                assert_eq!(work.query_cache_updates, 1);
            }
            assert_eq!(cached.apply(operation), Ok(()));
        }

        let integration = cached.operation_work(Operation::Integrate { ticks: 1 });
        assert_eq!(integration.integration_rows_scanned, 1);
        assert_eq!(integration.integrated_entities, 1);
        assert_eq!(integration.component_lookups, 0);

        let index = cached.index_stats();
        assert_eq!(index.entity_index_entries, 2);
        assert_eq!(index.component_index_slots, 2 * 256);
        assert_eq!(index.query_index_slots, 256);
        assert_eq!(index.query_rows, 1);

        let removal = cached.operation_work(Operation::RemovePosition(first));
        assert!(removal.query_cache_updates >= 1);
    }

    #[test]
    fn distant_entity_ids_keep_component_and_query_indices_bounded() {
        let high = EntityId(u32::MAX);
        let low = EntityId(2);
        let mut reference = ReferenceWorld::new();
        let mut cached = CachedSparseWorld::new();

        for operation in [
            Operation::Spawn(high),
            Operation::SetPosition(high, Position::new3(10, 20, 30)),
            Operation::SetVelocity(high, Velocity::new3(1, -2, 3)),
            Operation::Spawn(low),
            Operation::SetPosition(low, Position::new3(-4, 5, 6)),
            Operation::Integrate { ticks: 2 },
        ] {
            assert_eq!(cached.apply(operation), reference.apply(operation));
            assert_eq!(cached.snapshot(), reference.snapshot());
        }

        let stats = cached.index_stats();
        assert_eq!(stats.entity_index_entries, 2);
        assert_eq!(stats.component_index_slots, 3 * 256);
        assert_eq!(stats.query_index_slots, 256);
        assert_eq!(stats.query_rows, 1);
        assert!(stats.component_index_slots < u64::from(u32::MAX));
    }

    #[test]
    fn lifecycle_errors_match_reference_world() {
        let missing = EntityId(42);
        let mut reference = ReferenceWorld::new();
        let mut cached = CachedSparseWorld::new();

        for operation in [
            Operation::SetPosition(missing, Position::new(1, 1)),
            Operation::SetVelocity(missing, Velocity::new(1, 1)),
            Operation::Despawn(missing),
        ] {
            let expected = reference.apply(operation);
            assert_eq!(cached.apply(operation), expected);
            assert_eq!(expected, Err(WorkloadError::MissingEntity(missing)));
        }
    }
}
