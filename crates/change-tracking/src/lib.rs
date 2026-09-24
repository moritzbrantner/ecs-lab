use std::collections::{BTreeMap, BTreeSet};

use ecs_workload::{EntityId, Position};

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct ChangeTrackingStats {
    pub checkpoints: u64,
    pub dirty_marks: u64,
    pub full_rows_scanned: u64,
    pub full_recomputations: u64,
    pub incremental_rows_scanned: u64,
    pub incremental_recomputations: u64,
    pub incremental_removals: u64,
}

#[derive(Clone, Debug, Default, Eq, PartialEq)]
pub struct ChangeTrackingExperiment {
    positions: BTreeMap<EntityId, Position>,
    dirty: BTreeSet<EntityId>,
    full_projection: BTreeMap<EntityId, i128>,
    incremental_projection: BTreeMap<EntityId, i128>,
    stats: ChangeTrackingStats,
}

impl ChangeTrackingExperiment {
    #[must_use]
    pub const fn new() -> Self {
        Self {
            positions: BTreeMap::new(),
            dirty: BTreeSet::new(),
            full_projection: BTreeMap::new(),
            incremental_projection: BTreeMap::new(),
            stats: ChangeTrackingStats {
                checkpoints: 0,
                dirty_marks: 0,
                full_rows_scanned: 0,
                full_recomputations: 0,
                incremental_rows_scanned: 0,
                incremental_recomputations: 0,
                incremental_removals: 0,
            },
        }
    }

    pub fn set_position(&mut self, entity: EntityId, position: Position) {
        if self.positions.get(&entity).copied() == Some(position) {
            return;
        }
        self.positions.insert(entity, position);
        if self.dirty.insert(entity) {
            self.stats.dirty_marks = self.stats.dirty_marks.saturating_add(1);
        }
    }

    pub fn remove_position(&mut self, entity: EntityId) {
        if self.positions.remove(&entity).is_some() && self.dirty.insert(entity) {
            self.stats.dirty_marks = self.stats.dirty_marks.saturating_add(1);
        }
    }

    #[must_use]
    pub fn position(&self, entity: EntityId) -> Option<Position> {
        self.positions.get(&entity).copied()
    }

    /// Recomputes the reference projection and applies the accumulated dirty workset.
    ///
    /// # Panics
    ///
    /// Panics if the incremental projection diverges from the full recomputation oracle.
    pub fn checkpoint(&mut self) {
        self.full_projection.clear();
        self.stats.full_rows_scanned = self
            .stats
            .full_rows_scanned
            .saturating_add(as_u64(self.positions.len()));
        self.stats.full_recomputations = self
            .stats
            .full_recomputations
            .saturating_add(as_u64(self.positions.len()));
        for (&entity, &position) in &self.positions {
            self.full_projection.insert(entity, derive(position));
        }

        let dirty = std::mem::take(&mut self.dirty);
        self.stats.incremental_rows_scanned = self
            .stats
            .incremental_rows_scanned
            .saturating_add(as_u64(dirty.len()));
        for entity in dirty {
            if let Some(&position) = self.positions.get(&entity) {
                self.incremental_projection.insert(entity, derive(position));
                self.stats.incremental_recomputations =
                    self.stats.incremental_recomputations.saturating_add(1);
            } else if self.incremental_projection.remove(&entity).is_some() {
                self.stats.incremental_removals =
                    self.stats.incremental_removals.saturating_add(1);
            }
        }
        self.stats.checkpoints = self.stats.checkpoints.saturating_add(1);
        assert_eq!(
            self.incremental_projection, self.full_projection,
            "incremental projection must match a complete recomputation"
        );
    }

    #[must_use]
    pub const fn stats(&self) -> ChangeTrackingStats {
        self.stats
    }

    #[must_use]
    pub fn projection(&self) -> &BTreeMap<EntityId, i128> {
        &self.incremental_projection
    }
}

/// Runs the fixed deterministic change-density experiment.
///
/// # Panics
///
/// Panics when `changed_per_round` exceeds `entity_count`, or if an internal fixture
/// invariant is violated.
#[must_use]
pub fn run_fixed_scenario(
    entity_count: u32,
    rounds: u32,
    changed_per_round: u32,
) -> ChangeTrackingExperiment {
    assert!(
        changed_per_round <= entity_count,
        "changed_per_round must not exceed entity_count"
    );
    let mut experiment = ChangeTrackingExperiment::new();
    for raw_id in 0..entity_count {
        let value = i64::from(raw_id);
        experiment.set_position(
            EntityId(raw_id),
            Position::new3(value, value.saturating_mul(2), -value),
        );
    }
    experiment.checkpoint();

    if entity_count == 0 {
        for _ in 0..rounds {
            experiment.checkpoint();
        }
        return experiment;
    }

    for round in 0..rounds {
        for offset in 0..changed_per_round {
            let raw_id = offset.wrapping_add(round) % entity_count;
            let entity = EntityId(raw_id);
            let Some(current) = experiment.position(entity) else {
                panic!("scenario mutates only initialized positions");
            };
            experiment.set_position(
                entity,
                Position::new3(
                    current.x.saturating_add(1),
                    current.y.saturating_sub(1),
                    current.z.saturating_add(2),
                ),
            );
        }
        experiment.checkpoint();
    }
    experiment
}

fn derive(position: Position) -> i128 {
    let x = i128::from(position.x);
    let y = i128::from(position.y);
    let z = i128::from(position.z);
    x * x + y * y + z * z
}

fn as_u64(value: usize) -> u64 {
    u64::try_from(value).unwrap_or(u64::MAX)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fixed_scenarios_ratchet_incremental_work() {
        let entity_count = 1_024_u32;
        let rounds = 8_u32;
        for changed in [0_u32, 1, 16, 256, 1_024] {
            let experiment = run_fixed_scenario(entity_count, rounds, changed);
            let stats = experiment.stats();
            let expected_full = u64::from(entity_count) * u64::from(rounds + 1);
            let expected_incremental =
                u64::from(entity_count) + u64::from(changed) * u64::from(rounds);
            assert_eq!(stats.checkpoints, u64::from(rounds + 1));
            assert_eq!(stats.full_rows_scanned, expected_full);
            assert_eq!(stats.full_recomputations, expected_full);
            assert_eq!(stats.incremental_rows_scanned, expected_incremental);
            assert_eq!(stats.incremental_recomputations, expected_incremental);
            assert_eq!(stats.dirty_marks, expected_incremental);
            assert_eq!(stats.incremental_removals, 0);
        }
    }

    #[test]
    fn setting_the_same_value_does_not_dirty_the_component() {
        let entity = EntityId(3);
        let position = Position::new3(1, 2, 3);
        let mut experiment = ChangeTrackingExperiment::new();
        experiment.set_position(entity, position);
        experiment.checkpoint();
        let before = experiment.stats();

        experiment.set_position(entity, position);
        experiment.checkpoint();
        let after = experiment.stats();

        assert_eq!(after.dirty_marks, before.dirty_marks);
        assert_eq!(
            after.incremental_rows_scanned,
            before.incremental_rows_scanned
        );
        assert_eq!(after.full_rows_scanned, before.full_rows_scanned + 1);
    }

    #[test]
    fn removal_updates_incremental_projection_without_recomputing_survivors() {
        let mut experiment = run_fixed_scenario(8, 0, 0);
        experiment.remove_position(EntityId(3));
        experiment.checkpoint();

        assert!(!experiment.projection().contains_key(&EntityId(3)));
        assert_eq!(experiment.projection().len(), 7);
        let stats = experiment.stats();
        assert_eq!(stats.incremental_rows_scanned, 9);
        assert_eq!(stats.incremental_recomputations, 8);
        assert_eq!(stats.incremental_removals, 1);
        assert_eq!(stats.full_rows_scanned, 15);
    }

    #[test]
    fn empty_world_checkpoints_do_no_projection_work() {
        let experiment = run_fixed_scenario(0, 8, 0);
        let stats = experiment.stats();
        assert_eq!(stats.checkpoints, 9);
        assert_eq!(stats.full_rows_scanned, 0);
        assert_eq!(stats.incremental_rows_scanned, 0);
    }
}
