use ecs_workload::{EntityId, EntitySnapshot, Position, Velocity, WorldSnapshot};

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct Row {
    entity: EntityId,
    position: Position,
    velocity: Velocity,
}

impl Row {
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

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum Bucket {
    Enabled,
    Disabled,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct Location {
    bucket: Bucket,
    index: usize,
}

#[derive(Clone, Debug, Eq, PartialEq)]
struct StructuralEnableWorld {
    locations: Vec<Location>,
    enabled: Vec<Row>,
    disabled: Vec<Row>,
}

impl StructuralEnableWorld {
    fn new(entity_count: u32, enabled_stride: u32) -> Self {
        let mut world = Self {
            locations: Vec::with_capacity(entity_count as usize),
            enabled: Vec::new(),
            disabled: Vec::new(),
        };
        for raw_id in 0..entity_count {
            let row = initial_row(raw_id);
            let is_enabled = enabled_stride != 0 && raw_id % enabled_stride == 0;
            let (bucket, index) = if is_enabled {
                let index = world.enabled.len();
                world.enabled.push(row);
                (Bucket::Enabled, index)
            } else {
                let index = world.disabled.len();
                world.disabled.push(row);
                (Bucket::Disabled, index)
            };
            world.locations.push(Location { bucket, index });
        }
        world
    }

    fn is_enabled(&self, entity: EntityId) -> bool {
        self.locations
            .get(entity.0 as usize)
            .is_some_and(|location| location.bucket == Bucket::Enabled)
    }

    fn set_enabled(&mut self, entity: EntityId, enabled: bool) -> bool {
        let entity_index = entity.0 as usize;
        let Some(location) = self.locations.get(entity_index).copied() else {
            return false;
        };
        let target = if enabled {
            Bucket::Enabled
        } else {
            Bucket::Disabled
        };
        if location.bucket == target {
            return false;
        }

        let row = match location.bucket {
            Bucket::Enabled => {
                let row = self.enabled.swap_remove(location.index);
                if let Some(moved) = self.enabled.get(location.index) {
                    self.locations[moved.entity.0 as usize] = Location {
                        bucket: Bucket::Enabled,
                        index: location.index,
                    };
                }
                row
            }
            Bucket::Disabled => {
                let row = self.disabled.swap_remove(location.index);
                if let Some(moved) = self.disabled.get(location.index) {
                    self.locations[moved.entity.0 as usize] = Location {
                        bucket: Bucket::Disabled,
                        index: location.index,
                    };
                }
                row
            }
        };

        let target_index = match target {
            Bucket::Enabled => {
                let index = self.enabled.len();
                self.enabled.push(row);
                index
            }
            Bucket::Disabled => {
                let index = self.disabled.len();
                self.disabled.push(row);
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
        for row in &mut self.enabled {
            row.integrate(ticks);
        }
        as_u64(self.enabled.len())
    }

    fn snapshot(&self) -> WorldSnapshot {
        let mut entities = Vec::with_capacity(self.locations.len());
        entities.extend(self.enabled.iter().copied().map(Row::snapshot));
        entities.extend(self.disabled.iter().copied().map(Row::snapshot));
        WorldSnapshot::new(entities)
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
struct MaskEnableWorld {
    rows: Vec<Row>,
    enabled_words: Vec<u64>,
}

impl MaskEnableWorld {
    fn new(entity_count: u32, enabled_stride: u32) -> Self {
        let mut world = Self {
            rows: (0..entity_count).map(initial_row).collect(),
            enabled_words: vec![0; words_for(entity_count as usize)],
        };
        for raw_id in 0..entity_count {
            if enabled_stride != 0 && raw_id % enabled_stride == 0 {
                world.set_enabled(EntityId(raw_id), true);
            }
        }
        world
    }

    fn is_enabled(&self, entity: EntityId) -> bool {
        let index = entity.0 as usize;
        let word = index / 64;
        let bit = index % 64;
        self.enabled_words
            .get(word)
            .is_some_and(|value| value & (1_u64 << bit) != 0)
    }

    fn set_enabled(&mut self, entity: EntityId, enabled: bool) -> bool {
        let index = entity.0 as usize;
        if index >= self.rows.len() {
            return false;
        }
        let word = index / 64;
        let bit = index % 64;
        let mask = 1_u64 << bit;
        let before = self.enabled_words[word] & mask != 0;
        if before == enabled {
            return false;
        }
        if enabled {
            self.enabled_words[word] |= mask;
        } else {
            self.enabled_words[word] &= !mask;
        }
        true
    }

    fn integrate(&mut self, ticks: i32) -> (u64, u64) {
        let ticks = i64::from(ticks);
        let mut processed = 0_u64;
        for word_index in 0..self.enabled_words.len() {
            let mut bits = self.enabled_words[word_index];
            while bits != 0 {
                let bit = bits.trailing_zeros() as usize;
                let row_index = word_index * 64 + bit;
                if let Some(row) = self.rows.get_mut(row_index) {
                    row.integrate(ticks);
                    processed = processed.saturating_add(1);
                }
                bits &= bits - 1;
            }
        }
        (as_u64(self.enabled_words.len()), processed)
    }

    fn snapshot(&self) -> WorldSnapshot {
        WorldSnapshot::new(self.rows.iter().copied().map(Row::snapshot).collect())
    }
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct EnableableStats {
    pub toggle_changes: u64,
    pub structural_migrations: u64,
    pub structural_rows_processed: u64,
    pub mask_words_scanned: u64,
    pub mask_rows_processed: u64,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct EnableableEvidence {
    pub snapshot: WorldSnapshot,
    pub stats: EnableableStats,
}

/// Runs both enable/disable representations in lockstep and records deterministic work evidence.
///
/// # Panics
///
/// Panics when `toggles_per_round` exceeds `entity_count` or if the two representations
/// diverge in enabled state, mutations, snapshots, or useful integration work.
#[must_use]
pub fn run_toggle_scenario(
    entity_count: u32,
    rounds: u32,
    enabled_stride: u32,
    toggles_per_round: u32,
) -> EnableableEvidence {
    assert!(
        toggles_per_round <= entity_count,
        "toggles_per_round must not exceed entity_count"
    );
    let mut structural = StructuralEnableWorld::new(entity_count, enabled_stride);
    let mut mask = MaskEnableWorld::new(entity_count, enabled_stride);
    let mut stats = EnableableStats::default();

    assert_eq!(structural.snapshot(), mask.snapshot());
    if entity_count == 0 {
        return EnableableEvidence {
            snapshot: structural.snapshot(),
            stats,
        };
    }

    for round in 0..rounds {
        for offset in 0..toggles_per_round {
            let entity = EntityId(offset.wrapping_add(round) % entity_count);
            assert_eq!(structural.is_enabled(entity), mask.is_enabled(entity));
            let target = !mask.is_enabled(entity);
            let structural_changed = structural.set_enabled(entity, target);
            let mask_changed = mask.set_enabled(entity, target);
            assert_eq!(structural_changed, mask_changed);
            if mask_changed {
                stats.toggle_changes = stats.toggle_changes.saturating_add(1);
                stats.structural_migrations = stats.structural_migrations.saturating_add(1);
            }
        }

        stats.structural_rows_processed = stats
            .structural_rows_processed
            .saturating_add(structural.integrate(1));
        let (words, rows) = mask.integrate(1);
        stats.mask_words_scanned = stats.mask_words_scanned.saturating_add(words);
        stats.mask_rows_processed = stats.mask_rows_processed.saturating_add(rows);
        assert_eq!(structural.snapshot(), mask.snapshot());
    }

    assert_eq!(
        stats.structural_rows_processed, stats.mask_rows_processed,
        "both representations must perform the same useful integration work"
    );
    EnableableEvidence {
        snapshot: mask.snapshot(),
        stats,
    }
}

/// Replays only the structural enable/disable representation.
///
/// # Panics
///
/// Panics when `toggles_per_round` exceeds `entity_count`.
#[must_use]
pub fn replay_structural_scenario(
    entity_count: u32,
    rounds: u32,
    enabled_stride: u32,
    toggles_per_round: u32,
) -> WorldSnapshot {
    assert!(
        toggles_per_round <= entity_count,
        "toggles_per_round must not exceed entity_count"
    );
    let mut world = StructuralEnableWorld::new(entity_count, enabled_stride);
    if entity_count == 0 {
        return world.snapshot();
    }

    for round in 0..rounds {
        for offset in 0..toggles_per_round {
            let entity = EntityId(offset.wrapping_add(round) % entity_count);
            let target = !world.is_enabled(entity);
            world.set_enabled(entity, target);
        }
        world.integrate(1);
    }
    world.snapshot()
}

/// Replays only the stable-row enable-mask representation.
///
/// # Panics
///
/// Panics when `toggles_per_round` exceeds `entity_count`.
#[must_use]
pub fn replay_mask_scenario(
    entity_count: u32,
    rounds: u32,
    enabled_stride: u32,
    toggles_per_round: u32,
) -> WorldSnapshot {
    assert!(
        toggles_per_round <= entity_count,
        "toggles_per_round must not exceed entity_count"
    );
    let mut world = MaskEnableWorld::new(entity_count, enabled_stride);
    if entity_count == 0 {
        return world.snapshot();
    }

    for round in 0..rounds {
        for offset in 0..toggles_per_round {
            let entity = EntityId(offset.wrapping_add(round) % entity_count);
            let target = !world.is_enabled(entity);
            world.set_enabled(entity, target);
        }
        world.integrate(1);
    }
    world.snapshot()
}

fn initial_row(raw_id: u32) -> Row {
    let value = i64::from(raw_id);
    Row {
        entity: EntityId(raw_id),
        position: Position::new3(value, value.saturating_mul(2), -value),
        velocity: Velocity::new3(1, -2, 3),
    }
}

const fn words_for(rows: usize) -> usize {
    rows.div_ceil(64)
}

fn as_u64(value: usize) -> u64 {
    u64::try_from(value).unwrap_or(u64::MAX)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn enable_mask_ratchets_scan_words_without_structural_migrations() {
        let entity_count = 1_024_u32;
        let rounds = 8_u32;
        for (stride, toggles) in [(1, 0), (4, 0), (16, 0), (0, 0), (4, 256), (16, 512)] {
            let evidence = run_toggle_scenario(entity_count, rounds, stride, toggles);
            assert_eq!(
                evidence.stats.mask_words_scanned,
                16 * u64::from(rounds)
            );
            assert_eq!(
                evidence.stats.structural_migrations,
                evidence.stats.toggle_changes
            );
            assert_eq!(
                evidence.stats.structural_rows_processed,
                evidence.stats.mask_rows_processed
            );
            assert_eq!(
                evidence.stats.toggle_changes,
                u64::from(toggles) * u64::from(rounds)
            );
            assert_eq!(evidence.snapshot.entities().len(), entity_count as usize);
        }
    }

    #[test]
    fn dedicated_replays_match_lockstep_evidence() {
        for (stride, toggles) in [(1, 0), (4, 0), (0, 0), (4, 32)] {
            let expected = run_toggle_scenario(128, 4, stride, toggles).snapshot;
            assert_eq!(replay_structural_scenario(128, 4, stride, toggles), expected);
            assert_eq!(replay_mask_scenario(128, 4, stride, toggles), expected);
        }
    }

    #[test]
    fn disabled_entities_retain_component_values() {
        let mut world = MaskEnableWorld::new(4, 1);
        let before = world.snapshot();
        assert!(world.set_enabled(EntityId(2), false));
        let after_disable = world.snapshot();
        assert_eq!(before.entities()[2].velocity, after_disable.entities()[2].velocity);
        assert_eq!(before.entities()[2].position, after_disable.entities()[2].position);

        let (_, processed) = world.integrate(3);
        assert_eq!(processed, 3);
        let disabled_position = world.snapshot().entities()[2].position;
        assert_eq!(disabled_position, before.entities()[2].position);

        assert!(world.set_enabled(EntityId(2), true));
        let (_, processed) = world.integrate(1);
        assert_eq!(processed, 4);
    }

    #[test]
    fn zero_enabled_entities_do_zero_useful_work_but_scan_only_mask_words() {
        let evidence = run_toggle_scenario(1_024, 8, 0, 0);
        assert_eq!(evidence.stats.mask_rows_processed, 0);
        assert_eq!(evidence.stats.structural_rows_processed, 0);
        assert_eq!(evidence.stats.mask_words_scanned, 128);
        assert_eq!(evidence.stats.structural_migrations, 0);
    }
}
