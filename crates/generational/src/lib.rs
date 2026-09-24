use std::collections::{BTreeMap, BTreeSet};

use ecs_workload::EntityId;

#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub struct EntityHandle {
    pub slot: u32,
    pub generation: u32,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum HandleError {
    DuplicateLogicalEntity(EntityId),
    Stale(EntityHandle),
    MissingLogicalEntity(EntityId),
    SlotCapacityExhausted,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct Slot {
    generation: u32,
    logical: Option<EntityId>,
    retired: bool,
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct GenerationalStats {
    pub spawns: u64,
    pub despawns: u64,
    pub new_slots: u64,
    pub free_slot_reuses: u64,
    pub retired_slots: u64,
    pub stale_rejections: u64,
    pub peak_slots: u64,
}

#[derive(Clone, Debug, Default, Eq, PartialEq)]
pub struct GenerationalArena {
    slots: Vec<Slot>,
    free: BTreeSet<u32>,
    logical_to_handle: BTreeMap<EntityId, EntityHandle>,
    stats: GenerationalStats,
}

impl GenerationalArena {
    #[must_use]
    pub const fn new() -> Self {
        Self {
            slots: Vec::new(),
            free: BTreeSet::new(),
            logical_to_handle: BTreeMap::new(),
            stats: GenerationalStats {
                spawns: 0,
                despawns: 0,
                new_slots: 0,
                free_slot_reuses: 0,
                retired_slots: 0,
                stale_rejections: 0,
                peak_slots: 0,
            },
        }
    }

    /// Allocates a deterministic physical slot for a logical entity.
    ///
    /// # Errors
    ///
    /// Returns `DuplicateLogicalEntity` when the logical identity is already live, or
    /// `SlotCapacityExhausted` when no new slot can be represented.
    pub fn spawn(&mut self, logical: EntityId) -> Result<EntityHandle, HandleError> {
        if self.logical_to_handle.contains_key(&logical) {
            return Err(HandleError::DuplicateLogicalEntity(logical));
        }

        let (slot_index, reused) = if let Some(&slot) = self.free.first() {
            self.free.remove(&slot);
            (slot, true)
        } else {
            let slot = u32::try_from(self.slots.len())
                .map_err(|_| HandleError::SlotCapacityExhausted)?;
            self.slots.push(Slot {
                generation: 0,
                logical: None,
                retired: false,
            });
            (slot, false)
        };

        let slot = &mut self.slots[slot_index as usize];
        debug_assert!(!slot.retired);
        debug_assert!(slot.logical.is_none());
        slot.logical = Some(logical);
        let handle = EntityHandle {
            slot: slot_index,
            generation: slot.generation,
        };
        self.logical_to_handle.insert(logical, handle);

        self.stats.spawns = self.stats.spawns.saturating_add(1);
        if reused {
            self.stats.free_slot_reuses = self.stats.free_slot_reuses.saturating_add(1);
        } else {
            self.stats.new_slots = self.stats.new_slots.saturating_add(1);
            self.stats.peak_slots = self
                .stats
                .peak_slots
                .max(u64::try_from(self.slots.len()).unwrap_or(u64::MAX));
        }
        Ok(handle)
    }

    /// Removes the entity addressed by a live generation-qualified handle.
    ///
    /// # Errors
    ///
    /// Returns `Stale` when the handle does not name the current generation of a live slot.
    pub fn despawn(&mut self, handle: EntityHandle) -> Result<EntityId, HandleError> {
        let slot = self.valid_slot(handle)?;
        let logical = slot
            .logical
            .ok_or(HandleError::Stale(handle))?;
        self.logical_to_handle.remove(&logical);

        let slot = &mut self.slots[handle.slot as usize];
        slot.logical = None;
        if slot.generation == u32::MAX {
            slot.retired = true;
            self.stats.retired_slots = self.stats.retired_slots.saturating_add(1);
        } else {
            slot.generation += 1;
            self.free.insert(handle.slot);
        }
        self.stats.despawns = self.stats.despawns.saturating_add(1);
        Ok(logical)
    }

    /// Resolves a live generation-qualified handle to its logical entity identity.
    ///
    /// # Errors
    ///
    /// Returns `Stale` when the slot is absent, retired, generation-mismatched, or not live.
    pub fn resolve(&mut self, handle: EntityHandle) -> Result<EntityId, HandleError> {
        match self.valid_slot(handle).and_then(|slot| {
            slot.logical
                .ok_or(HandleError::Stale(handle))
        }) {
            Ok(logical) => Ok(logical),
            Err(error) => {
                if matches!(error, HandleError::Stale(_)) {
                    self.stats.stale_rejections =
                        self.stats.stale_rejections.saturating_add(1);
                }
                Err(error)
            }
        }
    }

    #[must_use]
    pub fn handle_for(&self, logical: EntityId) -> Option<EntityHandle> {
        self.logical_to_handle.get(&logical).copied()
    }

    #[must_use]
    pub fn canonical_logical_entities(&self) -> Vec<EntityId> {
        self.logical_to_handle.keys().copied().collect()
    }

    #[must_use]
    pub const fn stats(&self) -> GenerationalStats {
        self.stats
    }

    #[must_use]
    pub fn slot_count(&self) -> usize {
        self.slots.len()
    }

    fn valid_slot(&self, handle: EntityHandle) -> Result<&Slot, HandleError> {
        let Some(slot) = self.slots.get(handle.slot as usize) else {
            return Err(HandleError::Stale(handle));
        };
        if slot.retired || slot.generation != handle.generation {
            return Err(HandleError::Stale(handle));
        }
        Ok(slot)
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ReuseEvidence {
    pub live_entities: Vec<EntityId>,
    pub live_handles: Vec<EntityHandle>,
    pub stats: GenerationalStats,
}

/// Runs the deterministic spawn/despawn/reuse scenario.
///
/// # Panics
///
/// Panics only if the fixed scenario exhausts the representable slot space, which would violate
/// the experiment's bounded batch-size invariant.
#[must_use]
pub fn run_reuse_scenario(cycles: u32, batch_size: u32) -> ReuseEvidence {
    let mut arena = GenerationalArena::new();
    let mut previous_handles = Vec::new();
    let mut live_handles = Vec::new();

    for cycle in 0..cycles {
        live_handles.clear();
        for offset in 0..batch_size {
            let logical = EntityId(cycle.saturating_mul(batch_size).saturating_add(offset));
            let Ok(handle) = arena.spawn(logical) else {
                panic!("fixed scenario must have slot capacity");
            };
            live_handles.push(handle);
        }

        if cycle > 0 {
            for &stale in &previous_handles {
                assert_eq!(arena.resolve(stale), Err(HandleError::Stale(stale)));
            }
        }

        if cycle + 1 < cycles {
            previous_handles = live_handles.clone();
            for &handle in &live_handles {
                assert!(arena.despawn(handle).is_ok());
            }
        }
    }

    ReuseEvidence {
        live_entities: arena.canonical_logical_entities(),
        live_handles,
        stats: arena.stats(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn deterministic_reuse_prefers_the_lowest_free_slot() {
        let mut arena = GenerationalArena::new();
        let a = arena.spawn(EntityId(10)).expect("spawn");
        let b = arena.spawn(EntityId(11)).expect("spawn");
        let c = arena.spawn(EntityId(12)).expect("spawn");
        assert_eq!((a.slot, b.slot, c.slot), (0, 1, 2));

        assert_eq!(arena.despawn(b), Ok(EntityId(11)));
        assert_eq!(arena.despawn(a), Ok(EntityId(10)));

        let next = arena.spawn(EntityId(20)).expect("reuse");
        let next_two = arena.spawn(EntityId(21)).expect("reuse");
        assert_eq!(next.slot, 0);
        assert_eq!(next_two.slot, 1);
        assert_eq!(next.generation, 1);
        assert_eq!(next_two.generation, 1);
    }

    #[test]
    fn stale_handle_is_rejected_after_slot_reuse() {
        let mut arena = GenerationalArena::new();
        let old = arena.spawn(EntityId(1)).expect("spawn");
        assert_eq!(arena.despawn(old), Ok(EntityId(1)));
        let current = arena.spawn(EntityId(2)).expect("reuse");
        assert_eq!(old.slot, current.slot);
        assert_ne!(old.generation, current.generation);
        assert_eq!(arena.resolve(old), Err(HandleError::Stale(old)));
        assert_eq!(arena.resolve(current), Ok(EntityId(2)));
    }

    #[test]
    fn reuse_ratchet_keeps_slot_footprint_bounded_by_batch_size() {
        for (cycles, batch) in [(1_u32, 64_u32), (2, 64), (16, 64), (64, 4)] {
            let evidence = run_reuse_scenario(cycles, batch);
            let expected_spawns = u64::from(cycles) * u64::from(batch);
            let expected_despawns =
                u64::from(cycles.saturating_sub(1)) * u64::from(batch);
            assert_eq!(evidence.stats.spawns, expected_spawns);
            assert_eq!(evidence.stats.despawns, expected_despawns);
            assert_eq!(evidence.stats.new_slots, u64::from(batch));
            assert_eq!(
                evidence.stats.free_slot_reuses,
                expected_spawns.saturating_sub(u64::from(batch))
            );
            assert_eq!(evidence.stats.peak_slots, u64::from(batch));
            assert_eq!(
                evidence.stats.stale_rejections,
                expected_despawns
            );
            assert_eq!(evidence.live_entities.len(), batch as usize);
            assert_eq!(evidence.live_handles.len(), batch as usize);
        }
    }

    #[test]
    fn generation_exhaustion_retires_slot_instead_of_aliasing() {
        let logical = EntityId(7);
        let handle = EntityHandle {
            slot: 0,
            generation: u32::MAX,
        };
        let mut arena = GenerationalArena {
            slots: vec![Slot {
                generation: u32::MAX,
                logical: Some(logical),
                retired: false,
            }],
            free: BTreeSet::new(),
            logical_to_handle: BTreeMap::from([(logical, handle)]),
            stats: GenerationalStats {
                spawns: 1,
                new_slots: 1,
                peak_slots: 1,
                ..GenerationalStats::default()
            },
        };

        assert_eq!(arena.despawn(handle), Ok(logical));
        assert_eq!(arena.resolve(handle), Err(HandleError::Stale(handle)));
        let replacement = arena.spawn(EntityId(8)).expect("replacement slot");
        assert_eq!(replacement.slot, 1);
        assert_eq!(replacement.generation, 0);
        assert_eq!(arena.stats().retired_slots, 1);
        assert_eq!(arena.slot_count(), 2);
    }

    #[test]
    fn logical_identity_is_distinct_from_physical_handle() {
        let mut arena = GenerationalArena::new();
        let first = arena.spawn(EntityId(5)).expect("spawn");
        assert_eq!(arena.despawn(first), Ok(EntityId(5)));
        let second = arena.spawn(EntityId(5)).expect("respawn logical id");

        assert_eq!(arena.canonical_logical_entities(), vec![EntityId(5)]);
        assert_eq!(arena.handle_for(EntityId(5)), Some(second));
        assert_ne!(first, second);
    }
}
