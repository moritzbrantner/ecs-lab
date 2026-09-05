use std::collections::BTreeSet;

use ecs_workload::{
    EntityId, EntitySnapshot, Operation, Position, Velocity, Workload, WorkloadError, WorldSnapshot,
};

#[derive(Clone, Debug, Eq, PartialEq)]
struct SparseSet<T> {
    sparse: Vec<Option<usize>>,
    dense_entities: Vec<EntityId>,
    dense_values: Vec<T>,
}

impl<T> Default for SparseSet<T> {
    fn default() -> Self {
        Self {
            sparse: Vec::new(),
            dense_entities: Vec::new(),
            dense_values: Vec::new(),
        }
    }
}

impl<T> SparseSet<T> {
    fn insert(&mut self, entity: EntityId, value: T) {
        let slot = entity.0 as usize;
        if self.sparse.len() <= slot {
            self.sparse.resize(slot + 1, None);
        }
        if let Some(index) = self.sparse[slot] {
            self.dense_values[index] = value;
            return;
        }
        let index = self.dense_values.len();
        self.sparse[slot] = Some(index);
        self.dense_entities.push(entity);
        self.dense_values.push(value);
    }

    fn get(&self, entity: EntityId) -> Option<&T> {
        let index = self.sparse.get(entity.0 as usize).copied().flatten()?;
        self.dense_values.get(index)
    }

    fn remove(&mut self, entity: EntityId) -> Option<T> {
        let slot = entity.0 as usize;
        let index = self.sparse.get(slot).copied().flatten()?;
        self.sparse[slot] = None;
        self.dense_entities.swap_remove(index);
        let removed = self.dense_values.swap_remove(index);
        if index < self.dense_entities.len() {
            let moved = self.dense_entities[index];
            self.sparse[moved.0 as usize] = Some(index);
        }
        Some(removed)
    }
}

#[derive(Clone, Debug, Default, Eq, PartialEq)]
pub struct SparseWorld {
    alive: BTreeSet<EntityId>,
    positions: SparseSet<Position>,
    velocities: SparseSet<Velocity>,
}

impl SparseWorld {
    #[must_use]
    pub const fn new() -> Self {
        Self {
            alive: BTreeSet::new(),
            positions: SparseSet {
                sparse: Vec::new(),
                dense_entities: Vec::new(),
                dense_values: Vec::new(),
            },
            velocities: SparseSet {
                sparse: Vec::new(),
                dense_entities: Vec::new(),
                dense_values: Vec::new(),
            },
        }
    }

    /// Applies one operation using sparse-set component storage.
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
                self.alive.remove(&entity);
                self.positions.remove(entity);
                self.velocities.remove(entity);
                Ok(())
            }
            Operation::SetPosition(entity, position) => {
                self.require_alive(entity)?;
                self.positions.insert(entity, position);
                Ok(())
            }
            Operation::RemovePosition(entity) => {
                self.require_alive(entity)?;
                self.positions.remove(entity);
                Ok(())
            }
            Operation::SetVelocity(entity, velocity) => {
                self.require_alive(entity)?;
                self.velocities.insert(entity, velocity);
                Ok(())
            }
            Operation::RemoveVelocity(entity) => {
                self.require_alive(entity)?;
                self.velocities.remove(entity);
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

    fn require_alive(&self, entity: EntityId) -> Result<(), WorkloadError> {
        if self.alive.contains(&entity) {
            Ok(())
        } else {
            Err(WorkloadError::MissingEntity(entity))
        }
    }

    fn integrate(&mut self, ticks: i32) {
        let ticks = i64::from(ticks);
        for index in 0..self.positions.dense_entities.len() {
            let entity = self.positions.dense_entities[index];
            let Some(velocity) = self.velocities.get(entity).copied() else {
                continue;
            };
            let position = &mut self.positions.dense_values[index];
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

    use super::SparseWorld;

    #[test]
    fn sparse_set_repair_keeps_moved_dense_entry_addressable() {
        let mut world = SparseWorld::new();
        for raw_id in 0..3 {
            let entity = EntityId(raw_id);
            assert_eq!(world.apply(Operation::Spawn(entity)), Ok(()));
            assert_eq!(
                world.apply(Operation::SetPosition(
                    entity,
                    Position::new(i64::from(raw_id), 0)
                )),
                Ok(())
            );
        }
        assert_eq!(world.apply(Operation::RemovePosition(EntityId(1))), Ok(()));
        assert_eq!(
            world.apply(Operation::SetVelocity(EntityId(2), Velocity::new(2, 0))),
            Ok(())
        );
        assert_eq!(world.apply(Operation::Integrate { ticks: 2 }), Ok(()));
        assert_eq!(
            world
                .snapshot()
                .entities()
                .iter()
                .find(|entity| entity.id == EntityId(2))
                .and_then(|entity| entity.position),
            Some(Position::new(6, 0))
        );
    }

    #[test]
    fn sparse_set_integrates_z_and_matches_reference() {
        let entity = EntityId(7);
        let workload = Workload::new(vec![
            Operation::Spawn(entity),
            Operation::SetPosition(entity, Position::new3(1, 2, 3)),
            Operation::SetVelocity(entity, Velocity::new3(2, -1, 4)),
            Operation::Integrate { ticks: 3 },
        ]);
        let mut reference = ReferenceWorld::new();
        let mut sparse = SparseWorld::new();
        assert_eq!(reference.replay(&workload), Ok(()));
        assert_eq!(sparse.replay(&workload), Ok(()));
        assert_eq!(sparse.snapshot(), reference.snapshot());
    }

    #[test]
    fn deterministic_motion_scenarios_match_reference_world() {
        for seed in [0, 1, 17, 99, u32::MAX] {
            let workload = Workload::motion_scenario(seed, 128, 32);
            let mut reference = ReferenceWorld::new();
            let mut sparse = SparseWorld::new();

            assert_eq!(reference.replay(&workload), Ok(()));
            assert_eq!(sparse.replay(&workload), Ok(()));
            assert_eq!(sparse.snapshot(), reference.snapshot(), "seed {seed}");
        }
    }

    #[test]
    fn invalid_lifecycle_errors_match_reference_world() {
        let operations = [
            Operation::Despawn(EntityId(2)),
            Operation::SetPosition(EntityId(2), Position::new(1, 1)),
        ];
        for operation in operations {
            let mut reference = ReferenceWorld::new();
            let mut sparse = SparseWorld::new();
            let expected = reference.apply(operation);
            assert_eq!(expected, Err(WorkloadError::MissingEntity(EntityId(2))));
            assert_eq!(sparse.apply(operation), expected);
            assert_eq!(sparse.snapshot(), reference.snapshot());
        }
    }
}
