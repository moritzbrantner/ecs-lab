use std::collections::BTreeMap;

use ecs_workload::{
    EntityId, EntitySnapshot, Operation, Position, Velocity, Workload, WorkloadError, WorldSnapshot,
};

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
struct EntityState {
    position: Option<Position>,
    velocity: Option<Velocity>,
}

#[derive(Clone, Debug, Default, Eq, PartialEq)]
pub struct ReferenceWorld {
    entities: BTreeMap<EntityId, EntityState>,
}

impl ReferenceWorld {
    #[must_use]
    pub const fn new() -> Self {
        Self {
            entities: BTreeMap::new(),
        }
    }

    /// Applies one operation to the reference world.
    ///
    /// # Errors
    ///
    /// Returns [`WorkloadError`] when an operation addresses a missing entity or attempts to spawn
    /// an entity that is already alive.
    pub fn apply(&mut self, operation: Operation) -> Result<(), WorkloadError> {
        match operation {
            Operation::Spawn(entity) => self.spawn(entity),
            Operation::Despawn(entity) => self.despawn(entity),
            Operation::SetPosition(entity, position) => {
                self.entity_mut(entity)?.position = Some(position);
                Ok(())
            }
            Operation::RemovePosition(entity) => {
                self.entity_mut(entity)?.position = None;
                Ok(())
            }
            Operation::SetVelocity(entity, velocity) => {
                self.entity_mut(entity)?.velocity = Some(velocity);
                Ok(())
            }
            Operation::RemoveVelocity(entity) => {
                self.entity_mut(entity)?.velocity = None;
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
    /// Returns the first [`WorkloadError`] produced by the workload. Earlier successful operations
    /// remain applied, matching normal sequential execution.
    pub fn replay(&mut self, workload: &Workload) -> Result<(), WorkloadError> {
        for operation in workload.operations() {
            self.apply(*operation)?;
        }
        Ok(())
    }

    #[must_use]
    pub fn snapshot(&self) -> WorldSnapshot {
        WorldSnapshot::new(
            self.entities
                .iter()
                .map(|(&id, state)| EntitySnapshot {
                    id,
                    position: state.position,
                    velocity: state.velocity,
                })
                .collect(),
        )
    }

    fn spawn(&mut self, entity: EntityId) -> Result<(), WorkloadError> {
        if self.entities.contains_key(&entity) {
            return Err(WorkloadError::EntityAlreadyExists(entity));
        }
        self.entities.insert(entity, EntityState::default());
        Ok(())
    }

    fn despawn(&mut self, entity: EntityId) -> Result<(), WorkloadError> {
        self.entities
            .remove(&entity)
            .map(|_| ())
            .ok_or(WorkloadError::MissingEntity(entity))
    }

    fn entity_mut(&mut self, entity: EntityId) -> Result<&mut EntityState, WorkloadError> {
        self.entities
            .get_mut(&entity)
            .ok_or(WorkloadError::MissingEntity(entity))
    }

    fn integrate(&mut self, ticks: i32) {
        let ticks = i64::from(ticks);
        for state in self.entities.values_mut() {
            let (Some(position), Some(velocity)) = (&mut state.position, state.velocity) else {
                continue;
            };
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
    use ecs_workload::{
        EntityId, EntitySnapshot, Operation, Position, Velocity, Workload, WorkloadError,
        WorldSnapshot,
    };

    use super::ReferenceWorld;

    #[test]
    fn replays_component_lifecycle_and_integration() {
        let entity = EntityId(9);
        let workload = Workload::new(vec![
            Operation::Spawn(entity),
            Operation::SetPosition(entity, Position::new(10, -2)),
            Operation::SetVelocity(entity, Velocity::new(3, 4)),
            Operation::Integrate { ticks: 5 },
        ]);
        let mut world = ReferenceWorld::new();

        assert_eq!(world.replay(&workload), Ok(()));
        assert_eq!(
            world.snapshot(),
            WorldSnapshot::new(vec![EntitySnapshot {
                id: entity,
                position: Some(Position::new(25, 18)),
                velocity: Some(Velocity::new(3, 4)),
            }])
        );
    }

    #[test]
    fn integration_advances_all_three_axes() {
        let entity = EntityId(3);
        let workload = Workload::new(vec![
            Operation::Spawn(entity),
            Operation::SetPosition(entity, Position::new3(1, 2, 3)),
            Operation::SetVelocity(entity, Velocity::new3(2, -1, 4)),
            Operation::Integrate { ticks: 3 },
        ]);
        let mut world = ReferenceWorld::new();
        assert_eq!(world.replay(&workload), Ok(()));
        assert_eq!(
            world.snapshot().entities()[0].position,
            Some(Position::new3(7, -1, 15))
        );
    }

    #[test]
    fn rejects_duplicate_spawn_without_mutating_existing_entity() {
        let entity = EntityId(4);
        let mut world = ReferenceWorld::new();
        assert_eq!(world.apply(Operation::Spawn(entity)), Ok(()));
        assert_eq!(
            world.apply(Operation::Spawn(entity)),
            Err(WorkloadError::EntityAlreadyExists(entity))
        );
        assert_eq!(world.snapshot().entities().len(), 1);
    }

    #[test]
    fn rejects_component_updates_for_missing_entities() {
        let entity = EntityId(42);
        let mut world = ReferenceWorld::new();

        assert_eq!(
            world.apply(Operation::SetVelocity(entity, Velocity::new(1, 1))),
            Err(WorkloadError::MissingEntity(entity))
        );
        assert_eq!(world.snapshot(), WorldSnapshot::default());
    }
}
