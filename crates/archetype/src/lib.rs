use ecs_workload::{
    EntityId, EntitySnapshot, Operation, Position, Velocity, Workload, WorkloadError, WorldSnapshot,
};

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum TableKind {
    Empty,
    Position,
    Velocity,
    Motion,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct Location {
    table: TableKind,
    index: usize,
}

#[derive(Clone, Debug, Default, Eq, PartialEq)]
struct EntityTable {
    entities: Vec<EntityId>,
}

impl EntityTable {
    fn push(&mut self, entity: EntityId) -> usize {
        let index = self.entities.len();
        self.entities.push(entity);
        index
    }

    fn swap_remove(&mut self, index: usize) -> (EntityId, Option<EntityId>) {
        let removed = self.entities.swap_remove(index);
        let moved = self.entities.get(index).copied();
        (removed, moved)
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
struct ComponentTable<T> {
    entities: Vec<EntityId>,
    values: Vec<T>,
}

impl<T> Default for ComponentTable<T> {
    fn default() -> Self {
        Self {
            entities: Vec::new(),
            values: Vec::new(),
        }
    }
}

impl<T> ComponentTable<T> {
    fn push(&mut self, entity: EntityId, value: T) -> usize {
        let index = self.entities.len();
        self.entities.push(entity);
        self.values.push(value);
        index
    }

    fn swap_remove(&mut self, index: usize) -> (EntityId, T, Option<EntityId>) {
        let removed_entity = self.entities.swap_remove(index);
        let removed_value = self.values.swap_remove(index);
        let moved = self.entities.get(index).copied();
        (removed_entity, removed_value, moved)
    }
}

#[derive(Clone, Debug, Default, Eq, PartialEq)]
struct MotionTable {
    entities: Vec<EntityId>,
    positions: Vec<Position>,
    velocities: Vec<Velocity>,
}

impl MotionTable {
    fn push(&mut self, entity: EntityId, position: Position, velocity: Velocity) -> usize {
        let index = self.entities.len();
        self.entities.push(entity);
        self.positions.push(position);
        self.velocities.push(velocity);
        index
    }

    fn swap_remove(
        &mut self,
        index: usize,
    ) -> (EntityId, Position, Velocity, Option<EntityId>) {
        let removed_entity = self.entities.swap_remove(index);
        let removed_position = self.positions.swap_remove(index);
        let removed_velocity = self.velocities.swap_remove(index);
        let moved = self.entities.get(index).copied();
        (removed_entity, removed_position, removed_velocity, moved)
    }
}

/// Experimental archetype/table ECS backend.
///
/// Entities move between four compact tables according to whether they own Position and/or Velocity.
/// The Position+Velocity table is the integration hot path: both columns are traversed contiguously
/// with no per-row sparse lookup. Structural changes pay the table-move cost instead.
#[derive(Clone, Debug, Default, Eq, PartialEq)]
pub struct ArchetypeWorld {
    locations: Vec<Option<Location>>,
    empty: EntityTable,
    positions: ComponentTable<Position>,
    velocities: ComponentTable<Velocity>,
    motion: MotionTable,
}

impl ArchetypeWorld {
    #[must_use]
    pub const fn new() -> Self {
        Self {
            locations: Vec::new(),
            empty: EntityTable {
                entities: Vec::new(),
            },
            positions: ComponentTable {
                entities: Vec::new(),
                values: Vec::new(),
            },
            velocities: ComponentTable {
                entities: Vec::new(),
                values: Vec::new(),
            },
            motion: MotionTable {
                entities: Vec::new(),
                positions: Vec::new(),
                velocities: Vec::new(),
            },
        }
    }

    /// Applies one operation using archetype/table storage.
    ///
    /// # Errors
    ///
    /// Returns WorkloadError for invalid entity lifecycle operations.
    pub fn apply(&mut self, operation: Operation) -> Result<(), WorkloadError> {
        match operation {
            Operation::Spawn(entity) => self.spawn(entity),
            Operation::Despawn(entity) => self.despawn(entity),
            Operation::SetPosition(entity, position) => self.set_position(entity, position),
            Operation::RemovePosition(entity) => self.remove_position(entity),
            Operation::SetVelocity(entity, velocity) => self.set_velocity(entity, velocity),
            Operation::RemoveVelocity(entity) => self.remove_velocity(entity),
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
    /// Returns the first WorkloadError produced by the workload.
    pub fn replay(&mut self, workload: &Workload) -> Result<(), WorkloadError> {
        for operation in workload.operations() {
            self.apply(*operation)?;
        }
        Ok(())
    }

    #[must_use]
    pub fn snapshot(&self) -> WorldSnapshot {
        let mut entities = Vec::with_capacity(self.entity_count());

        entities.extend(self.empty.entities.iter().copied().map(|id| EntitySnapshot {
            id,
            position: None,
            velocity: None,
        }));
        entities.extend(
            self.positions
                .entities
                .iter()
                .copied()
                .zip(self.positions.values.iter().copied())
                .map(|(id, position)| EntitySnapshot {
                    id,
                    position: Some(position),
                    velocity: None,
                }),
        );
        entities.extend(
            self.velocities
                .entities
                .iter()
                .copied()
                .zip(self.velocities.values.iter().copied())
                .map(|(id, velocity)| EntitySnapshot {
                    id,
                    position: None,
                    velocity: Some(velocity),
                }),
        );
        entities.extend(
            self.motion
                .entities
                .iter()
                .copied()
                .zip(self.motion.positions.iter().copied())
                .zip(self.motion.velocities.iter().copied())
                .map(|((id, position), velocity)| EntitySnapshot {
                    id,
                    position: Some(position),
                    velocity: Some(velocity),
                }),
        );

        WorldSnapshot::new(entities)
    }

    fn spawn(&mut self, entity: EntityId) -> Result<(), WorkloadError> {
        self.ensure_slot(entity);
        let slot = entity.0 as usize;
        if self.locations[slot].is_some() {
            return Err(WorkloadError::EntityAlreadyExists(entity));
        }

        let index = self.empty.push(entity);
        self.locations[slot] = Some(Location {
            table: TableKind::Empty,
            index,
        });
        Ok(())
    }

    fn despawn(&mut self, entity: EntityId) -> Result<(), WorkloadError> {
        let location = self.location(entity)?;
        match location.table {
            TableKind::Empty => {
                let (removed, moved) = self.empty.swap_remove(location.index);
                debug_assert_eq!(removed, entity);
                self.repair_moved(moved, TableKind::Empty, location.index);
            }
            TableKind::Position => {
                let (removed, _position, moved) = self.positions.swap_remove(location.index);
                debug_assert_eq!(removed, entity);
                self.repair_moved(moved, TableKind::Position, location.index);
            }
            TableKind::Velocity => {
                let (removed, _velocity, moved) = self.velocities.swap_remove(location.index);
                debug_assert_eq!(removed, entity);
                self.repair_moved(moved, TableKind::Velocity, location.index);
            }
            TableKind::Motion => {
                let (removed, _position, _velocity, moved) =
                    self.motion.swap_remove(location.index);
                debug_assert_eq!(removed, entity);
                self.repair_moved(moved, TableKind::Motion, location.index);
            }
        }

        self.locations[entity.0 as usize] = None;
        Ok(())
    }

    fn set_position(
        &mut self,
        entity: EntityId,
        position: Position,
    ) -> Result<(), WorkloadError> {
        let location = self.location(entity)?;
        match location.table {
            TableKind::Empty => {
                let (removed, moved) = self.empty.swap_remove(location.index);
                debug_assert_eq!(removed, entity);
                self.repair_moved(moved, TableKind::Empty, location.index);
                let index = self.positions.push(entity, position);
                self.set_location(entity, TableKind::Position, index);
            }
            TableKind::Position => self.positions.values[location.index] = position,
            TableKind::Velocity => {
                let (removed, velocity, moved) = self.velocities.swap_remove(location.index);
                debug_assert_eq!(removed, entity);
                self.repair_moved(moved, TableKind::Velocity, location.index);
                let index = self.motion.push(entity, position, velocity);
                self.set_location(entity, TableKind::Motion, index);
            }
            TableKind::Motion => self.motion.positions[location.index] = position,
        }
        Ok(())
    }

    fn remove_position(&mut self, entity: EntityId) -> Result<(), WorkloadError> {
        let location = self.location(entity)?;
        match location.table {
            TableKind::Empty | TableKind::Velocity => {}
            TableKind::Position => {
                let (removed, _position, moved) = self.positions.swap_remove(location.index);
                debug_assert_eq!(removed, entity);
                self.repair_moved(moved, TableKind::Position, location.index);
                let index = self.empty.push(entity);
                self.set_location(entity, TableKind::Empty, index);
            }
            TableKind::Motion => {
                let (removed, _position, velocity, moved) =
                    self.motion.swap_remove(location.index);
                debug_assert_eq!(removed, entity);
                self.repair_moved(moved, TableKind::Motion, location.index);
                let index = self.velocities.push(entity, velocity);
                self.set_location(entity, TableKind::Velocity, index);
            }
        }
        Ok(())
    }

    fn set_velocity(
        &mut self,
        entity: EntityId,
        velocity: Velocity,
    ) -> Result<(), WorkloadError> {
        let location = self.location(entity)?;
        match location.table {
            TableKind::Empty => {
                let (removed, moved) = self.empty.swap_remove(location.index);
                debug_assert_eq!(removed, entity);
                self.repair_moved(moved, TableKind::Empty, location.index);
                let index = self.velocities.push(entity, velocity);
                self.set_location(entity, TableKind::Velocity, index);
            }
            TableKind::Position => {
                let (removed, position, moved) = self.positions.swap_remove(location.index);
                debug_assert_eq!(removed, entity);
                self.repair_moved(moved, TableKind::Position, location.index);
                let index = self.motion.push(entity, position, velocity);
                self.set_location(entity, TableKind::Motion, index);
            }
            TableKind::Velocity => self.velocities.values[location.index] = velocity,
            TableKind::Motion => self.motion.velocities[location.index] = velocity,
        }
        Ok(())
    }

    fn remove_velocity(&mut self, entity: EntityId) -> Result<(), WorkloadError> {
        let location = self.location(entity)?;
        match location.table {
            TableKind::Empty | TableKind::Position => {}
            TableKind::Velocity => {
                let (removed, _velocity, moved) = self.velocities.swap_remove(location.index);
                debug_assert_eq!(removed, entity);
                self.repair_moved(moved, TableKind::Velocity, location.index);
                let index = self.empty.push(entity);
                self.set_location(entity, TableKind::Empty, index);
            }
            TableKind::Motion => {
                let (removed, position, _velocity, moved) =
                    self.motion.swap_remove(location.index);
                debug_assert_eq!(removed, entity);
                self.repair_moved(moved, TableKind::Motion, location.index);
                let index = self.positions.push(entity, position);
                self.set_location(entity, TableKind::Position, index);
            }
        }
        Ok(())
    }

    fn integrate(&mut self, ticks: i32) {
        let ticks = i64::from(ticks);
        for (position, velocity) in self
            .motion
            .positions
            .iter_mut()
            .zip(self.motion.velocities.iter().copied())
        {
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

    fn entity_count(&self) -> usize {
        self.empty.entities.len()
            + self.positions.entities.len()
            + self.velocities.entities.len()
            + self.motion.entities.len()
    }

    fn ensure_slot(&mut self, entity: EntityId) {
        let slot = entity.0 as usize;
        if self.locations.len() <= slot {
            self.locations.resize(slot + 1, None);
        }
    }

    fn location(&self, entity: EntityId) -> Result<Location, WorkloadError> {
        self.locations
            .get(entity.0 as usize)
            .copied()
            .flatten()
            .ok_or(WorkloadError::MissingEntity(entity))
    }

    fn set_location(&mut self, entity: EntityId, table: TableKind, index: usize) {
        self.locations[entity.0 as usize] = Some(Location { table, index });
    }

    fn repair_moved(&mut self, moved: Option<EntityId>, table: TableKind, index: usize) {
        if let Some(entity) = moved {
            self.set_location(entity, table, index);
        }
    }
}

#[cfg(test)]
mod tests {
    use ecs_reference::ReferenceWorld;
    use ecs_workload::{EntityId, Operation, Position, Velocity, WorkloadError};

    use super::ArchetypeWorld;

    #[test]
    fn component_shape_transitions_match_reference_world() {
        let operations = [
            Operation::Spawn(EntityId(5)),
            Operation::Spawn(EntityId(2)),
            Operation::SetPosition(EntityId(5), Position::new3(1, 2, 3)),
            Operation::SetVelocity(EntityId(2), Velocity::new3(4, 5, 6)),
            Operation::SetVelocity(EntityId(5), Velocity::new3(2, -1, 4)),
            Operation::SetPosition(EntityId(2), Position::new3(-3, 8, 1)),
            Operation::Integrate { ticks: 3 },
            Operation::RemoveVelocity(EntityId(5)),
            Operation::Integrate { ticks: 2 },
            Operation::SetVelocity(EntityId(5), Velocity::new3(1, 1, 1)),
            Operation::RemovePosition(EntityId(2)),
            Operation::Despawn(EntityId(5)),
        ];

        let mut reference = ReferenceWorld::new();
        let mut archetype = ArchetypeWorld::new();
        for operation in operations {
            let expected = reference.apply(operation);
            assert_eq!(archetype.apply(operation), expected);
            assert_eq!(archetype.snapshot(), reference.snapshot());
        }
    }

    #[test]
    fn swap_remove_repairs_moved_motion_row_location() {
        let mut reference = ReferenceWorld::new();
        let mut archetype = ArchetypeWorld::new();

        for raw_id in 0..3 {
            let entity = EntityId(raw_id);
            for operation in [
                Operation::Spawn(entity),
                Operation::SetPosition(entity, Position::new3(i64::from(raw_id), 0, 0)),
                Operation::SetVelocity(entity, Velocity::new3(1, 2, 3)),
            ] {
                assert_eq!(archetype.apply(operation), reference.apply(operation));
            }
        }

        for operation in [
            Operation::RemoveVelocity(EntityId(1)),
            Operation::SetPosition(EntityId(2), Position::new3(20, 30, 40)),
            Operation::Integrate { ticks: 2 },
        ] {
            assert_eq!(archetype.apply(operation), reference.apply(operation));
        }

        assert_eq!(archetype.snapshot(), reference.snapshot());
    }

    #[test]
    fn integration_only_advances_entities_in_the_motion_table() {
        let workload = [
            Operation::Spawn(EntityId(0)),
            Operation::Spawn(EntityId(1)),
            Operation::Spawn(EntityId(2)),
            Operation::SetPosition(EntityId(0), Position::new(10, 10)),
            Operation::SetVelocity(EntityId(1), Velocity::new(3, 4)),
            Operation::SetPosition(EntityId(2), Position::new(5, 6)),
            Operation::SetVelocity(EntityId(2), Velocity::new(2, -1)),
            Operation::Integrate { ticks: 4 },
        ];

        let mut reference = ReferenceWorld::new();
        let mut archetype = ArchetypeWorld::new();
        for operation in workload {
            assert_eq!(archetype.apply(operation), reference.apply(operation));
        }

        assert_eq!(archetype.snapshot(), reference.snapshot());
    }

    #[test]
    fn lifecycle_errors_match_reference_world() {
        let mut reference = ReferenceWorld::new();
        let mut archetype = ArchetypeWorld::new();
        let missing = EntityId(42);

        for operation in [
            Operation::SetPosition(missing, Position::new(1, 1)),
            Operation::SetVelocity(missing, Velocity::new(1, 1)),
            Operation::Despawn(missing),
        ] {
            assert_eq!(
                archetype.apply(operation),
                reference.apply(operation),
                "operation {operation:?}"
            );
            assert_eq!(archetype.snapshot(), reference.snapshot());
        }

        let entity = EntityId(7);
        assert_eq!(archetype.apply(Operation::Spawn(entity)), Ok(()));
        assert_eq!(
            archetype.apply(Operation::Spawn(entity)),
            Err(WorkloadError::EntityAlreadyExists(entity))
        );
    }
}
