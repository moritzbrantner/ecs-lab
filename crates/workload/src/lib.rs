use std::fmt;

#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub struct EntityId(pub u32);

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Position {
    pub x: i64,
    pub y: i64,
}

impl Position {
    #[must_use]
    pub const fn new(x: i64, y: i64) -> Self {
        Self { x, y }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Velocity {
    pub x: i32,
    pub y: i32,
}

impl Velocity {
    #[must_use]
    pub const fn new(x: i32, y: i32) -> Self {
        Self { x, y }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Operation {
    Spawn(EntityId),
    Despawn(EntityId),
    SetPosition(EntityId, Position),
    RemovePosition(EntityId),
    SetVelocity(EntityId, Velocity),
    RemoveVelocity(EntityId),
    Integrate { ticks: i32 },
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Workload {
    operations: Vec<Operation>,
}

impl Workload {
    #[must_use]
    pub fn new(operations: impl Into<Vec<Operation>>) -> Self {
        Self {
            operations: operations.into(),
        }
    }

    #[must_use]
    pub fn motion_scenario(seed: u32, entity_count: u32, rounds: u32) -> Self {
        let mut generator = Generator::new(seed);
        let mut operations = Vec::new();

        for raw_id in 0..entity_count {
            let entity = EntityId(raw_id);
            operations.push(Operation::Spawn(entity));
            operations.push(Operation::SetPosition(
                entity,
                Position::new(generator.position(), generator.position()),
            ));
            operations.push(Operation::SetVelocity(
                entity,
                Velocity::new(generator.velocity(), generator.velocity()),
            ));
        }

        for _ in 0..rounds {
            let ticks = i32::from(generator.next_u32().to_le_bytes()[0] % 5 + 1);
            operations.push(Operation::Integrate { ticks });
            if entity_count > 0 {
                let entity = EntityId(generator.next_u32() % entity_count);
                operations.push(Operation::RemoveVelocity(entity));
                operations.push(Operation::SetVelocity(
                    entity,
                    Velocity::new(generator.velocity(), generator.velocity()),
                ));
            }
        }

        Self::new(operations)
    }

    #[must_use]
    pub fn operations(&self) -> &[Operation] {
        &self.operations
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct EntitySnapshot {
    pub id: EntityId,
    pub position: Option<Position>,
    pub velocity: Option<Velocity>,
}

#[derive(Clone, Debug, Default, Eq, PartialEq)]
pub struct WorldSnapshot {
    entities: Vec<EntitySnapshot>,
}

impl WorldSnapshot {
    #[must_use]
    pub fn new(entities: Vec<EntitySnapshot>) -> Self {
        Self { entities }
    }

    #[must_use]
    pub fn entities(&self) -> &[EntitySnapshot] {
        &self.entities
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum WorkloadError {
    EntityAlreadyExists(EntityId),
    MissingEntity(EntityId),
}

impl fmt::Display for WorkloadError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::EntityAlreadyExists(entity) => {
                write!(formatter, "entity {} already exists", entity.0)
            }
            Self::MissingEntity(entity) => write!(formatter, "entity {} does not exist", entity.0),
        }
    }
}

impl std::error::Error for WorkloadError {}

struct Generator {
    state: u32,
}

impl Generator {
    const fn new(seed: u32) -> Self {
        Self { state: seed }
    }

    fn next_u32(&mut self) -> u32 {
        self.state = self
            .state
            .wrapping_mul(1_664_525)
            .wrapping_add(1_013_904_223);
        self.state
    }

    fn position(&mut self) -> i64 {
        i64::from(self.next_u32() % 2_001) - 1_000
    }

    fn velocity(&mut self) -> i32 {
        i32::from(self.next_u32().to_le_bytes()[0] % 21) - 10
    }
}

#[cfg(test)]
mod tests {
    use super::{EntityId, Operation, Workload};

    #[test]
    fn workload_preserves_operation_order() {
        let operations = vec![
            Operation::Spawn(EntityId(7)),
            Operation::Despawn(EntityId(7)),
        ];
        let workload = Workload::new(operations.clone());

        assert_eq!(workload.operations(), operations);
    }

    #[test]
    fn motion_scenario_is_seed_deterministic() {
        assert_eq!(
            Workload::motion_scenario(17, 32, 5),
            Workload::motion_scenario(17, 32, 5)
        );
        assert_ne!(
            Workload::motion_scenario(17, 32, 5),
            Workload::motion_scenario(18, 32, 5)
        );
    }

    #[test]
    fn empty_motion_scenario_remains_valid() {
        let workload = Workload::motion_scenario(1, 0, 4);
        assert_eq!(workload.operations().len(), 4);
        assert!(
            workload
                .operations()
                .iter()
                .all(|operation| matches!(operation, Operation::Integrate { .. }))
        );
    }
}
