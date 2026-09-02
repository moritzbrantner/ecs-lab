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
}
