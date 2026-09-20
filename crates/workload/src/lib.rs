use std::fmt;

#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub struct EntityId(pub u32);

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Position {
    pub x: i64,
    pub y: i64,
    pub z: i64,
}

impl Position {
    #[must_use]
    pub const fn new(x: i64, y: i64) -> Self {
        Self { x, y, z: 0 }
    }

    #[must_use]
    pub const fn new3(x: i64, y: i64, z: i64) -> Self {
        Self { x, y, z }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Velocity {
    pub x: i32,
    pub y: i32,
    pub z: i32,
}

impl Velocity {
    #[must_use]
    pub const fn new(x: i32, y: i32) -> Self {
        Self { x, y, z: 0 }
    }

    #[must_use]
    pub const fn new3(x: i32, y: i32, z: i32) -> Self {
        Self { x, y, z }
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
    pub fn mixed_motion_scenario(
        seed: u32,
        entity_count: u32,
        rounds: u32,
        velocity_stride: u32,
    ) -> Self {
        let mut generator = Generator::new(seed);
        let mut operations = Vec::new();

        for raw_id in 0..entity_count {
            let entity = EntityId(raw_id);
            operations.push(Operation::Spawn(entity));
            operations.push(Operation::SetPosition(
                entity,
                Position::new(generator.position(), generator.position()),
            ));
            if velocity_stride != 0 && raw_id % velocity_stride == 0 {
                operations.push(Operation::SetVelocity(
                    entity,
                    Velocity::new(generator.velocity(), generator.velocity()),
                ));
            }
        }

        for _ in 0..rounds {
            let ticks = i32::from(generator.next_u32().to_le_bytes()[0] % 5 + 1);
            operations.push(Operation::Integrate { ticks });
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
    /// Creates the canonical observable world representation.
    ///
    /// Entity order is part of the snapshot foundation rather than something every consumer should
    /// repeatedly reconstruct. Backends that already emit canonical entity-id order keep their existing
    /// vector untouched; only noncanonical callers pay for sorting. Physics, controllers, liquids and
    /// other read-only systems can then perform deterministic binary lookup without allocating a
    /// temporary index.
    #[must_use]
    pub fn new(mut entities: Vec<EntitySnapshot>) -> Self {
        if !entities.windows(2).all(|pair| pair[0].id <= pair[1].id) {
            entities.sort_unstable_by_key(|entity| entity.id);
        }
        Self { entities }
    }

    #[must_use]
    pub fn entities(&self) -> &[EntitySnapshot] {
        &self.entities
    }

    /// Returns one entity from the canonical snapshot without allocating an auxiliary map.
    #[must_use]
    pub fn entity(&self, id: EntityId) -> Option<&EntitySnapshot> {
        self.entities
            .binary_search_by_key(&id, |entity| entity.id)
            .ok()
            .map(|index| &self.entities[index])
    }
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct StorageWorkStats {
    pub integration_rows_scanned: u64,
    pub integrated_entities: u64,
    pub component_lookups: u64,
    pub structural_table_transitions: u64,
}

impl StorageWorkStats {
    pub fn accumulate(&mut self, other: Self) {
        self.integration_rows_scanned = self
            .integration_rows_scanned
            .saturating_add(other.integration_rows_scanned);
        self.integrated_entities = self
            .integrated_entities
            .saturating_add(other.integrated_entities);
        self.component_lookups = self
            .component_lookups
            .saturating_add(other.component_lookups);
        self.structural_table_transitions = self
            .structural_table_transitions
            .saturating_add(other.structural_table_transitions);
    }
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct SnapshotWorkStats {
    pub slots_scanned: u64,
    pub entities_materialized: u64,
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
    use super::{EntityId, EntitySnapshot, Operation, Position, Velocity, Workload, WorldSnapshot};

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
    fn two_axis_constructors_preserve_the_legacy_z_zero_plane() {
        assert_eq!(Position::new(2, 3), Position::new3(2, 3, 0));
        assert_eq!(Velocity::new(4, 5), Velocity::new3(4, 5, 0));
    }

    #[test]
    fn world_snapshot_canonicalizes_once_and_supports_direct_lookup() {
        let first = EntitySnapshot {
            id: EntityId(1),
            position: Some(Position::new(1, 2)),
            velocity: None,
        };
        let third = EntitySnapshot {
            id: EntityId(3),
            position: Some(Position::new(3, 4)),
            velocity: Some(Velocity::new(5, 6)),
        };
        let snapshot = WorldSnapshot::new(vec![third, first]);

        assert_eq!(snapshot.entities(), [first, third]);
        assert_eq!(snapshot.entity(EntityId(1)), Some(&first));
        assert_eq!(snapshot.entity(EntityId(3)), Some(&third));
        assert_eq!(snapshot.entity(EntityId(2)), None);
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
    fn mixed_motion_scenario_keeps_component_mix_deterministic() {
        let workload = Workload::mixed_motion_scenario(17, 10, 3, 4);
        let velocity_sets = workload
            .operations()
            .iter()
            .filter(|operation| matches!(operation, Operation::SetVelocity(_, _)))
            .count();
        let integrations = workload
            .operations()
            .iter()
            .filter(|operation| matches!(operation, Operation::Integrate { .. }))
            .count();

        assert_eq!(velocity_sets, 3);
        assert_eq!(integrations, 3);
        assert_eq!(workload, Workload::mixed_motion_scenario(17, 10, 3, 4));
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
