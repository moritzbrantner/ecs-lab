use std::collections::VecDeque;

use ecs_archetype::ArchetypeWorld;
use ecs_workload::{
    EntityId, Operation, Position, Velocity, WorkloadError, WorldSnapshot,
};

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BufferedCommand {
    Operation(Operation),
    SpawnBundle {
        entity: EntityId,
        position: Option<Position>,
        velocity: Option<Velocity>,
    },
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct CommitStats {
    pub batches: u64,
    pub commands_committed: u64,
    pub structural_table_transitions: u64,
    pub bundle_spawns: u64,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct CommitFailure {
    pub error: WorkloadError,
    pub committed: CommitStats,
    pub remaining_commands: usize,
}

#[derive(Clone, Debug, Default, Eq, PartialEq)]
pub struct CommandBuffer {
    commands: VecDeque<BufferedCommand>,
}

impl CommandBuffer {
    #[must_use]
    pub const fn new() -> Self {
        Self {
            commands: VecDeque::new(),
        }
    }

    pub fn push_operation(&mut self, operation: Operation) {
        self.commands
            .push_back(BufferedCommand::Operation(operation));
    }

    pub fn push_spawn_bundle(
        &mut self,
        entity: EntityId,
        position: Option<Position>,
        velocity: Option<Velocity>,
    ) {
        self.commands.push_back(BufferedCommand::SpawnBundle {
            entity,
            position,
            velocity,
        });
    }

    #[must_use]
    pub fn len(&self) -> usize {
        self.commands.len()
    }

    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.commands.is_empty()
    }

    /// Commits buffered commands in FIFO order.
    ///
    /// # Errors
    ///
    /// Returns the first failed command together with stats for commands committed before it.
    /// The failing command and all later commands remain buffered.
    pub fn commit(&mut self, world: &mut ArchetypeWorld) -> Result<CommitStats, CommitFailure> {
        let mut stats = CommitStats {
            batches: u64::from(!self.commands.is_empty()),
            ..CommitStats::default()
        };

        while let Some(command) = self.commands.front().copied() {
            let result = match command {
                BufferedCommand::Operation(operation) => {
                    stats.structural_table_transitions = stats
                        .structural_table_transitions
                        .saturating_add(
                            world
                                .operation_work(operation)
                                .structural_table_transitions,
                        );
                    world.apply(operation)
                }
                BufferedCommand::SpawnBundle {
                    entity,
                    position,
                    velocity,
                } => {
                    let result = world.spawn_bundle(entity, position, velocity);
                    if result.is_ok() {
                        stats.bundle_spawns = stats.bundle_spawns.saturating_add(1);
                    }
                    result
                }
            };

            if let Err(error) = result {
                return Err(CommitFailure {
                    error,
                    committed: stats,
                    remaining_commands: self.commands.len(),
                });
            }
            self.commands.pop_front();
            stats.commands_committed = stats.commands_committed.saturating_add(1);
        }

        Ok(stats)
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ConstructionEvidence {
    pub snapshot: WorldSnapshot,
    pub immediate_commands: u64,
    pub immediate_transitions: u64,
    pub deferred: CommitStats,
    pub bundled: CommitStats,
}

/// Builds the fixed construction experiment and verifies parity across all strategies.
///
/// # Panics
///
/// Panics if a fixed valid construction command fails or if deferred/bundled snapshots diverge
/// from immediate construction.
#[must_use]
pub fn construction_scenario(entity_count: u32) -> ConstructionEvidence {
    let mut immediate = ArchetypeWorld::new();
    let mut immediate_commands = 0_u64;
    let mut immediate_transitions = 0_u64;

    let mut deferred_commands = CommandBuffer::new();
    let mut bundled_commands = CommandBuffer::new();

    for raw_id in 0..entity_count {
        let entity = EntityId(raw_id);
        let position = Position::new3(i64::from(raw_id), i64::from(raw_id) * 2, -i64::from(raw_id));
        let velocity = Velocity::new3(1, -2, 3);
        let operations = [
            Operation::Spawn(entity),
            Operation::SetPosition(entity, position),
            Operation::SetVelocity(entity, velocity),
        ];

        for operation in operations {
            immediate_transitions = immediate_transitions.saturating_add(
                immediate
                    .operation_work(operation)
                    .structural_table_transitions,
            );
            assert_eq!(immediate.apply(operation), Ok(()));
            immediate_commands = immediate_commands.saturating_add(1);
            deferred_commands.push_operation(operation);
        }
        bundled_commands.push_spawn_bundle(entity, Some(position), Some(velocity));
    }

    let expected = immediate.snapshot();

    let mut deferred_world = ArchetypeWorld::new();
    let Ok(deferred) = deferred_commands.commit(&mut deferred_world) else {
        panic!("valid deferred construction must commit");
    };
    assert_eq!(deferred_world.snapshot(), expected);

    let mut bundled_world = ArchetypeWorld::new();
    let Ok(bundled) = bundled_commands.commit(&mut bundled_world) else {
        panic!("valid bundled construction must commit");
    };
    assert_eq!(bundled_world.snapshot(), expected);

    ConstructionEvidence {
        snapshot: expected,
        immediate_commands,
        immediate_transitions,
        deferred,
        bundled,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn construction_ratchet_distinguishes_deferral_from_bulk_construction() {
        for entity_count in [0_u32, 1, 128, 1_024] {
            let evidence = construction_scenario(entity_count);
            let entities = u64::from(entity_count);
            assert_eq!(evidence.snapshot.entities().len(), entity_count as usize);
            assert_eq!(evidence.immediate_commands, entities * 3);
            assert_eq!(evidence.immediate_transitions, entities * 2);
            assert_eq!(evidence.deferred.commands_committed, entities * 3);
            assert_eq!(evidence.deferred.structural_table_transitions, entities * 2);
            assert_eq!(evidence.deferred.bundle_spawns, 0);
            assert_eq!(evidence.bundled.commands_committed, entities);
            assert_eq!(evidence.bundled.structural_table_transitions, 0);
            assert_eq!(evidence.bundled.bundle_spawns, entities);
        }
    }

    #[test]
    fn deferred_lifecycle_preserves_immediate_semantics_at_commit_boundary() {
        let entity = EntityId(7);
        let operations = [
            Operation::Spawn(entity),
            Operation::SetPosition(entity, Position::new3(1, 2, 3)),
            Operation::SetVelocity(entity, Velocity::new3(4, 5, 6)),
            Operation::RemovePosition(entity),
            Operation::SetPosition(entity, Position::new3(8, 9, 10)),
            Operation::RemoveVelocity(entity),
            Operation::Despawn(entity),
        ];

        let mut immediate = ArchetypeWorld::new();
        let mut buffered = CommandBuffer::new();
        for operation in operations {
            assert_eq!(immediate.apply(operation), Ok(()));
            buffered.push_operation(operation);
        }

        let mut deferred = ArchetypeWorld::new();
        let stats = buffered.commit(&mut deferred).expect("valid lifecycle");
        assert_eq!(deferred.snapshot(), immediate.snapshot());
        assert_eq!(stats.commands_committed, operations.len() as u64);
        assert!(buffered.is_empty());
    }

    #[test]
    fn commit_failure_preserves_failing_and_later_commands() {
        let missing = EntityId(42);
        let mut buffer = CommandBuffer::new();
        buffer.push_operation(Operation::SetPosition(missing, Position::new(1, 2)));
        buffer.push_operation(Operation::Spawn(EntityId(1)));

        let mut world = ArchetypeWorld::new();
        let failure = buffer.commit(&mut world).expect_err("first command must fail");

        assert_eq!(failure.error, WorkloadError::MissingEntity(missing));
        assert_eq!(failure.committed.commands_committed, 0);
        assert_eq!(failure.remaining_commands, 2);
        assert_eq!(buffer.len(), 2);
        assert!(world.snapshot().entities().is_empty());
    }

    #[test]
    fn duplicate_bundle_fails_without_consuming_the_command() {
        let entity = EntityId(1);
        let mut world = ArchetypeWorld::new();
        assert_eq!(world.spawn_bundle(entity, None, None), Ok(()));

        let mut buffer = CommandBuffer::new();
        buffer.push_spawn_bundle(entity, Some(Position::new(1, 2)), None);
        let failure = buffer.commit(&mut world).expect_err("duplicate must fail");

        assert_eq!(
            failure.error,
            WorkloadError::EntityAlreadyExists(entity)
        );
        assert_eq!(failure.remaining_commands, 1);
        assert_eq!(buffer.len(), 1);
    }
}
