use std::{hint::black_box, time::Instant};

use ecs_archetype::ArchetypeWorld;
use ecs_command_buffer::{construction_scenario, CommandBuffer};
use ecs_workload::{EntityId, Operation, Position, Velocity, WorldSnapshot};

const CASES: &[u32] = &[1, 128, 1_024, 16_384];

fn fixture(entity_count: u32) -> Vec<(EntityId, Position, Velocity)> {
    (0..entity_count)
        .map(|raw_id| {
            (
                EntityId(raw_id),
                Position::new3(
                    i64::from(raw_id),
                    i64::from(raw_id).saturating_mul(2),
                    -i64::from(raw_id),
                ),
                Velocity::new3(1, -2, 3),
            )
        })
        .collect()
}

fn replay_immediate(fixture: &[(EntityId, Position, Velocity)]) -> WorldSnapshot {
    let mut world = ArchetypeWorld::new();
    for &(entity, position, velocity) in fixture {
        for operation in [
            Operation::Spawn(entity),
            Operation::SetPosition(entity, position),
            Operation::SetVelocity(entity, velocity),
        ] {
            assert_eq!(world.apply(operation), Ok(()));
        }
    }
    world.snapshot()
}

fn replay_deferred(fixture: &[(EntityId, Position, Velocity)]) -> WorldSnapshot {
    let mut commands = CommandBuffer::new();
    for &(entity, position, velocity) in fixture {
        commands.push_operation(Operation::Spawn(entity));
        commands.push_operation(Operation::SetPosition(entity, position));
        commands.push_operation(Operation::SetVelocity(entity, velocity));
    }

    let mut world = ArchetypeWorld::new();
    let Ok(_) = commands.commit(&mut world) else {
        panic!("valid deferred benchmark fixture must commit");
    };
    world.snapshot()
}

fn replay_bundled(fixture: &[(EntityId, Position, Velocity)]) -> WorldSnapshot {
    let mut commands = CommandBuffer::new();
    for &(entity, position, velocity) in fixture {
        commands.push_spawn_bundle(entity, Some(position), Some(velocity));
    }

    let mut world = ArchetypeWorld::new();
    let Ok(_) = commands.commit(&mut world) else {
        panic!("valid bundled benchmark fixture must commit");
    };
    world.snapshot()
}

fn measure(
    entities: u32,
    implementation: &str,
    fixture: &[(EntityId, Position, Velocity)],
    replay: fn(&[(EntityId, Position, Velocity)]) -> WorldSnapshot,
) {
    drop(black_box(replay(black_box(fixture))));
    let mut samples = [0_u128; 7];
    for sample in &mut samples {
        let start = Instant::now();
        drop(black_box(replay(black_box(fixture))));
        *sample = start.elapsed().as_nanos();
    }
    samples.sort_unstable();
    println!(
        "command_buffer_timing entities={entities} implementation={implementation} samples=7 median_ns={} min_ns={} max_ns={} advisory=true",
        samples[3], samples[0], samples[6],
    );
}

fn parse_args(args: &[String]) -> Result<bool, String> {
    match args {
        [] => Ok(false),
        [mode] if mode == "--bench" => Ok(true),
        _ => Err("usage: command-buffer-benchmark [--bench]".to_owned()),
    }
}

fn main() -> Result<(), String> {
    let args: Vec<_> = std::env::args().skip(1).collect();
    let timed = parse_args(&args)?;

    for &entities in CASES {
        let evidence = construction_scenario(entities);
        println!(
            "command_buffer_work entities={entities} immediate_commands={} immediate_transitions={} deferred_commands={} deferred_transitions={} bundled_commands={} bundled_transitions={} bundled_spawns={}",
            evidence.immediate_commands,
            evidence.immediate_transitions,
            evidence.deferred.commands_committed,
            evidence.deferred.structural_table_transitions,
            evidence.bundled.commands_committed,
            evidence.bundled.structural_table_transitions,
            evidence.bundled.bundle_spawns,
        );

        if timed {
            let fixture = fixture(entities);
            for (implementation, replay) in [
                (
                    "immediate",
                    replay_immediate
                        as fn(&[(EntityId, Position, Velocity)]) -> WorldSnapshot,
                ),
                ("deferred", replay_deferred),
                ("bundled", replay_bundled),
            ] {
                let snapshot = replay(&fixture);
                assert_eq!(
                    snapshot, evidence.snapshot,
                    "{implementation} benchmark replay must preserve construction parity"
                );
                measure(entities, implementation, &fixture, replay);
            }
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn benchmark_replays_match_the_canonical_construction_snapshot() {
        for entities in [0_u32, 1, 128] {
            let fixture = fixture(entities);
            let expected = construction_scenario(entities).snapshot;
            assert_eq!(replay_immediate(&fixture), expected);
            assert_eq!(replay_deferred(&fixture), expected);
            assert_eq!(replay_bundled(&fixture), expected);
        }
    }

    #[test]
    fn benchmark_arguments_reject_trailing_values() {
        assert_eq!(parse_args(&[]), Ok(false));
        assert_eq!(parse_args(&["--bench".to_owned()]), Ok(true));
        assert!(parse_args(&["--bench".to_owned(), "extra".to_owned()]).is_err());
    }
}
