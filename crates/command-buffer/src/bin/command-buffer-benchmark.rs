use std::{hint::black_box, time::Instant};

use ecs_command_buffer::construction_scenario;

const CASES: &[u32] = &[1, 128, 1_024, 16_384];

fn main() -> Result<(), String> {
    let timed = match std::env::args().nth(1).as_deref() {
        None => false,
        Some("--bench") => true,
        Some(_) => return Err("usage: command-buffer-benchmark [--bench]".to_owned()),
    };

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
            drop(black_box(construction_scenario(entities)));
            let mut samples = [0_u128; 7];
            for sample in &mut samples {
                let start = Instant::now();
                drop(black_box(construction_scenario(black_box(entities))));
                *sample = start.elapsed().as_nanos();
            }
            samples.sort_unstable();
            println!(
                "command_buffer_timing entities={entities} samples=7 median_ns={} min_ns={} max_ns={} advisory=true",
                samples[3], samples[0], samples[6],
            );
        }
    }
    Ok(())
}
