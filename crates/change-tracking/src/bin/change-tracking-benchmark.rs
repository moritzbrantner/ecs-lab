use std::{hint::black_box, time::Instant};

use ecs_change_tracking::run_fixed_scenario;

const CASES: &[(u32, u32, u32)] = &[
    (1_024, 8, 0),
    (1_024, 8, 1),
    (1_024, 8, 16),
    (1_024, 8, 256),
    (1_024, 8, 1_024),
];

fn main() -> Result<(), String> {
    let timed = match std::env::args().nth(1).as_deref() {
        None => false,
        Some("--bench") => true,
        Some(_) => return Err("usage: change-tracking-benchmark [--bench]".to_owned()),
    };

    for &(entities, rounds, changed) in CASES {
        let experiment = run_fixed_scenario(entities, rounds, changed);
        let stats = experiment.stats();
        println!(
            "change_tracking_work entities={entities} rounds={rounds} changed_per_round={changed} checkpoints={} full_rows_scanned={} full_recomputations={} incremental_rows_scanned={} incremental_recomputations={} dirty_marks={} incremental_removals={}",
            stats.checkpoints,
            stats.full_rows_scanned,
            stats.full_recomputations,
            stats.incremental_rows_scanned,
            stats.incremental_recomputations,
            stats.dirty_marks,
            stats.incremental_removals,
        );

        if timed {
            drop(black_box(run_fixed_scenario(entities, rounds, changed)));
            let mut samples = [0_u128; 7];
            for sample in &mut samples {
                let start = Instant::now();
                drop(black_box(run_fixed_scenario(
                    black_box(entities),
                    black_box(rounds),
                    black_box(changed),
                )));
                *sample = start.elapsed().as_nanos();
            }
            samples.sort_unstable();
            println!(
                "change_tracking_timing entities={entities} rounds={rounds} changed_per_round={changed} samples=7 median_ns={} min_ns={} max_ns={} advisory=true",
                samples[3], samples[0], samples[6],
            );
        }
    }
    Ok(())
}
