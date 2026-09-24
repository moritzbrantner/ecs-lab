use std::{hint::black_box, time::Instant};

use ecs_enableable::run_toggle_scenario;

const CASES: &[(u32, u32)] = &[(1, 0), (4, 0), (16, 0), (0, 0), (4, 256), (16, 512)];

fn main() -> Result<(), String> {
    let timed = match std::env::args().nth(1).as_deref() {
        None => false,
        Some("--bench") => true,
        Some(_) => return Err("usage: enableable-benchmark [--bench]".to_owned()),
    };

    for &(stride, toggles) in CASES {
        let evidence = run_toggle_scenario(1_024, 8, stride, toggles);
        let stats = evidence.stats;
        println!(
            "enableable_work entities=1024 rounds=8 enabled_stride={stride} toggles_per_round={toggles} toggle_changes={} structural_migrations={} structural_rows_processed={} mask_words_scanned={} mask_rows_processed={}",
            stats.toggle_changes,
            stats.structural_migrations,
            stats.structural_rows_processed,
            stats.mask_words_scanned,
            stats.mask_rows_processed,
        );

        if timed {
            drop(black_box(run_toggle_scenario(1_024, 8, stride, toggles)));
            let mut samples = [0_u128; 7];
            for sample in &mut samples {
                let start = Instant::now();
                drop(black_box(run_toggle_scenario(
                    1_024,
                    8,
                    black_box(stride),
                    black_box(toggles),
                )));
                *sample = start.elapsed().as_nanos();
            }
            samples.sort_unstable();
            println!(
                "enableable_timing enabled_stride={stride} toggles_per_round={toggles} samples=7 median_ns={} min_ns={} max_ns={} advisory=true",
                samples[3], samples[0], samples[6],
            );
        }
    }
    Ok(())
}
