use std::{hint::black_box, time::Instant};

use ecs_hybrid_storage::run_status_churn_scenario;

const CASES: &[(u32, u32)] = &[(4, 0), (4, 64), (4, 256), (16, 512)];

fn main() -> Result<(), String> {
    let timed = match std::env::args().nth(1).as_deref() {
        None => false,
        Some("--bench") => true,
        Some(_) => return Err("usage: hybrid-storage-benchmark [--bench]".to_owned()),
    };

    for &(stride, toggles) in CASES {
        let evidence = run_status_churn_scenario(1_024, 8, stride, toggles);
        let stats = evidence.stats;
        println!(
            "hybrid_storage_work entities=1024 rounds=8 status_stride={stride} toggles_per_round={toggles} status_changes={} archetype_motion_row_moves={} hybrid_motion_row_moves={} sparse_motion_row_moves={} archetype_rows_scanned={} hybrid_rows_scanned={} sparse_rows_scanned={} sparse_component_lookups={} hybrid_status_peak_slots={} sparse_status_peak_slots={}",
            stats.status_changes,
            stats.archetype_motion_row_moves,
            stats.hybrid_motion_row_moves,
            stats.sparse_motion_row_moves,
            stats.archetype_rows_scanned,
            stats.hybrid_rows_scanned,
            stats.sparse_rows_scanned,
            stats.sparse_component_lookups,
            stats.hybrid_status_peak_slots,
            stats.sparse_status_peak_slots,
        );

        if timed {
            drop(black_box(run_status_churn_scenario(1_024, 8, stride, toggles)));
            let mut samples = [0_u128; 7];
            for sample in &mut samples {
                let start = Instant::now();
                drop(black_box(run_status_churn_scenario(
                    1_024,
                    8,
                    black_box(stride),
                    black_box(toggles),
                )));
                *sample = start.elapsed().as_nanos();
            }
            samples.sort_unstable();
            println!(
                "hybrid_storage_timing status_stride={stride} toggles_per_round={toggles} samples=7 median_ns={} min_ns={} max_ns={} advisory=true",
                samples[3], samples[0], samples[6],
            );
        }
    }
    Ok(())
}
