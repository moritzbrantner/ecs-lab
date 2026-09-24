use std::{hint::black_box, time::Instant};

use ecs_hybrid_storage::{
    replay_archetype_status_scenario, replay_hybrid_status_scenario,
    replay_sparse_status_scenario, run_status_churn_scenario, HybridSnapshot,
};

const CASES: &[(u32, u32)] = &[(4, 0), (4, 64), (4, 256), (16, 512)];

fn measure(
    implementation: &str,
    stride: u32,
    toggles: u32,
    replay: fn(u32, u32, u32, u32) -> HybridSnapshot,
) {
    drop(black_box(replay(
        1_024,
        8,
        black_box(stride),
        black_box(toggles),
    )));
    let mut samples = [0_u128; 7];
    for sample in &mut samples {
        let start = Instant::now();
        drop(black_box(replay(
            1_024,
            8,
            black_box(stride),
            black_box(toggles),
        )));
        *sample = start.elapsed().as_nanos();
    }
    samples.sort_unstable();
    println!(
        "hybrid_storage_timing implementation={implementation} status_stride={stride} toggles_per_round={toggles} samples=7 median_ns={} min_ns={} max_ns={} advisory=true",
        samples[3], samples[0], samples[6],
    );
}

fn parse_args(args: &[String]) -> Result<bool, String> {
    match args {
        [] => Ok(false),
        [mode] if mode == "--bench" => Ok(true),
        _ => Err("usage: hybrid-storage-benchmark [--bench]".to_owned()),
    }
}

fn main() -> Result<(), String> {
    let args: Vec<_> = std::env::args().skip(1).collect();
    let timed = parse_args(&args)?;

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
            for (implementation, replay) in [
                (
                    "archetype",
                    replay_archetype_status_scenario
                        as fn(u32, u32, u32, u32) -> HybridSnapshot,
                ),
                ("hybrid", replay_hybrid_status_scenario),
                ("sparse", replay_sparse_status_scenario),
            ] {
                assert_eq!(
                    replay(1_024, 8, stride, toggles),
                    evidence.snapshot,
                    "{implementation} timing replay must preserve status-churn parity"
                );
                measure(implementation, stride, toggles, replay);
            }
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn benchmark_arguments_reject_trailing_values() {
        assert_eq!(parse_args(&[]), Ok(false));
        assert_eq!(parse_args(&["--bench".to_owned()]), Ok(true));
        assert!(parse_args(&["--bench".to_owned(), "extra".to_owned()]).is_err());
    }
}
