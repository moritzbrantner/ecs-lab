use std::{hint::black_box, time::Instant};

use ecs_enableable::{
    replay_mask_scenario, replay_structural_scenario, run_toggle_scenario,
};
use ecs_workload::WorldSnapshot;

const CASES: &[(u32, u32)] = &[(1, 0), (4, 0), (16, 0), (0, 0), (4, 256), (16, 512)];

fn measure(
    implementation: &str,
    stride: u32,
    toggles: u32,
    replay: fn(u32, u32, u32, u32) -> WorldSnapshot,
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
        "enableable_timing implementation={implementation} enabled_stride={stride} toggles_per_round={toggles} samples=7 median_ns={} min_ns={} max_ns={} advisory=true",
        samples[3], samples[0], samples[6],
    );
}

fn parse_args(args: &[String]) -> Result<bool, String> {
    match args {
        [] => Ok(false),
        [mode] if mode == "--bench" => Ok(true),
        _ => Err("usage: enableable-benchmark [--bench]".to_owned()),
    }
}

fn main() -> Result<(), String> {
    let args: Vec<_> = std::env::args().skip(1).collect();
    let timed = parse_args(&args)?;

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
            for (implementation, replay) in [
                (
                    "structural",
                    replay_structural_scenario as fn(u32, u32, u32, u32) -> WorldSnapshot,
                ),
                ("mask", replay_mask_scenario),
            ] {
                assert_eq!(
                    replay(1_024, 8, stride, toggles),
                    evidence.snapshot,
                    "{implementation} benchmark replay must preserve enableable parity"
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
