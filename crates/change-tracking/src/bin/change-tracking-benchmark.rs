use std::{hint::black_box, time::Instant};

use ecs_change_tracking::run_fixed_scenario;

const CASES: &[(u32, u32, u32)] = &[
    (1_024, 8, 0),
    (1_024, 8, 1),
    (1_024, 8, 16),
    (1_024, 8, 256),
    (1_024, 8, 1_024),
];

fn parse_args(args: &[String]) -> Result<bool, String> {
    match args {
        [] => Ok(false),
        [mode] if mode == "--bench" => Ok(true),
        _ => Err("usage: change-tracking-benchmark [--bench]".to_owned()),
    }
}

fn main() -> Result<(), String> {
    let args: Vec<_> = std::env::args().skip(1).collect();
    let timed = parse_args(&args)?;

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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn benchmark_arguments_reject_trailing_values() {
        assert_eq!(parse_args(&[]), Ok(false));
        assert_eq!(parse_args(&["--bench".to_owned()]), Ok(true));
        assert!(parse_args(&["--bench".to_owned(), "--unexpected".to_owned()]).is_err());
        assert!(parse_args(&["--unexpected".to_owned()]).is_err());
    }
}
