use std::{hint::black_box, time::Instant};

use ecs_scheduler::{run_parallel_systems, run_partitioned, run_serial};

fn measure(label: &str, mut run: impl FnMut()) {
    run();
    let mut samples = [0_u128; 7];
    for sample in &mut samples {
        let start = Instant::now();
        run();
        *sample = start.elapsed().as_nanos();
    }
    samples.sort_unstable();
    println!(
        "scheduler_timing mode={label} samples=7 median_ns={} min_ns={} max_ns={} advisory=true",
        samples[3], samples[0], samples[6]
    );
}

fn parse_args(args: &[String]) -> Result<bool, String> {
    match args {
        [] => Ok(false),
        [mode] if mode == "--bench" => Ok(true),
        _ => Err("usage: scheduler-benchmark [--bench]".to_owned()),
    }
}

fn main() -> Result<(), String> {
    let args: Vec<_> = std::env::args().skip(1).collect();
    let timed = parse_args(&args)?;

    let serial = run_serial(16_384, 32);
    let parallel = run_parallel_systems(16_384, 32);
    let partitioned = run_partitioned(16_384, 32, 4);
    assert_eq!(serial.snapshot, parallel.snapshot);
    assert_eq!(serial.snapshot, partitioned.snapshot);

    println!(
        "scheduler_work entities=16384 rounds=32 batches_per_round={} parallel_batches_per_round={} barriers={} useful_entity_system_runs={} schedule_rebuilds={}",
        parallel.stats.batches_per_round,
        parallel.stats.parallel_batches_per_round,
        parallel.stats.barriers,
        parallel.stats.useful_entity_system_runs,
        parallel.stats.schedule_rebuilds,
    );

    if timed {
        measure("serial", || drop(black_box(run_serial(16_384, 32))));
        measure("parallel-systems", || {
            drop(black_box(run_parallel_systems(16_384, 32)));
        });
        measure("partitioned-4", || {
            drop(black_box(run_partitioned(16_384, 32, 4)));
        });
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
