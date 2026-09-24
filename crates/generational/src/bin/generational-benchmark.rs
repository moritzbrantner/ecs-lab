use std::{hint::black_box, time::Instant};

use ecs_generational::run_reuse_scenario;

const CASES: &[(u32, u32)] = &[(2, 1_024), (16, 1_024), (256, 64)];

fn parse_args(args: &[String]) -> Result<bool, String> {
    match args {
        [] => Ok(false),
        [mode] if mode == "--bench" => Ok(true),
        _ => Err("usage: generational-benchmark [--bench]".to_owned()),
    }
}

fn main() -> Result<(), String> {
    let args: Vec<_> = std::env::args().skip(1).collect();
    let timed = parse_args(&args)?;

    for &(cycles, batch) in CASES {
        let evidence = run_reuse_scenario(cycles, batch);
        let stats = evidence.stats;
        println!(
            "generational_work cycles={cycles} batch_size={batch} spawns={} despawns={} new_slots={} free_slot_reuses={} retired_slots={} stale_rejections={} peak_slots={} live_entities={}",
            stats.spawns,
            stats.despawns,
            stats.new_slots,
            stats.free_slot_reuses,
            stats.retired_slots,
            stats.stale_rejections,
            stats.peak_slots,
            evidence.live_entities.len(),
        );

        if timed {
            drop(black_box(run_reuse_scenario(cycles, batch)));
            let mut samples = [0_u128; 7];
            for sample in &mut samples {
                let start = Instant::now();
                drop(black_box(run_reuse_scenario(
                    black_box(cycles),
                    black_box(batch),
                )));
                *sample = start.elapsed().as_nanos();
            }
            samples.sort_unstable();
            println!(
                "generational_timing cycles={cycles} batch_size={batch} samples=7 median_ns={} min_ns={} max_ns={} advisory=true",
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
        assert!(parse_args(&["--bench".to_owned(), "extra".to_owned()]).is_err());
    }
}
