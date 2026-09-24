use std::{hint::black_box, time::Instant};

use ecs_chunked::{replay_chunked_scenario, replay_flat_scenario, run_chunked_scenario};
use ecs_workload::WorldSnapshot;

const ENTITY_COUNTS: &[u32] = &[1_024, 65_536];
const CHUNK_SIZES: &[usize] = &[16, 64, 256];

fn measure<F>(entities: u32, chunk_size: usize, implementation: &str, mut replay: F)
where
    F: FnMut() -> WorldSnapshot,
{
    drop(black_box(replay()));
    let mut samples = [0_u128; 7];
    for sample in &mut samples {
        let start = Instant::now();
        drop(black_box(replay()));
        *sample = start.elapsed().as_nanos();
    }
    samples.sort_unstable();
    println!(
        "chunked_timing entities={entities} chunk_size={chunk_size} implementation={implementation} samples=7 median_ns={} min_ns={} max_ns={} advisory=true",
        samples[3], samples[0], samples[6],
    );
}

fn parse_args(args: &[String]) -> Result<bool, String> {
    match args {
        [] => Ok(false),
        [mode] if mode == "--bench" => Ok(true),
        _ => Err("usage: chunked-benchmark [--bench]".to_owned()),
    }
}

fn main() -> Result<(), String> {
    let args: Vec<_> = std::env::args().skip(1).collect();
    let timed = parse_args(&args)?;

    for &entities in ENTITY_COUNTS {
        for &chunk_size in CHUNK_SIZES {
            let evidence = run_chunked_scenario(entities, 32, chunk_size);
            let stats = evidence.stats;
            println!(
                "chunked_work entities={entities} rounds=32 chunk_size={chunk_size} flat_rows_processed={} chunk_rows_processed={} chunk_blocks_visited={} chunk_count={} partial_chunk_rows={} logical_chunk_allocations={} rows_copied_during_growth={}",
                stats.flat_rows_processed,
                stats.chunk_rows_processed,
                stats.chunk_blocks_visited,
                stats.chunk_count,
                stats.partial_chunk_rows,
                stats.logical_chunk_allocations,
                stats.rows_copied_during_growth,
            );

            if timed {
                let flat = replay_flat_scenario(entities, 32);
                assert_eq!(flat, evidence.snapshot, "flat timing replay must preserve parity");
                let chunked = replay_chunked_scenario(entities, 32, chunk_size);
                assert_eq!(
                    chunked, evidence.snapshot,
                    "chunked timing replay must preserve parity"
                );

                measure(entities, chunk_size, "flat", || {
                    replay_flat_scenario(black_box(entities), 32)
                });
                measure(entities, chunk_size, "chunked", || {
                    replay_chunked_scenario(black_box(entities), 32, black_box(chunk_size))
                });
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
