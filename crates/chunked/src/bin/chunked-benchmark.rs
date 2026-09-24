use std::{hint::black_box, time::Instant};

use ecs_chunked::run_chunked_scenario;

const ENTITY_COUNTS: &[u32] = &[1_024, 65_536];
const CHUNK_SIZES: &[usize] = &[16, 64, 256];

fn main() -> Result<(), String> {
    let timed = match std::env::args().nth(1).as_deref() {
        None => false,
        Some("--bench") => true,
        Some(_) => return Err("usage: chunked-benchmark [--bench]".to_owned()),
    };

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
                drop(black_box(run_chunked_scenario(entities, 32, chunk_size)));
                let mut samples = [0_u128; 7];
                for sample in &mut samples {
                    let start = Instant::now();
                    drop(black_box(run_chunked_scenario(
                        black_box(entities),
                        32,
                        black_box(chunk_size),
                    )));
                    *sample = start.elapsed().as_nanos();
                }
                samples.sort_unstable();
                println!(
                    "chunked_timing entities={entities} chunk_size={chunk_size} samples=7 median_ns={} min_ns={} max_ns={} advisory=true",
                    samples[3], samples[0], samples[6],
                );
            }
        }
    }
    Ok(())
}
