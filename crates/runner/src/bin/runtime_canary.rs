use std::hint::black_box;

use ecs_reference::ReferenceWorld;
use ecs_sparse_set::SparseWorld;
use ecs_workload::{Workload, WorkloadError};

const ENTITY_COUNT: u32 = 5_000;
const ROUNDS: u32 = 25;
const SEEDS: [u32; 3] = [0x5EED_CAFE, 17, 99];

fn main() -> Result<(), WorkloadError> {
    for seed in SEEDS {
        let workload = Workload::motion_scenario(seed, ENTITY_COUNT, ROUNDS);

        let mut reference = ReferenceWorld::new();
        reference.replay(&workload)?;
        let reference_snapshot = reference.snapshot();

        let mut sparse = SparseWorld::new();
        sparse.replay(&workload)?;
        let sparse_snapshot = sparse.snapshot();

        assert_eq!(
            sparse_snapshot, reference_snapshot,
            "runtime canary requires exact reference/sparse parity for seed {seed}"
        );
        black_box((reference_snapshot, sparse_snapshot));
    }

    Ok(())
}
