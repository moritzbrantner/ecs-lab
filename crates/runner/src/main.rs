mod benchmarks;

use benchmarks::run_benchmarks;
use ecs_sparse_set::SparseWorld;
use ecs_workload::{EntityId, Operation, Position, Velocity, Workload};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut arguments = std::env::args().skip(1);
    match arguments.next().as_deref() {
        Some("benchmark") => {
            let fingerprint = benchmark_fingerprint(arguments.next());
            run_benchmarks(false, &fingerprint);
        }
        Some("benchmark-smoke") => {
            let fingerprint = benchmark_fingerprint(arguments.next());
            run_benchmarks(true, &fingerprint);
        }
        _ => run_demo()?,
    }
    Ok(())
}

fn benchmark_fingerprint(argument: Option<String>) -> String {
    argument.unwrap_or_else(|| "unverified".to_owned())
}

fn run_demo() -> Result<(), Box<dyn std::error::Error>> {
    let entity = EntityId(1);
    let workload = Workload::new(vec![
        Operation::Spawn(entity),
        Operation::SetPosition(entity, Position::new(0, 0)),
        Operation::SetVelocity(entity, Velocity::new(2, 1)),
        Operation::Integrate { ticks: 3 },
    ]);

    let mut world = SparseWorld::new();
    world.replay(&workload)?;
    println!("{:#?}", world.snapshot());
    Ok(())
}
