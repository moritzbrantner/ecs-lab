use ecs_reference::ReferenceWorld;
use ecs_workload::{EntityId, Operation, Position, Velocity, Workload};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let entity = EntityId(1);
    let workload = Workload::new(vec![
        Operation::Spawn(entity),
        Operation::SetPosition(entity, Position::new(0, 0)),
        Operation::SetVelocity(entity, Velocity::new(2, 1)),
        Operation::Integrate { ticks: 3 },
    ]);

    let mut world = ReferenceWorld::new();
    world.replay(&workload)?;
    println!("{:#?}", world.snapshot());
    Ok(())
}
