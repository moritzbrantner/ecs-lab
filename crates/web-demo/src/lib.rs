use ecs_reference::ReferenceWorld;
use ecs_workload::{EntityId, Operation, Position, Velocity};

const ENTITY: EntityId = EntityId(1);
const START: Position = Position::new(8, 6);
const VELOCITY: Velocity = Velocity::new(3, 2);

fn simulated_position(ticks: i32) -> Position {
    let mut world = ReferenceWorld::new();
    let operations = [
        Operation::Spawn(ENTITY),
        Operation::SetPosition(ENTITY, START),
        Operation::SetVelocity(ENTITY, VELOCITY),
        Operation::Integrate { ticks },
    ];

    for operation in operations {
        if world.apply(operation).is_err() {
            return START;
        }
    }

    world
        .snapshot()
        .entities()
        .first()
        .and_then(|entity| entity.position)
        .unwrap_or(START)
}

#[unsafe(no_mangle)]
pub extern "C" fn start_x() -> i64 {
    START.x
}

#[unsafe(no_mangle)]
pub extern "C" fn start_y() -> i64 {
    START.y
}

#[unsafe(no_mangle)]
pub extern "C" fn velocity_x() -> i32 {
    VELOCITY.x
}

#[unsafe(no_mangle)]
pub extern "C" fn velocity_y() -> i32 {
    VELOCITY.y
}

#[unsafe(no_mangle)]
pub extern "C" fn position_x_after(ticks: i32) -> i64 {
    simulated_position(ticks).x
}

#[unsafe(no_mangle)]
pub extern "C" fn position_y_after(ticks: i32) -> i64 {
    simulated_position(ticks).y
}

#[cfg(test)]
mod tests {
    use super::{position_x_after, position_y_after, start_x, start_y, velocity_x, velocity_y};

    #[test]
    fn exported_fixture_uses_reference_world_integration() {
        assert_eq!((start_x(), start_y()), (8, 6));
        assert_eq!((velocity_x(), velocity_y()), (3, 2));
        assert_eq!((position_x_after(0), position_y_after(0)), (8, 6));
        assert_eq!((position_x_after(5), position_y_after(5)), (23, 16));
    }
}
