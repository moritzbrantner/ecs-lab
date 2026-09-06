use ecs_workload::{EntityId, Operation, Velocity, WorldSnapshot};

use crate::PhysicsStep3d;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ControllerConfig3d {
    pub move_speed: i32,
    pub jump_speed: i32,
}

impl Default for ControllerConfig3d {
    fn default() -> Self {
        Self {
            move_speed: 6,
            jump_speed: 12,
        }
    }
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct ControllerInput3d {
    pub move_x: i8,
    pub move_z: i8,
    pub jump: bool,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ControllerError3d {
    MissingEntity(EntityId),
    MissingVelocity(EntityId),
    InvalidMoveAxis(i8),
    NegativeMoveSpeed(i32),
    NegativeJumpSpeed(i32),
}

/// Converts one replayable controller input into an ordinary ECS velocity mutation.
///
/// Horizontal policy belongs to the controller rather than the collision solver. Jumping is allowed
/// only when the immediately preceding canonical Rust physics step reports support for the entity.
/// Browser input can therefore be transported as this small discrete command without becoming physics
/// authority.
///
/// # Errors
///
/// Returns [`ControllerError3d`] for missing ECS state or invalid controller configuration/input.
pub fn controller_operations(
    snapshot: &WorldSnapshot,
    preceding_physics: &PhysicsStep3d,
    entity: EntityId,
    input: ControllerInput3d,
    config: ControllerConfig3d,
) -> Result<Vec<Operation>, ControllerError3d> {
    validate_input(input)?;
    validate_config(config)?;
    let state = snapshot
        .entities()
        .iter()
        .find(|candidate| candidate.id == entity)
        .ok_or(ControllerError3d::MissingEntity(entity))?;
    let current = state
        .velocity
        .ok_or(ControllerError3d::MissingVelocity(entity))?;

    let next = Velocity::new3(
        i32::from(input.move_x).saturating_mul(config.move_speed),
        if input.jump && preceding_physics.is_supported(entity) {
            config.jump_speed
        } else {
            current.y
        },
        i32::from(input.move_z).saturating_mul(config.move_speed),
    );
    if next == current {
        Ok(Vec::new())
    } else {
        Ok(vec![Operation::SetVelocity(entity, next)])
    }
}

fn validate_input(input: ControllerInput3d) -> Result<(), ControllerError3d> {
    if !(-1..=1).contains(&input.move_x) {
        return Err(ControllerError3d::InvalidMoveAxis(input.move_x));
    }
    if !(-1..=1).contains(&input.move_z) {
        return Err(ControllerError3d::InvalidMoveAxis(input.move_z));
    }
    Ok(())
}

fn validate_config(config: ControllerConfig3d) -> Result<(), ControllerError3d> {
    if config.move_speed < 0 {
        return Err(ControllerError3d::NegativeMoveSpeed(config.move_speed));
    }
    if config.jump_speed < 0 {
        return Err(ControllerError3d::NegativeJumpSpeed(config.jump_speed));
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use ecs_workload::{EntityId, EntitySnapshot, Position, Velocity, WorldSnapshot};

    use crate::{PhysicsStep3d, PhysicsStep3dStats};

    use super::{ControllerConfig3d, ControllerError3d, ControllerInput3d, controller_operations};

    const PLAYER: EntityId = EntityId(7);

    fn snapshot(velocity: Velocity) -> WorldSnapshot {
        WorldSnapshot::new(vec![EntitySnapshot {
            id: PLAYER,
            position: Some(Position::new3(0, 2, 0)),
            velocity: Some(velocity),
        }])
    }

    fn physics(supported: bool) -> PhysicsStep3d {
        PhysicsStep3d {
            operations: Vec::new(),
            stats: PhysicsStep3dStats::default(),
            contacts: Vec::new(),
            supporting_entities: if supported { vec![PLAYER] } else { Vec::new() },
        }
    }

    #[test]
    fn grounded_jump_sets_vertical_speed() {
        let operations = controller_operations(
            &snapshot(Velocity::new3(0, -1, 0)),
            &physics(true),
            PLAYER,
            ControllerInput3d {
                move_x: 1,
                move_z: -1,
                jump: true,
            },
            ControllerConfig3d::default(),
        )
        .expect("valid controller command");
        assert_eq!(
            operations,
            vec![ecs_workload::Operation::SetVelocity(
                PLAYER,
                Velocity::new3(6, 12, -6)
            )]
        );
    }

    #[test]
    fn airborne_jump_does_not_reset_vertical_speed() {
        let operations = controller_operations(
            &snapshot(Velocity::new3(0, -4, 0)),
            &physics(false),
            PLAYER,
            ControllerInput3d {
                jump: true,
                ..ControllerInput3d::default()
            },
            ControllerConfig3d::default(),
        )
        .expect("valid controller command");
        assert!(operations.is_empty());
    }

    #[test]
    fn repeated_grounded_command_is_idempotent() {
        let state = snapshot(Velocity::new3(6, 0, 0));
        let operations = controller_operations(
            &state,
            &physics(true),
            PLAYER,
            ControllerInput3d {
                move_x: 1,
                ..ControllerInput3d::default()
            },
            ControllerConfig3d::default(),
        )
        .expect("valid controller command");
        assert!(operations.is_empty());
    }

    #[test]
    fn invalid_axis_fails_closed() {
        assert_eq!(
            controller_operations(
                &snapshot(Velocity::new3(0, 0, 0)),
                &physics(true),
                PLAYER,
                ControllerInput3d {
                    move_x: 2,
                    ..ControllerInput3d::default()
                },
                ControllerConfig3d::default(),
            ),
            Err(ControllerError3d::InvalidMoveAxis(2))
        );
    }
}
