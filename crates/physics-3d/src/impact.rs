use std::collections::BTreeMap;

use ecs_physics::BodyKind;
use ecs_workload::{EntityId, Operation, Position, Velocity, WorldSnapshot};

use crate::{ContactNormal3d, PhysicsBody3d, PhysicsConfig3d, PhysicsStep3d};

const MAX_FRAGMENT_COUNT: u32 = 8;
const FRAGMENT_OFFSETS: [[i32; 3]; MAX_FRAGMENT_COUNT as usize] = [
    [-1, -1, -1],
    [1, -1, -1],
    [-1, 1, -1],
    [1, 1, -1],
    [-1, -1, 1],
    [1, -1, 1],
    [-1, 1, 1],
    [1, 1, 1],
];

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ImpactEvidence3d {
    pub left: EntityId,
    pub right: EntityId,
    pub normal: ContactNormal3d,
    /// Mass-weighted normal velocity change caused by collision response beyond free-flight gravity.
    ///
    /// This is an integer solver-step response measure rather than a continuous SI impulse. It is
    /// suitable for deterministic gameplay thresholds and remains replayable across ECS storage backends.
    pub response_momentum_units: u64,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct DestructionRecipe3d {
    pub threshold_momentum_units: u64,
    pub fragment_count: u32,
    pub separation_speed: i32,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ImpactError3d {
    NonPositiveTicks(i32),
    MissingEntity(EntityId),
    MissingVelocity(EntityId),
    MissingPosition(EntityId),
    MissingBody(EntityId),
    InvalidFragmentCount(u32),
    NegativeSeparationSpeed(i32),
    FragmentIdOverflow,
    ArithmeticOverflow,
}

/// Derives deterministic impact response evidence from one authoritative physics step.
///
/// Free-flight gravity is removed before measuring velocity change, so ordinary falling does not look
/// like an impact. The evidence is then attached to canonical contact pairs from the Rust solver. When a
/// body participates in several contacts during the same step, the body-level response is intentionally
/// shared across those contacts; downstream systems should use a stable maximum/threshold policy rather
/// than sum the records as if they were independent impulses.
///
/// # Errors
///
/// Returns [`ImpactError3d`] for invalid ticks, missing body/ECS state, or arithmetic overflow.
pub fn impact_evidence(
    snapshot: &WorldSnapshot,
    bodies: &[PhysicsBody3d],
    step: &PhysicsStep3d,
    config: PhysicsConfig3d,
    ticks: i32,
) -> Result<Vec<ImpactEvidence3d>, ImpactError3d> {
    if ticks <= 0 {
        return Err(ImpactError3d::NonPositiveTicks(ticks));
    }

    let body_map = bodies
        .iter()
        .map(|body| (body.entity, *body))
        .collect::<BTreeMap<_, _>>();
    let snapshots = snapshot
        .entities()
        .iter()
        .map(|entity| (entity.id, *entity))
        .collect::<BTreeMap<_, _>>();
    let mut final_velocities = snapshots
        .iter()
        .filter_map(|(entity, state)| state.velocity.map(|velocity| (*entity, velocity)))
        .collect::<BTreeMap<_, _>>();

    for operation in step.operations() {
        if let Operation::SetVelocity(entity, velocity) = *operation {
            final_velocities.insert(entity, velocity);
        }
    }

    let mut response_by_entity = BTreeMap::new();
    for body in bodies {
        if body.kind == BodyKind::Fixed {
            continue;
        }
        let state = snapshots
            .get(&body.entity)
            .ok_or(ImpactError3d::MissingEntity(body.entity))?;
        let initial = state
            .velocity
            .ok_or(ImpactError3d::MissingVelocity(body.entity))?;
        let final_velocity = final_velocities
            .get(&body.entity)
            .copied()
            .ok_or(ImpactError3d::MissingVelocity(body.entity))?;
        let free = Velocity::new3(
            initial
                .x
                .saturating_add(config.gravity.x.saturating_mul(ticks)),
            initial
                .y
                .saturating_add(config.gravity.y.saturating_mul(ticks)),
            initial
                .z
                .saturating_add(config.gravity.z.saturating_mul(ticks)),
        );
        response_by_entity.insert(body.entity, velocity_delta(final_velocity, free));
    }

    let mut evidence = BTreeMap::new();
    for contact in step.contacts() {
        let left_body = body_map
            .get(&contact.left)
            .ok_or(ImpactError3d::MissingBody(contact.left))?;
        let right_body = body_map
            .get(&contact.right)
            .ok_or(ImpactError3d::MissingBody(contact.right))?;
        let left_momentum = normal_response_momentum(
            *left_body,
            response_by_entity.get(&contact.left).copied(),
            contact.normal,
        )?;
        let right_momentum = normal_response_momentum(
            *right_body,
            response_by_entity.get(&contact.right).copied(),
            contact.normal,
        )?;
        let response_momentum_units = left_momentum.max(right_momentum);
        let key = (
            contact.left,
            contact.right,
            contact.normal.x,
            contact.normal.y,
            contact.normal.z,
        );
        evidence
            .entry(key)
            .and_modify(|current: &mut ImpactEvidence3d| {
                current.response_momentum_units =
                    current.response_momentum_units.max(response_momentum_units);
            })
            .or_insert(ImpactEvidence3d {
                left: contact.left,
                right: contact.right,
                normal: contact.normal,
                response_momentum_units,
            });
    }
    Ok(evidence.into_values().collect())
}

/// Produces a predefined deterministic fragment pattern when the strongest target impact crosses the
/// recipe threshold.
///
/// Destruction remains a separate ECS behavior: physics supplies impact evidence, while this function
/// emits ordinary despawn/spawn/component mutations. Stable sequential fragment ids and a fixed offset
/// table make repeated replay bit-for-bit identical.
///
/// # Errors
///
/// Returns [`ImpactError3d`] for invalid recipes, missing target state, id overflow, or arithmetic
/// overflow.
pub fn destruction_operations(
    snapshot: &WorldSnapshot,
    evidence: &[ImpactEvidence3d],
    target: EntityId,
    first_fragment_id: EntityId,
    recipe: DestructionRecipe3d,
) -> Result<Vec<Operation>, ImpactError3d> {
    validate_recipe(recipe)?;
    let strongest = evidence
        .iter()
        .filter(|impact| impact.left == target || impact.right == target)
        .map(|impact| impact.response_momentum_units)
        .max()
        .unwrap_or(0);
    if strongest < recipe.threshold_momentum_units {
        return Ok(Vec::new());
    }

    let state = snapshot
        .entities()
        .iter()
        .find(|entity| entity.id == target)
        .ok_or(ImpactError3d::MissingEntity(target))?;
    let position = state
        .position
        .ok_or(ImpactError3d::MissingPosition(target))?;
    let velocity = state.velocity.unwrap_or_else(|| Velocity::new3(0, 0, 0));
    let mut operations = Vec::with_capacity(1 + recipe.fragment_count as usize * 3);
    operations.push(Operation::Despawn(target));

    for fragment_index in 0..recipe.fragment_count {
        let id = first_fragment_id
            .0
            .checked_add(fragment_index)
            .map(EntityId)
            .ok_or(ImpactError3d::FragmentIdOverflow)?;
        let offset = FRAGMENT_OFFSETS[fragment_index as usize];
        let fragment_position = Position::new3(
            position
                .x
                .checked_add(i64::from(offset[0]))
                .ok_or(ImpactError3d::ArithmeticOverflow)?,
            position
                .y
                .checked_add(i64::from(offset[1]))
                .ok_or(ImpactError3d::ArithmeticOverflow)?,
            position
                .z
                .checked_add(i64::from(offset[2]))
                .ok_or(ImpactError3d::ArithmeticOverflow)?,
        );
        let fragment_velocity = Velocity::new3(
            velocity
                .x
                .saturating_add(offset[0].saturating_mul(recipe.separation_speed)),
            velocity
                .y
                .saturating_add(offset[1].saturating_mul(recipe.separation_speed)),
            velocity
                .z
                .saturating_add(offset[2].saturating_mul(recipe.separation_speed)),
        );
        operations.push(Operation::Spawn(id));
        operations.push(Operation::SetPosition(id, fragment_position));
        operations.push(Operation::SetVelocity(id, fragment_velocity));
    }
    Ok(operations)
}

fn velocity_delta(left: Velocity, right: Velocity) -> [i64; 3] {
    [
        i64::from(left.x) - i64::from(right.x),
        i64::from(left.y) - i64::from(right.y),
        i64::from(left.z) - i64::from(right.z),
    ]
}

fn normal_response_momentum(
    body: PhysicsBody3d,
    response: Option<[i64; 3]>,
    normal: ContactNormal3d,
) -> Result<u64, ImpactError3d> {
    if body.kind == BodyKind::Fixed {
        return Ok(0);
    }
    let response = response.ok_or(ImpactError3d::MissingVelocity(body.entity))?;
    let normal_delta = i128::from(response[0])
        .checked_mul(i128::from(normal.x))
        .and_then(|value| value.checked_add(i128::from(response[1]) * i128::from(normal.y)))
        .and_then(|value| value.checked_add(i128::from(response[2]) * i128::from(normal.z)))
        .ok_or(ImpactError3d::ArithmeticOverflow)?
        .abs();
    let momentum = normal_delta
        .checked_mul(i128::from(body.mass_units))
        .ok_or(ImpactError3d::ArithmeticOverflow)?;
    u64::try_from(momentum).map_err(|_| ImpactError3d::ArithmeticOverflow)
}

fn validate_recipe(recipe: DestructionRecipe3d) -> Result<(), ImpactError3d> {
    if recipe.fragment_count == 0 || recipe.fragment_count > MAX_FRAGMENT_COUNT {
        return Err(ImpactError3d::InvalidFragmentCount(recipe.fragment_count));
    }
    if recipe.separation_speed < 0 {
        return Err(ImpactError3d::NegativeSeparationSpeed(
            recipe.separation_speed,
        ));
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use ecs_physics::PhysicsMaterial;
    use ecs_workload::{EntitySnapshot, Position};

    use crate::{PhysicsContact3d, PhysicsStep3dStats};

    use super::*;

    const BODY: EntityId = EntityId(1);
    const WALL: EntityId = EntityId(2);

    fn snapshot() -> WorldSnapshot {
        WorldSnapshot::new(vec![
            EntitySnapshot {
                id: BODY,
                position: Some(Position::new3(0, 2, 0)),
                velocity: Some(Velocity::new3(0, -5, 0)),
            },
            EntitySnapshot {
                id: WALL,
                position: Some(Position::new3(0, 0, 0)),
                velocity: None,
            },
        ])
    }

    fn bodies() -> Vec<PhysicsBody3d> {
        vec![
            PhysicsBody3d::dynamic(BODY, [1, 1, 1])
                .with_mass(2)
                .with_material(PhysicsMaterial::new(1_000, 0)),
            PhysicsBody3d::fixed(WALL, [4, 1, 4]),
        ]
    }

    fn bounced_step() -> PhysicsStep3d {
        PhysicsStep3d {
            operations: vec![Operation::SetVelocity(BODY, Velocity::new3(0, 6, 0))],
            stats: PhysicsStep3dStats::default(),
            contacts: vec![PhysicsContact3d {
                left: BODY,
                right: WALL,
                normal: ContactNormal3d { x: 0, y: -1, z: 0 },
                penetration: 0,
            }],
            supporting_entities: vec![BODY],
        }
    }

    #[test]
    fn impact_measure_removes_free_fall_gravity() {
        let evidence = impact_evidence(
            &snapshot(),
            &bodies(),
            &bounced_step(),
            PhysicsConfig3d {
                gravity: Velocity::new3(0, -1, 0),
            },
            1,
        )
        .expect("impact evidence should be valid");
        assert_eq!(evidence.len(), 1);
        assert_eq!(evidence[0].response_momentum_units, 24);
    }

    #[test]
    fn destruction_uses_stable_fragment_ids_and_order() {
        let evidence = vec![ImpactEvidence3d {
            left: BODY,
            right: WALL,
            normal: ContactNormal3d { x: 0, y: -1, z: 0 },
            response_momentum_units: 24,
        }];
        let operations = destruction_operations(
            &snapshot(),
            &evidence,
            BODY,
            EntityId(100),
            DestructionRecipe3d {
                threshold_momentum_units: 20,
                fragment_count: 2,
                separation_speed: 3,
            },
        )
        .expect("destruction should be valid");
        assert_eq!(operations[0], Operation::Despawn(BODY));
        assert_eq!(operations[1], Operation::Spawn(EntityId(100)));
        assert_eq!(operations[4], Operation::Spawn(EntityId(101)));
        assert_eq!(operations.len(), 7);
    }

    #[test]
    fn below_threshold_is_idempotently_unchanged() {
        let evidence = vec![ImpactEvidence3d {
            left: BODY,
            right: WALL,
            normal: ContactNormal3d { x: 0, y: -1, z: 0 },
            response_momentum_units: 4,
        }];
        let recipe = DestructionRecipe3d {
            threshold_momentum_units: 20,
            fragment_count: 4,
            separation_speed: 2,
        };
        assert!(
            destruction_operations(&snapshot(), &evidence, BODY, EntityId(100), recipe)
                .expect("valid recipe")
                .is_empty()
        );
    }
}
