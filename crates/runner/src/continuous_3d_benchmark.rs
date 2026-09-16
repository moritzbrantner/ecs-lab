use std::hint::black_box;

use ecs_physics::PhysicsMaterial;
use ecs_physics_3d::{PhysicsBody3d, PhysicsConfig3d, step_3d};
use ecs_workload::{EntityId, EntitySnapshot, Position, Velocity, WorldSnapshot};

use super::{benchmark, must};

const CONTINUOUS_3D_SEED: u32 = 0x3DCC_D001;
const LANE_SPACING: i64 = 4;
const WALL_X: i64 = 12;
const BOUNCE_SPEED: i32 = 100;
const NO_GRAVITY_3D: PhysicsConfig3d = PhysicsConfig3d {
    gravity: Velocity::new3(0, 0, 0),
};

pub(super) fn run(smoke: bool, fingerprint: &str) {
    let (dynamic_count, repetitions) = if smoke { (24, 3) } else { (96, 5) };
    let (snapshot, bodies) = fixture(dynamic_count);

    let preflight = must(
        step_3d(&snapshot, &bodies, NO_GRAVITY_3D, 1),
        "continuous 3D benchmark preflight must succeed",
    );
    let replay = must(
        step_3d(&snapshot, &bodies, NO_GRAVITY_3D, 1),
        "continuous 3D benchmark replay must succeed",
    );
    assert_eq!(
        preflight, replay,
        "continuous 3D benchmark must replay exactly before timing"
    );
    assert!(
        preflight.stats().ccd_contacts > dynamic_count as usize,
        "continuous 3D benchmark must traverse more than one global CCD event"
    );
    assert!(
        preflight.stats().ccd_candidate_pairs >= preflight.stats().ccd_contacts,
        "every continuous contact must originate from a broad-phase candidate"
    );

    benchmark(
        "continuous-3d-bouncing-lanes",
        "rust-continuous-step",
        dynamic_count.saturating_add(2),
        1,
        CONTINUOUS_3D_SEED,
        repetitions,
        fingerprint,
        || {
            must(
                step_3d(
                    black_box(&snapshot),
                    black_box(&bodies),
                    NO_GRAVITY_3D,
                    1,
                ),
                "validated continuous 3D benchmark step must succeed",
            )
        },
    );
}

fn fixture(dynamic_count: u32) -> (WorldSnapshot, Vec<PhysicsBody3d>) {
    let mut entities = Vec::with_capacity(dynamic_count.saturating_add(2) as usize);
    let mut bodies = Vec::with_capacity(dynamic_count.saturating_add(2) as usize);
    let bouncy = PhysicsMaterial::new(1_000, 0);

    for raw_id in 0..dynamic_count {
        let entity = EntityId(raw_id);
        let y = i64::from(raw_id) * LANE_SPACING;
        entities.push(EntitySnapshot {
            id: entity,
            position: Some(Position::new3(0, y, 0)),
            velocity: Some(Velocity::new3(BOUNCE_SPEED, 0, 0)),
        });
        bodies.push(PhysicsBody3d::dynamic(entity, [1, 1, 1]).with_material(bouncy));
    }

    let lane_max = i64::from(dynamic_count.saturating_sub(1)) * LANE_SPACING;
    let wall_center_y = lane_max / 2;
    let wall_half_y = must(
        i32::try_from(wall_center_y + 2),
        "continuous 3D benchmark wall extent must fit i32",
    );
    for (offset, x) in [(-WALL_X), WALL_X].into_iter().enumerate() {
        let raw_id = dynamic_count.saturating_add(offset as u32);
        let entity = EntityId(raw_id);
        entities.push(EntitySnapshot {
            id: entity,
            position: Some(Position::new3(x, wall_center_y, 0)),
            velocity: None,
        });
        bodies.push(PhysicsBody3d::fixed(entity, [1, wall_half_y, 2]));
    }

    (WorldSnapshot::new(entities), bodies)
}
