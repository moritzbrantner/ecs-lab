use std::hint::black_box;

use ecs_physics::{PhysicsBody, PhysicsConfig, PhysicsMaterial, step};
use ecs_workload::{EntityId, EntitySnapshot, Position, Velocity, WorldSnapshot};

use super::{benchmark, must};

const MATERIAL_FIXTURE_SEED: u32 = 0x0BAD_5EED;
const MATERIAL_COLUMNS: u32 = 16;
const NO_GRAVITY: PhysicsConfig = PhysicsConfig {
    gravity: Velocity::new(0, 0),
};

pub(super) fn run_material_step_benchmarks(smoke: bool, fingerprint: &str) {
    let (sparse_count, dense_count, repetitions) = if smoke { (96, 64, 3) } else { (512, 192, 5) };
    let (sparse_snapshot, sparse_bodies) = material_step_fixture(sparse_count, 8);
    let (dense_snapshot, dense_bodies) = material_step_fixture(dense_count, 1);

    let sparse_preflight = must(
        step(&sparse_snapshot, &sparse_bodies, NO_GRAVITY, 1),
        "sparse material benchmark preflight must succeed",
    );
    assert_eq!(
        sparse_preflight.stats().contacts,
        0,
        "sparse material benchmark must isolate candidate-pair traversal"
    );
    let dense_preflight = must(
        step(&dense_snapshot, &dense_bodies, NO_GRAVITY, 1),
        "dense material benchmark preflight must succeed",
    );
    assert!(
        dense_preflight.stats().contacts > 0,
        "dense material benchmark must exercise contact/material response"
    );

    benchmark(
        "material-step-sparse",
        "rust-step",
        sparse_count,
        1,
        MATERIAL_FIXTURE_SEED,
        repetitions,
        fingerprint,
        || {
            must(
                step(
                    black_box(&sparse_snapshot),
                    black_box(&sparse_bodies),
                    NO_GRAVITY,
                    1,
                ),
                "validated sparse material benchmark step must succeed",
            )
        },
    );
    benchmark(
        "material-step-dense",
        "rust-step",
        dense_count,
        1,
        MATERIAL_FIXTURE_SEED,
        repetitions,
        fingerprint,
        || {
            must(
                step(
                    black_box(&dense_snapshot),
                    black_box(&dense_bodies),
                    NO_GRAVITY,
                    1,
                ),
                "validated dense material benchmark step must succeed",
            )
        },
    );
}

fn material_step_fixture(body_count: u32, spacing: i64) -> (WorldSnapshot, Vec<PhysicsBody>) {
    let mut entities = Vec::new();
    let mut bodies = Vec::new();

    for raw_id in 0..body_count {
        let entity = EntityId(raw_id);
        let column = i64::from(raw_id % MATERIAL_COLUMNS);
        let row = i64::from(raw_id / MATERIAL_COLUMNS);
        let velocity_x = must(
            i32::try_from(raw_id % 5),
            "benchmark velocity residue must fit i32",
        ) - 2;
        let velocity_y = must(
            i32::try_from((raw_id / 5) % 5),
            "benchmark velocity residue must fit i32",
        ) - 2;
        let restitution = must(
            u16::try_from((raw_id % 5) * 250),
            "benchmark restitution must fit u16",
        );
        let friction = must(
            u16::try_from(((raw_id + 2) % 5) * 250),
            "benchmark friction must fit u16",
        );

        entities.push(EntitySnapshot {
            id: entity,
            position: Some(Position::new(column * spacing, row * spacing)),
            velocity: Some(Velocity::new(velocity_x, velocity_y)),
        });
        bodies.push(
            PhysicsBody::dynamic(entity, [1, 1])
                .with_mass(1 + raw_id % 7)
                .with_material(PhysicsMaterial::new(restitution, friction)),
        );
    }

    (WorldSnapshot::new(entities), bodies)
}
