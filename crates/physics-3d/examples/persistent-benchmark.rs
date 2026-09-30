//! Complete adapter-call timing. Run each route in a separate process for RSS evidence.
use std::{hint::black_box, time::Instant};

use ecs_physics_3d::{
    AngularState3d, AngularSubstepPolicy3d, AngularVelocity3d, Orientation3d,
    PersistentPhysicsWorld3d, PhysicsBody3d, RigidBox3d, RigidBoxState3d, RigidBoxWorldConfig3d,
    RotatingContactSearchConfig3d, step_rigid_box_world_with_physics_engine,
};
use ecs_workload::{EntityId, Position, Velocity};

const WARM_FRAMES: usize = 40;
const MEASURED_FRAMES: usize = 120;

fn main() {
    let args = std::env::args().collect::<Vec<_>>();
    let mode = args.get(1).expect("route: compare, persistent or rebuild");
    let population: u32 = args.get(2).expect("population").parse().unwrap();
    let sparse = args.get(3).is_some_and(|value| value == "sparse");
    let mut boxes = (0..population)
        .map(|id| {
            RigidBox3d::new(
                PhysicsBody3d::dynamic(EntityId(id), [10, 10, 10]),
                RigidBoxState3d::new(
                    Position::new3(i64::from(id) * 4000, 0, 0),
                    Velocity::new3(if sparse && id == 0 { -600 } else { 0 }, 0, 0),
                    AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
                ),
            )
        })
        .collect::<Vec<_>>();
    let config = RigidBoxWorldConfig3d {
        gravity: Velocity::new3(0, 0, 0),
        timestep_numerator: 1,
        timestep_denominator: 60,
        angular_damping_milli: 1000,
        solver_passes: 6,
    };
    let search = RotatingContactSearchConfig3d {
        coarse_samples: 3,
        refinement_steps: 0,
    };
    let policy = AngularSubstepPolicy3d::default();
    let rebuild = |boxes: &[RigidBox3d]| {
        step_rigid_box_world_with_physics_engine(boxes, config, search, policy).unwrap()
    };
    if mode == "compare" {
        let mut world = PersistentPhysicsWorld3d::new(&boxes, config, search, policy).unwrap();
        for _ in 0..WARM_FRAMES + MEASURED_FRAMES {
            let expected = rebuild(&boxes);
            assert_eq!(world.advance().unwrap(), expected);
            boxes = expected.boxes;
        }
        println!(
            "PARITY {population} {sparse} {}",
            WARM_FRAMES + MEASURED_FRAMES
        );
        return;
    }
    assert!(mode == "persistent" || mode == "rebuild");
    let construction_start = Instant::now();
    let mut world = (mode == "persistent")
        .then(|| PersistentPhysicsWorld3d::new(&boxes, config, search, policy).unwrap());
    let construction_ns = construction_start.elapsed().as_nanos();
    for _ in 0..WARM_FRAMES {
        boxes = match &mut world {
            Some(world) => world.advance().unwrap().boxes,
            None => rebuild(&boxes).boxes,
        };
    }
    if let Some(world) = &world {
        println!("WORK_BEFORE {:?}", world.work());
    }
    let start = Instant::now();
    for _ in 0..MEASURED_FRAMES {
        let next = match &mut world {
            Some(world) => world.advance().unwrap(),
            None => rebuild(&boxes),
        };
        black_box(&next);
        boxes = next.boxes;
    }
    let elapsed_ns = start.elapsed().as_nanos();
    black_box(&boxes);
    println!(
        "BENCHMARK {mode} {population} {sparse} {MEASURED_FRAMES} {elapsed_ns} {construction_ns}"
    );
    if let Some(world) = &world {
        println!("WORK_AFTER {:?}", world.work());
        println!("RETAINED {:?}", world.retained());
    }
}
