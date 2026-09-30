//! Public adapter fixtures shared by native tests and the separate WASM contract driver.
use ecs_physics::{BodyKind, PhysicsMaterial};
use ecs_workload::{EntityId, Position, Velocity};

use crate::{
    AngularState3d, AngularSubstepPolicy3d, AngularVelocity3d, Orientation3d,
    PersistentPhysicsWorld3d, PhysicsBody3d, PhysicsEngineAdapterError3d,
    PhysicsEngineMotionAuthority3d, RigidBox3d, RigidBoxState3d, RigidBoxWorldConfig3d,
    RotatingContactSearchConfig3d, step_rigid_box_world_with_physics_engine,
};

fn config() -> RigidBoxWorldConfig3d {
    RigidBoxWorldConfig3d {
        gravity: Velocity::new3(0, 0, 0),
        timestep_numerator: 1,
        timestep_denominator: 60,
        angular_damping_milli: 1000,
        solver_passes: 6,
    }
}

fn search() -> RotatingContactSearchConfig3d {
    RotatingContactSearchConfig3d {
        coarse_samples: 3,
        refinement_steps: 0,
    }
}

fn body(entity: u32, x: i64, y: i64, velocity: i32) -> RigidBox3d {
    RigidBox3d::new(
        PhysicsBody3d::dynamic(EntityId(entity), [10, 10, 10]),
        RigidBoxState3d::new(
            Position::new3(x, y, 0),
            Velocity::new3(velocity, 0, 0),
            AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
        ),
    )
}

fn create(boxes: &[RigidBox3d]) -> PersistentPhysicsWorld3d {
    PersistentPhysicsWorld3d::new(boxes, config(), search(), AngularSubstepPolicy3d::default())
        .unwrap()
}

#[cfg_attr(test, test)]
pub fn stationary_and_sparse_frames_match_rebuild_without_reconstruction() {
    for moving in [false, true] {
        let mut reference = (0..128)
            .rev()
            .map(|id| body(id, i64::from(id) * 4000, 0, 0))
            .collect::<Vec<_>>();
        if moving {
            reference.last_mut().unwrap().state.linear_velocity.x = -600;
        }
        let mut persistent = create(&reference);
        let initial = persistent.work();
        for _ in 0..64 {
            let expected = step_rigid_box_world_with_physics_engine(
                &reference,
                config(),
                search(),
                AngularSubstepPolicy3d::default(),
            )
            .unwrap();
            let actual = persistent.advance().unwrap();
            assert_eq!(actual, expected);
            reference = expected.boxes;
        }
        let work = persistent.work();
        assert_eq!(work.world_constructions, 1);
        assert_eq!(work.insertions, 128);
        assert_eq!(work.input_conversions, initial.input_conversions);
        assert_eq!(work.angular_scan_visits, 128 * 64);
        assert_eq!(work.output_conversions, 128 * 64);
        assert_eq!(work.descriptor_commands, 0);
        assert_eq!(work.motion_commands, 0);
        assert_eq!(persistent.retained().entity_mappings, 128);
        assert_eq!(persistent.retained().body_metadata_entries, 128);
    }
}

#[cfg_attr(test, test)]
pub fn damping_is_authoritative_once_per_frame_including_zero_time() {
    let mut spinning = body(1, 0, 0, 600);
    spinning.state.angular.angular_velocity = AngularVelocity3d::new(0, 0, 12_000_000);
    let mut cfg = config();
    cfg.angular_damping_milli = 800;
    let policy = AngularSubstepPolicy3d {
        max_angular_step_units: 50_000,
        max_substeps: 8,
    };
    let mut persistent = PersistentPhysicsWorld3d::new(&[spinning], cfg, search(), policy).unwrap();
    let first = persistent.advance().unwrap();
    assert_eq!(
        first,
        step_rigid_box_world_with_physics_engine(&[spinning], cfg, search(), policy).unwrap()
    );
    assert_eq!(first.substeps, 4);
    assert_eq!(first.boxes[0].state.angular.angular_velocity.z, 9_600_000);
    assert_eq!(
        persistent.advance().unwrap().boxes[0]
            .state
            .angular
            .angular_velocity
            .z,
        7_680_000
    );
    cfg.timestep_numerator = 0;
    cfg.angular_damping_milli = 500;
    spinning.state.angular.angular_velocity = AngularVelocity3d::new(5, -5, 0);
    let mut zero = PersistentPhysicsWorld3d::new(&[spinning], cfg, search(), policy).unwrap();
    let first = zero.advance().unwrap();
    assert_eq!(first.boxes[0].state.center, spinning.state.center);
    assert_eq!(
        first.boxes[0].state.angular.angular_velocity,
        AngularVelocity3d::new(3, -3, 0)
    );
    assert_eq!(
        zero.advance().unwrap().boxes[0]
            .state
            .angular
            .angular_velocity,
        AngularVelocity3d::new(2, -2, 0)
    );
}

#[cfg_attr(test, test)]
pub fn output_views_do_not_overwrite_physics_and_metadata_edits_preserve_motion() {
    let original = body(1, 0, 0, 600);
    let mut persistent = create(&[original]);
    let mut view = persistent.advance().unwrap().boxes;
    view[0].state.center.x = 1_000_000;
    view[0].state.linear_velocity.x = -1;
    view[0].body.mass_units = 900;
    let next = persistent.advance().unwrap();
    assert_eq!(next.boxes[0].state.center.x, 20);
    assert_eq!(next.boxes[0].state.linear_velocity.x, 600);
    assert_eq!(next.boxes[0].body.mass_units, 1);
    let changed = original
        .body
        .with_mass(7)
        .with_material(PhysicsMaterial::new(300, 700));
    assert!(persistent.update_metadata(EntityId(1), changed).unwrap());
    let updated = persistent.snapshot()[0];
    assert_eq!(updated.state, next.boxes[0].state);
    assert_eq!(updated.body, changed);
    assert!(!persistent.update_metadata(EntityId(1), changed).unwrap());
    assert_eq!(persistent.work().changed_descriptors, 1);
    assert_eq!(persistent.work().motion_commands, 0);
}

#[cfg_attr(test, test)]
pub fn remapping_and_reuse_keep_stable_engine_identity_and_canonical_output() {
    let mut persistent = create(&[body(4, 0, 0, 0), body(9, 1000, 0, 0)]);
    let id = persistent.body_id(EntityId(4)).unwrap();
    assert!(persistent.remap(EntityId(4), EntityId(20)).unwrap());
    assert_eq!(persistent.body_id(EntityId(20)), Some(id));
    assert_eq!(
        persistent.remap(EntityId(20), EntityId(9)),
        Err(PhysicsEngineAdapterError3d::DuplicateEntity(EntityId(9)))
    );
    assert_eq!(
        persistent.insert(body(9, 2000, 0, 0)),
        Err(PhysicsEngineAdapterError3d::DuplicateEntity(EntityId(9)))
    );
    persistent.insert(body(4, 3000, 0, 0)).unwrap();
    assert_ne!(persistent.body_id(EntityId(4)), Some(id));
    let first_reused = persistent.body_id(EntityId(4)).unwrap();
    persistent.remove(EntityId(4)).unwrap();
    persistent.insert(body(4, 4000, 0, 0)).unwrap();
    assert_ne!(persistent.body_id(EntityId(4)), Some(first_reused));
    let rows = persistent.advance().unwrap().boxes;
    assert_eq!(
        rows.iter()
            .map(|body| body.body.entity.0)
            .collect::<Vec<_>>(),
        vec![4, 9, 20]
    );
    assert_eq!(rows[2].state.center.x, 0);
    assert_eq!(persistent.work().world_constructions, 1);
}

#[cfg_attr(test, test)]
pub fn late_frame_failure_restores_physics_and_keeps_intended_commands() {
    let mut rotating = body(1, 0, 0, 0);
    rotating.state.angular.angular_velocity.z = 12_000_000;
    let failing = body(2, i64::from(i32::MAX) - 140, 1000, 8400);
    let mut cfg = config();
    cfg.angular_damping_milli = 800;
    let policy = AngularSubstepPolicy3d {
        max_angular_step_units: 50_000,
        max_substeps: 8,
    };
    let mut persistent =
        PersistentPhysicsWorld3d::new(&[rotating, failing], cfg, search(), policy).unwrap();
    let before = persistent.snapshot();
    let identity = persistent.body_id(EntityId(1));
    assert!(persistent.advance().is_err());
    assert_eq!(persistent.snapshot(), before);
    assert_eq!(persistent.body_id(EntityId(1)), identity);
    assert_eq!(persistent.work().completed_frames, 0);
    assert_eq!(persistent.work().completed_substeps, 3);
    persistent
        .set_motion(
            EntityId(2),
            Velocity::new3(0, 0, 0),
            AngularVelocity3d::default(),
        )
        .unwrap();
    let mut repaired = before;
    repaired[1].state.linear_velocity = Velocity::new3(0, 0, 0);
    let expected =
        step_rigid_box_world_with_physics_engine(&repaired, cfg, search(), policy).unwrap();
    assert_eq!(persistent.advance().unwrap(), expected);
    assert_eq!(persistent.work().world_constructions, 1);
}

#[cfg_attr(test, test)]
pub fn reset_and_independent_worlds_have_explicit_lifetimes() {
    let original = body(1, 0, 0, 600);
    let mut first = create(&[original]);
    let mut second = create(&[original]);
    first.advance().unwrap();
    let before = first.snapshot();
    assert_eq!(
        first.reset(&[original, original]),
        Err(PhysicsEngineAdapterError3d::DuplicateEntity(EntityId(1)))
    );
    assert_eq!(first.snapshot(), before);
    first.reset(&[original]).unwrap();
    assert_eq!(first.work().world_constructions, 2);
    assert_eq!(first.work().completed_frames, 1);
    assert_eq!(first.advance().unwrap(), second.advance().unwrap());
    drop(first);
    assert_eq!(second.advance().unwrap().boxes[0].state.center.x, 20);
}

#[cfg_attr(test, test)]
pub fn real_motion_teleports_and_authority_changes_use_only_intended_commands() {
    let mut cfg = config();
    cfg.gravity.y = -600;
    let mut persistent = PersistentPhysicsWorld3d::new(
        &[body(1, 0, 1000, 0)],
        cfg,
        search(),
        AngularSubstepPolicy3d::default(),
    )
    .unwrap();
    persistent
        .set_authority(EntityId(1), PhysicsEngineMotionAuthority3d::External)
        .unwrap();
    persistent
        .set_motion(
            EntityId(1),
            Velocity::new3(600, 0, 0),
            AngularVelocity3d::new(100_000, 0, 0),
        )
        .unwrap();
    let first = persistent.advance().unwrap().boxes[0];
    assert_eq!(first.state.linear_velocity, Velocity::new3(600, 0, 0));
    assert_eq!(first.state.angular.angular_velocity.x, 100_000);
    persistent
        .teleport(
            EntityId(1),
            Position::new3(100, 1000, 0),
            first.state.angular.orientation,
        )
        .unwrap();
    let teleported = persistent.snapshot()[0];
    assert_eq!(teleported.state.angular, first.state.angular);
    assert_eq!(teleported.state.center.x, 100);
    assert!(
        !persistent
            .teleport(
                EntityId(1),
                teleported.state.center,
                teleported.state.angular.orientation
            )
            .unwrap()
    );
    persistent
        .set_authority(EntityId(1), PhysicsEngineMotionAuthority3d::Physics)
        .unwrap();
    assert_eq!(
        persistent.advance().unwrap().boxes[0]
            .state
            .linear_velocity
            .y,
        -10
    );
    let before = persistent.snapshot();
    assert!(
        persistent
            .teleport(
                EntityId(1),
                Position::new3(i64::MAX, 0, 0),
                Orientation3d::IDENTITY
            )
            .is_err()
    );
    assert!(
        persistent
            .teleport(
                EntityId(1),
                Position::new3(0, 0, 0),
                Orientation3d::new(0, 0, 0, 0)
            )
            .is_err()
    );
    assert_eq!(persistent.snapshot(), before);
}

#[cfg_attr(test, test)]
pub fn near_misses_preserve_parked_bodies_and_real_contacts_restore_dynamic_mass() {
    let mut persistent = create(&[body(1, 0, 0, 0)]);
    for _ in 0..16 {
        persistent.advance().unwrap();
    }
    assert_eq!(persistent.retained().sleeping_bodies, 1);
    persistent.insert(body(2, -100, 21, 600)).unwrap();
    for _ in 0..20 {
        persistent.advance().unwrap();
    }
    assert_eq!(persistent.retained().sleeping_bodies, 1);
    assert_eq!(persistent.snapshot()[0].state.center.x, 0);
    persistent.insert(body(3, -100, 0, 600)).unwrap();
    for _ in 0..20 {
        persistent.advance().unwrap();
    }
    let target = persistent.snapshot()[0];
    assert_eq!(target.body.kind, BodyKind::Dynamic);
    assert_eq!(target.body.mass_units, 1);
    assert!(target.state.center.x > 0);
    assert!(target.state.linear_velocity.x > 0);
    assert_eq!(persistent.work().world_constructions, 1);
}

#[cfg_attr(test, test)]
pub fn support_removal_wakes_the_dependent_without_connecting_fixed_floor_islands() {
    let mut floor = body(0, 0, -3600, 0);
    floor.body = PhysicsBody3d::fixed(EntityId(0), [200_000, 3600, 50_000])
        .with_material(PhysicsMaterial::new(0, 1000));
    let mut lower = body(1, 0, 3600, 0);
    lower.body.half_extents = [3600; 3];
    lower.body.material = PhysicsMaterial::new(0, 1000);
    let mut upper = lower;
    upper.body.entity = EntityId(2);
    upper.state.center.y = 10_800;
    let mut independent = lower;
    independent.body.entity = EntityId(3);
    independent.state.center.x = 100_000;
    let mut cfg = config();
    cfg.gravity.y = -36_000;
    let mut persistent = PersistentPhysicsWorld3d::new(
        &[floor, lower, upper, independent],
        cfg,
        search(),
        AngularSubstepPolicy3d::default(),
    )
    .unwrap();
    for _ in 0..180 {
        persistent.advance().unwrap();
        if persistent.retained().sleeping_bodies == 3 {
            break;
        }
    }
    assert_eq!(persistent.retained().sleeping_bodies, 3);
    let before = persistent.snapshot();
    persistent.remove(EntityId(1)).unwrap();
    assert_eq!(persistent.retained().sleeping_bodies, 1);
    let after = persistent.advance().unwrap().boxes;
    assert!(after[1].state.center.y < before[2].state.center.y);
    assert_eq!(after[2].state, before[3].state);
    assert_eq!(after[2].body.kind, BodyKind::Dynamic);
}

#[cfg_attr(test, test)]
pub fn contact_frames_match_rebuild_before_sleep_history_diverges() {
    let mut reference = vec![body(1, -40, 0, 1200), body(2, 0, 0, 0)];
    let mut world = create(&reference);
    let mut events = 0;
    for _ in 0..8 {
        let expected = step_rigid_box_world_with_physics_engine(
            &reference,
            config(),
            search(),
            AngularSubstepPolicy3d::default(),
        )
        .unwrap();
        let actual = world.advance().unwrap();
        assert_eq!(actual, expected);
        events += actual.sampled_events + actual.tail_contacts;
        reference = expected.boxes;
    }
    assert!(events > 0);
}

#[cfg_attr(test, test)]
pub fn invalid_edits_preserve_physics_metadata_and_parked_state() {
    let mut world = create(&[body(1, 0, 0, 0)]);
    for _ in 0..16 {
        world.advance().unwrap();
    }
    assert_eq!(world.retained().sleeping_bodies, 1);
    let before = world.snapshot();
    let id = world.body_id(EntityId(1));
    let mut invalid = before[0].body;
    invalid.half_extents[0] = 0;
    assert!(world.update_metadata(EntityId(1), invalid).is_err());
    invalid = before[0].body;
    invalid.material.restitution_milli = 1001;
    assert!(world.update_metadata(EntityId(1), invalid).is_err());
    assert!(world.insert(before[0]).is_err());
    assert!(world.remap(EntityId(9), EntityId(2)).is_err());
    assert!(
        world
            .set_motion(
                EntityId(9),
                Velocity::new3(1, 0, 0),
                AngularVelocity3d::default()
            )
            .is_err()
    );
    assert_eq!(world.snapshot(), before);
    assert_eq!(world.body_id(EntityId(1)), id);
    assert_eq!(world.retained().sleeping_bodies, 1);
    let mut replacement = before[0].body;
    replacement.half_extents[0] = 12;
    assert!(world.update_metadata(EntityId(1), replacement).unwrap());
    let after = world.snapshot();
    assert_eq!(after[0].body.half_extents[0], 12);
    assert_eq!(after[0].state, before[0].state);
}
