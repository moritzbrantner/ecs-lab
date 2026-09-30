pub use ecs_physics_3d::{
    AngularState3d, AngularSubstepPolicy3d, AngularVelocity3d, Orientation3d,
    PersistentPhysicsWorld3d, PhysicsBody3d, PhysicsEngineAdapterError3d,
    PhysicsEngineMotionAuthority3d, RigidBox3d, RigidBoxState3d, RigidBoxWorldConfig3d,
    RotatingContactSearchConfig3d, step_rigid_box_world_with_physics_engine,
};

#[path = "../../physics-3d/src/persistent_physics_tests.rs"]
mod contract;

#[unsafe(no_mangle)]
pub extern "C" fn run_persistent_contract() -> u32 {
    contract::stationary_and_sparse_frames_match_rebuild_without_reconstruction();
    contract::damping_is_authoritative_once_per_frame_including_zero_time();
    contract::output_views_do_not_overwrite_physics_and_metadata_edits_preserve_motion();
    contract::remapping_and_reuse_keep_stable_engine_identity_and_canonical_output();
    contract::late_frame_failure_restores_physics_and_keeps_intended_commands();
    contract::reset_and_independent_worlds_have_explicit_lifetimes();
    contract::real_motion_teleports_and_authority_changes_use_only_intended_commands();
    contract::near_misses_preserve_parked_bodies_and_real_contacts_restore_dynamic_mass();
    contract::support_removal_wakes_the_dependent_without_connecting_fixed_floor_islands();
    contract::contact_frames_match_rebuild_before_sleep_history_diverges();
    contract::invalid_edits_preserve_physics_metadata_and_parked_state();
    11
}
