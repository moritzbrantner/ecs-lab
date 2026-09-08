mod angular;
mod angular_substep;
mod box_box;
mod box_box_friction;
mod box_box_stabilization;
mod box_plane;
mod colliders;
mod continuous_solver;
mod controller;
mod impact;
mod interactions;
mod liquid;
mod oriented_box;
mod repeated_rotating_frame;
mod rigid_box_free_flight;
mod rigid_box_world;
mod rotating_contact_frontier;
mod rotating_contact_response;
mod rotating_contact_search;
mod rotating_new_contact_search;
mod rotational_sweep;
mod sampled_rotating_frame;
mod scenario;
#[allow(dead_code)]
mod solver;
mod sphere_obb;
mod swept_broad_phase;
mod types;

pub use angular::{
    ANGULAR_VELOCITY_SCALE, AngularError3d, AngularState3d, AngularVelocity3d, BoxInertia3d,
    ORIENTATION_SCALE, Orientation3d, box_inertia, contact_angular_impulse, integrate_orientation,
};
pub use angular_substep::{
    AngularSubstepError3d, AngularSubstepPolicy3d, DEFAULT_MAX_ANGULAR_STEP_UNITS,
    DEFAULT_MAX_ANGULAR_SUBSTEPS, MAX_ANGULAR_SUBSTEPS, required_angular_substeps,
    step_rigid_box_world_substepped,
};
pub use box_box::{
    BoxBoxContact3d, BoxBoxError3d, BoxBoxStep3d, RigidBoxState3d, resolve_box_box_contact,
};
pub use box_box_friction::stabilize_box_box_contact;
pub use box_box_stabilization::BoxBoxStabilizationError3d;
pub use box_plane::{
    BoxPlaneConfig3d, BoxPlaneContact3d, BoxPlaneError3d, BoxPlaneState3d, BoxPlaneStep3d,
    oriented_box_vertices, step_box_on_plane,
};
pub use colliders::{
    Collider3d, ColliderContact3d, ColliderError3d, ColliderShape3d, collider_contact,
};
pub use continuous_solver::step_3d;
pub use controller::{
    ControllerConfig3d, ControllerError3d, ControllerInput3d, controller_operations,
};
pub use impact::{
    DestructionRecipe3d, ImpactError3d, ImpactEvidence3d, destruction_operations, impact_evidence,
};
pub use interactions::{
    ColliderRole3d, CollisionFilter3d, InteractiveCollider3d, PairInteraction3d, SensorEvent3d,
    pair_interaction, sensor_events,
};
pub use liquid::{LiquidError3d, LiquidVolume3d, liquid_operations};
pub use oriented_box::{
    ObbAxisFeature3d, ObbContactSeed3d, OrientedBox3d, OrientedBoxError3d, obb_contact_seed,
};
pub use repeated_rotating_frame::{
    MAX_REPEATED_ROTATING_EVENTS, MAX_REPEATED_ROTATING_TAIL_SEGMENTS,
    RepeatedRotatingFrameError3d, RepeatedRotatingFrameStep3d,
    step_rigid_box_world_repeated_rotating,
};
pub use rigid_box_free_flight::{
    RigidBoxFreeFlightConfig3d, RigidBoxFreeFlightError3d, sample_rigid_box_free_flight,
    sample_rigid_box_world_free_flight,
};
pub use rigid_box_world::{
    RigidBox3d, RigidBoxWorldConfig3d, RigidBoxWorldError3d, RigidBoxWorldStats3d,
    RigidBoxWorldStep3d, step_rigid_box_world,
};
pub use rotating_contact_frontier::{
    RotatingContactFrontier3d, RotatingContactFrontierError3d,
    advance_to_earliest_rotating_contact_set,
};
pub use rotating_contact_response::{
    MAX_ROTATING_CONTACT_RESPONSE_PASSES, RotatingContactResponse3d,
    RotatingContactResponseError3d, resolve_rotating_contact_frontier,
};
pub use rotating_contact_search::{
    MAX_ROTATING_CONTACT_REFINEMENTS, MAX_ROTATING_CONTACT_SAMPLES, RotatingContactBracket3d,
    RotatingContactSearchConfig3d, RotatingContactSearchError3d, RotatingContactSet3d,
    bracket_rotating_contact, earliest_rotating_contact_set, search_rotating_contacts,
};
pub use rotating_new_contact_search::{
    bracket_new_rotating_contact, earliest_new_rotating_contact_set, search_new_rotating_contacts,
};
pub use rotational_sweep::{
    RotationalSweepBounds3d, RotationalSweepError3d, RotationalSweepPair3d,
    rotational_sweep_bounds, rotational_sweep_candidate_pairs,
};
pub use sampled_rotating_frame::{
    MAX_SAMPLED_REMAINDER_SEGMENTS, SampledRotatingFrameError3d, SampledRotatingFrameStep3d,
    step_rigid_box_world_sampled_rotating,
};
pub use scenario::{BouncingRoom3dScenario, BroadPhaseBody3d, BroadPhaseFrame3d, ScenarioError3d};
pub use sphere_obb::{Sphere3d, SphereObbContact3d, SphereObbError3d, sphere_obb_contact};
pub use types::{
    ContactNormal3d, PhysicsBody3d, PhysicsConfig3d, PhysicsContact3d, PhysicsError3d,
    PhysicsStep3d, PhysicsStep3dStats,
};
