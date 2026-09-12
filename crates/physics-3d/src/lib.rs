mod angular;
mod angular_substep;
mod box_plane;
mod colliders;
mod continuous_solver;
mod controller;
mod impact;
mod interactions;
mod liquid;
mod oriented_box;
mod physics_engine_adapter;
mod rigid_box;
mod scenario;
#[allow(dead_code)]
mod solver;
mod sphere_obb;
mod sphere_obb_response;
mod swept_broad_phase;
mod types;

pub use angular::{
    ANGULAR_VELOCITY_SCALE, AngularError3d, AngularState3d, AngularVelocity3d, BoxInertia3d,
    ORIENTATION_SCALE, Orientation3d, box_inertia, contact_angular_impulse, integrate_orientation,
};
pub use angular_substep::{
    AngularSubstepError3d, AngularSubstepPolicy3d, DEFAULT_MAX_ANGULAR_STEP_UNITS,
    DEFAULT_MAX_ANGULAR_SUBSTEPS, MAX_ANGULAR_SUBSTEPS, required_angular_substeps,
};
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
pub use physics_engine_adapter::{
    PhysicsEngineAdapterError3d, PhysicsEngineAdapterStep3d,
    step_rigid_box_world_with_physics_engine,
};
pub use rigid_box::{
    RigidBox3d, RigidBoxState3d, RigidBoxWorldConfig3d, RotatingContactSearchConfig3d,
};
pub use scenario::{BouncingRoom3dScenario, BroadPhaseBody3d, BroadPhaseFrame3d, ScenarioError3d};
pub use sphere_obb::{Sphere3d, SphereObbContact3d, SphereObbError3d, sphere_obb_contact};
pub use sphere_obb_response::{
    RigidSphereState3d, SphereBody3d, SphereObbResponseContact3d, SphereObbResponseError3d,
    SphereObbStep3d, resolve_sphere_obb_contact, stabilize_sphere_obb_contact,
};
pub use types::{
    ContactNormal3d, PhysicsBody3d, PhysicsConfig3d, PhysicsContact3d, PhysicsError3d,
    PhysicsStep3d, PhysicsStep3dStats,
};
