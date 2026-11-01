//! Rapier-backed rigid-box physics for ECS Lab.
//!
//! Rapier (`rapier3d`, 3D f32, default features) is the rigid-body authority. The ECS keeps its
//! integer components: every frame quantizes Rapier's state into [`RigidBox3d`] views and integer
//! [`ecs_workload::Operation`]s. Rapier with default features replays bit-identically only on the
//! same machine and binary, so scenarios assert tolerance-based physical-quality bounds plus
//! same-machine replay, not cross-platform integer parity.
#![allow(
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
    reason = "this crate is the f32 Rapier <-> integer ECS quantization seam; casts round or saturate by design"
)]

mod body;
mod geometry;
mod scenario;
mod world;

pub use body::{
    ANGULAR_VELOCITY_SCALE, AngularState3d, AngularVelocity3d, ORIENTATION_SCALE, Orientation3d,
    PhysicsBody3d, RigidBox3d, RigidBoxState3d,
};
pub use geometry::oriented_box_vertices;
pub use scenario::BouncingRoom3dScenario;
pub use world::{PairContact3d, PhysicsError3d, RapierFrame3d, RapierWorld3d, RapierWorldConfig3d};
