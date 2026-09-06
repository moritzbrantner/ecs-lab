use ecs_workload::EntityId;

use crate::{Collider3d, ColliderError3d, collider_contact};

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct CollisionFilter3d {
    pub memberships: u32,
    pub mask: u32,
}

impl CollisionFilter3d {
    #[must_use]
    pub const fn new(memberships: u32, mask: u32) -> Self {
        Self { memberships, mask }
    }

    #[must_use]
    pub const fn all() -> Self {
        Self {
            memberships: u32::MAX,
            mask: u32::MAX,
        }
    }

    #[must_use]
    pub const fn interacts_with(self, other: Self) -> bool {
        self.memberships & other.mask != 0 && other.memberships & self.mask != 0
    }
}

impl Default for CollisionFilter3d {
    fn default() -> Self {
        Self::all()
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ColliderRole3d {
    Solid,
    Sensor,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct InteractiveCollider3d {
    pub entity: EntityId,
    pub collider: Collider3d,
    pub filter: CollisionFilter3d,
    pub role: ColliderRole3d,
}

impl InteractiveCollider3d {
    #[must_use]
    pub const fn solid(entity: EntityId, collider: Collider3d) -> Self {
        Self {
            entity,
            collider,
            filter: CollisionFilter3d::all(),
            role: ColliderRole3d::Solid,
        }
    }

    #[must_use]
    pub const fn sensor(entity: EntityId, collider: Collider3d) -> Self {
        Self {
            entity,
            collider,
            filter: CollisionFilter3d::all(),
            role: ColliderRole3d::Sensor,
        }
    }

    #[must_use]
    pub const fn with_filter(mut self, filter: CollisionFilter3d) -> Self {
        self.filter = filter;
        self
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum PairInteraction3d {
    Ignore,
    Solid,
    Sensor,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct SensorEvent3d {
    pub left: EntityId,
    pub right: EntityId,
}

#[must_use]
pub const fn pair_interaction(
    left: &InteractiveCollider3d,
    right: &InteractiveCollider3d,
) -> PairInteraction3d {
    if !left.filter.interacts_with(right.filter) {
        return PairInteraction3d::Ignore;
    }
    if matches!(left.role, ColliderRole3d::Sensor) || matches!(right.role, ColliderRole3d::Sensor) {
        PairInteraction3d::Sensor
    } else {
        PairInteraction3d::Solid
    }
}

/// Collects deterministic final-state sensor overlaps in canonical entity-pair order.
///
/// Sensor overlap is observation only: this function cannot mutate ECS state or apply collision response.
/// Filtering happens before shape evaluation, so masked pairs are not reported.
///
/// # Errors
///
/// Returns [`ColliderError3d`] when a participating collider has invalid shape data or overflows the
/// integer contact calculation.
pub fn sensor_events(
    colliders: &[InteractiveCollider3d],
) -> Result<Vec<SensorEvent3d>, ColliderError3d> {
    let mut ordered = colliders.to_vec();
    ordered.sort_unstable_by_key(|entry| entry.entity);
    let mut events = Vec::new();

    for left_index in 0..ordered.len() {
        for right_index in (left_index + 1)..ordered.len() {
            let left = ordered[left_index];
            let right = ordered[right_index];
            if pair_interaction(&left, &right) != PairInteraction3d::Sensor {
                continue;
            }
            if collider_contact(left.collider, right.collider)?.overlaps() {
                events.push(SensorEvent3d {
                    left: left.entity,
                    right: right.entity,
                });
            }
        }
    }
    Ok(events)
}

#[cfg(test)]
mod tests {
    use ecs_workload::{EntityId, Position};

    use crate::{Collider3d, ColliderShape3d};

    use super::{
        ColliderRole3d, CollisionFilter3d, InteractiveCollider3d, PairInteraction3d, SensorEvent3d,
        pair_interaction, sensor_events,
    };

    fn sphere(entity: u32, x: i64, role: ColliderRole3d) -> InteractiveCollider3d {
        InteractiveCollider3d {
            entity: EntityId(entity),
            collider: Collider3d::new(Position::new3(x, 0, 0), ColliderShape3d::sphere(2)),
            filter: CollisionFilter3d::all(),
            role,
        }
    }

    #[test]
    fn layer_masks_must_agree_in_both_directions() {
        let left =
            sphere(1, 0, ColliderRole3d::Solid).with_filter(CollisionFilter3d::new(0b0001, 0b0010));
        let right =
            sphere(2, 0, ColliderRole3d::Solid).with_filter(CollisionFilter3d::new(0b0010, 0b0001));
        assert_eq!(pair_interaction(&left, &right), PairInteraction3d::Solid);

        let masked = right.with_filter(CollisionFilter3d::new(0b0010, 0b0100));
        assert_eq!(pair_interaction(&left, &masked), PairInteraction3d::Ignore);
    }

    #[test]
    fn either_sensor_role_makes_the_pair_observation_only() {
        let solid = sphere(1, 0, ColliderRole3d::Solid);
        let sensor = sphere(2, 0, ColliderRole3d::Sensor);
        assert_eq!(pair_interaction(&solid, &sensor), PairInteraction3d::Sensor);
        assert_eq!(pair_interaction(&sensor, &solid), PairInteraction3d::Sensor);
    }

    #[test]
    fn overlapping_sensor_events_are_sorted_by_entity_pair() {
        let colliders = [
            sphere(9, 0, ColliderRole3d::Solid),
            sphere(2, 1, ColliderRole3d::Sensor),
            sphere(5, 20, ColliderRole3d::Sensor),
        ];
        assert_eq!(
            sensor_events(&colliders).expect("valid sensor shapes"),
            vec![SensorEvent3d {
                left: EntityId(2),
                right: EntityId(9),
            }]
        );
    }

    #[test]
    fn masked_sensor_overlap_is_not_reported() {
        let sensor = sphere(1, 0, ColliderRole3d::Sensor)
            .with_filter(CollisionFilter3d::new(0b0001, 0b0010));
        let solid =
            sphere(2, 0, ColliderRole3d::Solid).with_filter(CollisionFilter3d::new(0b0100, 0b0001));
        assert!(
            sensor_events(&[sensor, solid])
                .expect("valid shapes")
                .is_empty()
        );
    }
}
