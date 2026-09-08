use crate::{
    RigidBox3d, RigidBoxWorldConfig3d, RotatingContactBracket3d, RotatingContactSearchConfig3d,
    RotatingContactSearchError3d, RotatingContactSet3d, bracket_rotating_contact,
    search_rotating_contacts,
};

/// Finds the first sampled rotating contact only when the pair is clear at the segment start.
///
/// [`bracket_rotating_contact`] deliberately reports an existing start contact at fraction zero. That is
/// useful for diagnostics and initial stabilization, but it cannot drive a repeated event loop because a
/// resting pair would be rediscovered forever without consuming time. This positive-progress wrapper keeps
/// the original API unchanged and suppresses only zero-fraction results. A pair that is already touching at
/// the segment start is treated as an existing constraint for that segment rather than a new impact.
///
/// This remains bounded sampled evidence, not exact rotational CCD. In particular, a pair that starts in
/// contact, separates, and recontacts inside the same segment is intentionally not rediscovered here; a
/// later solver stage must shorten/restart the segment if it needs that distinction.
///
/// # Errors
///
/// Returns the same [`RotatingContactSearchError3d`] variants as [`bracket_rotating_contact`].
pub fn bracket_new_rotating_contact(
    left: RigidBox3d,
    right: RigidBox3d,
    frame_config: RigidBoxWorldConfig3d,
    search_config: RotatingContactSearchConfig3d,
) -> Result<Option<RotatingContactBracket3d>, RotatingContactSearchError3d> {
    Ok(
        bracket_rotating_contact(left, right, frame_config, search_config)?
            .filter(|contact| contact.contact_numerator > 0),
    )
}

/// Returns sampled rotating contacts that make strictly positive progress from the segment start.
///
/// The canonical world search still owns conservative sweep candidate generation, rational sampling,
/// refinement, and deterministic ordering. This wrapper removes only pairs whose first sampled contact is
/// already present at fraction zero. Remaining results preserve the canonical ordering by sampled contact
/// fraction and entity ids.
///
/// # Errors
///
/// Returns the same [`RotatingContactSearchError3d`] variants as [`search_rotating_contacts`].
pub fn search_new_rotating_contacts(
    boxes: &[RigidBox3d],
    frame_config: RigidBoxWorldConfig3d,
    search_config: RotatingContactSearchConfig3d,
) -> Result<Vec<RotatingContactBracket3d>, RotatingContactSearchError3d> {
    Ok(
        search_rotating_contacts(boxes, frame_config, search_config)?
            .into_iter()
            .filter(|contact| contact.contact_numerator > 0)
            .collect(),
    )
}

/// Collects the globally earliest strictly-positive sampled rotating contact set.
///
/// Existing start contacts are absent from this result, so any returned set proves positive sampled time
/// progress. Equal-time new impacts are still grouped together before response, preserving the same
/// deterministic simultaneous-contact boundary as the initial sampled frontier path.
///
/// # Errors
///
/// Returns the same [`RotatingContactSearchError3d`] variants as [`search_new_rotating_contacts`].
pub fn earliest_new_rotating_contact_set(
    boxes: &[RigidBox3d],
    frame_config: RigidBoxWorldConfig3d,
    search_config: RotatingContactSearchConfig3d,
) -> Result<Option<RotatingContactSet3d>, RotatingContactSearchError3d> {
    let contacts = search_new_rotating_contacts(boxes, frame_config, search_config)?;
    let Some(first) = contacts.first().copied() else {
        return Ok(None);
    };
    let contact_numerator = first.contact_numerator;
    let denominator = first.denominator;
    let contacts = contacts
        .into_iter()
        .take_while(|contact| {
            contact.contact_numerator == contact_numerator && contact.denominator == denominator
        })
        .collect();
    Ok(Some(RotatingContactSet3d {
        contact_numerator,
        denominator,
        contacts,
    }))
}

#[cfg(test)]
mod tests {
    use ecs_physics::{BodyKind, MATERIAL_SCALE};
    use ecs_workload::{EntityId, Position, Velocity};

    use crate::{
        ANGULAR_VELOCITY_SCALE, AngularState3d, AngularVelocity3d, Orientation3d, PhysicsBody3d,
        RigidBoxState3d,
    };

    use super::*;

    fn frame_config() -> RigidBoxWorldConfig3d {
        RigidBoxWorldConfig3d {
            gravity: Velocity::new3(0, 0, 0),
            timestep_numerator: 1,
            timestep_denominator: 60,
            angular_damping_milli: MATERIAL_SCALE,
            solver_passes: 8,
        }
    }

    fn body(
        entity: u32,
        kind: BodyKind,
        half_extents: [i32; 3],
        center: Position,
        angular_velocity: AngularVelocity3d,
    ) -> RigidBox3d {
        let physics_body = match kind {
            BodyKind::Dynamic => PhysicsBody3d::dynamic(EntityId(entity), half_extents),
            BodyKind::Fixed => PhysicsBody3d::fixed(EntityId(entity), half_extents),
        };
        RigidBox3d::new(
            physics_body,
            RigidBoxState3d::new(
                center,
                Velocity::new3(0, 0, 0),
                AngularState3d::new(Orientation3d::IDENTITY, angular_velocity),
            ),
        )
    }

    fn rotating_rod() -> RigidBox3d {
        body(
            1,
            BodyKind::Dynamic,
            [20, 2, 2],
            Position::new3(0, 0, 0),
            AngularVelocity3d::new(0, 0, 230 * ANGULAR_VELOCITY_SCALE),
        )
    }

    fn obstacle(entity: u32, x: i64, y: i64) -> RigidBox3d {
        body(
            entity,
            BodyKind::Fixed,
            [2, 2, 2],
            Position::new3(x, y, 0),
            AngularVelocity3d::default(),
        )
    }

    #[test]
    fn existing_start_contact_is_not_a_new_impact() {
        let rod = rotating_rod();
        let touching = obstacle(2, 18, 0);
        let ordinary = bracket_rotating_contact(
            rod,
            touching,
            frame_config(),
            RotatingContactSearchConfig3d::default(),
        )
        .expect("valid ordinary search")
        .expect("fixture should already touch");

        assert_eq!(ordinary.contact_numerator, 0);
        assert_eq!(
            bracket_new_rotating_contact(
                rod,
                touching,
                frame_config(),
                RotatingContactSearchConfig3d::default(),
            )
            .expect("valid positive-progress search"),
            None
        );
    }

    #[test]
    fn later_sampled_impact_is_retained_with_positive_fraction() {
        let contact = bracket_new_rotating_contact(
            rotating_rod(),
            obstacle(2, 10, 10),
            frame_config(),
            RotatingContactSearchConfig3d::default(),
        )
        .expect("valid positive-progress search")
        .expect("rotating rod should reach the later obstacle");

        assert!(contact.contact_numerator > 0);
        assert!(contact.contact_numerator < contact.denominator);
    }

    #[test]
    fn world_search_ignores_resting_pair_but_keeps_later_new_impact() {
        let contacts = search_new_rotating_contacts(
            &[
                rotating_rod(),
                obstacle(2, 18, 0),
                obstacle(3, 10, 10),
                obstacle(4, 120, 120),
            ],
            frame_config(),
            RotatingContactSearchConfig3d::default(),
        )
        .expect("valid positive-progress world search");

        assert!(contacts.iter().all(|contact| contact.contact_numerator > 0));
        assert!(contacts.iter().all(|contact| contact.right != EntityId(2)));
        assert!(contacts.iter().any(|contact| contact.right == EntityId(3)));
        assert!(contacts.iter().all(|contact| contact.right != EntityId(4)));
    }

    #[test]
    fn earliest_new_set_groups_only_strictly_positive_contacts() {
        let set = earliest_new_rotating_contact_set(
            &[
                rotating_rod(),
                obstacle(2, 18, 0),
                obstacle(3, 10, 10),
                obstacle(4, -10, -10),
            ],
            frame_config(),
            RotatingContactSearchConfig3d::default(),
        )
        .expect("valid earliest new-impact search")
        .expect("symmetric later contacts should be sampled");

        assert!(set.contact_numerator > 0);
        assert!(
            set.contacts
                .iter()
                .all(|contact| contact.right != EntityId(2))
        );
        assert_eq!(set.contacts.len(), 2);
        assert!(set.contacts.iter().all(|contact| {
            contact.contact_numerator == set.contact_numerator
                && contact.denominator == set.denominator
        }));
    }

    #[test]
    fn only_existing_start_contacts_produce_no_new_event_set() {
        assert_eq!(
            earliest_new_rotating_contact_set(
                &[rotating_rod(), obstacle(2, 18, 0)],
                frame_config(),
                RotatingContactSearchConfig3d::default(),
            )
            .expect("valid empty new-event search"),
            None
        );
    }
}
