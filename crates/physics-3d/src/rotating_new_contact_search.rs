use std::collections::BTreeMap;

use crate::{
    MAX_ROTATING_CONTACT_REFINEMENTS, MAX_ROTATING_CONTACT_SAMPLES, ObbContactSeed3d,
    OrientedBox3d, RigidBox3d, RigidBoxFreeFlightConfig3d, RigidBoxWorldConfig3d,
    RigidBoxWorldError3d, RotatingContactBracket3d, RotatingContactSearchConfig3d,
    RotatingContactSearchError3d, RotatingContactSet3d, bracket_rotating_contact, obb_contact_seed,
    rotational_sweep_candidate_pairs, sample_rigid_box_free_flight,
};

/// Finds the first strictly-positive sampled rotating impact for one pair.
///
/// A pair that starts clear delegates to [`bracket_rotating_contact`]. A pair that starts in contact is
/// treated as an existing constraint until the bounded coarse grid observes a clear sample. Once clear,
/// the first later sampled contact becomes a re-contact bracket and is refined with the same deterministic
/// bisection policy used by the ordinary first-contact search. Persistent start contacts therefore do not
/// stall a repeated event loop, while a sampled clear-then-recontact transition is no longer discarded.
///
/// This remains bounded sampled evidence rather than analytic rotational CCD. A separation or contact
/// island that exists entirely between configured coarse samples can still be missed. Re-contact is only
/// admitted when the configured grid observes at least one clear state followed by a later contact state.
///
/// # Errors
///
/// Returns [`RotatingContactSearchError3d`] for malformed pair/configuration inputs, invalid sampled OBB
/// geometry, or checked arithmetic failure.
pub fn bracket_new_rotating_contact(
    left: RigidBox3d,
    right: RigidBox3d,
    frame_config: RigidBoxWorldConfig3d,
    search_config: RotatingContactSearchConfig3d,
) -> Result<Option<RotatingContactBracket3d>, RotatingContactSearchError3d> {
    let Some(first) = bracket_rotating_contact(left, right, frame_config, search_config)? else {
        return Ok(None);
    };
    if first.contact_numerator > 0 {
        return Ok(Some(first));
    }

    bracket_recontact_after_start_contact(left, right, frame_config, search_config)
}

/// Returns all strictly-positive sampled rotating impacts on conservative sweep candidates.
///
/// World validation and broad-phase admission mirror [`crate::search_rotating_contacts`]. Each candidate
/// is then searched with [`bracket_new_rotating_contact`], so persistent start constraints disappear from
/// the event stream while sampled clear-then-recontact transitions remain eligible. Results are sorted by
/// sampled contact fraction and canonical entity ids.
///
/// # Errors
///
/// Returns [`RotatingContactSearchError3d`] for malformed world/search inputs, conservative sweep failure,
/// invalid sampled geometry, or checked arithmetic failure.
pub fn search_new_rotating_contacts(
    boxes: &[RigidBox3d],
    frame_config: RigidBoxWorldConfig3d,
    search_config: RotatingContactSearchConfig3d,
) -> Result<Vec<RotatingContactBracket3d>, RotatingContactSearchError3d> {
    validate_configs(frame_config, search_config)?;
    for pair in boxes.windows(2) {
        if pair[0].body.entity >= pair[1].body.entity {
            return Err(RotatingContactSearchError3d::NonCanonicalWorld(
                pair[0].body.entity,
                pair[1].body.entity,
            ));
        }
    }

    let end = boxes
        .iter()
        .copied()
        .map(|body| sampled_body(body, frame_config, 1, 1))
        .collect::<Result<Vec<_>, _>>()?;
    let candidates = rotational_sweep_candidate_pairs(boxes, &end)?;
    let indices = boxes
        .iter()
        .enumerate()
        .map(|(index, body)| (body.body.entity, index))
        .collect::<BTreeMap<_, _>>();
    let mut contacts = Vec::new();
    for candidate in candidates {
        let left_index = *indices
            .get(&candidate.left)
            .ok_or(RotatingContactSearchError3d::ArithmeticOverflow)?;
        let right_index = *indices
            .get(&candidate.right)
            .ok_or(RotatingContactSearchError3d::ArithmeticOverflow)?;
        if let Some(contact) = bracket_new_rotating_contact(
            boxes[left_index],
            boxes[right_index],
            frame_config,
            search_config,
        )? {
            contacts.push(contact);
        }
    }
    contacts.sort_by_key(|contact| (contact.contact_numerator, contact.left, contact.right));
    Ok(contacts)
}

/// Collects the globally earliest strictly-positive sampled rotating contact set.
///
/// Persistent start contacts are absent, sampled re-contacts are retained, and equal-time impacts are
/// grouped before response. Any returned set therefore proves positive sampled time progress while keeping
/// the deterministic simultaneous-contact boundary used by the initial frontier path.
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

fn bracket_recontact_after_start_contact(
    left: RigidBox3d,
    right: RigidBox3d,
    frame_config: RigidBoxWorldConfig3d,
    search_config: RotatingContactSearchConfig3d,
) -> Result<Option<RotatingContactBracket3d>, RotatingContactSearchError3d> {
    let coarse_denominator = u32::from(search_config.coarse_samples);
    let mut last_clear = None;
    let mut bracket = None;

    for sample in 1..=search_config.coarse_samples {
        let numerator = u32::from(sample);
        match sampled_contact(left, right, frame_config, numerator, coarse_denominator)? {
            Some(contact) => {
                if let Some(clear) = last_clear {
                    bracket = Some((clear, numerator, coarse_denominator, contact));
                    break;
                }
            }
            None => last_clear = Some(numerator),
        }
    }

    let Some((mut clear, mut contact_at, mut denominator, mut contact)) = bracket else {
        return Ok(None);
    };

    for _ in 0..search_config.refinement_steps {
        clear = clear
            .checked_mul(2)
            .ok_or(RotatingContactSearchError3d::ArithmeticOverflow)?;
        contact_at = contact_at
            .checked_mul(2)
            .ok_or(RotatingContactSearchError3d::ArithmeticOverflow)?;
        denominator = denominator
            .checked_mul(2)
            .ok_or(RotatingContactSearchError3d::ArithmeticOverflow)?;
        let midpoint = clear + (contact_at - clear) / 2;
        if let Some(midpoint_contact) =
            sampled_contact(left, right, frame_config, midpoint, denominator)?
        {
            contact_at = midpoint;
            contact = midpoint_contact;
        } else {
            clear = midpoint;
        }
    }

    Ok(Some(RotatingContactBracket3d {
        left: left.body.entity,
        right: right.body.entity,
        clear_numerator: clear,
        contact_numerator: contact_at,
        denominator,
        contact,
    }))
}

fn validate_configs(
    frame_config: RigidBoxWorldConfig3d,
    search_config: RotatingContactSearchConfig3d,
) -> Result<(), RotatingContactSearchError3d> {
    if frame_config.timestep_numerator < 0 {
        return Err(RotatingContactSearchError3d::World(
            RigidBoxWorldError3d::NegativeTimestepNumerator(frame_config.timestep_numerator),
        ));
    }
    if frame_config.timestep_denominator <= 0 {
        return Err(RotatingContactSearchError3d::World(
            RigidBoxWorldError3d::NonPositiveTimestepDenominator(frame_config.timestep_denominator),
        ));
    }
    if search_config.coarse_samples == 0
        || search_config.coarse_samples > MAX_ROTATING_CONTACT_SAMPLES
    {
        return Err(RotatingContactSearchError3d::CoarseSamplesOutOfRange(
            search_config.coarse_samples,
        ));
    }
    if search_config.refinement_steps > MAX_ROTATING_CONTACT_REFINEMENTS {
        return Err(RotatingContactSearchError3d::RefinementStepsOutOfRange(
            search_config.refinement_steps,
        ));
    }
    Ok(())
}

fn sampled_contact(
    left: RigidBox3d,
    right: RigidBox3d,
    frame_config: RigidBoxWorldConfig3d,
    fraction_numerator: u32,
    fraction_denominator: u32,
) -> Result<Option<ObbContactSeed3d>, RotatingContactSearchError3d> {
    let left = sampled_body(left, frame_config, fraction_numerator, fraction_denominator)?;
    let right = sampled_body(
        right,
        frame_config,
        fraction_numerator,
        fraction_denominator,
    )?;
    Ok(obb_contact_seed(
        OrientedBox3d::new(
            left.state.center,
            left.body.half_extents,
            left.state.angular.orientation,
        ),
        OrientedBox3d::new(
            right.state.center,
            right.body.half_extents,
            right.state.angular.orientation,
        ),
    )?)
}

fn sampled_body(
    body: RigidBox3d,
    frame_config: RigidBoxWorldConfig3d,
    fraction_numerator: u32,
    fraction_denominator: u32,
) -> Result<RigidBox3d, RotatingContactSearchError3d> {
    Ok(sample_rigid_box_free_flight(
        body,
        RigidBoxFreeFlightConfig3d {
            gravity: frame_config.gravity,
            timestep_numerator: frame_config.timestep_numerator,
            timestep_denominator: frame_config.timestep_denominator,
        },
        fraction_numerator,
        fraction_denominator,
    )?)
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

    fn recontacting_rod() -> RigidBox3d {
        body(
            1,
            BodyKind::Dynamic,
            [20, 2, 2],
            Position::new3(0, 0, 0),
            AngularVelocity3d::new(0, 0, 230 * ANGULAR_VELOCITY_SCALE),
        )
    }

    fn resting_rod() -> RigidBox3d {
        body(
            1,
            BodyKind::Dynamic,
            [20, 2, 2],
            Position::new3(0, 0, 0),
            AngularVelocity3d::default(),
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
    fn persistent_start_contact_is_not_a_new_impact() {
        let rod = resting_rod();
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
    fn start_contact_clear_then_recontact_is_reported() {
        let rod = recontacting_rod();
        let touching = obstacle(2, -6, 3);
        let ordinary = bracket_rotating_contact(
            rod,
            touching,
            frame_config(),
            RotatingContactSearchConfig3d::default(),
        )
        .expect("valid ordinary search")
        .expect("fixture should start in contact");
        let recontact = bracket_new_rotating_contact(
            rod,
            touching,
            frame_config(),
            RotatingContactSearchConfig3d::default(),
        )
        .expect("valid recontact search")
        .expect("rotating rod should separate and later recontact");

        assert_eq!(ordinary.contact_numerator, 0);
        assert!(recontact.clear_numerator > 0);
        assert!(recontact.contact_numerator > recontact.clear_numerator);
        assert!(recontact.contact_numerator < recontact.denominator);
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
    fn world_search_ignores_persistent_contact_but_keeps_later_new_impact() {
        let contacts = search_new_rotating_contacts(
            &[
                rotating_rod(),
                obstacle(2, 0, 0),
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
    fn world_search_retains_sampled_recontact() {
        let contacts = search_new_rotating_contacts(
            &[recontacting_rod(), obstacle(2, -6, 3)],
            frame_config(),
            RotatingContactSearchConfig3d::default(),
        )
        .expect("valid recontact world search");

        assert_eq!(contacts.len(), 1);
        assert_eq!(contacts[0].left, EntityId(1));
        assert_eq!(contacts[0].right, EntityId(2));
        assert!(contacts[0].clear_numerator > 0);
        assert!(contacts[0].contact_numerator > contacts[0].clear_numerator);
    }

    #[test]
    fn earliest_new_set_groups_only_strictly_positive_contacts() {
        let set = earliest_new_rotating_contact_set(
            &[
                rotating_rod(),
                obstacle(2, 0, 0),
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
    fn only_persistent_start_contacts_produce_no_new_event_set() {
        assert_eq!(
            earliest_new_rotating_contact_set(
                &[rotating_rod(), obstacle(2, 0, 0)],
                frame_config(),
                RotatingContactSearchConfig3d::default(),
            )
            .expect("valid empty new-event search"),
            None
        );
    }
}
