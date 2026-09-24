use std::thread;

use ecs_workload::{Position, Velocity};

#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub enum SystemId {
    Integrate,
    RegenerateEnergy,
    Accelerate,
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct Access {
    reads: u8,
    writes: u8,
}

impl Access {
    const POSITION: u8 = 1;
    const VELOCITY: u8 = 1 << 1;
    const ENERGY: u8 = 1 << 2;

    #[must_use]
    pub const fn integrate() -> Self {
        Self {
            reads: Self::VELOCITY,
            writes: Self::POSITION,
        }
    }

    #[must_use]
    pub const fn regenerate_energy() -> Self {
        Self {
            reads: Self::ENERGY,
            writes: Self::ENERGY,
        }
    }

    #[must_use]
    pub const fn accelerate() -> Self {
        Self {
            reads: Self::POSITION,
            writes: Self::VELOCITY,
        }
    }

    #[must_use]
    pub const fn conflicts(self, other: Self) -> bool {
        let self_touches = self.reads | self.writes;
        let other_touches = other.reads | other.writes;
        self.writes & other_touches != 0 || other.writes & self_touches != 0
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct SystemSpec {
    pub id: SystemId,
    pub access: Access,
}

#[must_use]
pub fn system_specs() -> [SystemSpec; 3] {
    [
        SystemSpec {
            id: SystemId::Integrate,
            access: Access::integrate(),
        },
        SystemSpec {
            id: SystemId::RegenerateEnergy,
            access: Access::regenerate_energy(),
        },
        SystemSpec {
            id: SystemId::Accelerate,
            access: Access::accelerate(),
        },
    ]
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Schedule {
    pub batches: Vec<Vec<SystemId>>,
}

#[must_use]
pub fn build_schedule(mut systems: Vec<SystemSpec>) -> Schedule {
    systems.sort_unstable_by_key(|system| system.id);
    let mut batches: Vec<Vec<SystemSpec>> = Vec::new();

    for system in systems {
        if let Some(batch) = batches.iter_mut().find(|batch| {
            batch
                .iter()
                .all(|scheduled| !scheduled.access.conflicts(system.access))
        }) {
            batch.push(system);
        } else {
            batches.push(vec![system]);
        }
    }

    Schedule {
        batches: batches
            .into_iter()
            .map(|batch| batch.into_iter().map(|system| system.id).collect())
            .collect(),
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct SimulationSnapshot {
    pub positions: Vec<Position>,
    pub velocities: Vec<Velocity>,
    pub energy: Vec<i64>,
}

#[derive(Clone, Debug, Eq, PartialEq)]
struct SimulationWorld {
    positions: Vec<Position>,
    velocities: Vec<Velocity>,
    energy: Vec<i64>,
}

impl SimulationWorld {
    fn new(entity_count: u32) -> Self {
        let mut positions = Vec::with_capacity(entity_count as usize);
        let mut velocities = Vec::with_capacity(entity_count as usize);
        let mut energy = Vec::with_capacity(entity_count as usize);
        for raw_id in 0..entity_count {
            let value = i64::from(raw_id);
            positions.push(Position::new3(value, -value, value / 2));
            velocities.push(Velocity::new3(
                i32::try_from(raw_id % 7).unwrap_or_default() - 3,
                2,
                -1,
            ));
            energy.push(value % 11);
        }
        Self {
            positions,
            velocities,
            energy,
        }
    }

    fn snapshot(&self) -> SimulationSnapshot {
        SimulationSnapshot {
            positions: self.positions.clone(),
            velocities: self.velocities.clone(),
            energy: self.energy.clone(),
        }
    }
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct SchedulerStats {
    pub schedule_rebuilds: u64,
    pub batches_per_round: u64,
    pub parallel_batches_per_round: u64,
    pub barriers: u64,
    pub useful_entity_system_runs: u64,
    pub worker_count: u64,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct SchedulerEvidence {
    pub snapshot: SimulationSnapshot,
    pub stats: SchedulerStats,
}

#[must_use]
pub fn run_serial(entity_count: u32, rounds: u32) -> SchedulerEvidence {
    let schedule = build_schedule(system_specs().to_vec());
    let mut world = SimulationWorld::new(entity_count);
    for _ in 0..rounds {
        run_integrate(&mut world.positions, &world.velocities);
        run_regenerate_energy(&mut world.energy);
        run_accelerate(&world.positions, &mut world.velocities);
    }
    SchedulerEvidence {
        snapshot: world.snapshot(),
        stats: schedule_stats(&schedule, entity_count, rounds, 1),
    }
}

/// Executes the deterministic access-aware schedule with independent systems in parallel.
///
/// # Panics
///
/// Panics if the fixed scheduler contract no longer produces the expected two deterministic batches.
#[must_use]
pub fn run_parallel_systems(entity_count: u32, rounds: u32) -> SchedulerEvidence {
    let schedule = build_schedule(system_specs().to_vec());
    assert_eq!(
        schedule.batches,
        vec![
            vec![SystemId::Integrate, SystemId::RegenerateEnergy],
            vec![SystemId::Accelerate],
        ]
    );
    let mut world = SimulationWorld::new(entity_count);

    for _ in 0..rounds {
        let SimulationWorld {
            positions,
            velocities,
            energy,
        } = &mut world;
        thread::scope(|scope| {
            scope.spawn(|| run_integrate(positions, velocities));
            scope.spawn(|| run_regenerate_energy(energy));
        });
        run_accelerate(&world.positions, &mut world.velocities);
    }

    SchedulerEvidence {
        snapshot: world.snapshot(),
        stats: schedule_stats(&schedule, entity_count, rounds, 2),
    }
}

/// Executes each system with deterministic entity partitions.
///
/// # Panics
///
/// Panics when `worker_count` is zero.
#[must_use]
pub fn run_partitioned(
    entity_count: u32,
    rounds: u32,
    worker_count: usize,
) -> SchedulerEvidence {
    assert!(worker_count > 0, "worker_count must be non-zero");
    let schedule = build_schedule(system_specs().to_vec());
    let mut world = SimulationWorld::new(entity_count);

    for _ in 0..rounds {
        run_integrate_partitioned(&mut world.positions, &world.velocities, worker_count);
        run_regenerate_partitioned(&mut world.energy, worker_count);
        run_accelerate_partitioned(&world.positions, &mut world.velocities, worker_count);
    }

    SchedulerEvidence {
        snapshot: world.snapshot(),
        stats: schedule_stats(
            &schedule,
            entity_count,
            rounds,
            u64::try_from(worker_count).unwrap_or(u64::MAX),
        ),
    }
}

fn schedule_stats(
    schedule: &Schedule,
    entity_count: u32,
    rounds: u32,
    worker_count: impl Into<u64>,
) -> SchedulerStats {
    let batches = u64::try_from(schedule.batches.len()).unwrap_or(u64::MAX);
    let parallel_batches = u64::try_from(
        schedule
            .batches
            .iter()
            .filter(|batch| batch.len() > 1)
            .count(),
    )
    .unwrap_or(u64::MAX);
    SchedulerStats {
        schedule_rebuilds: 1,
        batches_per_round: batches,
        parallel_batches_per_round: parallel_batches,
        barriers: batches.saturating_mul(u64::from(rounds)),
        useful_entity_system_runs: u64::from(entity_count)
            .saturating_mul(u64::from(rounds))
            .saturating_mul(3),
        worker_count: worker_count.into(),
    }
}

fn run_integrate(positions: &mut [Position], velocities: &[Velocity]) {
    for (position, velocity) in positions.iter_mut().zip(velocities.iter().copied()) {
        integrate_one(position, velocity);
    }
}

fn run_regenerate_energy(energy: &mut [i64]) {
    for value in energy {
        *value = value.saturating_add(1);
    }
}

fn run_accelerate(positions: &[Position], velocities: &mut [Velocity]) {
    for (position, velocity) in positions.iter().zip(velocities) {
        accelerate_one(*position, velocity);
    }
}

fn run_integrate_partitioned(
    positions: &mut [Position],
    velocities: &[Velocity],
    workers: usize,
) {
    let chunk = partition_size(positions.len(), workers);
    if chunk == 0 {
        return;
    }
    thread::scope(|scope| {
        for (positions, velocities) in positions
            .chunks_mut(chunk)
            .zip(velocities.chunks(chunk))
        {
            scope.spawn(move || run_integrate(positions, velocities));
        }
    });
}

fn run_regenerate_partitioned(energy: &mut [i64], workers: usize) {
    let chunk = partition_size(energy.len(), workers);
    if chunk == 0 {
        return;
    }
    thread::scope(|scope| {
        for energy in energy.chunks_mut(chunk) {
            scope.spawn(move || run_regenerate_energy(energy));
        }
    });
}

fn run_accelerate_partitioned(
    positions: &[Position],
    velocities: &mut [Velocity],
    workers: usize,
) {
    let chunk = partition_size(velocities.len(), workers);
    if chunk == 0 {
        return;
    }
    thread::scope(|scope| {
        for (positions, velocities) in positions
            .chunks(chunk)
            .zip(velocities.chunks_mut(chunk))
        {
            scope.spawn(move || run_accelerate(positions, velocities));
        }
    });
}

const fn partition_size(len: usize, workers: usize) -> usize {
    if len == 0 {
        0
    } else {
        len.div_ceil(workers)
    }
}

fn integrate_one(position: &mut Position, velocity: Velocity) {
    position.x = position.x.saturating_add(i64::from(velocity.x));
    position.y = position.y.saturating_add(i64::from(velocity.y));
    position.z = position.z.saturating_add(i64::from(velocity.z));
}

fn accelerate_one(position: Position, velocity: &mut Velocity) {
    let delta = if position.x & 1 == 0 { 1 } else { -1 };
    velocity.x = velocity.x.saturating_add(delta);
    velocity.z = velocity.z.saturating_sub(delta);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn access_conflicts_are_explicit() {
        assert!(!Access::integrate().conflicts(Access::regenerate_energy()));
        assert!(Access::integrate().conflicts(Access::accelerate()));
        assert!(!Access::regenerate_energy().conflicts(Access::accelerate()));
    }

    #[test]
    fn schedule_is_deterministic_across_input_orders() {
        let expected = Schedule {
            batches: vec![
                vec![SystemId::Integrate, SystemId::RegenerateEnergy],
                vec![SystemId::Accelerate],
            ],
        };
        let specs = system_specs();
        for order in [
            vec![specs[0], specs[1], specs[2]],
            vec![specs[2], specs[0], specs[1]],
            vec![specs[1], specs[2], specs[0]],
        ] {
            assert_eq!(build_schedule(order), expected);
        }
    }

    #[test]
    fn parallel_and_partitioned_execution_match_serial_for_worker_counts() {
        for entities in [0_u32, 1, 17, 1_024] {
            let expected = run_serial(entities, 16);
            let parallel = run_parallel_systems(entities, 16);
            assert_eq!(parallel.snapshot, expected.snapshot);
            for workers in [1_usize, 2, 4, 7] {
                let partitioned = run_partitioned(entities, 16, workers);
                assert_eq!(
                    partitioned.snapshot, expected.snapshot,
                    "entities={entities}, workers={workers}"
                );
            }
        }
    }

    #[test]
    fn scheduler_work_ratchet_has_two_batches_and_one_parallel_batch() {
        let evidence = run_parallel_systems(1_024, 8);
        assert_eq!(evidence.stats.schedule_rebuilds, 1);
        assert_eq!(evidence.stats.batches_per_round, 2);
        assert_eq!(evidence.stats.parallel_batches_per_round, 1);
        assert_eq!(evidence.stats.barriers, 16);
        assert_eq!(evidence.stats.useful_entity_system_runs, 24_576);
        assert_eq!(evidence.stats.worker_count, 2);
    }
}
