use crate::map::GeoCoord;
use crate::simulation::config::{
    DefenseConfigRegistry, MissileConfigRegistry, MissileType,
    SensorConfigRegistry, InterceptorConfigRegistry, SatelliteConfigRegistry
};
use crate::simulation::detection::DetectionSystem;
use crate::simulation::entities::*;
use crate::simulation::physics::{haversine_distance, interpolate_great_circle, BallisticTrajectory, FlightPhase};
use rand::Rng;
use std::path::Path;

/// Time scale options for simulation speed
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum TimeScale {
    Paused,
    RealTime,      // 1x
    Fast,          // 10x
    VeryFast,      // 60x (1 minute per second)
    UltraFast,     // 300x (5 minutes per second)
}

impl TimeScale {
    pub fn multiplier(&self) -> f64 {
        match self {
            TimeScale::Paused => 0.0,
            TimeScale::RealTime => 1.0,
            TimeScale::Fast => 10.0,
            TimeScale::VeryFast => 60.0,
            TimeScale::UltraFast => 300.0,
        }
    }

    pub fn name(&self) -> &'static str {
        match self {
            TimeScale::Paused => "Paused",
            TimeScale::RealTime => "1x",
            TimeScale::Fast => "10x",
            TimeScale::VeryFast => "60x",
            TimeScale::UltraFast => "300x",
        }
    }

    pub fn all() -> &'static [TimeScale] {
        &[
            TimeScale::Paused,
            TimeScale::RealTime,
            TimeScale::Fast,
            TimeScale::VeryFast,
            TimeScale::UltraFast,
        ]
    }
}

/// The main simulation engine
pub struct SimulationEngine {
    /// Current simulation time in seconds
    pub sim_time: f64,
    /// Time scale for simulation speed
    pub time_scale: TimeScale,
    /// Next entity ID to assign
    next_id: EntityId,
    /// All missiles in the simulation
    pub missiles: Vec<Missile>,
    /// All defense units
    pub defense_units: Vec<DefenseUnit>,
    /// All satellites
    pub satellites: Vec<Satellite>,
    /// All radar stations
    pub radar_stations: Vec<RadarStation>,
    /// All interceptors in flight
    pub interceptors: Vec<Interceptor>,
    /// Cached trajectories for missiles
    trajectories: Vec<(EntityId, BallisticTrajectory)>,
    /// Detection system for tracking sensors and targets
    pub detection: DetectionSystem,
    /// Track interceptors in flight per target (for salvo fire)
    interceptors_per_target: std::collections::HashMap<EntityId, u32>,
    /// Maximum interceptors to fire per target (salvo size)
    pub salvo_size: u32,
    /// Delay between interceptor launches in a salvo (seconds)
    pub salvo_delay: f64,
    /// Targets that need follow-up shots after a miss (for shoot-look-shoot)
    targets_needing_followup: std::collections::HashSet<EntityId>,
    /// Track total shots fired at each target (to enforce salvo_size limit)
    shots_fired_per_target: std::collections::HashMap<EntityId, u32>,
    /// Defense system configurations loaded from TOML files
    pub defense_configs: DefenseConfigRegistry,
    /// Missile configurations loaded from TOML files
    pub missile_configs: MissileConfigRegistry,
    /// Sensor configurations loaded from TOML files
    pub sensor_configs: SensorConfigRegistry,
    /// Interceptor configurations loaded from TOML files
    pub interceptor_configs: InterceptorConfigRegistry,
    /// Satellite configurations loaded from TOML files
    pub satellite_configs: SatelliteConfigRegistry,
}

impl SimulationEngine {
    pub fn new() -> Self {
        // Try to load configs from config directory, fall back to defaults
        let defense_configs = DefenseConfigRegistry::load(Path::new("config"))
            .unwrap_or_else(|e| {
                eprintln!("Warning: Failed to load defense configs: {}, using defaults", e);
                DefenseConfigRegistry::with_defaults()
            });

        let missile_configs = MissileConfigRegistry::load(Path::new("config"))
            .unwrap_or_else(|e| {
                eprintln!("Warning: Failed to load missile configs: {}, using defaults", e);
                MissileConfigRegistry::with_defaults()
            });

        let sensor_configs = SensorConfigRegistry::load(Path::new("config"))
            .unwrap_or_else(|e| {
                eprintln!("Warning: Failed to load sensor configs: {}, using defaults", e);
                SensorConfigRegistry::with_defaults()
            });

        let interceptor_configs = InterceptorConfigRegistry::load(Path::new("config"))
            .unwrap_or_else(|e| {
                eprintln!("Warning: Failed to load interceptor configs: {}, using defaults", e);
                InterceptorConfigRegistry::with_defaults()
            });

        let satellite_configs = SatelliteConfigRegistry::load(Path::new("config"))
            .unwrap_or_else(|e| {
                eprintln!("Warning: Failed to load satellite configs: {}, using defaults", e);
                SatelliteConfigRegistry::with_defaults()
            });

        Self {
            sim_time: 0.0,
            time_scale: TimeScale::Paused,
            next_id: 1,
            missiles: Vec::new(),
            defense_units: Vec::new(),
            satellites: Vec::new(),
            radar_stations: Vec::new(),
            interceptors: Vec::new(),
            trajectories: Vec::new(),
            detection: DetectionSystem::new(),
            interceptors_per_target: std::collections::HashMap::new(),
            salvo_size: 2, // Default: fire 2 interceptors per target
            salvo_delay: 5.0, // 5 seconds between interceptor launches in a salvo
            targets_needing_followup: std::collections::HashSet::new(),
            shots_fired_per_target: std::collections::HashMap::new(),
            defense_configs,
            missile_configs,
            sensor_configs,
            interceptor_configs,
            satellite_configs,
        }
    }

    /// Generate a new unique entity ID
    fn new_id(&mut self) -> EntityId {
        let id = self.next_id;
        self.next_id += 1;
        id
    }

    /// Add a missile to the simulation (auto-detects type from range)
    pub fn add_missile(
        &mut self,
        name: String,
        affiliation: Affiliation,
        origin: GeoCoord,
        target: GeoCoord,
        launch_time: f64,
    ) -> EntityId {
        // Determine missile type from range
        let range_km = haversine_distance(origin, target);
        let missile_type = self.missile_configs.type_for_range(range_km);
        self.add_missile_typed(name, affiliation, origin, target, launch_time, missile_type)
    }

    /// Add a missile with explicit type
    pub fn add_missile_typed(
        &mut self,
        name: String,
        affiliation: Affiliation,
        origin: GeoCoord,
        target: GeoCoord,
        launch_time: f64,
        missile_type: MissileType,
    ) -> EntityId {
        let id = self.new_id();
        let range_km = haversine_distance(origin, target);

        // Get trajectory parameters from config (by name first, fallback to type)
        let config = self.missile_configs.get_by_name(&name);
        let apogee = config.trajectory.apogee_base_km + range_km * config.trajectory.apogee_range_factor;
        let flight_time = config.trajectory.flight_time_base_sec + range_km * config.trajectory.flight_time_range_factor;

        let trajectory = BallisticTrajectory::with_params(origin, target, apogee, flight_time);

        let mut missile = Missile::new(id, name, affiliation, origin, target, flight_time);
        missile.launch_time = launch_time;
        missile.missile_type = config.classification.missile_type;

        // Apply default countermeasures from config
        if config.countermeasures.has_countermeasures {
            missile.has_countermeasures = true;
            missile.decoys_deployed = 0;
            missile.max_decoys = config.countermeasures.default_decoys;
        }

        self.missiles.push(missile);
        self.trajectories.push((id, trajectory));

        id
    }

    /// Add a missile with countermeasures capability
    pub fn add_missile_with_countermeasures(
        &mut self,
        name: String,
        affiliation: Affiliation,
        origin: GeoCoord,
        target: GeoCoord,
        launch_time: f64,
        max_decoys: u32,
    ) -> EntityId {
        let range_km = haversine_distance(origin, target);
        let missile_type = self.missile_configs.type_for_range(range_km);
        self.add_missile_with_countermeasures_typed(
            name, affiliation, origin, target, launch_time, max_decoys, missile_type
        )
    }

    /// Add a missile with explicit type and countermeasures
    pub fn add_missile_with_countermeasures_typed(
        &mut self,
        name: String,
        affiliation: Affiliation,
        origin: GeoCoord,
        target: GeoCoord,
        launch_time: f64,
        max_decoys: u32,
        missile_type: MissileType,
    ) -> EntityId {
        let id = self.new_id();
        let range_km = haversine_distance(origin, target);

        // Get trajectory parameters from config (by name first, fallback to type)
        let config = self.missile_configs.get_by_name(&name);
        let apogee = config.trajectory.apogee_base_km + range_km * config.trajectory.apogee_range_factor;
        let flight_time = config.trajectory.flight_time_base_sec + range_km * config.trajectory.flight_time_range_factor;

        let trajectory = BallisticTrajectory::with_params(origin, target, apogee, flight_time);

        let mut missile = Missile::new(id, name, affiliation, origin, target, flight_time)
            .with_countermeasures(max_decoys);
        missile.launch_time = launch_time;
        missile.missile_type = config.classification.missile_type;

        self.missiles.push(missile);
        self.trajectories.push((id, trajectory));

        id
    }

    /// Add a defense unit to the simulation
    pub fn add_defense_unit(
        &mut self,
        name: String,
        affiliation: Affiliation,
        position: GeoCoord,
        defense_type: DefenseType,
        interceptors: u32,
    ) -> EntityId {
        let id = self.new_id();
        let unit = DefenseUnit::new(id, name, affiliation, position, defense_type, interceptors);
        self.defense_units.push(unit);
        id
    }

    /// Add a satellite to the simulation
    pub fn add_satellite(
        &mut self,
        name: String,
        affiliation: Affiliation,
        position: GeoCoord,
        altitude_km: f64,
        sensor_type: SensorType,
    ) -> EntityId {
        let id = self.new_id();
        let satellite = Satellite::new(id, name, affiliation, position, altitude_km, sensor_type);
        self.satellites.push(satellite);
        id
    }

    /// Add a radar station to the simulation
    pub fn add_radar_station(
        &mut self,
        name: String,
        affiliation: Affiliation,
        position: GeoCoord,
        detection_range_km: f64,
    ) -> EntityId {
        let id = self.new_id();
        let station = RadarStation::new(id, name, affiliation, position, detection_range_km);
        self.radar_stations.push(station);
        id
    }

    /// Update the simulation by a real-time delta (in seconds)
    pub fn update(&mut self, real_dt: f64) {
        let sim_dt = real_dt * self.time_scale.multiplier();

        if sim_dt == 0.0 {
            return;
        }

        self.sim_time += sim_dt;

        // Update all missiles
        let sim_time = self.sim_time;
        for missile in &mut self.missiles {
            // Find the trajectory for this missile
            let trajectory = self.trajectories.iter()
                .find(|(id, _)| *id == missile.id)
                .map(|(_, t)| t);
            Self::update_missile(missile, sim_time, trajectory);
        }

        // Deploy decoys for missiles being tracked (during midcourse phase)
        self.deploy_missile_decoys();

        // Update detection system
        self.detection.update(
            &self.missiles,
            &self.defense_units,
            &self.radar_stations,
            &self.satellites,
            sim_dt,
        );

        // Update defense unit status based on detections
        for unit in &mut self.defense_units {
            let has_detections = self
                .detection
                .detections_for_sensor(unit.id)
                .len() > 0;

            unit.status = if has_detections {
                UnitStatus::Tracking
            } else {
                UnitStatus::Idle
            };
        }

        // Update interceptors in flight
        self.update_interceptors(sim_time);

        // Launch interceptors at detected threats
        self.launch_interceptors_at_threats(sim_time);

        // Check for intercept completions
        self.resolve_intercepts();
    }

    /// Deploy decoys from missiles that have countermeasures and are being tracked
    fn deploy_missile_decoys(&mut self) {
        // Get IDs of missiles being targeted by interceptors
        let targeted_missile_ids: std::collections::HashSet<EntityId> = self
            .interceptors
            .iter()
            .filter(|i| i.status == InterceptorStatus::InFlight)
            .map(|i| i.target_id)
            .collect();

        for missile in &mut self.missiles {
            // Only deploy decoys during midcourse phase when being tracked
            if missile.status != MissileStatus::Midcourse {
                continue;
            }

            // Check if this missile is being targeted
            if !targeted_missile_ids.contains(&missile.id) {
                continue;
            }

            // Deploy a decoy if available (one per update cycle to spread them out)
            missile.deploy_decoy();
        }
    }

    /// Update all interceptors in flight - realistic kinematics with terminal homing
    fn update_interceptors(&mut self, sim_time: f64) {
        use crate::simulation::entities::{InterceptorKinematics, InterceptorPhase};

        // Launch pending interceptors whose time has come
        for interceptor in &mut self.interceptors {
            if interceptor.status == InterceptorStatus::Pending && sim_time >= interceptor.launch_time {
                interceptor.status = InterceptorStatus::InFlight;
            }
        }

        // Collect current missile positions for terminal homing
        let missile_positions: std::collections::HashMap<EntityId, (GeoCoord, f64)> = self
            .missiles
            .iter()
            .filter(|m| matches!(m.status, MissileStatus::Boost | MissileStatus::Midcourse | MissileStatus::Terminal))
            .map(|m| (m.id, (m.position, m.altitude_km)))
            .collect();

        for interceptor in &mut self.interceptors {
            if interceptor.status != InterceptorStatus::InFlight {
                continue;
            }

            interceptor.current_flight_time = sim_time - interceptor.launch_time;

            // Update kinematics (velocity and phase)
            interceptor.update_kinematics();

            let kin = InterceptorKinematics::for_defense_type(interceptor.defense_type);
            let flight_duration = interceptor.intercept_time - interceptor.launch_time;

            // Calculate position based on actual distance traveled using kinematics
            let distance_traveled = kin.distance_at_time(interceptor.current_flight_time);
            let total_distance = haversine_distance(interceptor.launch_position, interceptor.target_position);
            let horizontal_distance = (total_distance.powi(2) + interceptor.target_altitude_km.powi(2)).sqrt();

            // Progress based on distance, not time (accounts for acceleration)
            let distance_progress = if horizontal_distance > 0.0 {
                (distance_traveled / horizontal_distance).clamp(0.0, 1.5)
            } else {
                1.0
            };

            // Get actual target position for terminal homing
            let (actual_target_pos, actual_target_alt) = missile_positions
                .get(&interceptor.target_id)
                .copied()
                .unwrap_or((interceptor.target_position, interceptor.target_altitude_km));

            // Terminal homing phase: blend toward actual missile position
            // Exoatmospheric interceptors (SM-3, GBI, Arrow-3) have very limited divert capability
            // Endoatmospheric interceptors (Patriot, THAAD, Iron Dome) can maneuver more aggressively
            let in_terminal = interceptor.phase == InterceptorPhase::Terminal;
            let max_terminal_blend = self.defense_configs.get(interceptor.defense_type)
                .engagement.terminal_blend_factor;

            let terminal_blend = if in_terminal {
                let time_progress = interceptor.flight_progress();
                let raw_blend = ((time_progress - 0.7) / 0.3).clamp(0.0, 1.0);
                raw_blend * max_terminal_blend
            } else {
                0.0
            };

            // Compute position along original predicted path using distance-based progress
            let predicted_pos = interpolate_great_circle(
                interceptor.launch_position,
                interceptor.target_position,
                distance_progress.min(1.0),
            );

            // Blend between predicted path and actual target for terminal homing
            let pos = if terminal_blend > 0.0 && in_terminal {
                // Proportional navigation: adjust course toward target
                GeoCoord::new(
                    predicted_pos.lat * (1.0 - terminal_blend) + actual_target_pos.lat * terminal_blend,
                    predicted_pos.lon * (1.0 - terminal_blend) + actual_target_pos.lon * terminal_blend,
                )
            } else {
                predicted_pos
            };
            interceptor.position = pos;

            // Altitude profile based on flight phase
            let altitude = match interceptor.phase {
                InterceptorPhase::Boost => {
                    // Climbing rapidly during boost
                    let boost_progress = interceptor.current_flight_time / kin.boost_duration_sec;
                    // Quadratic climb during boost (accelerating upward)
                    kin.burnout_altitude_km * boost_progress.powi(2).min(1.0)
                }
                InterceptorPhase::Coast => {
                    // Coasting toward intercept altitude
                    let coast_start = kin.burnout_altitude_km;
                    let coast_progress = (distance_progress - 0.2) / 0.5; // Normalize to coast phase
                    let coast_progress = coast_progress.clamp(0.0, 1.0);
                    // Linear interpolation from burnout to target altitude
                    coast_start + (interceptor.target_altitude_km - coast_start) * coast_progress
                }
                InterceptorPhase::Terminal => {
                    // Terminal phase - home toward actual target altitude
                    let base_alt = interceptor.target_altitude_km;
                    base_alt * (1.0 - terminal_blend) + actual_target_alt * terminal_blend
                }
            };
            interceptor.altitude_km = altitude.max(0.0);
        }
    }

    /// Launch interceptors from defense units at detected hostile missiles
    /// Uses Shoot-Look-Shoot doctrine when there's time, Shoot-Shoot-Look when not
    fn launch_interceptors_at_threats(&mut self, sim_time: f64) {
        // Collect launch decisions first to avoid borrow issues
        let mut launches: Vec<(usize, EntityId, GeoCoord, f64, bool)> = Vec::new(); // Added: is_followup flag

        // First, handle follow-up shots for targets that had a miss (Shoot-Look-Shoot)
        let followup_targets: Vec<EntityId> = self.targets_needing_followup.iter().copied().collect();
        for target_id in followup_targets {
            // Check if target is still valid
            let missile = match self.missiles.iter().find(|m| m.id == target_id) {
                Some(m) => m,
                None => {
                    self.targets_needing_followup.remove(&target_id);
                    continue;
                }
            };

            if !matches!(missile.status, MissileStatus::Boost | MissileStatus::Midcourse | MissileStatus::Terminal) {
                self.targets_needing_followup.remove(&target_id);
                continue;
            }

            // Check if we've already fired max shots at this target
            let shots_fired = self.shots_fired_per_target.get(&target_id).copied().unwrap_or(0);
            if shots_fired >= self.salvo_size {
                self.targets_needing_followup.remove(&target_id);
                continue;
            }

            // Find a defense unit that can engage
            for (unit_idx, unit) in self.defense_units.iter().enumerate() {
                if unit.interceptors_remaining == 0 {
                    continue;
                }

                let distance = haversine_distance(unit.position, missile.position);
                if distance > unit.detection_range_km() {
                    continue;
                }

                if let Some((_, intercept_pos, intercept_alt)) = self.calculate_intercept_solution(unit, missile) {
                    launches.push((unit_idx, target_id, intercept_pos, intercept_alt, true));
                    self.targets_needing_followup.remove(&target_id);
                    break;
                }
            }
        }

        // Now handle new engagements
        for (unit_idx, unit) in self.defense_units.iter().enumerate() {
            if unit.interceptors_remaining == 0 {
                continue;
            }

            let detections = self.detection.detections_for_sensor(unit.id);
            let mut unit_launches_this_cycle = 0;
            let max_launches_per_cycle = (self.salvo_size * 4).min(unit.interceptors_remaining) as usize;

            for detection in detections {
                let target_id = detection.target_id;

                if unit_launches_this_cycle >= max_launches_per_cycle {
                    break;
                }

                // Check if we already have interceptors in flight or planned for this target
                let in_flight = self.interceptors_per_target.get(&target_id).copied().unwrap_or(0);
                let already_launching = launches.iter().filter(|(_, tid, _, _, _)| *tid == target_id).count() as u32;
                let shots_fired = self.shots_fired_per_target.get(&target_id).copied().unwrap_or(0);

                // Skip if we already have shots in flight or have fired max shots
                if in_flight > 0 || already_launching > 0 || shots_fired >= self.salvo_size {
                    continue;
                }

                let missile = match self.missiles.iter().find(|m| m.id == target_id) {
                    Some(m) => m,
                    None => continue,
                };

                if missile.affiliation != Affiliation::Hostile {
                    continue;
                }
                if !matches!(missile.status, MissileStatus::Boost | MissileStatus::Midcourse | MissileStatus::Terminal) {
                    continue;
                }

                let distance = haversine_distance(unit.position, missile.position);
                if distance > unit.detection_range_km() {
                    continue;
                }

                if let Some((time_to_intercept, intercept_pos, intercept_alt)) = self.calculate_intercept_solution(unit, missile) {
                    // Determine doctrine: Shoot-Look-Shoot vs Shoot-Shoot-Look
                    // SLS requires time for: first intercept + assessment + second intercept
                    let time_to_impact = missile.flight_time - missile.current_flight_time;
                    let assessment_time = 10.0; // Time to determine hit/miss

                    // Estimate time for a second shot (roughly similar to first)
                    let second_shot_time = time_to_intercept * 0.8; // Second shot has less distance

                    let sls_total_time = time_to_intercept + assessment_time + second_shot_time;
                    let use_shoot_look_shoot = sls_total_time < time_to_impact - 5.0; // 5 second safety margin

                    // First shot
                    launches.push((unit_idx, target_id, intercept_pos, intercept_alt, false));
                    unit_launches_this_cycle += 1;

                    // Second shot only if using Shoot-Shoot-Look (not enough time for SLS)
                    if !use_shoot_look_shoot && self.salvo_size > 1
                       && unit_launches_this_cycle < max_launches_per_cycle
                       && (unit_launches_this_cycle as u32) < unit.interceptors_remaining {
                        launches.push((unit_idx, target_id, intercept_pos, intercept_alt, false));
                        unit_launches_this_cycle += 1;
                    }
                }
            }
        }

        // Execute launches - track salvo index per target for timing offsets (SSL only)
        let mut salvo_index_per_target: std::collections::HashMap<EntityId, u32> =
            std::collections::HashMap::new();

        for (unit_idx, target_id, intercept_pos, intercept_alt, is_followup) in launches {
            // Check if unit still has interceptors
            if self.defense_units[unit_idx].interceptors_remaining == 0 {
                continue;
            }

            // For SSL (multiple shots at once), add delay between shots
            // For SLS follow-up shots, launch immediately
            let launch_time = if is_followup {
                sim_time // Follow-up shots launch immediately
            } else {
                let salvo_index = *salvo_index_per_target.get(&target_id).unwrap_or(&0);
                *salvo_index_per_target.entry(target_id).or_insert(0) += 1;
                sim_time + (salvo_index as f64 * self.salvo_delay)
            };

            // Track total shots fired at this target
            *self.shots_fired_per_target.entry(target_id).or_insert(0) += 1;

            // Get ID first before any borrows
            let interceptor_id = self.new_id();

            // Extract needed values from unit
            let unit = &self.defense_units[unit_idx];
            let unit_id = unit.id;
            let affiliation = unit.affiliation;
            let defense_type = unit.defense_type;
            let position = unit.position;

            // Calculate flight time using kinematics
            let time_to_intercept =
                Self::calculate_flight_time(defense_type, position, intercept_pos, intercept_alt);

            let mut interceptor = Interceptor::new(
                interceptor_id,
                unit_id,
                target_id,
                affiliation,
                defense_type,
                position,
                intercept_pos,
                intercept_alt,
                launch_time,
                launch_time + time_to_intercept,
            );
            // Override hit probability from config
            interceptor.hit_probability = self.defense_configs.get(defense_type).engagement.hit_probability;

            self.interceptors.push(interceptor);
            *self.interceptors_per_target.entry(target_id).or_insert(0) += 1;
            self.defense_units[unit_idx].interceptors_remaining -= 1;
            self.defense_units[unit_idx].status = UnitStatus::Engaged;
        }
    }

    /// Get average interceptor speed in km/s (accounting for boost and coast phases)
    fn interceptor_avg_speed(&self, defense_type: DefenseType) -> f64 {
        self.defense_configs.get(defense_type).kinematics.average_speed_km_s
    }

    /// Calculate intercept solution using iterative method with proportional navigation
    /// Returns (intercept_time, intercept_position, intercept_altitude) or None if no solution
    fn calculate_intercept_solution(
        &self,
        unit: &DefenseUnit,
        missile: &Missile,
    ) -> Option<(f64, GeoCoord, f64)> {
        // Use the stored trajectory (created with missile config values) if available
        let trajectory = self.trajectories.iter()
            .find(|(id, _)| *id == missile.id)
            .map(|(_, t)| t.clone())
            .unwrap_or_else(|| BallisticTrajectory::new(missile.origin, missile.target));

        // Time remaining until missile impact
        let time_to_impact = missile.flight_time - missile.current_flight_time;
        if time_to_impact <= 0.0 {
            return None;
        }

        // Calculate missile's current velocity (approximate from trajectory)
        let current_progress = missile.current_flight_time / missile.flight_time;
        let (current_pos, _current_alt) = trajectory.position_at(current_progress);

        // Get position slightly ahead to estimate velocity direction
        let lookahead_progress = (current_progress + 0.01).min(0.99);
        let (lookahead_pos, _lookahead_alt) = trajectory.position_at(lookahead_progress);

        // Missile ground speed estimate (km/s)
        let missile_ground_speed = haversine_distance(current_pos, lookahead_pos) /
            (0.01 * missile.flight_time);

        // Initial guess using average interceptor speed
        let avg_speed = self.interceptor_avg_speed(unit.defense_type);
        let initial_range = haversine_distance(unit.position, missile.position);
        let closing_speed = avg_speed + missile_ground_speed * 0.5;
        let mut t_intercept = initial_range / closing_speed;

        // Iterative refinement using ACTUAL kinematic flight time calculation
        let mut best_solution: Option<(f64, GeoCoord, f64)> = None;
        let mut best_error = f64::MAX;

        for iteration in 0..20 {
            // Where will missile be at time t_intercept from now?
            let missile_progress = (missile.current_flight_time + t_intercept) / missile.flight_time;

            if missile_progress >= 0.98 {
                // Missile will have nearly impacted - try earlier intercept
                t_intercept *= 0.7;
                continue;
            }

            let (predicted_pos, predicted_alt) = trajectory.position_at(missile_progress);

            // Use the SAME flight time calculation that will be used for actual launch
            let t_required = Self::calculate_flight_time(
                unit.defense_type,
                unit.position,
                predicted_pos,
                predicted_alt,
            );

            // Calculate timing error: positive means interceptor arrives late, negative means early
            let timing_error = t_required - t_intercept;
            let abs_error = timing_error.abs();

            // Track best solution found
            let dist_to_intercept = haversine_distance(unit.position, predicted_pos);
            if abs_error < best_error {
                // Validate this solution
                let alt_ok = predicted_alt >= unit.defense_type.min_engagement_altitude_km()
                    && predicted_alt <= unit.defense_type.max_engagement_altitude_km();
                let range_ok = dist_to_intercept <= unit.engagement_range_km();
                let time_ok = t_intercept <= time_to_impact - 3.0;

                if alt_ok && range_ok && time_ok {
                    best_error = abs_error;
                    best_solution = Some((t_required, predicted_pos, predicted_alt));
                }
            }

            // Check convergence (within 1 second is good)
            if abs_error < 1.0 {
                if let Some(solution) = best_solution {
                    return Some(solution);
                }
            }

            // Adjust estimate using Newton-like iteration
            // If timing_error > 0, interceptor arrives late -> aim further ahead (increase t)
            // If timing_error < 0, interceptor arrives early -> aim closer (decrease t)
            let adjustment = if iteration < 10 {
                timing_error * 0.7 // Faster convergence early
            } else {
                timing_error * 0.4 // More conservative later
            };
            t_intercept += adjustment;

            // Bound the estimate - leave margin before impact
            let max_time = (time_to_impact - 5.0).max(5.0);
            t_intercept = t_intercept.clamp(2.0, max_time);
        }

        // Return best solution found even if not perfectly converged
        if let Some(solution) = best_solution {
            if best_error < 5.0 {
                return Some(solution);
            }
        }

        // Fallback: scan trajectory for valid intercept points
        // Start from current position and work forward
        for step in 1..20 {
            let future_time = step as f64 * 5.0; // Check every 5 seconds
            if future_time > time_to_impact - 5.0 {
                break;
            }

            let future_progress = (missile.current_flight_time + future_time) / missile.flight_time;
            if future_progress >= 0.98 {
                continue;
            }

            let (pos, alt) = trajectory.position_at(future_progress);

            // Check altitude envelope
            if alt < unit.defense_type.min_engagement_altitude_km()
                || alt > unit.defense_type.max_engagement_altitude_km()
            {
                continue;
            }

            let dist = haversine_distance(unit.position, pos);
            if dist > unit.engagement_range_km() {
                continue;
            }

            let t_to_reach = Self::calculate_flight_time(unit.defense_type, unit.position, pos, alt);

            // Interceptor must arrive before or within 5 seconds of missile
            if t_to_reach <= future_time + 5.0 {
                return Some((t_to_reach, pos, alt));
            }
        }

        None
    }

    /// Calculate flight time for interceptor using kinematics model
    fn calculate_flight_time(defense_type: DefenseType, from: GeoCoord, to: GeoCoord, target_altitude: f64) -> f64 {
        use crate::simulation::entities::InterceptorKinematics;

        let kin = InterceptorKinematics::for_defense_type(defense_type);
        let horizontal_distance = haversine_distance(from, to);
        let total_distance = (horizontal_distance.powi(2) + target_altitude.powi(2)).sqrt();

        // Calculate time accounting for boost acceleration
        let g_to_km_s2 = 0.00981;
        let acceleration = kin.boost_acceleration_g * g_to_km_s2;

        // Distance covered during boost: d = 0.5 * a * t²
        let boost_distance = 0.5 * acceleration * kin.boost_duration_sec.powi(2);

        if total_distance <= boost_distance {
            // Intercept during boost phase - solve d = 0.5 * a * t²
            (2.0 * total_distance / acceleration).sqrt()
        } else {
            // Boost phase + coast phase
            let remaining_distance = total_distance - boost_distance;
            let coast_time = remaining_distance / kin.max_velocity_km_s;
            kin.boost_duration_sec + coast_time
        }
    }

    /// Resolve intercepts - determine hit or miss based on actual proximity to assigned target only
    fn resolve_intercepts(&mut self) {
        let mut rng = rand::thread_rng();

        let mut hits: Vec<EntityId> = Vec::new();
        let mut just_resolved: Vec<EntityId> = Vec::new();

        // Build maps for quick lookup - only for the assigned target
        let missile_data: std::collections::HashMap<EntityId, (GeoCoord, f64, f64)> = self
            .missiles
            .iter()
            .map(|m| (m.id, (m.position, m.altitude_km, m.decoy_effectiveness())))
            .collect();

        // Check if missile is still a valid target (not already intercepted/impacted)
        let valid_targets: std::collections::HashSet<EntityId> = self
            .missiles
            .iter()
            .filter(|m| matches!(m.status, MissileStatus::Boost | MissileStatus::Midcourse | MissileStatus::Terminal))
            .map(|m| m.id)
            .collect();

        for interceptor in &mut self.interceptors {
            if interceptor.status != InterceptorStatus::InFlight {
                continue;
            }

            let target_id = interceptor.target_id;

            // Check if target missile is still valid
            if !valid_targets.contains(&target_id) {
                // Target already destroyed or impacted - self destruct
                interceptor.status = InterceptorStatus::SelfDestruct;
                just_resolved.push(target_id);
                continue;
            }

            // Get ONLY the assigned target's position - ignore all other missiles
            let (missile_pos, missile_alt, decoy_factor) = match missile_data.get(&target_id) {
                Some(data) => *data,
                None => {
                    interceptor.status = InterceptorStatus::SelfDestruct;
                    just_resolved.push(target_id);
                    continue;
                }
            };

            // Calculate distance between interceptor and its assigned target ONLY
            let horizontal_distance = haversine_distance(interceptor.position, missile_pos);
            let altitude_diff = (interceptor.altitude_km - missile_alt).abs();

            // 3D distance to assigned target
            let distance_3d = (horizontal_distance.powi(2) + altitude_diff.powi(2)).sqrt();

            // Kill envelope from config
            let config = self.defense_configs.get(interceptor.defense_type);
            let seeker_range_km = config.kill_envelope.seeker_range_km;
            let kill_radius_km = config.kill_envelope.kill_radius_km;
            let base_pk = config.kill_envelope.base_pk;

            // Check flight progress
            let flight_duration = interceptor.intercept_time - interceptor.launch_time;
            let progress = if flight_duration > 0.0 {
                interceptor.current_flight_time / flight_duration
            } else {
                1.0
            };

            // Terminal homing phase - actively tracking the target
            let in_terminal = progress >= 0.7;

            // Only attempt intercept against the ASSIGNED target
            if in_terminal {
                if distance_3d <= kill_radius_km {
                    // Within kill radius - attempt intercept
                    // Closer = higher hit probability
                    let proximity_factor = 1.0 - (distance_3d / kill_radius_km) * 0.3;
                    let adjusted_pk = base_pk * proximity_factor * decoy_factor;

                    let roll: f64 = rng.gen();
                    if roll < adjusted_pk {
                        interceptor.status = InterceptorStatus::Hit;
                        hits.push(target_id);
                    } else {
                        interceptor.status = InterceptorStatus::Miss;
                    }
                    just_resolved.push(target_id);
                } else if distance_3d <= seeker_range_km && progress >= 0.9 {
                    // In seeker range but not kill radius - can still attempt at reduced Pk
                    let distance_factor = 1.0 - (distance_3d - kill_radius_km) / (seeker_range_km - kill_radius_km);
                    let adjusted_pk = base_pk * 0.4 * distance_factor.max(0.0) * decoy_factor;

                    if progress >= 1.0 {
                        // Time is up - must resolve now
                        let roll: f64 = rng.gen();
                        if roll < adjusted_pk {
                            interceptor.status = InterceptorStatus::Hit;
                            hits.push(target_id);
                        } else {
                            interceptor.status = InterceptorStatus::Miss;
                        }
                        just_resolved.push(target_id);
                    }
                    // Otherwise continue homing
                } else if progress >= 1.3 {
                    // Well past intercept time and not close - definite miss
                    // This is the ONLY way to get a miss - being far from assigned target
                    interceptor.status = InterceptorStatus::Miss;
                    just_resolved.push(target_id);
                }
                // Continue homing if still in terminal phase but not resolved
            }
            // Pre-terminal: still flying toward intercept point
        }

        // Collect misses for shoot-look-shoot follow-up
        let misses: Vec<EntityId> = self.interceptors
            .iter()
            .filter(|i| i.status == InterceptorStatus::Miss)
            .map(|i| i.target_id)
            .collect();

        // Mark ONLY hit missiles as intercepted (misses should NOT stop the missile)
        for missile in &mut self.missiles {
            if hits.contains(&missile.id) {
                missile.status = MissileStatus::Intercepted;
                // Clear tracking for destroyed targets
                self.shots_fired_per_target.remove(&missile.id);
                self.targets_needing_followup.remove(&missile.id);
            }
        }

        // Queue follow-up shots for misses (Shoot-Look-Shoot doctrine)
        for target_id in &misses {
            // Only queue if we haven't fired max shots yet and target is still valid
            let shots_fired = self.shots_fired_per_target.get(target_id).copied().unwrap_or(0);
            if shots_fired < self.salvo_size {
                // Check if target is still a valid threat
                let target_still_valid = self.missiles.iter().any(|m| {
                    m.id == *target_id &&
                    matches!(m.status, MissileStatus::Boost | MissileStatus::Midcourse | MissileStatus::Terminal)
                });
                if target_still_valid {
                    self.targets_needing_followup.insert(*target_id);
                }
            }
        }

        // Update interceptor counts for resolved intercepts
        for target_id in just_resolved {
            if let Some(count) = self.interceptors_per_target.get_mut(&target_id) {
                *count = count.saturating_sub(1);
                if *count == 0 {
                    self.interceptors_per_target.remove(&target_id);
                }
            }
        }
    }

    /// Update a single missile's state
    fn update_missile(missile: &mut Missile, sim_time: f64, trajectory: Option<&BallisticTrajectory>) {
        match missile.status {
            MissileStatus::PreLaunch => {
                if sim_time >= missile.launch_time {
                    missile.status = MissileStatus::Boost;
                    missile.current_flight_time = 0.0;
                }
            }
            MissileStatus::Boost | MissileStatus::Midcourse | MissileStatus::Terminal => {
                missile.current_flight_time = sim_time - missile.launch_time;

                let progress = missile.flight_progress();

                // Update status based on flight phase
                if progress >= 1.0 {
                    missile.status = MissileStatus::Impacted;
                    missile.position = missile.target;
                    missile.altitude_km = 0.0;
                } else if let Some(traj) = trajectory {
                    // Update position using stored trajectory (created with missile config values)
                    let (pos, alt) = traj.position_at(progress);
                    missile.position = pos;
                    missile.altitude_km = alt;

                    // Update phase
                    match traj.phase_at(progress) {
                        FlightPhase::Boost => missile.status = MissileStatus::Boost,
                        FlightPhase::Midcourse => missile.status = MissileStatus::Midcourse,
                        FlightPhase::Terminal => missile.status = MissileStatus::Terminal,
                        _ => {}
                    }
                }
            }
            MissileStatus::Intercepted | MissileStatus::Impacted => {
                // No update needed
            }
        }
    }

    /// Get trajectory for a missile
    pub fn get_trajectory(&self, missile_id: EntityId) -> Option<&BallisticTrajectory> {
        self.trajectories
            .iter()
            .find(|(id, _)| *id == missile_id)
            .map(|(_, t)| t)
    }

    /// Set the time scale
    pub fn set_time_scale(&mut self, scale: TimeScale) {
        self.time_scale = scale;
    }

    /// Toggle between paused and previous speed (or real-time)
    pub fn toggle_pause(&mut self) {
        if self.time_scale == TimeScale::Paused {
            self.time_scale = TimeScale::Fast;
        } else {
            self.time_scale = TimeScale::Paused;
        }
    }

    /// Reset the simulation
    pub fn reset(&mut self) {
        self.sim_time = 0.0;
        self.missiles.clear();
        self.defense_units.clear();
        self.satellites.clear();
        self.radar_stations.clear();
        self.interceptors.clear();
        self.trajectories.clear();
        self.interceptors_per_target.clear();
        self.targets_needing_followup.clear();
        self.shots_fired_per_target.clear();
        self.detection = DetectionSystem::new();
        self.next_id = 1;
    }

    /// Format simulation time as HH:MM:SS
    pub fn format_time(&self) -> String {
        let total_secs = self.sim_time as u64;
        let hours = total_secs / 3600;
        let minutes = (total_secs % 3600) / 60;
        let seconds = total_secs % 60;
        format!("{:02}:{:02}:{:02}", hours, minutes, seconds)
    }

    /// Create a sample scenario for testing
    pub fn load_sample_scenario(&mut self) {
        self.reset();

        // Add some defense units
        self.add_defense_unit(
            "THAAD Battery Alpha".to_string(),
            Affiliation::Friendly,
            GeoCoord::new(37.5, -122.0), // California
            DefenseType::THAAD,
            48,
        );

        self.add_defense_unit(
            "Aegis Cruiser Tokyo".to_string(),
            Affiliation::Friendly,
            GeoCoord::new(35.0, 140.0), // Japan coast
            DefenseType::Aegis,
            96,
        );

        // Aegis ship in Sea of Japan - positioned to detect NK launches
        self.add_defense_unit(
            "Aegis Cruiser Sea of Japan".to_string(),
            Affiliation::Friendly,
            GeoCoord::new(38.0, 132.0), // Sea of Japan
            DefenseType::Aegis,
            96,
        );

        self.add_defense_unit(
            "GBI Site".to_string(),
            Affiliation::Friendly,
            GeoCoord::new(64.0, -146.0), // Alaska
            DefenseType::GBI,
            44,
        );

        // Add a radar station
        self.add_radar_station(
            "Early Warning Radar".to_string(),
            Affiliation::Friendly,
            GeoCoord::new(71.0, -156.0), // Alaska north
            2000.0,
        );

        // Add satellites - positioned for global coverage
        self.add_satellite(
            "SBIRS GEO-1".to_string(),
            Affiliation::Friendly,
            GeoCoord::new(0.0, -100.0), // Over Central Pacific
            35786.0, // Geostationary
            SensorType::Infrared,
        );

        // Satellite with view of East Asia
        self.add_satellite(
            "SBIRS GEO-2".to_string(),
            Affiliation::Friendly,
            GeoCoord::new(0.0, 120.0), // Over Western Pacific/East Asia
            35786.0,
            SensorType::Infrared,
        );

        // Add some missiles (will launch at different times)
        // ICBM with countermeasures (decoys)
        self.add_missile_with_countermeasures(
            "Hostile ICBM 1 (CM)".to_string(),
            Affiliation::Hostile,
            GeoCoord::new(39.0, 125.5), // North Korea
            GeoCoord::new(37.5, -122.0), // San Francisco
            60.0, // Launch at T+60s
            5,    // 5 decoys
        );

        // ICBM without countermeasures
        self.add_missile(
            "Hostile ICBM 2".to_string(),
            Affiliation::Hostile,
            GeoCoord::new(39.0, 125.5), // North Korea
            GeoCoord::new(47.6, -122.3), // Seattle
            120.0, // Launch at T+120s
        );

        // MRBM with countermeasures
        self.add_missile_with_countermeasures(
            "Hostile MRBM (CM)".to_string(),
            Affiliation::Hostile,
            GeoCoord::new(35.0, 51.0), // Iran
            GeoCoord::new(32.0, 34.8), // Tel Aviv
            30.0,  // Launch at T+30s
            3,     // 3 decoys
        );
    }
}

impl Default for SimulationEngine {
    fn default() -> Self {
        Self::new()
    }
}
