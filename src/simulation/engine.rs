use crate::simulation::config::{
    InterceptorConfig, InterceptorConfigRegistry, MissileConfigRegistry, MissileType,
    PhysicsConfig, PkWeights, PlatformConfig, PlatformConfigRegistry, SatelliteConfigRegistry,
    SensorConfigRegistry,
};
use crate::simulation::detection::calculate_position_from_bearing_range;
use crate::simulation::detection::DetectionSystem;
use crate::simulation::entities::*;
use crate::simulation::physics::{
    bearing, haversine_distance, lambert_guidance, BallisticTrajectory, FlightPhase,
};
use crate::simulation::SimEventType;
use crate::types::GeoCoord;
use rand::Rng;
use rayon::prelude::*;
use std::collections::HashMap;
use std::path::Path;

/// Time scale options for simulation speed
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum TimeScale {
    Paused,
    RealTime,  // 1x
    Fast,      // 10x
    VeryFast,  // 60x (1 minute per second)
    UltraFast, // 300x (5 minutes per second)
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

/// Debris cloud from a successful intercept - can damage other interceptors
#[derive(Clone, Debug)]
pub struct DebrisCloud {
    /// Position of the debris cloud center
    pub position: GeoCoord,
    /// Altitude of the debris cloud center in km
    pub altitude_km: f64,
    /// Time of intercept that created the debris
    pub creation_time: f64,
    /// Initial radius of debris cloud in km
    pub initial_radius_km: f64,
    /// Expansion rate in km/s
    pub expansion_rate_km_s: f64,
    /// Duration the cloud remains hazardous in seconds
    pub hazard_duration_sec: f64,
}

impl DebrisCloud {
    /// Calculate current radius based on time since creation
    pub fn current_radius_km(&self, current_time: f64) -> f64 {
        let elapsed = current_time - self.creation_time;
        self.initial_radius_km + self.expansion_rate_km_s * elapsed
    }

    /// Check if debris cloud is still hazardous
    pub fn is_hazardous(&self, current_time: f64) -> bool {
        current_time - self.creation_time < self.hazard_duration_sec
    }

    /// Check if a point is within the debris cloud
    pub fn contains(&self, pos: GeoCoord, alt_km: f64, current_time: f64) -> bool {
        if !self.is_hazardous(current_time) {
            return false;
        }
        let horiz_dist = haversine_distance(self.position, pos);
        let vert_dist = (alt_km - self.altitude_km).abs();
        let dist_3d = (horiz_dist.powi(2) + vert_dist.powi(2)).sqrt();
        dist_3d < self.current_radius_km(current_time)
    }
}

/// Kill assessment record for shoot-look-shoot doctrine
#[derive(Clone, Debug)]
pub struct KillAssessment {
    /// The target missile being assessed
    pub target_id: EntityId,
    /// Time of the intercept attempt
    pub intercept_time: f64,
    /// Time when assessment will be complete (intercept_time + assessment_delay)
    pub assessment_complete_time: f64,
    /// Whether the assessment is complete
    pub assessment_complete: bool,
    /// Whether the intercept was successful (only valid if assessment_complete)
    pub was_kill: bool,
}

impl KillAssessment {
    /// Assessment delay in seconds (typical radar confirmation time)
    pub const ASSESSMENT_DELAY_SEC: f64 = 3.0;

    pub fn new(target_id: EntityId, intercept_time: f64, was_kill: bool) -> Self {
        Self {
            target_id,
            intercept_time,
            assessment_complete_time: intercept_time + Self::ASSESSMENT_DELAY_SEC,
            assessment_complete: false,
            was_kill,
        }
    }

    /// Update assessment status based on current time
    pub fn update(&mut self, current_time: f64) {
        if !self.assessment_complete && current_time >= self.assessment_complete_time {
            self.assessment_complete = true;
        }
    }
}

// ============================================================================
// Pk Calculation Utilities (Weighted Log-Odds)
// ============================================================================

/// Convert probability to log-odds (logit)
/// logit(p) = ln(p / (1-p))
fn logit(p: f64) -> f64 {
    let p = p.clamp(0.001, 0.999); // Avoid infinity
    (p / (1.0 - p)).ln()
}

/// Convert log-odds to probability (sigmoid)
/// sigmoid(x) = 1 / (1 + e^(-x))
fn sigmoid(x: f64) -> f64 {
    1.0 / (1.0 + (-x).exp())
}

/// Calculate Pk using weighted log-odds approach
///
/// This avoids the harsh compounding of multiplicative factors.
/// Instead of pk = base * f1 * f2 * f3, we use:
///   penalty = Σ(weight_i * (1 - factor_i)) / total_weight * severity
///   final_pk = sigmoid(logit(base_pk) - penalty)
///
/// # Arguments
/// * `base_pk` - Base probability of kill (from interceptor config)
/// * `factors` - Slice of (factor_value, weight) pairs where factor_value is 0.0-1.0
/// * `severity_scale` - Overall penalty magnitude in logit space
///
/// # Returns
/// Final probability of kill, clamped to [0.01, 0.99]
fn calculate_pk_weighted(
    base_pk: f64,
    factors: &[(f64, f64)], // (value, weight)
    severity_scale: f64,
) -> f64 {
    let base_logit = logit(base_pk);

    let total_weight: f64 = factors.iter().map(|(_, w)| *w).sum();
    if total_weight <= 0.0 {
        return base_pk;
    }

    // Calculate weighted penalty
    // Each factor's penalty is proportional to its deviation from ideal (1.0)
    let mut total_penalty = 0.0;
    for (factor, weight) in factors {
        let deviation = 1.0 - factor.clamp(0.0, 1.0);
        let normalized_weight = weight / total_weight;
        total_penalty += normalized_weight * deviation * severity_scale;
    }

    let final_logit = base_logit - total_penalty;
    sigmoid(final_logit).clamp(0.01, 0.99)
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
    /// Cached trajectories for missiles (HashMap for O(1) lookup)
    trajectories: HashMap<EntityId, BallisticTrajectory>,
    /// Detection system for tracking sensors and targets
    pub detection: DetectionSystem,
    /// Track interceptors in flight per target (for salvo fire)
    interceptors_per_target: std::collections::HashMap<EntityId, u32>,
    /// Maximum interceptors to fire per target (salvo size)
    pub salvo_size: u32,
    /// Delay between interceptor launches in a salvo (seconds)
    pub salvo_delay: f64,
    /// Maximum TOTAL shots per target across the whole engagement.
    /// Previously the per-salvo size doubled as a total-shot cap (2 for the
    /// entire flight) — one wasted salvo ended the engagement even with
    /// interceptors and time remaining. Doctrine-configurable.
    pub max_shots_per_target: u32,
    /// Targets that need follow-up shots after a miss (for shoot-look-shoot)
    targets_needing_followup: std::collections::HashSet<EntityId>,
    /// Track total shots fired at each target (to enforce salvo_size limit)
    shots_fired_per_target: std::collections::HashMap<EntityId, u32>,
    /// Missile configurations loaded from TOML files
    pub missile_configs: MissileConfigRegistry,
    /// Sensor configurations loaded from TOML files
    pub sensor_configs: SensorConfigRegistry,
    /// Platform/launcher configurations loaded from TOML files
    pub platform_configs: PlatformConfigRegistry,
    /// Interceptor configurations loaded from TOML files
    pub interceptor_configs: InterceptorConfigRegistry,
    /// Satellite configurations loaded from TOML files
    pub satellite_configs: SatelliteConfigRegistry,
    /// Use Kalman-filtered estimates for intercept calculations instead of ground truth
    /// Legacy flag — all intercept solutions are sensor-derived now
    /// (converged trajectory projection). Kept for API compatibility.
    pub use_filtered_intercepts: bool,
    /// Weights for Pk factor contributions using weighted log-odds calculation
    pub pk_weights: PkWeights,
    /// Physics simulation settings (sub-stepping for precision)
    pub physics_config: PhysicsConfig,
    /// Pending events to be consumed by the event tracker
    pub pending_events: Vec<(f64, SimEventType)>,
    /// Debris clouds from successful intercepts
    pub debris_clouds: Vec<DebrisCloud>,
    /// Kill assessments in progress (shoot-look-shoot doctrine)
    pub kill_assessments: Vec<KillAssessment>,
}

impl SimulationEngine {
    pub fn new() -> Self {
        // Try to load configs from config directory, fall back to defaults
        let missile_configs =
            MissileConfigRegistry::load(Path::new("config")).unwrap_or_else(|e| {
                eprintln!(
                    "Warning: Failed to load missile configs: {}, using defaults",
                    e
                );
                MissileConfigRegistry::with_defaults()
            });

        let sensor_configs = SensorConfigRegistry::load(Path::new("config")).unwrap_or_else(|e| {
            eprintln!(
                "Warning: Failed to load sensor configs: {}, using defaults",
                e
            );
            SensorConfigRegistry::with_defaults()
        });

        let platform_configs =
            PlatformConfigRegistry::load(Path::new("config")).unwrap_or_else(|e| {
                eprintln!(
                    "Warning: Failed to load platform configs: {}, using defaults",
                    e
                );
                PlatformConfigRegistry::with_defaults()
            });

        let interceptor_configs = InterceptorConfigRegistry::load(Path::new("config"))
            .unwrap_or_else(|e| {
                eprintln!(
                    "Warning: Failed to load interceptor configs: {}, using defaults",
                    e
                );
                InterceptorConfigRegistry::with_defaults()
            });

        let satellite_configs =
            SatelliteConfigRegistry::load(Path::new("config")).unwrap_or_else(|e| {
                eprintln!(
                    "Warning: Failed to load satellite configs: {}, using defaults",
                    e
                );
                SatelliteConfigRegistry::with_defaults()
            });

        // Load Pk weights from simulation.toml (falls back to defaults if not found)
        let pk_weights = PkWeights::load(Path::new("config"));

        // Load physics config from simulation.toml (sub-stepping for precision)
        let physics_config = PhysicsConfig::load(Path::new("config"));

        Self {
            sim_time: 0.0,
            time_scale: TimeScale::Paused,
            next_id: 1,
            missiles: Vec::new(),
            defense_units: Vec::new(),
            satellites: Vec::new(),
            radar_stations: Vec::new(),
            interceptors: Vec::new(),
            trajectories: HashMap::new(),
            detection: DetectionSystem::new(),
            interceptors_per_target: std::collections::HashMap::new(),
            salvo_size: 2,           // Default: fire 2 interceptors per target
            salvo_delay: 5.0,        // 5 seconds between interceptor launches in a salvo
            max_shots_per_target: 4, // Total shots per target (SLS can keep firing)
            targets_needing_followup: std::collections::HashSet::new(),
            shots_fired_per_target: std::collections::HashMap::new(),
            missile_configs,
            sensor_configs,
            platform_configs,
            interceptor_configs,
            satellite_configs,
            use_filtered_intercepts: true, // Enable Kalman-filtered intercept calculations using sensor track data
            pk_weights,
            physics_config,
            pending_events: Vec::new(),
            debris_clouds: Vec::new(),
            kill_assessments: Vec::new(),
        }
    }

    /// Drain pending events for external consumption
    pub fn drain_events(&mut self) -> Vec<(f64, SimEventType)> {
        std::mem::take(&mut self.pending_events)
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
        _missile_type: MissileType,
    ) -> EntityId {
        let id = self.new_id();
        let range_km = haversine_distance(origin, target);

        // Get trajectory parameters from config (by name first, fallback to type)
        let config = self.missile_configs.get_by_name(&name);
        let apogee =
            config.trajectory.apogee_base_km + range_km * config.trajectory.apogee_range_factor;
        let flight_time = config.trajectory.flight_time_base_sec
            + range_km * config.trajectory.flight_time_range_factor;

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

        // Apply radar signature from config
        missile.rcs_boost_dbsm = config.radar_signature.rcs_boost_dbsm;
        missile.rcs_midcourse_dbsm = config.radar_signature.rcs_midcourse_dbsm;
        missile.rcs_terminal_dbsm = config.radar_signature.rcs_terminal_dbsm;

        self.missiles.push(missile);
        self.trajectories.insert(id, trajectory);

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
            name,
            affiliation,
            origin,
            target,
            launch_time,
            max_decoys,
            missile_type,
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
        _missile_type: MissileType,
    ) -> EntityId {
        let id = self.new_id();
        let range_km = haversine_distance(origin, target);

        // Get trajectory parameters from config (by name first, fallback to type)
        let config = self.missile_configs.get_by_name(&name);
        let apogee =
            config.trajectory.apogee_base_km + range_km * config.trajectory.apogee_range_factor;
        let flight_time = config.trajectory.flight_time_base_sec
            + range_km * config.trajectory.flight_time_range_factor;

        let trajectory = BallisticTrajectory::with_params(origin, target, apogee, flight_time);

        let mut missile = Missile::new(id, name, affiliation, origin, target, flight_time)
            .with_countermeasures(max_decoys);
        missile.launch_time = launch_time;
        missile.missile_type = config.classification.missile_type;

        // Apply radar signature from config
        missile.rcs_boost_dbsm = config.radar_signature.rcs_boost_dbsm;
        missile.rcs_midcourse_dbsm = config.radar_signature.rcs_midcourse_dbsm;
        missile.rcs_terminal_dbsm = config.radar_signature.rcs_terminal_dbsm;

        self.missiles.push(missile);
        self.trajectories.insert(id, trajectory);

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
        let mut unit =
            DefenseUnit::new(id, name, affiliation, position, defense_type, interceptors);

        // Initialize multi-sensor support from platform config
        let platform_config = self.platform_configs.get_by_defense_type(defense_type);
        let platform_sensors = platform_config.launcher.get_sensors();

        // Update sensor_config_name from platform config (for legacy single-sensor code paths)
        if !platform_config.launcher.sensor_config_name.is_empty() {
            unit.sensor_config_name = platform_config.launcher.sensor_config_name.clone();
        } else if let Some(first_sensor) = platform_sensors.first() {
            unit.sensor_config_name = first_sensor.config_name.clone();
        }

        // Create runtime sensor instances with unique IDs
        for sensor_config in platform_sensors {
            let sensor_id = self.new_id();

            // Get sensor detection config to determine coverage
            let detection_config = self.sensor_configs.get_by_name(&sensor_config.config_name);

            let azimuth_coverage = sensor_config
                .azimuth_coverage_override_deg
                .unwrap_or(detection_config.detection.azimuth_coverage_deg);

            unit.sensors.push(DefenseUnitSensor {
                sensor_id,
                config_name: sensor_config.config_name.clone(),
                role: sensor_config.role,
                azimuth_center_deg: sensor_config.azimuth_center_deg,
                azimuth_coverage_deg: azimuth_coverage,
            });
        }

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

    /// Add a radar station with explicit sensor configuration
    pub fn add_radar_station_with_config(
        &mut self,
        name: String,
        affiliation: Affiliation,
        position: GeoCoord,
        detection_range_km: f64,
        sensor_config: Option<String>,
        facing_deg: Option<f64>,
    ) -> EntityId {
        let id = self.new_id();
        let mut station = RadarStation::new(id, name, affiliation, position, detection_range_km);

        // Apply explicit sensor config if provided
        if let Some(config_name) = sensor_config {
            station.sensor_config_name = config_name;
        }

        // Apply facing direction if provided
        if let Some(facing) = facing_deg {
            station.facing_deg = facing;
        }

        // Look up sensor config to set azimuth coverage
        let sensor_config = self.sensor_configs.get_by_name(&station.sensor_config_name);
        station.azimuth_coverage_deg = sensor_config.detection.azimuth_coverage_deg;

        self.radar_stations.push(station);
        id
    }

    /// Update the simulation by a real-time delta (in seconds)
    pub fn update(&mut self, real_dt: f64) {
        let sim_dt = real_dt * self.time_scale.multiplier();

        if sim_dt == 0.0 {
            return;
        }

        // Determine how many sub-steps we need based on interceptor proximity
        let sub_steps = self.determine_sub_steps();
        let sub_dt = sim_dt / sub_steps as f64;

        // Run physics sub-steps for precision intercept calculations
        for step in 0..sub_steps {
            // Advance simulation time for this sub-step
            self.sim_time += sub_dt;
            let sim_time = self.sim_time;

            // Update all missiles in parallel (need accurate positions for intercept calculations)
            let trajectories = &self.trajectories;
            self.missiles.par_iter_mut().for_each(|missile| {
                let trajectory = trajectories.get(&missile.id);
                Self::update_missile(missile, sim_time, trajectory);
            });

            // Update interceptors in flight with sub-step precision
            self.update_interceptors(sim_time, sub_dt);

            // Check for intercept completions at each sub-step
            // This is critical for precise hit detection
            self.resolve_intercepts();

            // Only do these once per frame (first sub-step), not every sub-step
            if step == 0 {
                // Deploy decoys for missiles being tracked
                self.deploy_missile_decoys();

                // Update detection system (doesn't need sub-step precision)
                self.detection.update(
                    &self.missiles,
                    &self.defense_units,
                    &self.radar_stations,
                    &self.satellites,
                    &self.interceptors,
                    &self.sensor_configs,
                    sim_dt, // Full frame dt for detection
                    sim_time,
                );

                // Update defense unit status based on detections
                for unit in &mut self.defense_units {
                    let has_detections = self.detection.detections_for_sensor(unit.id).len() > 0;

                    unit.status = if has_detections {
                        UnitStatus::Tracking
                    } else {
                        UnitStatus::Idle
                    };
                }

                // Launch interceptors at detected threats
                self.launch_interceptors_at_threats(sim_time);
            }
        }
    }

    /// Determine how many physics sub-steps to use based on interceptor proximity
    /// Uses full precision when interceptors are close to targets, minimum otherwise
    fn determine_sub_steps(&self) -> u32 {
        let config = &self.physics_config;

        // Check if any interceptor is within precision distance of its target
        let needs_precision = self.interceptors.iter().any(|interceptor| {
            if interceptor.status != InterceptorStatus::InFlight {
                return false;
            }

            // Find the target missile
            if let Some(missile) = self.missiles.iter().find(|m| m.id == interceptor.target_id) {
                let horiz_dist = haversine_distance(interceptor.position, missile.position);
                let vert_dist = (interceptor.altitude_km - missile.altitude_km).abs();
                let distance = (horiz_dist.powi(2) + vert_dist.powi(2)).sqrt();

                distance < config.precision_distance_km
            } else {
                false
            }
        });

        if needs_precision {
            config.sub_steps
        } else {
            config.min_sub_steps
        }
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
    fn update_interceptors(&mut self, sim_time: f64, sim_dt: f64) {
        use crate::simulation::entities::{InterceptorKinematics, InterceptorPhase};

        // Launch pending interceptors whose time has come
        for interceptor in &mut self.interceptors {
            if interceptor.status == InterceptorStatus::Pending
                && sim_time >= interceptor.launch_time
            {
                interceptor.status = InterceptorStatus::InFlight;
            }
        }

        // Collect current missile positions for terminal homing
        let missile_positions: std::collections::HashMap<EntityId, (GeoCoord, f64)> = self
            .missiles
            .iter()
            .filter(|m| {
                matches!(
                    m.status,
                    MissileStatus::Boost | MissileStatus::Midcourse | MissileStatus::Terminal
                )
            })
            .map(|m| (m.id, (m.position, m.altitude_km)))
            .collect();

        // Collect active debris clouds for interference checking
        let active_debris: Vec<_> = self
            .debris_clouds
            .iter()
            .filter(|dc| dc.is_hazardous(sim_time))
            .map(|dc| (dc.position, dc.altitude_km, dc.current_radius_km(sim_time)))
            .collect();

        // First pass: collect defense types to look up configs (avoiding borrow checker issues)
        let interceptor_configs_cache: Vec<_> = self
            .interceptors
            .iter()
            .map(|i| {
                let config = self.get_interceptor_config(i.defense_type);
                (
                    i.defense_type,
                    config.engagement.terminal_blend_factor,
                    config.engagement.midcourse_guidance.clone(),
                    config.kill_envelope.seeker_range_km,
                )
            })
            .collect();

        // ========================================================================
        // MID-COURSE GUIDANCE UPDATE PASS
        // Ground/ship radars send updated intercept solutions during coast phase
        // ========================================================================

        // Pre-collect defense unit IDs for track lookup
        let defense_unit_ids: std::collections::HashSet<EntityId> =
            self.defense_units.iter().map(|u| u.id).collect();

        // Collect updated intercept solutions for interceptors that need guidance updates
        // Mid-course guidance uses SENSOR-FUSED TRACK DATA (not ground truth) for realism
        // Result: Ok((pos, alt, correction_km)) for success, Err(reason) for blocked
        let guidance_results: Vec<Result<(GeoCoord, f64, f64), Option<String>>> = self
            .interceptors
            .iter()
            .enumerate()
            .map(|(idx, interceptor)| {
                // Only update interceptors that are in flight
                if interceptor.status != InterceptorStatus::InFlight {
                    return Err(None); // Don't log - not in flight
                }

                // Allow updates during Coast phase, or late Boost phase (after 15% of flight)
                let early_boost = interceptor.phase == InterceptorPhase::Boost
                    && interceptor.flight_progress() < 0.15;
                if early_boost {
                    return Err(None); // Don't log - early boost is expected
                }

                // In terminal phase: only stop guidance if seeker has acquired target
                // If seeker hasn't acquired, continue midcourse guidance to correct track errors
                // This allows large prediction errors to be corrected before seeker handoff
                if interceptor.phase == InterceptorPhase::Terminal && interceptor.seeker_acquired {
                    return Err(None); // Don't log - seeker has taken over as expected
                }

                // Get mid-course guidance config
                let (_, _, ref guidance_config, _) = interceptor_configs_cache[idx];
                if !guidance_config.enabled {
                    return Err(None); // Don't log - guidance disabled by config
                }

                // Check if enough time has passed since last update
                let time_since_update = sim_time - interceptor.last_guidance_update_time;
                if time_since_update < guidance_config.update_interval_sec {
                    return Err(None); // Don't log - just waiting for next update interval
                }

                // REQUIRE sensor-fused track data for mid-course guidance
                // This is the fire control system's best estimate of target position/velocity
                let fused_track = match self.detection.get_fused_track(
                    interceptor.target_id,
                    &defense_unit_ids,
                    sim_time,
                ) {
                    Some(track) => track,
                    None => return Err(Some("No fused track available".to_string())),
                };

                // Log track quality info
                let has_fc_lock = fused_track.has_fire_control_lock;
                let track_quality = fused_track.fused_quality;
                let sensor_count = fused_track.sensor_count;

                // Calculate time remaining until current predicted intercept
                let remaining_flight_time = interceptor.intercept_time - sim_time;
                if remaining_flight_time <= 2.0 {
                    return Err(None); // Don't log - close to intercept is expected
                }

                // SENSOR-DERIVED TARGET PROJECTION (realism requirement):
                // Mid-course guidance projects the target through the converged
                // trajectory estimated from radar measurements — never the
                // ground-truth trajectory. Previously this read
                // self.trajectories (omniscient), which violated the project's
                // "sensor data only" fire control rule.
                let (converged, _est_flight_time) = match self
                    .detection
                    .get_converged_trajectory(interceptor.target_id)
                {
                    Some(ct) => ct,
                    None => return Err(Some("No converged trajectory estimate yet".to_string())),
                };

                // Rebuild the profile the fit says the threat flies
                let trajectory = BallisticTrajectory::with_params(
                    converged.origin,
                    converged.target,
                    converged.apogee_km,
                    converged.flight_time_sec,
                );

                // Current progress from the fused track position along the
                // estimated path (constant ground speed model)
                let along_track =
                    haversine_distance(converged.origin, fused_track.estimated_position);
                let current_progress =
                    (along_track / trajectory.range_km.max(1.0)).clamp(0.0, 0.99);

                // Get interceptor kinematics for timing calculations
                let kin = InterceptorKinematics::for_defense_type(interceptor.defense_type);

                // ITERATIVE TIME-SYNCHRONIZED INTERCEPT SOLUTION
                // Find a time T where both interceptor and missile arrive at
                // the same point on the sensor-derived profile.
                let mut time_to_intercept = remaining_flight_time;
                let mut best_pos = fused_track.estimated_position;
                let mut best_alt = fused_track.estimated_altitude;

                for _iteration in 0..5 {
                    // Project missile position at current time estimate
                    let future_progress = (current_progress
                        + time_to_intercept / converged.flight_time_sec.max(1.0))
                    .clamp(0.0, 0.98);
                    let (projected_pos, projected_alt) = trajectory.position_at(future_progress);

                    // Calculate how long it would take interceptor to reach this position
                    let h_dist = haversine_distance(interceptor.position, projected_pos);
                    let v_dist = (interceptor.altitude_km - projected_alt).abs();
                    let dist_3d = (h_dist.powi(2) + v_dist.powi(2)).sqrt();

                    let interceptor_time_to_point = kin
                        .time_to_cover_distance(dist_3d, interceptor.current_flight_time, {
                            // Endo systems coasting below 100 km lose speed to drag
                            let endo = matches!(
                                interceptor.defense_type,
                                DefenseType::Patriot
                                    | DefenseType::THAAD
                                    | DefenseType::IronDome
                                    | DefenseType::DavidsSling
                            );
                            endo && interceptor.altitude_km < 100.0
                        })
                        .unwrap_or(dist_3d / kin.max_velocity_km_s.max(0.5));

                    // Update best solution
                    best_pos = projected_pos;
                    best_alt = projected_alt;

                    // Check for convergence (times match within 1 second)
                    let time_diff = (interceptor_time_to_point - time_to_intercept).abs();
                    if time_diff < 1.0 {
                        break; // Converged - times are synchronized
                    }

                    // Adjust time estimate: average of current and calculated time
                    // This helps convergence
                    time_to_intercept = (time_to_intercept + interceptor_time_to_point) / 2.0;

                    // Bound the time estimate to reasonable values
                    time_to_intercept = time_to_intercept.clamp(2.0, remaining_flight_time * 2.0);
                }

                // Check if correction is significant enough to warrant an update
                let horizontal_diff = haversine_distance(interceptor.target_position, best_pos);
                let vertical_diff = (interceptor.target_altitude_km - best_alt).abs();
                let total_diff = (horizontal_diff.powi(2) + vertical_diff.powi(2)).sqrt();

                // Apply correction if difference exceeds threshold
                if total_diff < 0.5 {
                    return Err(None); // Don't log - small correction not needed
                }

                // Return the time-synchronized intercept point
                Ok((best_pos, best_alt, total_diff))
            })
            .collect();

        // Convert to Option for the apply loop, extracting just the success cases
        let guidance_updates: Vec<Option<(GeoCoord, f64, f64)>> = guidance_results
            .iter()
            .map(|r| r.as_ref().ok().copied())
            .collect();

        // Collect interceptor/target info for event logging (before mutable borrow)
        let interceptor_info: Vec<(String, String, EntityId)> = self
            .interceptors
            .iter()
            .map(|i| {
                let target_name = self
                    .missiles
                    .iter()
                    .find(|m| m.id == i.target_id)
                    .map(|m| m.name.clone())
                    .unwrap_or_else(|| "Unknown".to_string());
                (i.defense_type.name().to_string(), target_name, i.target_id)
            })
            .collect();

        // Log guidance blocked events (only log once per target to avoid spam)
        let mut logged_blocked_targets: std::collections::HashSet<EntityId> =
            std::collections::HashSet::new();
        for (idx, result) in guidance_results.iter().enumerate() {
            if let Err(Some(reason)) = result {
                let (interceptor_type, target_name, target_id) = &interceptor_info[idx];
                // Only log once per target to reduce spam
                if !logged_blocked_targets.contains(target_id) {
                    logged_blocked_targets.insert(*target_id);
                    eprintln!(
                        "[GUIDANCE BLOCKED] {} -> {}: {}",
                        interceptor_type, target_name, reason
                    );
                    self.pending_events.push((
                        sim_time,
                        SimEventType::GuidanceBlocked {
                            interceptor_type: interceptor_type.clone(),
                            target: target_name.clone(),
                            reason: reason.clone(),
                        },
                    ));
                }
            }
        }

        // Apply guidance updates and log success events
        // Note: Pk will be recalculated in the Pk pass below using the updated target_position,
        // so improved guidance leads to improved Pk (better prediction_factor and timing_factor)
        for (idx, interceptor) in self.interceptors.iter_mut().enumerate() {
            if let Some((new_pos, new_alt, correction_km)) = guidance_updates[idx] {
                // Cap maximum correction based on remaining distance to intercept
                // Early in flight: allow larger corrections since track is still being refined
                // Later in flight: require smaller corrections for precision
                // Reference: Real BMD systems allow substantial mid-course corrections
                let remaining_distance = {
                    let h_dist =
                        haversine_distance(interceptor.position, interceptor.target_position);
                    let v_dist = (interceptor.altitude_km - interceptor.target_altitude_km).abs();
                    (h_dist.powi(2) + v_dist.powi(2)).sqrt()
                };

                // Scale max correction based on flight phase:
                // Early in flight: Allow unlimited corrections as track converges
                // Mid-flight: Moderate corrections as track refines
                // Terminal: Smaller corrections - seeker handles fine adjustments
                //
                // Note: Track quality was already checked when computing guidance_updates,
                // so if we have an update, the track was good enough to trust.
                //
                // IMPORTANT: Initial intercept may use ground truth while guidance uses EKF,
                // causing large discrepancies (1000+ km). Allow unlimited early corrections.
                let max_correction = if interceptor.guidance_updates_count < 10 {
                    // Early updates - NO LIMIT
                    // Initial intercept may have used ground truth, guidance uses EKF
                    // Must allow full correction to sync with filtered predictions
                    f64::MAX
                } else if remaining_distance > 200.0 {
                    // Far - allow corrections up to 100% of remaining distance
                    remaining_distance * 1.0
                } else if remaining_distance > 100.0 {
                    // Medium range - allow 80% corrections
                    (remaining_distance * 0.8).min(300.0)
                } else if remaining_distance > 50.0 {
                    // Close - precision phase, 50% corrections
                    (remaining_distance * 0.5).min(100.0)
                } else {
                    // Terminal - seeker handles fine adjustments
                    25.0
                };

                if correction_km > max_correction {
                    eprintln!(
                        "[GUIDANCE BLOCKED] Correction {:.1}km > max {:.1}km (remaining: {:.0}km)",
                        correction_km, max_correction, remaining_distance
                    );
                    continue;
                }

                interceptor.target_position = new_pos;
                interceptor.target_altitude_km = new_alt;
                // Divert budget tracking disabled for now
                // interceptor.divert_budget_remaining_km -= divert_used;
                // interceptor.total_divert_used_km += divert_used;
                interceptor.last_guidance_update_time = sim_time;
                interceptor.guidance_updates_count += 1;

                // Log successful guidance update
                let (interceptor_type, target_name, _) = &interceptor_info[idx];
                eprintln!(
                    "[GUIDANCE UPDATE] {} -> {}: correction={:.1}km, updates=#{}",
                    interceptor_type,
                    target_name,
                    correction_km,
                    interceptor.guidance_updates_count
                );
                self.pending_events.push((
                    sim_time,
                    SimEventType::GuidanceUpdate {
                        interceptor_type: interceptor_type.clone(),
                        target: target_name.clone(),
                        correction_km,
                        update_count: interceptor.guidance_updates_count,
                    },
                ));

                // Recalculate intercept time based on new target position
                // Uses the SAME unified arrival-time function as launch and
                // guidance (boost-aware, drag-aware) so all three agree.
                let kin = InterceptorKinematics::for_defense_type(interceptor.defense_type);
                let distance_remaining = {
                    let h_dist = haversine_distance(interceptor.position, new_pos);
                    let v_dist = (interceptor.altitude_km - new_alt).abs();
                    (h_dist.powi(2) + v_dist.powi(2)).sqrt()
                };

                let endo_coasting = matches!(
                    interceptor.defense_type,
                    DefenseType::Patriot
                        | DefenseType::THAAD
                        | DefenseType::IronDome
                        | DefenseType::DavidsSling
                ) && interceptor.altitude_km < 100.0;

                let time_remaining = kin
                    .time_to_cover_distance(
                        distance_remaining,
                        interceptor.current_flight_time,
                        endo_coasting,
                    )
                    .unwrap_or(distance_remaining / kin.max_velocity_km_s.max(0.5));

                interceptor.intercept_time = sim_time + time_remaining;
            }
        }

        for (idx, interceptor) in self.interceptors.iter_mut().enumerate() {
            if interceptor.status != InterceptorStatus::InFlight {
                continue;
            }

            interceptor.current_flight_time = sim_time - interceptor.launch_time;

            // Update kinematics (velocity, phase) — dt needed for cumulative drag
            interceptor.update_kinematics(sim_dt);

            let kin = InterceptorKinematics::for_defense_type(interceptor.defense_type);
            let _flight_duration = interceptor.intercept_time - interceptor.launch_time;

            // Actual distance traveled from the stateful kinematics integration
            // (reflects cumulative drag, unlike the ideal kin.distance_at_time)
            let distance_traveled = interceptor.distance_traveled_km;
            let total_distance =
                haversine_distance(interceptor.launch_position, interceptor.target_position);
            let horizontal_distance =
                (total_distance.powi(2) + interceptor.target_altitude_km.powi(2)).sqrt();

            // NOTE: The "skid maneuver" (perpendicular divert to bleed speed and
            // fix early-arrival timing) has been REMOVED. Exo-atmospheric
            // interceptors cannot bleed speed in vacuum, and with unified
            // arrival-time math (Phase 1.1) early arrival is corrected by
            // mid-course guidance re-solving the intercept point instead.

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

            // Calculate seeker off-boresight angle (angle between flight direction and target)
            // Seeker can only acquire targets within its gimbal limits
            // Use ACTUAL flight direction (from previous to current position), not the fixed launch-to-predicted line
            let prev_to_current_dist =
                haversine_distance(interceptor.previous_position, interceptor.position);
            let flight_heading = if prev_to_current_dist > 0.001 {
                bearing(interceptor.previous_position, interceptor.position)
            } else {
                // Fallback if no movement yet - use launch to target
                bearing(interceptor.launch_position, interceptor.target_position)
            };
            let target_bearing = bearing(interceptor.position, actual_target_pos);

            // Calculate angle difference (0° = on boresight, 180° = directly behind)
            // Use rem_euclid to handle negative numbers correctly (Rust's % preserves sign)
            let diff = (target_bearing - flight_heading).rem_euclid(360.0);
            let bearing_diff = if diff > 180.0 { 360.0 - diff } else { diff };
            interceptor.off_boresight_angle_deg = bearing_diff;

            // Check if target is within seeker gimbal limits
            let within_gimbal =
                interceptor.off_boresight_angle_deg <= interceptor.seeker_gimbal_limit_deg;

            // Seeker acquisition: requires target within gimbal limits AND
            // within seeker acquisition range during terminal phase.
            // Previously there was no range gate at all — seekers "acquired"
            // at 170-190 km against 50-80 km published seeker ranges.
            // 1.5x margin accounts for closure geometry (target moving toward
            // the seeker extends effective acquisition distance).
            let seeker_range_limit = interceptor_configs_cache[idx].3 * 1.5;
            let dist_to_actual_target = {
                let h = haversine_distance(interceptor.position, actual_target_pos);
                let v = (interceptor.altitude_km - actual_target_alt).abs();
                (h.powi(2) + v.powi(2)).sqrt()
            };
            let within_seeker_range = dist_to_actual_target <= seeker_range_limit;

            // Seeker acquisition: requires target within gimbal limits during terminal phase
            // Once acquired, seeker tracks the target (can lose lock if target leaves gimbal)
            if interceptor.phase == InterceptorPhase::Terminal {
                if within_gimbal && within_seeker_range && !interceptor.seeker_acquired {
                    // Start acquisition timer
                    if interceptor.seeker_acquisition_time == 0.0 {
                        interceptor.seeker_acquisition_time = sim_time;
                    }
                    // Seeker acquires after ~0.5 seconds of tracking
                    let acquisition_delay = 0.5;
                    if sim_time - interceptor.seeker_acquisition_time >= acquisition_delay {
                        interceptor.seeker_acquired = true;
                        eprintln!("[SEEKER ACQUIRED] {:?} acquired target at {:.1}km (limit {:.1}km), off-boresight={:.1}°",
                                 interceptor.defense_type,
                                 haversine_distance(interceptor.position, actual_target_pos),
                                 seeker_range_limit,
                                 interceptor.off_boresight_angle_deg);
                    }
                } else if !within_seeker_range && interceptor.seeker_acquired {
                    // Target out of seeker range — lost lock
                    eprintln!(
                        "[SEEKER LOST LOCK] {:?} lost target, range {:.1}km > {:.1}km",
                        interceptor.defense_type, dist_to_actual_target, seeker_range_limit
                    );
                    interceptor.seeker_acquired = false;
                    interceptor.seeker_acquisition_time = 0.0;
                } else if !within_gimbal && interceptor.seeker_acquired {
                    // Lost lock - target outside gimbal limits
                    // In reality, seeker has some hysteresis before losing lock
                    if interceptor.off_boresight_angle_deg
                        > interceptor.seeker_gimbal_limit_deg * 1.2
                    {
                        eprintln!(
                            "[SEEKER LOST LOCK] {:?} lost target, off-boresight={:.1}° > {:.1}°",
                            interceptor.defense_type,
                            interceptor.off_boresight_angle_deg,
                            interceptor.seeker_gimbal_limit_deg * 1.2
                        );
                        interceptor.seeker_acquired = false;
                        interceptor.seeker_acquisition_time = 0.0;
                    }
                } else if (!within_gimbal || !within_seeker_range) && !interceptor.seeker_acquired {
                    // Reset acquisition timer if target leaves gimbal/range before lock
                    interceptor.seeker_acquisition_time = 0.0;
                }
            }

            // INCREMENTAL POSITION UPDATE
            // Move from current position toward current intercept point (updated by mid-course guidance)
            // This ensures the interceptor always steers toward the latest calculated merge point

            // Calculate the horizontal component of velocity using the INITIAL trajectory geometry
            // The interceptor velocity is the 3D velocity (used for flight time calculation)
            // The climb angle is fixed based on launch-to-target, not current-to-target
            // This ensures position update is consistent with flight time calculation
            let total_horiz_dist =
                haversine_distance(interceptor.launch_position, interceptor.target_position);
            let total_alt_change = interceptor.target_altitude_km; // From sea level (0km)
            let total_3d_dist = (total_horiz_dist.powi(2) + total_alt_change.powi(2)).sqrt();

            // Horizontal velocity component: v_h = v_total × cos(climb_angle)
            // cos(climb_angle) = horizontal_distance / 3d_distance
            let horizontal_velocity_factor = if total_3d_dist > 0.1 {
                total_horiz_dist / total_3d_dist
            } else {
                1.0 // Vertical intercept (unlikely)
            };

            // Debug velocity factor - only log periodically to avoid spam
            // Remove this debug code later

            // Use actual simulation dt for position update (accounts for time scaling)
            let horizontal_velocity =
                interceptor.current_velocity_km_s * horizontal_velocity_factor;
            let distance_this_frame = horizontal_velocity * sim_dt;

            // Determine which target to fly toward:
            // - Pre-terminal: fly toward current intercept point (target_position, updated by mid-course guidance)
            // - Terminal with seeker: fly toward actual missile position (actual_target_pos)
            let in_terminal = interceptor.phase == InterceptorPhase::Terminal;
            let _max_terminal_blend = interceptor_configs_cache[idx].1;

            // Calculate guidance target for terminal homing
            let (guidance_target, terminal_lead_altitude) = if in_terminal
                && interceptor.seeker_acquired
            {
                // Lead calculation uses the SENSOR-DERIVED velocity estimate
                // (fused track, uplinked to the interceptor) — a seeker measures
                // Doppler/LOS rate, never the target's true aimpoint.
                let defense_unit_ids_term: std::collections::HashSet<EntityId> =
                    self.defense_units.iter().map(|u| u.id).collect();
                let track_velocity = self
                    .detection
                    .get_fused_track(interceptor.target_id, &defense_unit_ids_term, sim_time)
                    .and_then(|ft| ft.estimated_velocity.clone());

                if let Some(velocity) = track_velocity {
                    let h_dist = haversine_distance(interceptor.position, actual_target_pos);
                    let v_dist = (interceptor.altitude_km - actual_target_alt).abs();
                    let dist_3d = (h_dist.powi(2) + v_dist.powi(2)).sqrt();

                    // Closing velocity from geometry and the measured velocities
                    let missile_heading = velocity.heading_deg;
                    let missile_to_interceptor = bearing(actual_target_pos, interceptor.position);
                    let approach_angle =
                        ((missile_heading - missile_to_interceptor + 180.0) % 360.0 - 180.0).abs();
                    let closure_component = approach_angle.to_radians().cos();
                    let closing_velocity = (interceptor.current_velocity_km_s
                        + velocity.ground_speed_km_s * closure_component)
                        .max(0.5);

                    let time_to_intercept = dist_3d / closing_velocity;

                    // Conservative lead: only lead by 30% of what pure geometry suggests
                    let lead_factor = 0.3;
                    let lead_distance =
                        velocity.ground_speed_km_s * time_to_intercept * lead_factor;

                    // Project missile position forward along the measured heading
                    let lead_pos = calculate_position_from_bearing_range(
                        actual_target_pos,
                        missile_heading,
                        lead_distance,
                    );

                    (lead_pos, Some(actual_target_alt))
                } else {
                    (actual_target_pos, Some(actual_target_alt))
                }
            } else {
                // Pre-terminal or no seeker lock: fly toward predicted intercept point
                (interceptor.target_position, None)
            };

            // CPA CHECK IN GUIDANCE: If we've passed closest point of approach, stop steering
            // This prevents flip-flopping behavior where interceptor turns around after missing
            let current_distance_to_target = {
                let h = haversine_distance(interceptor.position, actual_target_pos);
                let v = (interceptor.altitude_km - actual_target_alt).abs();
                (h.powi(2) + v.powi(2)).sqrt()
            };

            // Detect CPA: distance increasing consistently.
            // Use hysteresis to avoid false CPA triggers from momentary distance
            // fluctuations (terminal altitude corrections, PN heading changes).
            // Runs regardless of seeker state: resolve_intercepts now consumes
            // ONLY this confirmed flag, so blind flybys must also detect CPA
            // to resolve as misses instead of flying on to the 1.2x timeout.
            if in_terminal && !interceptor.passed_cpa {
                // Tolerance: distance must increase by at least 0.1km to count as "increasing"
                let distance_tolerance_km = 0.1;

                if interceptor.previous_distance_to_target_km < f64::MAX {
                    if current_distance_to_target
                        > interceptor.previous_distance_to_target_km + distance_tolerance_km
                    {
                        // Distance is increasing - increment counter
                        interceptor.cpa_increasing_frames += 1;

                        // Require 10 consecutive frames of increasing distance before declaring CPA
                        // At high closing speeds, this is ~0.1-0.2 seconds of consistent divergence
                        const CPA_HYSTERESIS_FRAMES: u32 = 10;

                        if interceptor.cpa_increasing_frames >= CPA_HYSTERESIS_FRAMES {
                            // CPA confirmed - set flag permanently and store current heading
                            interceptor.passed_cpa = true;
                            // Store the heading toward the guidance target
                            interceptor.heading_at_cpa =
                                bearing(interceptor.position, guidance_target);
                            eprintln!("[CPA] {:?} passed CPA at {:.3}km from target, locking heading={:.1}° (after {} frames)",
                                     interceptor.defense_type, interceptor.previous_distance_to_target_km,
                                     interceptor.heading_at_cpa, interceptor.cpa_increasing_frames);
                        }
                    } else if current_distance_to_target
                        < interceptor.previous_distance_to_target_km - distance_tolerance_km
                    {
                        // Distance is decreasing - reset counter (still closing)
                        interceptor.cpa_increasing_frames = 0;
                    }
                    // If distance is within tolerance, don't change counter (neutral)
                }
                // Update distance tracking for next frame
                interceptor.previous_distance_to_target_km = current_distance_to_target;
            }

            // TERMINAL GUIDANCE
            // Uses different algorithms based on altitude regime:
            // - Exo-atmospheric (>100km): Lambert guidance for optimal transfer trajectory
            // - Endo-atmospheric: Proportional Navigation (PN)
            // Reference: Zarchan, "Tactical and Strategic Missile Guidance"
            //            Vallado, "Fundamentals of Astrodynamics and Applications"

            // Check if this is an exo-atmospheric system operating above Kármán line
            let is_exo_system = matches!(
                interceptor.defense_type,
                DefenseType::GBI | DefenseType::Aegis | DefenseType::Arrow3
            );
            let is_exo_altitude = interceptor.altitude_km > 100.0;

            let heading_to_target = if interceptor.passed_cpa {
                // We've passed CPA - maintain the exact heading we had when CPA was detected
                // Don't recalculate - use the stored value to prevent flip-flopping
                interceptor.heading_at_cpa
            } else if is_exo_system && is_exo_altitude && !in_terminal {
                // LAMBERT GUIDANCE for exo-atmospheric midcourse
                // Computes optimal trajectory to intercept point using orbital mechanics
                let time_to_intercept =
                    interceptor.intercept_time - interceptor.current_flight_time;
                if time_to_intercept > 1.0 {
                    if let Some((heading, _climb, _speed)) = lambert_guidance(
                        interceptor.position,
                        interceptor.altitude_km,
                        interceptor.target_position,
                        interceptor.target_altitude_km,
                        time_to_intercept,
                    ) {
                        heading
                    } else {
                        // Lambert solver didn't converge - fall back to direct pursuit
                        bearing(interceptor.position, guidance_target)
                    }
                } else {
                    // Too close to intercept - use direct pursuit
                    bearing(interceptor.position, guidance_target)
                }
            } else if in_terminal
                && interceptor.seeker_acquired
                && interceptor.los_rate_rad_s.abs() > 1e-6
            {
                // PROPORTIONAL NAVIGATION for terminal phase with seeker lock
                // PN Law: heading_rate = N * LOS_rate
                // Get navigation constant from config (typical N=3-5 for missiles)
                let guidance_config = &interceptor_configs_cache[idx].2;
                let nav_constant = guidance_config.navigation_constant;

                // Current flight heading (from velocity vector)
                let current_heading_rad =
                    bearing(interceptor.previous_position, interceptor.position).to_radians();

                // PN commanded heading rate (rad/s)
                let commanded_heading_rate = nav_constant * interceptor.los_rate_rad_s;

                // Limit heading rate based on interceptor maneuverability
                // Max turn rate depends on g-capability and velocity
                let kin = InterceptorKinematics::for_defense_type(interceptor.defense_type);
                let max_lateral_accel = kin.terminal_maneuver_g * 0.00981; // km/s²
                let max_heading_rate =
                    max_lateral_accel / interceptor.current_velocity_km_s.max(0.1);
                let limited_heading_rate =
                    commanded_heading_rate.clamp(-max_heading_rate, max_heading_rate);

                // Apply heading change
                let new_heading_rad = current_heading_rad + limited_heading_rate * sim_dt;

                // Convert back to degrees and normalize to [0, 360)
                let new_heading_deg = new_heading_rad.to_degrees().rem_euclid(360.0);

                new_heading_deg
            } else {
                // Pre-terminal or no seeker lock: fly toward predicted intercept point (lead pursuit)
                bearing(interceptor.position, guidance_target)
            };

            let pos = calculate_position_from_bearing_range(
                interceptor.position,
                heading_to_target,
                distance_this_frame,
            );

            // Update LOS tracking for diagnostics
            if in_terminal && interceptor.seeker_acquired {
                let delta_lat =
                    actual_target_pos.lat.to_radians() - interceptor.position.lat.to_radians();
                let delta_lon =
                    actual_target_pos.lon.to_radians() - interceptor.position.lon.to_radians();
                let current_los_angle = delta_lon.atan2(delta_lat);

                if interceptor.last_los_angle_rad != 0.0 && sim_dt > 0.0 {
                    let angle_diff = current_los_angle - interceptor.last_los_angle_rad;
                    let normalized_diff = if angle_diff > std::f64::consts::PI {
                        angle_diff - 2.0 * std::f64::consts::PI
                    } else if angle_diff < -std::f64::consts::PI {
                        angle_diff + 2.0 * std::f64::consts::PI
                    } else {
                        angle_diff
                    };
                    interceptor.los_rate_rad_s = normalized_diff / sim_dt;
                }
                interceptor.last_los_angle_rad = current_los_angle;
            }

            // Track previous position for PN calculations
            interceptor.previous_position = interceptor.position;
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
                    // Terminal phase: actively track target altitude for hit-to-kill
                    if interceptor.seeker_acquired {
                        // Use lead altitude (predicted missile altitude at intercept)
                        let target_alt = terminal_lead_altitude.unwrap_or(actual_target_alt);

                        // Calculate altitude error
                        let alt_error = target_alt - interceptor.altitude_km;

                        // Calculate time to intercept based on horizontal distance and closing speed
                        let h_dist = haversine_distance(interceptor.position, actual_target_pos);
                        let closing_speed = interceptor.current_velocity_km_s * 1.5; // Approximate closing speed
                        let time_to_target = (h_dist / closing_speed.max(0.5)).max(0.01);

                        // Required vertical velocity to reach target altitude at intercept time
                        let required_vert_vel = alt_error / time_to_target;

                        // Maximum vertical velocity component (can redirect ~30% of speed vertically)
                        let max_vert_vel = interceptor.current_velocity_km_s * 0.5;

                        // Clamp to achievable vertical velocity
                        let actual_vert_vel = required_vert_vel.clamp(-max_vert_vel, max_vert_vel);

                        // Apply altitude change
                        let alt_change = actual_vert_vel * sim_dt;
                        (interceptor.altitude_km + alt_change).max(0.0)
                    } else {
                        // Seeker not acquired - continue toward predicted intercept altitude
                        interceptor.target_altitude_km
                    }
                }
            };
            interceptor.altitude_km = altitude.max(0.0);

            // Energy management: track interceptor's remaining maneuver capability
            // Energy state starts at 1.0 (full) and decreases with:
            // - Time (fuel consumption for attitude control)
            // - Maneuvers (divert fuel consumption)
            // - Aerodynamic drag (endoatmospheric systems)
            let divert_used_fraction =
                interceptor.total_maneuver_delta_v_used / interceptor.max_maneuver_delta_v.max(0.1);
            let time_decay = 1.0 - (interceptor.current_flight_time / 120.0).min(0.2); // Gradual decay over 2 minutes

            // Exoatmospheric systems: energy = divert fuel remaining
            // Endoatmospheric systems: some energy recovery through aerodynamic lift
            let is_exoatmospheric = matches!(
                interceptor.defense_type,
                DefenseType::Aegis | DefenseType::GBI | DefenseType::Arrow3
            );

            if is_exoatmospheric {
                // Purely fuel-based energy - no recovery
                interceptor.energy_state =
                    ((1.0 - divert_used_fraction) * time_decay).clamp(0.0, 1.0);
            } else {
                // Endoatmospheric: aerodynamic lift helps, but drag hurts
                let altitude_factor = if altitude < 20.0 {
                    0.95 // Dense atmosphere - good lift but high drag
                } else if altitude < 40.0 {
                    1.0 // Sweet spot
                } else {
                    0.9 + (60.0 - altitude) / 200.0 // Thin air - less lift
                };
                interceptor.energy_state =
                    ((1.0 - divert_used_fraction * 0.8) * time_decay * altitude_factor)
                        .clamp(0.0, 1.0);
            }

            // Check for debris cloud interference
            // If interceptor passes through debris from a previous intercept, it may be damaged
            for (debris_pos, debris_alt, debris_radius) in &active_debris {
                let horiz_dist = haversine_distance(interceptor.position, *debris_pos);
                let vert_dist = (interceptor.altitude_km - debris_alt).abs();
                let dist_3d = (horiz_dist.powi(2) + vert_dist.powi(2)).sqrt();

                if dist_3d < *debris_radius {
                    // Interceptor passed through debris cloud - probabilistic damage
                    // Probability based on how deep into the cloud
                    let penetration_depth = 1.0 - (dist_3d / debris_radius);
                    let damage_probability = penetration_depth * 0.3; // 30% max damage chance at center

                    if rand::thread_rng().gen::<f64>() < damage_probability {
                        interceptor.status = InterceptorStatus::Miss;
                        interceptor.miss_reason = MissReason::DebrisDamage;
                        // Could add an event here for debris damage
                        break;
                    }
                }
            }
        }

        // Second pass: update Pk for each interceptor (needs to be separate to avoid borrow issues)
        // Pk is recalculated AFTER mid-course guidance updates, so improved guidance
        // (updated target_position closer to actual missile path) improves Pk
        //
        // Pk calculation using WEIGHTED LOG-ODDS approach
        // Instead of multiplicative (pk = base * f1 * f2 * ...), we use:
        //   penalty = Σ(weight_i * (1 - factor_i)) / total_weight * severity
        //   final_pk = sigmoid(logit(base_pk) - penalty)
        //
        // This prevents harsh compounding and allows tuning factor importance.
        //
        // Factors (0.0-1.0 where 1.0 = ideal):
        // - Aspect angle: head-on (180°) is best, gives seeker longest look time
        // - Track quality: better track = interceptor pointed more accurately
        // - Closure speed: higher = less time for seeker to acquire and maneuver
        // - Energy state: fuel/thruster capacity for terminal corrections
        // - Countermeasures: decoys degrade tracking
        // - Prediction error: how close intercept point is to missile's future path
        // - Timing sync: interceptor and missile must arrive at intercept point together
        let pk_weights = &self.pk_weights;
        let pk_data: Vec<_> = self
            .interceptors
            .iter()
            .filter(|i| i.status == InterceptorStatus::InFlight)
            .map(|i| {
                let target = self.missiles.iter().find(|m| m.id == i.target_id);
                let track_quality = self
                    .detection
                    .active_tracks
                    .iter()
                    .filter(|t| t.target_id == i.target_id)
                    .map(|t| t.track_quality)
                    .max_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal))
                    .unwrap_or(0.5);

                // Get base Pk from interceptor config (ideal conditions)
                let config = self.get_interceptor_config(i.defense_type);
                let base_pk = config.kill_envelope.base_pk;

                let pk = if let Some(missile) = target {
                    // Sensor-derived state for Pk factors: fire control assesses
                    // geometry from the FUSED TRACK (position/velocity), never
                    // from the missile's true origin/target/flight clock.
                    let defense_unit_ids_pk: std::collections::HashSet<EntityId> =
                        self.defense_units.iter().map(|u| u.id).collect();
                    let ft_state = self.detection.get_fused_track(
                        i.target_id,
                        &defense_unit_ids_pk,
                        self.sim_time,
                    );
                    let ft_heading = ft_state
                        .as_ref()
                        .and_then(|ft| ft.estimated_velocity.as_ref().map(|v| v.heading_deg));
                    let ft_vert_rate = ft_state.as_ref().and_then(|ft| {
                        ft.estimated_velocity.as_ref().map(|v| v.vertical_rate_km_s)
                    });
                    let ft_speed = ft_state
                        .as_ref()
                        .and_then(|ft| ft.estimated_velocity.as_ref().map(|v| v.ground_speed_km_s));

                    // 1. ASPECT ANGLE FACTOR (3D geometry)
                    // Best case: head-on intercept (vectors are opposite, 180° apart)
                    // Worst case: tail chase, perpendicular crossing, or high crossing angle

                    // Horizontal aspect angle (heading from the fused velocity estimate;
                    // fall back to launch-to-intercept geometry when no velocity yet)
                    let interceptor_heading = bearing(i.launch_position, i.target_position);
                    let missile_heading =
                        ft_heading.unwrap_or_else(|| bearing(missile.origin, missile.target));
                    let h_angle_diff = ((interceptor_heading - missile_heading).abs() % 360.0)
                        .min(360.0 - ((interceptor_heading - missile_heading).abs() % 360.0));

                    // Vertical crossing factor (diving/climbing) — from the
                    // fused vertical rate when available
                    let interceptor_climb_rate = {
                        let h_dist = haversine_distance(i.launch_position, i.target_position);
                        let v_dist = i.target_altitude_km;
                        (v_dist / h_dist.max(0.1)).atan().to_degrees()
                    };
                    let missile_dive_rate = {
                        // Convert fused vertical rate to a flight-path angle estimate:
                        // dive angle ~ atan(vertical rate / ground speed)
                        match (ft_vert_rate, ft_speed) {
                            (Some(vz), Some(vg)) if vg > 0.05 => -(vz / vg).atan().to_degrees(),
                            _ => {
                                // Fallback: nominal midcourse descent profile
                                let progress = missile.flight_progress();
                                if progress < 0.3 {
                                    30.0 - progress * 50.0
                                } else if progress < 0.5 {
                                    15.0 - (progress - 0.3) * 75.0
                                } else {
                                    -15.0 - (progress - 0.5) * 30.0
                                }
                            }
                        }
                    };

                    let v_angle_diff = (interceptor_climb_rate - missile_dive_rate).abs();

                    // Horizontal: Head-on (180°) is best
                    let h_factor = if h_angle_diff >= 150.0 {
                        1.0
                    } else if h_angle_diff >= 90.0 {
                        0.85 + (h_angle_diff - 90.0) / 60.0 * 0.15
                    } else if h_angle_diff >= 45.0 {
                        0.7 + (h_angle_diff - 45.0) / 45.0 * 0.15
                    } else {
                        0.5 + h_angle_diff / 45.0 * 0.2
                    };

                    // Vertical: steep crossing angles are harder
                    let v_factor = if v_angle_diff <= 20.0 {
                        1.0
                    } else if v_angle_diff <= 40.0 {
                        0.95 - (v_angle_diff - 20.0) / 20.0 * 0.1
                    } else if v_angle_diff <= 60.0 {
                        0.85 - (v_angle_diff - 40.0) / 20.0 * 0.15
                    } else {
                        (0.7 - (v_angle_diff - 60.0) / 30.0 * 0.2).max(0.5)
                    };

                    // Combined aspect factor (weight horizontal more heavily)
                    let aspect_factor = h_factor * 0.7 + v_factor * 0.3;

                    // 2. TRACK QUALITY FACTOR
                    let track_factor = 0.6 + track_quality * 0.4; // Range: 0.6 to 1.0

                    // 3. CLOSURE SPEED FACTOR — from the fused track speed estimate
                    let closure_speed = i.current_velocity_km_s + ft_speed.unwrap_or(3.0);
                    let closure_factor = if closure_speed <= 4.0 {
                        1.0
                    } else if closure_speed <= 8.0 {
                        1.0 - (closure_speed - 4.0) / 4.0 * 0.15
                    } else if closure_speed <= 12.0 {
                        0.85 - (closure_speed - 8.0) / 4.0 * 0.15
                    } else {
                        0.7 - ((closure_speed - 12.0) / 8.0 * 0.2).min(0.2)
                    };

                    // 4. ENERGY STATE FACTOR
                    let energy_factor = if i.energy_state >= 0.5 {
                        1.0
                    } else if i.energy_state >= 0.2 {
                        0.9 + i.energy_state * 0.2
                    } else {
                        0.7 + i.energy_state * 1.0
                    };

                    // 5. COUNTERMEASURE FACTOR
                    let countermeasure_factor = missile.decoy_effectiveness();

                    // 6. PREDICTION ERROR FACTOR (sensor-derived)
                    // Compares the planned intercept point against the CONVERGED
                    // trajectory projection (sensor-derived), not ground truth.
                    let dist_to_intercept = haversine_distance(i.position, i.target_position);
                    let alt_to_intercept = (i.target_altitude_km - i.altitude_km).abs();
                    let dist_3d = (dist_to_intercept.powi(2) + alt_to_intercept.powi(2)).sqrt();
                    let time_to_intercept = if i.current_velocity_km_s > 0.0 {
                        dist_3d / i.current_velocity_km_s
                    } else {
                        10.0
                    };

                    // Sensor-derived trajectory projection; if no converged
                    // estimate exists the prediction error is unknowable —
                    // use a conservative mid-range factor.
                    let (prediction_factor, timing_factor) = match self
                        .detection
                        .get_converged_trajectory(i.target_id)
                    {
                        Some((converged, _)) => {
                            let trajectory = BallisticTrajectory::with_params(
                                converged.origin,
                                converged.target,
                                converged.apogee_km,
                                converged.flight_time_sec,
                            );
                            // Current progress from the interceptor's own view:
                            // fused position projected on the estimated path
                            let defense_unit_ids: std::collections::HashSet<EntityId> =
                                self.defense_units.iter().map(|u| u.id).collect();
                            let prog = self
                                .detection
                                .get_fused_track(i.target_id, &defense_unit_ids, self.sim_time)
                                .map(|ft| {
                                    (haversine_distance(converged.origin, ft.estimated_position)
                                        / trajectory.range_km.max(1.0))
                                    .clamp(0.0, 0.99)
                                })
                                .unwrap_or(0.5);

                            let future_progress = (prog
                                + time_to_intercept / converged.flight_time_sec.max(1.0))
                            .clamp(0.0, 0.99);
                            let (predicted_pos, predicted_alt) =
                                trajectory.position_at(future_progress);

                            let horizontal_error =
                                haversine_distance(i.target_position, predicted_pos);
                            let vertical_error = (i.target_altitude_km - predicted_alt).abs();
                            let total_error =
                                (horizontal_error.powi(2) + vertical_error.powi(2)).sqrt();
                            let seeker_range = config.kill_envelope.seeker_range_km;

                            let prediction_factor = if total_error <= seeker_range * 0.5 {
                                1.0
                            } else if total_error <= seeker_range {
                                0.95 - (total_error - seeker_range * 0.5) / (seeker_range * 0.5)
                                    * 0.1
                            } else if total_error <= seeker_range * 2.0 {
                                0.85 - (total_error - seeker_range) / seeker_range * 0.25
                            } else {
                                (0.6 - (total_error - seeker_range * 2.0) / seeker_range * 0.3)
                                    .max(0.2)
                            };

                            // 7. TIMING SYNCHRONIZATION FACTOR
                            // Time for the missile to reach the planned intercept
                            // point along the sensor-derived profile
                            let missile_time_to_intercept =
                                ((future_progress - prog) * converged.flight_time_sec).max(0.0);

                            let timing_diff = time_to_intercept - missile_time_to_intercept;
                            let timing_margin_sec =
                                (seeker_range / closure_speed.max(1.0)).max(1.0).min(5.0);

                            let timing_factor = if timing_diff.abs() <= timing_margin_sec {
                                1.0
                            } else if timing_diff.abs() <= timing_margin_sec * 2.0 {
                                1.0 - (timing_diff.abs() - timing_margin_sec) / timing_margin_sec
                            } else {
                                0.0 // Too far off - intercept impossible
                            };

                            (prediction_factor, timing_factor)
                        }
                        None => (0.5, 0.5), // Unknown prediction quality — neutral-low
                    };

                    // Combine all factors using weighted log-odds
                    let factors = [
                        (timing_factor, pk_weights.timing_sync),
                        (track_factor, pk_weights.track_quality),
                        (prediction_factor, pk_weights.prediction_error),
                        (countermeasure_factor, pk_weights.countermeasures),
                        (closure_factor, pk_weights.closure_speed),
                        (aspect_factor, pk_weights.aspect_angle),
                        (energy_factor, pk_weights.energy_state),
                    ];

                    calculate_pk_weighted(base_pk, &factors, pk_weights.severity_scale)
                } else {
                    // Target lost - use a very low factor for all categories
                    let factors = [
                        (0.1, pk_weights.timing_sync),
                        (0.1, pk_weights.track_quality),
                        (0.1, pk_weights.prediction_error),
                        (1.0, pk_weights.countermeasures),
                        (1.0, pk_weights.closure_speed),
                        (1.0, pk_weights.aspect_angle),
                        (1.0, pk_weights.energy_state),
                    ];
                    calculate_pk_weighted(base_pk, &factors, pk_weights.severity_scale)
                };

                (i.id, pk)
            })
            .collect();

        // Apply Pk updates
        for (id, pk) in pk_data {
            if let Some(interceptor) = self.interceptors.iter_mut().find(|i| i.id == id) {
                interceptor.hit_probability = pk;
            }
        }
    }

    /// Launch interceptors from defense units at detected hostile missiles
    /// Uses Shoot-Look-Shoot doctrine when there's time, Shoot-Shoot-Look when not
    fn launch_interceptors_at_threats(&mut self, sim_time: f64) {
        // Collect launch decisions first to avoid borrow issues
        let mut launches: Vec<(usize, EntityId, GeoCoord, f64, bool)> = Vec::new(); // Added: is_followup flag

        // First, handle follow-up shots for targets that had a miss (Shoot-Look-Shoot)
        let followup_targets: Vec<EntityId> =
            self.targets_needing_followup.iter().copied().collect();
        for target_id in followup_targets {
            // Check if target is still valid
            let missile = match self.missiles.iter().find(|m| m.id == target_id) {
                Some(m) => m,
                None => {
                    self.targets_needing_followup.remove(&target_id);
                    continue;
                }
            };

            if !matches!(
                missile.status,
                MissileStatus::Boost | MissileStatus::Midcourse | MissileStatus::Terminal
            ) {
                self.targets_needing_followup.remove(&target_id);
                continue;
            }

            // Check if we've already fired max shots at this target
            let shots_fired = self
                .shots_fired_per_target
                .get(&target_id)
                .copied()
                .unwrap_or(0);
            if shots_fired >= self.max_shots_per_target {
                self.targets_needing_followup.remove(&target_id);
                continue;
            }

            // Find a defense unit that can engage
            for (unit_idx, unit) in self.defense_units.iter().enumerate() {
                if unit.interceptors_remaining == 0 {
                    continue;
                }

                // Fire control range gate — measured to the FUSED TRACK position
                // (sensor-derived), never the missile's true position.
                let defense_unit_ids_f: std::collections::HashSet<EntityId> =
                    self.defense_units.iter().map(|u| u.id).collect();
                let fused_pos =
                    match self
                        .detection
                        .get_fused_track(target_id, &defense_unit_ids_f, sim_time)
                    {
                        Some(ft) => ft.estimated_position,
                        None => break, // No sensor track — no unit can engage
                    };

                let distance = haversine_distance(unit.position, fused_pos);
                let sensor_config = self.sensor_configs.get_by_name(&unit.sensor_config_name);
                let fire_control_range = sensor_config.detection.detection_range_km
                    * sensor_config.detection.fire_control_range_multiplier;
                if distance > fire_control_range {
                    continue;
                }

                if let Some((_, intercept_pos, intercept_alt, _uncertainty)) =
                    self.calculate_intercept_solution_from_track(unit, target_id)
                {
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

            // Collect detections from all of this unit's sensors
            let detections: Vec<_> = if !unit.sensors.is_empty() {
                // Multi-sensor platform: gather detections from all sensors
                unit.sensors
                    .iter()
                    .flat_map(|s| self.detection.detections_for_sensor(s.sensor_id))
                    .collect()
            } else {
                // Legacy single-sensor: use unit.id (fallback)
                self.detection.detections_for_sensor(unit.id)
            };
            let mut unit_launches_this_cycle = 0;
            let max_launches_per_cycle =
                (self.salvo_size * 4).min(unit.interceptors_remaining) as usize;

            for detection in detections {
                let target_id = detection.target_id;

                // Skip false alarms - they're not real targets
                if detection.is_false_alarm {
                    continue;
                }

                // Verify we have a valid track (not just a detection)
                // Without a stable track, we can't calculate intercept
                if !self.detection.is_target_tracked(target_id) {
                    continue;
                }

                if unit_launches_this_cycle >= max_launches_per_cycle {
                    break;
                }

                // Check if we already have interceptors in flight or planned for this target
                let in_flight = self
                    .interceptors_per_target
                    .get(&target_id)
                    .copied()
                    .unwrap_or(0);
                let already_launching = launches
                    .iter()
                    .filter(|(_, tid, _, _, _)| *tid == target_id)
                    .count() as u32;
                let shots_fired = self
                    .shots_fired_per_target
                    .get(&target_id)
                    .copied()
                    .unwrap_or(0);

                // Skip if we already have shots in flight or have fired max shots
                if in_flight > 0
                    || already_launching > 0
                    || shots_fired >= self.max_shots_per_target
                {
                    continue;
                }

                let missile = match self.missiles.iter().find(|m| m.id == target_id) {
                    Some(m) => m,
                    None => continue,
                };

                if missile.affiliation != Affiliation::Hostile {
                    continue;
                }
                if !matches!(
                    missile.status,
                    MissileStatus::Boost | MissileStatus::Midcourse | MissileStatus::Terminal
                ) {
                    continue;
                }

                // Fire control range gate — measured to the FUSED TRACK position
                // (sensor-derived), never the missile's true position.
                let defense_unit_ids_fc: std::collections::HashSet<EntityId> =
                    self.defense_units.iter().map(|u| u.id).collect();
                let fused_pos =
                    match self
                        .detection
                        .get_fused_track(target_id, &defense_unit_ids_fc, sim_time)
                    {
                        Some(ft) => ft.estimated_position,
                        None => continue, // No sensor track — cannot engage
                    };

                let distance = haversine_distance(unit.position, fused_pos);
                let sensor_config = self.sensor_configs.get_by_name(&unit.sensor_config_name);
                let fire_control_range = sensor_config.detection.detection_range_km
                    * sensor_config.detection.fire_control_range_multiplier;
                if distance > fire_control_range {
                    continue;
                }

                if let Some((time_to_intercept, intercept_pos, intercept_alt, _uncertainty)) =
                    self.calculate_intercept_solution_from_track(unit, target_id)
                {
                    // TERMINAL CLOSURE FEASIBILITY CHECK
                    // Reject solutions where the interceptor cannot physically
                    // reach the intercept point before the missile passes it,
                    // even though mid-flight geometry looked legal. This
                    // organically prevents role mismatches (e.g., a 0.7 km/s
                    // Tamir against a 2-3 km/s MRBM reentry vehicle) without
                    // a hardcoded role table: the solution just doesn't close.
                    {
                        let kin = InterceptorKinematics::for_defense_type(unit.defense_type);
                        let closure_required = haversine_distance(unit.position, intercept_pos);
                        // Required average speed over the whole engagement
                        let required_avg_speed = closure_required / time_to_intercept.max(0.1);
                        // Interceptor's achievable average (boost+coast upper bound):
                        // max velocity is the ceiling; average can't exceed it by >20%
                        if required_avg_speed > kin.max_velocity_km_s * 1.2 {
                            eprintln!(
                                "[ENGAGE REJECT] {:?} -> target {}: closure infeasible (needs {:.2} km/s avg > {:.2} max)",
                                unit.defense_type, target_id, required_avg_speed,
                                kin.max_velocity_km_s
                            );
                            continue;
                        }
                    }

                    // Get track confidence for doctrine decision
                    let defense_unit_ids: std::collections::HashSet<EntityId> =
                        self.defense_units.iter().map(|u| u.id).collect();
                    let track_confidence = self
                        .detection
                        .get_fused_track(target_id, &defense_unit_ids, sim_time)
                        .map(|ft| {
                            // Combined confidence: track quality + velocity confidence
                            let vel_conf = ft
                                .estimated_velocity
                                .as_ref()
                                .map(|v| v.confidence)
                                .unwrap_or(0.5);
                            (ft.fused_quality + vel_conf) / 2.0
                        })
                        .unwrap_or(0.5);

                    // Determine doctrine: Shoot-Look-Shoot vs Shoot-Shoot-Look
                    // SLS requires: sufficient time AND high track confidence
                    // Reference: Real BMD systems use SLS only when confident in track accuracy
                    // Time-to-impact comes from the sensor-derived converged
                    // trajectory estimate, not the missile's true flight clock.
                    let time_to_impact = self
                        .detection
                        .get_converged_trajectory(target_id)
                        .map(|(ct, _)| ct.flight_time_sec)
                        .unwrap_or(0.0);
                    let assessment_time = 10.0; // Time to determine hit/miss

                    // Estimate time for a second shot (roughly similar to first)
                    let second_shot_time = time_to_intercept * 0.8; // Second shot has less distance

                    let sls_total_time = time_to_intercept + assessment_time + second_shot_time;
                    let has_time_for_sls = sls_total_time < time_to_impact - 5.0; // 5 second margin

                    // Confidence-gated SLS: only use SLS if track is high confidence (>75%)
                    // Low confidence tracks warrant immediate salvo fire
                    const SLS_CONFIDENCE_THRESHOLD: f64 = 0.75;
                    let use_shoot_look_shoot =
                        has_time_for_sls && track_confidence >= SLS_CONFIDENCE_THRESHOLD;

                    // Confidence-based salvo size:
                    // - High confidence (>80%): Single shot, rely on accuracy
                    // - Medium confidence (60-80%): 2 shots (default salvo)
                    // - Lower confidence (50-60%): 3 shots if available
                    let confidence_salvo_size = if track_confidence >= 0.8 {
                        1 // High confidence - single precision shot
                    } else if track_confidence >= 0.6 {
                        2 // Medium confidence - standard salvo
                    } else {
                        3 // Lower confidence - increased salvo
                    };
                    let effective_salvo_size = confidence_salvo_size.min(self.salvo_size);

                    // First shot
                    launches.push((unit_idx, target_id, intercept_pos, intercept_alt, false));
                    unit_launches_this_cycle += 1;

                    // Additional shots based on confidence and doctrine
                    // SSL (Shoot-Shoot-Look): fire full salvo immediately
                    if !use_shoot_look_shoot && effective_salvo_size > 1 {
                        for _ in 1..effective_salvo_size {
                            if unit_launches_this_cycle < max_launches_per_cycle
                                && (unit_launches_this_cycle as u32) < unit.interceptors_remaining
                            {
                                launches.push((
                                    unit_idx,
                                    target_id,
                                    intercept_pos,
                                    intercept_alt,
                                    false,
                                ));
                                unit_launches_this_cycle += 1;
                            }
                        }
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
                self.calculate_flight_time(defense_type, position, intercept_pos, intercept_alt);

            // Debug: trace the launch parameters (sensor-derived only)
            {
                let expected_intercept_time = launch_time + time_to_intercept;
                eprintln!(
                    "\n[LAUNCH TRACE] {:?} -> target {}",
                    defense_type, target_id
                );
                eprintln!(
                    "  Launch time: {:.1}s, Intercept time: {:.1}s (in {:.1}s)",
                    launch_time, expected_intercept_time, time_to_intercept
                );
                eprintln!(
                    "  Intercept point: pos=({:.2},{:.2}), alt={:.0}km",
                    intercept_pos.lat, intercept_pos.lon, intercept_alt
                );
                if let Some((converged, _)) = self.detection.get_converged_trajectory(target_id) {
                    eprintln!(
                        "  Converged est: origin=({:.2},{:.2}) target=({:.2},{:.2}) apogee={:.0}km T={:.0}s conf={:.2}",
                        converged.origin.lat,
                        converged.origin.lon,
                        converged.target.lat,
                        converged.target.lon,
                        converged.apogee_km,
                        converged.flight_time_sec,
                        converged.confidence
                    );
                }
            }

            // Get interceptor config for engagement parameters
            let interceptor_config = self.get_interceptor_config(defense_type);
            let divert_budget_km = interceptor_config
                .engagement
                .midcourse_guidance
                .divert_budget_km;

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
                divert_budget_km,
            );
            // Override hit probability from config
            interceptor.hit_probability = interceptor_config.engagement.hit_probability;

            self.interceptors.push(interceptor);
            *self.interceptors_per_target.entry(target_id).or_insert(0) += 1;
            self.defense_units[unit_idx].interceptors_remaining -= 1;
            self.defense_units[unit_idx].status = UnitStatus::Engaged;
        }
    }

    // ========================================================================
    // Config Helper Methods
    // ========================================================================

    /// Get interceptor config for a defense unit via platform config
    fn get_interceptor_config_for_unit(&self, unit: &DefenseUnit) -> &InterceptorConfig {
        let platform_config = self.platform_configs.get_by_defense_type(unit.defense_type);
        self.interceptor_configs
            .get_by_name(&platform_config.launcher.interceptor_type)
    }

    /// Get interceptor config by defense type
    fn get_interceptor_config(&self, defense_type: DefenseType) -> &InterceptorConfig {
        let platform_config = self.platform_configs.get_by_defense_type(defense_type);
        self.interceptor_configs
            .get_by_name(&platform_config.launcher.interceptor_type)
    }

    /// Get platform config by defense type
    fn get_platform_config(&self, defense_type: DefenseType) -> &PlatformConfig {
        self.platform_configs.get_by_defense_type(defense_type)
    }

    // ========================================================================
    // Interceptor Performance Methods
    // ========================================================================

    /// Calculate a fire control intercept solution using ONLY sensor-derived
    /// track data. Returns None if the target is untracked, the track is
    /// insufficient, or no valid intercept geometry exists — in that case the
    /// launch is refused (no ground-truth fallback).
    ///
    /// Realistic BMD approach:
    /// 1. Search/track radars detect and establish initial track (3+ measurements)
    /// 2. Once track is established, defense system's fire control radar locks on
    /// 3. Fire control radar provides high-precision data for intercept calculation
    ///
    /// Fire control radars (TPY-2, SPY-1, MPQ-53/65) have:
    /// - High update rates (10-100 Hz vs 1-5 Hz for search radars)
    /// - Precision tracking (<1km position accuracy within range)
    /// - Doppler velocity measurement
    ///
    /// Returns (intercept_time, intercept_position, intercept_altitude, uncertainty_km)
    fn calculate_intercept_solution_from_track(
        &self,
        unit: &DefenseUnit,
        target_id: EntityId,
    ) -> Option<(f64, GeoCoord, f64, f64)> {
        // ====================================================================
        // SENSOR-DERIVED FIRE CONTROL SOLUTION
        //
        // Realism requirements (AGENTS.md): fire control uses ONLY sensor-
        // detected data. The missile projection model is the converged
        // trajectory estimator (quadratic altitude fit + running mean), which
        // reconstructs the exact parabolic profile the missile actually flies
        // — from measurements alone, never ground truth. If the sensor chain
        // cannot produce a solution, the launch is REFUSED (no fallback).
        // ====================================================================

        // 1. Get fused sensor track - if no track, cannot engage
        let defense_unit_ids: std::collections::HashSet<EntityId> =
            self.defense_units.iter().map(|u| u.id).collect();
        let fused_track =
            self.detection
                .get_fused_track(target_id, &defense_unit_ids, self.sim_time)?;

        // 2. Require minimum track quality and recent update
        // Real BMD systems require "fire control quality" tracks
        const MIN_ENGAGEMENT_QUALITY: f64 = 0.6;
        if fused_track.fused_quality < MIN_ENGAGEMENT_QUALITY {
            return None; // Track quality too poor for engagement
        }
        if fused_track.staleness_seconds > 5.0 {
            return None; // Track too stale (no updates in 5 seconds)
        }

        // 3. Require sufficient measurements for track establishment
        if fused_track.measurement_count < 10 {
            return None; // Need at least 10 measurements for stable track
        }

        // 4. Require velocity estimate with minimum confidence
        let velocity = match &fused_track.estimated_velocity {
            Some(v) => v,
            None => return None,
        };
        const MIN_VELOCITY_CONFIDENCE: f64 = 0.55;
        if velocity.confidence < MIN_VELOCITY_CONFIDENCE {
            return None; // Velocity estimate too uncertain for engagement
        }

        // 5. Require a converged trajectory estimate — fire control projects
        //    the target ONLY through the sensor-derived trajectory model.
        let (converged, _est_flight_time) = self.detection.get_converged_trajectory(target_id)?;

        self.calculate_intercept_from_converged_trajectory(
            unit,
            &converged,
            velocity,
            fused_track.uncertainty_radius_km,
            fused_track.estimated_position,
        )
    }

    /// Compute an intercept solution from the sensor-derived converged
    /// trajectory. The converged trajectory stores (origin, target, apogee,
    /// flight_time) estimated from radar measurements; position_at(progress)
    /// reproduces the parabolic profile h(τ) = 4A·τ(1−τ) with constant ground
    /// speed — the same kinematics the threat actually flies, so the vertical
    /// geometry error that plagued the old gravity-model solvers (60-95 km
    /// aim bias for MRBM-class targets) is eliminated.
    ///
    /// Returns (time_to_intercept, intercept_pos, intercept_alt, uncertainty_km).
    fn calculate_intercept_from_converged_trajectory(
        &self,
        unit: &DefenseUnit,
        converged: &crate::simulation::detection::ConvergedTrajectory,
        velocity: &crate::simulation::detection::VelocityEstimate,
        base_uncertainty_km: f64,
        fused_pos: GeoCoord,
    ) -> Option<(f64, GeoCoord, f64, f64)> {
        use crate::simulation::physics::BallisticTrajectory;

        // Reconstruct the profile from the converged estimate
        let trajectory = BallisticTrajectory::with_params(
            converged.origin,
            converged.target,
            converged.apogee_km,
            converged.flight_time_sec,
        );

        // Estimate current flight progress from the fused position along the
        // estimated origin->target path. Progress = fraction of range already
        // flown (constant ground speed model).
        let total_range = trajectory.range_km.max(1.0);
        let along_track = haversine_distance(converged.origin, fused_pos);
        let progress = (along_track / total_range).clamp(0.0, 0.99);

        // Remaining flight time in the estimated profile
        let remaining_flight_time = converged.flight_time_sec * (1.0 - progress);
        if remaining_flight_time <= 5.0 {
            return None; // Too late to engage
        }

        let interceptor_config = self.get_interceptor_config_for_unit(unit);
        let platform_config = self.get_platform_config(unit.defense_type);
        let max_engagement_range = platform_config.launcher.engagement_range_km;
        let min_alt = interceptor_config
            .altitude_envelope
            .min_engagement_altitude_km;
        let max_alt = interceptor_config
            .altitude_envelope
            .max_engagement_altitude_km;

        // Scan future progress values for the EARLIEST intercept both parties
        // can make: interceptor arrives no later than the missile (0.5 s
        // sub-step quantization margin; the old code allowed +5 s late which
        // guaranteed misses at 0.1 km kill radius).
        let scan_steps = 60;
        let progress_step = (1.0 - progress).min(0.98) / scan_steps as f64;

        let mut best: Option<(f64, GeoCoord, f64, f64)> = None;

        for step in 1..=scan_steps {
            let candidate_progress = progress + step as f64 * progress_step;
            if candidate_progress >= 0.995 {
                break;
            }
            let (pos, alt) = trajectory.position_at(candidate_progress);

            // Altitude envelope check
            if alt < min_alt || alt > max_alt {
                continue;
            }

            // Range check
            let dist_to_target = haversine_distance(unit.position, pos);
            if dist_to_target > max_engagement_range {
                continue;
            }

            // Missile time to this point (constant ground speed profile)
            let missile_time = (candidate_progress - progress) * converged.flight_time_sec;

            // Interceptor arrival time (unified boost/drag-aware estimator)
            let interceptor_time =
                self.calculate_flight_time(unit.defense_type, unit.position, pos, alt);

            // Interceptor must arrive BEFORE the missile (0.5 s quantization margin)
            if interceptor_time > missile_time + 0.5 {
                continue;
            }

            // Earliest feasible point wins (maximizes time for follow-up shots)
            let time_to_intercept = missile_time;
            let uncertainty = base_uncertainty_km + velocity.staleness * velocity.ground_speed_km_s;
            best = Some((time_to_intercept, pos, alt, uncertainty));
            break;
        }

        best
    }

    /// Calculate flight time for interceptor using kinematics model
    fn calculate_flight_time(
        &self,
        defense_type: DefenseType,
        from: GeoCoord,
        to: GeoCoord,
        target_altitude: f64,
    ) -> f64 {
        // SINGLE SOURCE OF TRUTH for arrival-time planning: delegates to
        // InterceptorKinematics::time_to_cover_distance (boost-aware, drag-
        // aware). Previously this used config kinematics while guidance used
        // a constant-max-velocity estimate — systematic 1-81 s disagreements.
        let kin = InterceptorKinematics::for_defense_type(defense_type);
        let horizontal_distance = haversine_distance(from, to);
        let total_distance = (horizontal_distance.powi(2) + target_altitude.powi(2)).sqrt();

        let is_endo = matches!(
            defense_type,
            DefenseType::Patriot
                | DefenseType::THAAD
                | DefenseType::IronDome
                | DefenseType::DavidsSling
        );

        kin.time_to_cover_distance(total_distance, 0.0, is_endo)
            .unwrap_or(f64::MAX / 4.0) // Unreachable — huge but finite to avoid NaN downstream
    }

    /// Resolve intercepts - determine hit or miss based on actual proximity to assigned target only
    fn resolve_intercepts(&mut self) {
        let mut hits: Vec<EntityId> = Vec::new();
        let mut just_resolved: Vec<EntityId> = Vec::new();
        // Collect data for debris cloud creation: (position, altitude)
        let mut debris_data: Vec<(GeoCoord, f64)> = Vec::new();

        // Build maps for quick lookup - only for the assigned target
        let missile_data: std::collections::HashMap<EntityId, (GeoCoord, f64, f64)> = self
            .missiles
            .iter()
            .map(|m| (m.id, (m.position, m.altitude_km, m.decoy_effectiveness())))
            .collect();

        // Missile name lookup for logging
        let missile_names: std::collections::HashMap<EntityId, String> = self
            .missiles
            .iter()
            .map(|m| (m.id, m.name.clone()))
            .collect();

        // Check if missile is still a valid target (not already intercepted/impacted)
        let valid_targets: std::collections::HashSet<EntityId> = self
            .missiles
            .iter()
            .filter(|m| {
                matches!(
                    m.status,
                    MissileStatus::Boost | MissileStatus::Midcourse | MissileStatus::Terminal
                )
            })
            .map(|m| m.id)
            .collect();

        // Pre-fetch kill envelope configs to avoid borrow checker issues
        let kill_envelope_cache: Vec<_> = self
            .interceptors
            .iter()
            .map(|i| {
                let config = self.get_interceptor_config(i.defense_type);
                (
                    config.kill_envelope.seeker_range_km,
                    config.kill_envelope.kill_radius_km,
                    config.kill_envelope.base_pk,
                    config.engagement.terminal_blend_factor,
                )
            })
            .collect();

        for (idx, interceptor) in self.interceptors.iter_mut().enumerate() {
            if interceptor.status != InterceptorStatus::InFlight {
                continue;
            }

            let target_id = interceptor.target_id;

            // Debug output for THAAD and Aegis
            let is_debug_system = matches!(
                interceptor.defense_type,
                DefenseType::THAAD | DefenseType::Aegis
            );

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

            // Kill envelope from config (cached)
            let (seeker_range_km, kill_radius_km, base_pk, terminal_blend_factor) =
                kill_envelope_cache[idx];

            // Check flight progress
            let flight_duration = interceptor.intercept_time - interceptor.launch_time;
            let progress = if flight_duration > 0.0 {
                interceptor.current_flight_time / flight_duration
            } else {
                1.0
            };

            // Terminal homing phase - actively tracking the target
            let in_terminal = progress >= 0.7;

            if is_debug_system && in_terminal {
                // Calculate error between predicted and actual missile position
                let predicted_error_horiz =
                    haversine_distance(interceptor.target_position, missile_pos);
                let predicted_error_vert = (interceptor.target_altitude_km - missile_alt).abs();
                let predicted_error_3d =
                    (predicted_error_horiz.powi(2) + predicted_error_vert.powi(2)).sqrt();

                eprintln!(
                    "\n[{:?} Intercept Check] Target={}, Progress={:.2}, InTerminal={}",
                    interceptor.defense_type, target_id, progress, in_terminal
                );
                eprintln!(
                    "  Interceptor: pos={:.2},{:.2}, alt={:.0}km",
                    interceptor.position.lat, interceptor.position.lon, interceptor.altitude_km
                );
                eprintln!(
                    "  Target intercept point (PREDICTED): pos={:.2},{:.2}, alt={:.0}km",
                    interceptor.target_position.lat,
                    interceptor.target_position.lon,
                    interceptor.target_altitude_km
                );
                eprintln!(
                    "  Actual missile: pos={:.2},{:.2}, alt={:.0}km",
                    missile_pos.lat, missile_pos.lon, missile_alt
                );
                // Check what the trajectory would predict at missile's actual position
                if let Some(target_missile) = self.missiles.iter().find(|m| m.id == target_id) {
                    let actual_progress = target_missile.flight_progress();
                    eprintln!(
                        "  Missile flight progress: {:.2}%, flight_time: {:.1}s, current: {:.1}s",
                        actual_progress * 100.0,
                        target_missile.flight_time,
                        target_missile.current_flight_time
                    );
                    eprintln!(
                        "  Interceptor planned intercept_time: {:.1}s",
                        interceptor.intercept_time
                    );
                    if let Some(traj) = self.trajectories.get(&target_id) {
                        let (traj_pos, traj_alt) = traj.position_at(actual_progress);
                        eprintln!("  Trajectory at actual progress {:.2}%: pos=({:.2},{:.2}), alt={:.0}km",
                                 actual_progress * 100.0, traj_pos.lat, traj_pos.lon, traj_alt);
                    }
                }
                eprintln!(
                    "  PREDICTION ERROR: {:.1}km horiz, {:.1}km vert, {:.1}km 3D",
                    predicted_error_horiz, predicted_error_vert, predicted_error_3d
                );
                eprintln!(
                    "  Interceptor to missile: {:.1}km 3D (horiz={:.1}km, vert={:.1}km)",
                    distance_3d, horizontal_distance, altitude_diff
                );
                eprintln!(
                    "  Kill radius: {:.1}km, Seeker range: {:.1}km, Terminal blend: {:.2}",
                    kill_radius_km, seeker_range_km, terminal_blend_factor
                );
                eprintln!(
                    "  Seeker: acquired={}, off-boresight={:.1}°, gimbal_limit={:.1}°",
                    interceptor.seeker_acquired,
                    interceptor.off_boresight_angle_deg,
                    interceptor.seeker_gimbal_limit_deg
                );
            }

            // Only attempt intercept against the ASSIGNED target
            if in_terminal {
                // CLOSEST POINT OF APPROACH (CPA) RESOLUTION
                // Hit-to-kill interceptors can't turn around. The guidance-side
                // tracker (update_interceptors) is the SOLE owner of CPA state:
                // it uses 10-frame hysteresis to filter terminal-maneuver
                // transients before setting passed_cpa. Here we only consume the
                // confirmed flag — previously resolve_intercepts ran its own
                // single-frame check on the same shared field, declaring misses
                // on transients the hysteresis was designed to filter.
                if interceptor.passed_cpa && distance_3d > kill_radius_km {
                    // We've passed CPA and missed the kill radius - immediate miss
                    let cpa_distance = interceptor.previous_distance_to_target_km;
                    let missile_name = missile_names
                        .get(&target_id)
                        .map(|s| s.as_str())
                        .unwrap_or("Unknown");
                    eprintln!("[INTERCEPT] {} -> {} | MISS | CPA={:.3}km | kill_radius={:.3}km | seeker={}",
                             interceptor.defense_type.name(), missile_name, cpa_distance, kill_radius_km,
                             if interceptor.seeker_acquired { "acquired" } else { "NOT acquired" });
                    interceptor.status = InterceptorStatus::Miss;
                    interceptor.miss_reason = MissReason::OffCourse;
                    interceptor.final_miss_distance_km = Some(cpa_distance);
                    interceptor.final_pk = Some(0.0);
                    just_resolved.push(target_id);
                    continue;
                }

                // Seeker acquisition factor: if seeker hasn't acquired, Pk is severely reduced
                // Interceptor is essentially flying blind without active seeker track
                let seeker_factor = if interceptor.seeker_acquired {
                    1.0
                } else if interceptor.off_boresight_angle_deg
                    <= interceptor.seeker_gimbal_limit_deg * 0.5
                {
                    0.3 // Target in seeker FOV but not yet acquired - some chance of last-second lock
                } else {
                    0.05 // Target outside seeker FOV - very low chance
                };

                if distance_3d <= kill_radius_km {
                    // Within kill radius - DETERMINISTIC HIT (no Pk roll)
                    // Pk is calculated for reference (e.g., follow-up shot decisions) but not used for hit determination
                    let proximity_factor = 1.0 - (distance_3d / kill_radius_km) * 0.3;
                    let adjusted_pk = base_pk * proximity_factor * decoy_factor * seeker_factor;

                    let missile_name = missile_names
                        .get(&target_id)
                        .map(|s| s.as_str())
                        .unwrap_or("Unknown");
                    eprintln!("[INTERCEPT] {} -> {} | HIT | dist={:.3}km | kill_radius={:.3}km | Pk={:.2}",
                             interceptor.defense_type.name(), missile_name, distance_3d, kill_radius_km, adjusted_pk);

                    interceptor.status = InterceptorStatus::Hit;
                    interceptor.final_miss_distance_km = Some(distance_3d);
                    interceptor.final_pk = Some(adjusted_pk); // Stored for reference, not used for hit determination
                    hits.push(target_id);
                    // Record position for debris cloud creation
                    debris_data.push((interceptor.position, interceptor.altitude_km));
                    just_resolved.push(target_id);
                } else if distance_3d <= seeker_range_km && progress >= 0.9 {
                    // In seeker range but not kill radius - continue homing
                    // Hit-to-kill interceptors must reach kill radius for a hit
                    if is_debug_system {
                        eprintln!("  ○ In seeker range ({:.1}km <= {:.1}km) but outside kill radius ({:.2}km)",
                                 distance_3d, seeker_range_km, kill_radius_km);
                        eprintln!(
                            "    Progress={:.2}, continuing to close distance...",
                            progress
                        );
                    }

                    if progress >= 1.2 {
                        // Past nominal intercept time and still not in kill radius - miss
                        let missile_name = missile_names
                            .get(&target_id)
                            .map(|s| s.as_str())
                            .unwrap_or("Unknown");
                        eprintln!("[INTERCEPT] {} -> {} | MISS | dist={:.3}km | kill_radius={:.3}km | timeout (progress={:.2})",
                                 interceptor.defense_type.name(), missile_name, distance_3d, kill_radius_km, progress);
                        interceptor.status = InterceptorStatus::Miss;
                        interceptor.miss_reason = MissReason::OffCourse;
                        interceptor.final_miss_distance_km = Some(distance_3d);
                        interceptor.final_pk = Some(0.0);
                        just_resolved.push(target_id);
                    }
                    // Otherwise continue homing toward kill radius
                } else if progress >= 1.3 {
                    // Well past intercept time and not close - definite miss
                    let missile_name = missile_names
                        .get(&target_id)
                        .map(|s| s.as_str())
                        .unwrap_or("Unknown");
                    eprintln!("[INTERCEPT] {} -> {} | MISS | dist={:.3}km | kill_radius={:.3}km | way past intercept time",
                             interceptor.defense_type.name(), missile_name, distance_3d, kill_radius_km);
                    interceptor.status = InterceptorStatus::Miss;
                    interceptor.miss_reason = MissReason::OffCourse;
                    interceptor.final_miss_distance_km = Some(distance_3d);
                    interceptor.final_pk = Some(0.0);
                    just_resolved.push(target_id);
                } else if is_debug_system {
                    eprintln!(
                        "  → Still homing: distance={:.1}km > kill_radius={:.1}km, progress={:.2}",
                        distance_3d, kill_radius_km, progress
                    );
                }
                // Continue homing if still in terminal phase but not resolved
            }
            // Pre-terminal: still flying toward intercept point
        }

        // Create debris clouds for all hits
        for (pos, alt) in debris_data {
            self.debris_clouds.push(DebrisCloud {
                position: pos,
                altitude_km: alt,
                creation_time: self.sim_time,
                // Debris starts with small radius and expands
                initial_radius_km: 0.5,
                // Expansion rate depends on closure velocity and altitude
                expansion_rate_km_s: 0.2,
                // Debris remains hazardous for ~10 seconds
                hazard_duration_sec: 10.0,
            });
        }

        // Create kill assessments for both hits and misses
        for interceptor in &self.interceptors {
            if interceptor.status == InterceptorStatus::Hit {
                self.kill_assessments.push(KillAssessment::new(
                    interceptor.target_id,
                    self.sim_time,
                    true, // Was a hit
                ));
            } else if interceptor.status == InterceptorStatus::Miss {
                self.kill_assessments.push(KillAssessment::new(
                    interceptor.target_id,
                    self.sim_time,
                    false, // Was a miss
                ));
            }
        }

        // Update existing kill assessments
        for assessment in &mut self.kill_assessments {
            assessment.update(self.sim_time);
        }

        // Remove old debris clouds that are no longer hazardous
        self.debris_clouds
            .retain(|dc| dc.is_hazardous(self.sim_time));

        // Collect misses for shoot-look-shoot follow-up (after assessment delay)
        let confirmed_misses: Vec<EntityId> = self
            .kill_assessments
            .iter()
            .filter(|ka| ka.assessment_complete && !ka.was_kill)
            .map(|ka| ka.target_id)
            .collect();

        // Remove completed assessments
        self.kill_assessments.retain(|ka| !ka.assessment_complete);

        // Mark ONLY hit missiles as intercepted (misses should NOT stop the missile)
        for missile in &mut self.missiles {
            if hits.contains(&missile.id) {
                missile.status = MissileStatus::Intercepted;
                // Clear tracking for destroyed targets
                self.shots_fired_per_target.remove(&missile.id);
                self.targets_needing_followup.remove(&missile.id);
            }
        }

        // Queue follow-up shots for confirmed misses (Shoot-Look-Shoot doctrine)
        // Uses kill assessment delay - don't queue follow-up until we've confirmed the miss
        for target_id in &confirmed_misses {
            // Only queue if we haven't fired max shots yet and target is still valid
            let shots_fired = self
                .shots_fired_per_target
                .get(target_id)
                .copied()
                .unwrap_or(0);
            if shots_fired < self.max_shots_per_target {
                // Check if target is still a valid threat
                let target_still_valid = self.missiles.iter().any(|m| {
                    m.id == *target_id
                        && matches!(
                            m.status,
                            MissileStatus::Boost
                                | MissileStatus::Midcourse
                                | MissileStatus::Terminal
                        )
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
    fn update_missile(
        missile: &mut Missile,
        sim_time: f64,
        trajectory: Option<&BallisticTrajectory>,
    ) {
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

                    // Update velocity with atmospheric drag modeling
                    missile.update_velocity(traj.range_km);
                }
            }
            MissileStatus::Intercepted | MissileStatus::Impacted => {
                // No update needed
            }
        }
    }

    /// Get trajectory for a missile (O(1) HashMap lookup)
    pub fn get_trajectory(&self, missile_id: EntityId) -> Option<&BallisticTrajectory> {
        self.trajectories.get(&missile_id)
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
        self.debris_clouds.clear();
        self.kill_assessments.clear();
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
            35786.0,                    // Geostationary
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
            GeoCoord::new(39.0, 125.5),  // North Korea
            GeoCoord::new(37.5, -122.0), // San Francisco
            60.0,                        // Launch at T+60s
            5,                           // 5 decoys
        );

        // ICBM without countermeasures
        self.add_missile(
            "Hostile ICBM 2".to_string(),
            Affiliation::Hostile,
            GeoCoord::new(39.0, 125.5),  // North Korea
            GeoCoord::new(47.6, -122.3), // Seattle
            120.0,                       // Launch at T+120s
        );

        // MRBM with countermeasures
        self.add_missile_with_countermeasures(
            "Hostile MRBM (CM)".to_string(),
            Affiliation::Hostile,
            GeoCoord::new(35.0, 51.0), // Iran
            GeoCoord::new(32.0, 34.8), // Tel Aviv
            30.0,                      // Launch at T+30s
            3,                         // 3 decoys
        );
    }
}

impl Default for SimulationEngine {
    fn default() -> Self {
        Self::new()
    }
}
