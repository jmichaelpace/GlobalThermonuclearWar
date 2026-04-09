use std::collections::{HashMap, HashSet};

use rand::Rng;

use crate::map::GeoCoord;
use crate::simulation::{
    bearing, haversine_distance, normalize_angle_diff, DefenseUnit, EntityId, Interceptor,
    InterceptorStatus, Missile, MissileStatus, RadarStation, Satellite, SensorConfig,
    SensorConfigRegistry, SensorType,
};
use crate::simulation::config::{RadarBand, RadarMode, RadarType, SensorRole, SensorTrackingConfig};
use crate::simulation::kalman::BallisticState;

/// Type of Kalman filter to use for tracking
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum FilterType {
    /// Linear Kalman Filter in local ENU coordinates (simpler, faster)
    LinearKalman,
    /// Extended Kalman Filter in geodetic coordinates (more accurate for long range)
    #[default]
    ExtendedKalman,
}

/// Detection event - a sensor detecting a threat
#[derive(Clone, Debug)]
pub struct Detection {
    pub sensor_id: EntityId,
    pub sensor_type: SensorKind,
    pub target_id: EntityId,
    pub detection_quality: f64, // 0.0 to 1.0
    pub bearing_deg: f64,
    pub range_km: f64,
    pub altitude_km: f64,
    /// True if this is a false alarm (clutter/noise)
    pub is_false_alarm: bool,
    /// Radar band used for this detection (None for non-radar sensors)
    pub radar_band: Option<RadarBand>,
    /// Raw radar measurement for EKF (range/azimuth/elevation)
    pub radar_measurement: Option<crate::simulation::ekf::RadarMeasurement>,
}

/// Kind of sensor that made the detection
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SensorKind {
    GroundRadar,
    ShipRadar,
    SatelliteIR,
    SatelliteRadar,
    DefenseUnitRadar,
}

/// Single position measurement from a sensor at a specific time
#[derive(Clone, Debug)]
pub struct PositionMeasurement {
    pub timestamp: f64,
    pub position: GeoCoord,
    pub altitude_km: f64,
    pub measurement_quality: f64,
}

/// Velocity estimate derived from position history
#[derive(Clone, Debug)]
pub struct VelocityEstimate {
    pub ground_speed_km_s: f64,
    pub heading_deg: f64,
    pub vertical_rate_km_s: f64,
    pub confidence: f64,
    pub staleness: f64,
}

/// Maximum position measurements to retain for velocity estimation
const MAX_POSITION_HISTORY: usize = 5;

/// Tracking state for a defense unit
#[derive(Clone, Debug)]
pub struct TrackingState {
    pub tracker_id: EntityId,
    pub target_id: EntityId,
    pub track_quality: f64,    // 0.0 to 1.0, degrades without updates
    pub time_since_update: f64,
    pub predicted_position: GeoCoord,
    pub predicted_altitude: f64,
    /// Position history for velocity estimation (newest first, max 5 measurements)
    pub position_history: Vec<PositionMeasurement>,
    /// Estimated velocity from position history
    pub estimated_velocity: Option<VelocityEstimate>,
    /// Estimated heading derived from velocity (degrees, 0=North)
    pub estimated_heading_deg: Option<f64>,
    /// Linear Kalman filter for optimal state estimation (ENU coordinates)
    pub kalman_filter: Option<BallisticState>,
    /// Extended Kalman filter for optimal state estimation (geodetic coordinates)
    pub ekf_state: Option<crate::simulation::ekf::EKFState>,
}

impl TrackingState {
    /// Add a position measurement to the history
    pub fn add_position_measurement(
        &mut self,
        timestamp: f64,
        position: GeoCoord,
        altitude: f64,
        quality: f64,
    ) {
        // Skip if position hasn't changed significantly from most recent measurement
        if let Some(last) = self.position_history.first() {
            let ground_dist = haversine_distance(last.position, position);
            let altitude_change = (altitude - last.altitude_km).abs();
            let time_diff = timestamp - last.timestamp;

            // Only add if position changed by > 0.05 km OR altitude changed by > 0.05 km
            // AND enough time has passed (> 0.01s to avoid duplicate timestamps)
            if ground_dist < 0.05 && altitude_change < 0.05 && time_diff < 1.0 {
                // Position hasn't changed enough - skip this measurement
                return;
            }
        }

        // Create measurement
        let measurement = PositionMeasurement {
            timestamp,
            position,
            altitude_km: altitude,
            measurement_quality: quality,
        };

        // Update Kalman filter
        if let Some(ref mut kf) = self.kalman_filter {
            // Predict forward to current time
            let dt = timestamp - kf.timestamp;
            if dt > 0.0 {
                kf.predict(dt);
            }
            // Update with measurement
            kf.update(&measurement);
        } else {
            // Initialize Kalman filter with first measurement
            self.kalman_filter = Some(BallisticState::new(&measurement));
        }

        // Add to front of history (newest first)
        self.position_history.insert(0, measurement);

        // Trim to max history
        if self.position_history.len() > MAX_POSITION_HISTORY {
            self.position_history.truncate(MAX_POSITION_HISTORY);
        }
    }

    /// Add a radar measurement and update the Extended Kalman Filter
    pub fn add_radar_measurement(
        &mut self,
        measurement: &crate::simulation::ekf::RadarMeasurement,
    ) {
        use crate::simulation::ekf::EKFState;

        // Update EKF
        if let Some(ref mut ekf) = self.ekf_state {
            // Predict forward to measurement time
            let dt = measurement.timestamp - ekf.timestamp;
            if dt > 0.0 {
                ekf.predict(dt);
            }
            // Update with radar measurement
            ekf.update(measurement);
        } else {
            // Initialize EKF with first measurement
            self.ekf_state = Some(EKFState::new(measurement));
        }

        // Also update position history for visualization/backup
        // Convert radar measurement to position
        let (pos, alt) = crate::simulation::ekf::radar_to_geodetic(
            measurement.sensor_position,
            measurement.sensor_altitude_km,
            measurement.range_km,
            measurement.azimuth_rad,
            measurement.elevation_rad,
        );

        let position_measurement = PositionMeasurement {
            timestamp: measurement.timestamp,
            position: pos,
            altitude_km: alt,
            measurement_quality: 0.8, // Radar measurements are generally good quality
        };

        // Add to front of history (newest first)
        self.position_history.insert(0, position_measurement);

        // Trim to max history
        if self.position_history.len() > MAX_POSITION_HISTORY {
            self.position_history.truncate(MAX_POSITION_HISTORY);
        }
    }

    /// Calculate velocity estimate from position history
    /// Uses weighted least-squares over multiple measurements
    pub fn update_velocity_estimate(&mut self, current_time: f64) {
        // Need at least 2 measurements for velocity
        if self.position_history.len() < 2 {
            self.estimated_velocity = None;
            self.estimated_heading_deg = None;
            return;
        }

        // Use weighted least-squares fit over all measurements
        // More recent measurements weighted higher
        let mut total_weight = 0.0;
        let mut weighted_ground_speed = 0.0;
        let mut weighted_heading_x = 0.0; // cos(heading)
        let mut weighted_heading_y = 0.0; // sin(heading)
        let mut weighted_vertical_rate = 0.0;

        // Calculate pairwise velocities
        for i in 0..self.position_history.len() - 1 {
            let newer = &self.position_history[i];
            let older = &self.position_history[i + 1];

            let dt = newer.timestamp - older.timestamp;

            if dt <= 0.001 {
                continue; // Skip near-simultaneous measurements
            }

            // Ground distance and direction
            let ground_distance = haversine_distance(older.position, newer.position);
            let heading = bearing(older.position, newer.position);
            let ground_speed = ground_distance / dt;

            // Vertical rate
            let altitude_change = newer.altitude_km - older.altitude_km;
            let vertical_rate = altitude_change / dt;

            // Weight: recent measurements matter more, quality matters
            let age_factor = 1.0 / (1.0 + i as f64 * 0.5); // Decay with index
            let quality_factor = (newer.measurement_quality + older.measurement_quality) / 2.0;
            let weight = age_factor * quality_factor;

            total_weight += weight;
            weighted_ground_speed += ground_speed * weight;
            weighted_heading_x += heading.to_radians().cos() * weight;
            weighted_heading_y += heading.to_radians().sin() * weight;
            weighted_vertical_rate += vertical_rate * weight;
        }

        if total_weight <= 0.0 {
            return;
        }

        // Compute weighted averages
        let avg_ground_speed = weighted_ground_speed / total_weight;
        let heading_deg = (weighted_heading_y / total_weight)
            .atan2(weighted_heading_x / total_weight)
            .to_degrees()
            .rem_euclid(360.0);
        let avg_vertical_rate = weighted_vertical_rate / total_weight;

        // Confidence based on measurement consistency and quality
        let avg_quality = self
            .position_history
            .iter()
            .map(|m| m.measurement_quality)
            .sum::<f64>()
            / self.position_history.len() as f64;

        // Confidence increases with more measurements (more data = higher confidence)
        // 2 measurements: 0.6, 3: 0.75, 4: 0.85, 5: 0.95
        let measurement_factor = (self.position_history.len() as f64 * 0.15 + 0.3).min(0.95);
        let confidence = avg_quality * measurement_factor;

        let staleness = current_time - self.position_history[0].timestamp;

        self.estimated_velocity = Some(VelocityEstimate {
            ground_speed_km_s: avg_ground_speed,
            heading_deg,
            vertical_rate_km_s: avg_vertical_rate,
            confidence,
            staleness,
        });

        self.estimated_heading_deg = Some(heading_deg);
    }
}

/// Per-sensor radar mode tracking state (phased arrays only)
#[derive(Clone, Debug)]
pub struct RadarModeState {
    pub sensor_id: EntityId,
    /// Map of target_id -> (mode assignment, selected band) for that target
    pub target_assignments: HashMap<EntityId, (RadarMode, RadarBand)>,
    /// Debug statistics
    pub stats: RadarModeStatistics,
}

/// Statistics for radar mode performance tracking
#[derive(Clone, Debug)]
pub struct RadarModeStatistics {
    /// Total number of mode switches (any mode change for any target)
    pub mode_switch_count: u32,
    /// Total number of targets dropped due to capacity/time budget
    pub targets_dropped_count: u32,
    /// Last scan's time budget utilization (0.0-1.0, >1.0 means over budget)
    pub last_time_budget_utilization: f64,
    /// Number of targets in each mode during last scan
    pub last_fc_count: u32,
    pub last_track_count: u32,
    pub last_search_count: u32,
}

impl Default for RadarModeStatistics {
    fn default() -> Self {
        Self {
            mode_switch_count: 0,
            targets_dropped_count: 0,
            last_time_budget_utilization: 0.0,
            last_fc_count: 0,
            last_track_count: 0,
            last_search_count: 0,
        }
    }
}

impl RadarModeState {
    pub fn new(sensor_id: EntityId) -> Self {
        Self {
            sensor_id,
            target_assignments: HashMap::new(),
            stats: RadarModeStatistics::default(),
        }
    }
}

/// A fused track combining detections from multiple sensors
/// Used for rendering in Detected Track View mode
#[derive(Clone, Debug)]
pub struct FusedTrack {
    pub target_id: EntityId,
    /// Fused position estimate (weighted average of sensor tracks)
    pub estimated_position: GeoCoord,
    /// Estimated altitude
    pub estimated_altitude: f64,
    /// Combined track quality (0.0 to 1.0)
    pub fused_quality: f64,
    /// Position uncertainty radius in km (grows with lower quality)
    pub uncertainty_radius_km: f64,
    /// Number of sensors contributing to this track
    pub sensor_count: usize,
    /// Time since last sensor update (affects uncertainty)
    pub staleness_seconds: f64,
    /// Predicted heading (degrees, 0=North)
    pub predicted_heading_deg: f64,
    /// Fused velocity estimate from all tracking sensors
    pub estimated_velocity: Option<VelocityEstimate>,
    /// Number of position measurements in best track (for Kalman filter convergence)
    pub measurement_count: usize,
    /// Kalman filter position uncertainty (km, None if no Kalman filter)
    pub kalman_position_uncertainty_km: Option<f64>,
    /// Whether a fire control radar (defense unit) is tracking this target
    pub has_fire_control_lock: bool,
}

impl FusedTrack {
    /// Project position forward to compensate for measurement lag
    /// Returns (projected_position, projected_altitude, increased_uncertainty)
    pub fn project_forward(&self, dt_seconds: f64) -> (GeoCoord, f64, f64) {
        if let Some(velocity) = &self.estimated_velocity {
            // Project position forward along velocity vector
            let distance = velocity.ground_speed_km_s * dt_seconds;
            let projected_position = calculate_position_from_bearing_range(
                self.estimated_position,
                velocity.heading_deg,
                distance,
            );

            let projected_altitude = self.estimated_altitude + velocity.vertical_rate_km_s * dt_seconds;

            // Uncertainty grows: ~5km/s * dt * (1 - confidence)
            let uncertainty_growth = 5.0 * dt_seconds * (1.0 - velocity.confidence);
            let total_uncertainty = self.uncertainty_radius_km + uncertainty_growth;

            (projected_position, projected_altitude, total_uncertainty)
        } else {
            // No velocity - rapid uncertainty growth
            let stale_uncertainty = self.uncertainty_radius_km + 10.0 * dt_seconds;
            (self.estimated_position, self.estimated_altitude, stale_uncertainty)
        }
    }
}

/// Detection system that manages all sensor-target relationships
pub struct DetectionSystem {
    pub active_detections: Vec<Detection>,
    pub active_tracks: Vec<TrackingState>,
    /// Time since last scan for each sensor (for scan rate limiting)
    scan_accumulators: HashMap<EntityId, f64>,
    /// Radar mode state per sensor (phased arrays only)
    pub radar_mode_states: HashMap<EntityId, RadarModeState>,
    /// Type of Kalman filter to use for tracking
    pub filter_type: FilterType,
}

impl Default for DetectionSystem {
    fn default() -> Self {
        Self::new()
    }
}

impl DetectionSystem {
    pub fn new() -> Self {
        Self {
            active_detections: Vec::new(),
            active_tracks: Vec::new(),
            scan_accumulators: HashMap::new(),
            radar_mode_states: HashMap::new(),
            filter_type: FilterType::default(),
        }
    }

    /// Create a new detection system with specified filter type
    pub fn with_filter_type(filter_type: FilterType) -> Self {
        Self {
            active_detections: Vec::new(),
            active_tracks: Vec::new(),
            scan_accumulators: HashMap::new(),
            radar_mode_states: HashMap::new(),
            filter_type,
        }
    }

    /// Create a RadarMeasurement from detection data for EKF
    fn create_radar_measurement(
        sensor_position: GeoCoord,
        sensor_altitude_km: f64,
        range_km: f64,
        bearing_deg: f64,
        target_altitude_km: f64,
        timestamp: f64,
        detection_quality: f64,
    ) -> crate::simulation::ekf::RadarMeasurement {
        // Calculate elevation angle from range and altitude difference
        let ground_range = range_km.max(0.01); // Avoid division by zero
        let altitude_diff = target_altitude_km - sensor_altitude_km;
        let elevation_rad = (altitude_diff / ground_range).atan();

        // Measurement noise depends on detection quality
        // Higher quality = lower noise
        let quality_factor = (1.0 - detection_quality).max(0.1);
        let range_noise = 0.1 * quality_factor; // 0.01 - 0.1 km noise
        let angle_noise = (0.5_f64).to_radians() * quality_factor; // 0.05 - 0.5 deg noise

        crate::simulation::ekf::RadarMeasurement {
            range_km,
            azimuth_rad: bearing_deg.to_radians(),
            elevation_rad,
            sensor_position,
            sensor_altitude_km,
            timestamp,
            noise_std: [range_noise, angle_noise, angle_noise],
        }
    }

    /// Update all detections based on current entity positions
    /// Uses scan rate timing - sensors only update at their configured refresh rate
    pub fn update(
        &mut self,
        missiles: &[Missile],
        defense_units: &[DefenseUnit],
        radar_stations: &[RadarStation],
        satellites: &[Satellite],
        interceptors: &[Interceptor],
        sensor_configs: &SensorConfigRegistry,
        dt: f64,
        current_sim_time: f64,
    ) {
        // Filter to only active missiles
        let active_missiles: Vec<&Missile> = missiles
            .iter()
            .filter(|m| {
                matches!(
                    m.status,
                    MissileStatus::Boost | MissileStatus::Midcourse | MissileStatus::Terminal
                )
            })
            .collect();

        // Collect track limits for each sensor
        let mut track_limits: HashMap<EntityId, u32> = HashMap::new();

        // False alarm rate (probability per scan) - lower for better sensors
        const BASE_FALSE_ALARM_RATE: f64 = 0.05;

        // Process defense unit sensors with scan timing
        for unit in defense_units {
            // Handle multi-sensor platforms
            if !unit.sensors.is_empty() {
                for sensor in &unit.sensors {
                    let config = sensor_configs.get_by_name(&sensor.config_name);
                    track_limits.insert(sensor.sensor_id, config.tracking.max_simultaneous_tracks);

                    // Update radar modes for phased arrays BEFORE scanning
                    self.update_radar_modes(
                        sensor.sensor_id,
                        unit.position,
                        config.tracking.max_simultaneous_tracks,
                        config.tracking.max_fire_control_tracks,
                        interceptors,
                        config.detection.radar_type,
                        &config.tracking,
                        config,
                    );

                    if self.should_scan(sensor.sensor_id, config.tracking.track_update_rate_hz, dt) {
                        // Clear old detections for this sensor
                        self.active_detections.retain(|d| d.sensor_id != sensor.sensor_id);

                        // Process new detections
                        for missile in &active_missiles {
                            if unit.affiliation == missile.affiliation {
                                continue;
                            }

                            // Check bearing coverage
                            let bearing = calculate_bearing(unit.position, missile.position);
                            if !sensor.is_bearing_in_coverage(bearing) {
                                continue;
                            }

                            // Get mode and band assignment for this target
                            let mode_and_band = self
                                .radar_mode_states
                                .get(&sensor.sensor_id)
                                .and_then(|state| state.target_assignments.get(&missile.id))
                                .copied();

                            // Check detection based on radar type and mode assignment
                            if let Some((radar_mode, band)) = mode_and_band {
                                // Target has mode assignment - use mode-specific coverage for phased arrays
                                if config.detection.radar_type == RadarType::PhasedArray {
                                    let effective_coverage = config.detection.get_effective_azimuth_coverage(radar_mode);
                                    let half_coverage = effective_coverage / 2.0;
                                    let relative_bearing = normalize_angle_diff(bearing - sensor.azimuth_center_deg);
                                    if relative_bearing.abs() > half_coverage {
                                        continue;  // Target outside mode-specific coverage
                                    }
                                }

                                if let Some(detection) =
                                    Self::check_defense_unit_detection(unit, missile, config, Some(radar_mode), band, current_sim_time)
                                {
                                    // Use unique sensor ID for detection
                                    let mut detection = detection;
                                    detection.sensor_id = sensor.sensor_id;
                                    self.active_detections.push(detection);
                                }
                            } else {
                                // No mode assignment yet - use default Search mode
                                // This allows initial detection to establish tracks
                                let band = config.detection.get_band_for_mode(RadarMode::Search);

                                // For phased arrays, check Search mode azimuth coverage
                                if config.detection.radar_type == RadarType::PhasedArray {
                                    let effective_coverage = config.detection.get_effective_azimuth_coverage(RadarMode::Search);
                                    let half_coverage = effective_coverage / 2.0;
                                    let relative_bearing = normalize_angle_diff(bearing - sensor.azimuth_center_deg);
                                    if relative_bearing.abs() > half_coverage {
                                        continue;  // Target outside Search coverage
                                    }
                                }

                                if let Some(detection) =
                                    Self::check_defense_unit_detection(unit, missile, config, None, band, current_sim_time)
                                {
                                    let mut detection = detection;
                                    detection.sensor_id = sensor.sensor_id;
                                    self.active_detections.push(detection);
                                }
                            }
                        }

                        // Generate false alarms (clutter/noise)
                        let false_alarms = Self::generate_false_alarms(
                            sensor.sensor_id,
                            SensorKind::DefenseUnitRadar,
                            unit.position,
                            config.detection.detection_range_km,
                            BASE_FALSE_ALARM_RATE,
                        );
                        self.active_detections.extend(false_alarms);
                    }
                }
            } else {
                // Legacy fallback: single sensor via sensor_config_name
                if !unit.sensor_config_name.is_empty() {
                    let config = sensor_configs.get_by_name(&unit.sensor_config_name);
                    track_limits.insert(unit.id, config.tracking.max_simultaneous_tracks);

                    // Update radar modes for phased arrays BEFORE scanning
                    self.update_radar_modes(
                        unit.id,
                        unit.position,
                        config.tracking.max_simultaneous_tracks,
                        config.tracking.max_fire_control_tracks,
                        interceptors,
                        config.detection.radar_type,
                        &config.tracking,
                        config,
                    );

                    if self.should_scan(unit.id, config.tracking.track_update_rate_hz, dt) {
                        // Clear old detections for this sensor
                        self.active_detections.retain(|d| d.sensor_id != unit.id);

                        // Process new detections
                        for missile in &active_missiles {
                            if unit.affiliation == missile.affiliation {
                                continue;
                            }

                            // Get mode and band assignment for this target
                            let mode_and_band = self
                                .radar_mode_states
                                .get(&unit.id)
                                .and_then(|state| state.target_assignments.get(&missile.id))
                                .copied();

                            // Check detection based on radar type and mode assignment
                            if let Some((radar_mode, band)) = mode_and_band {
                                // Target has mode assignment - use assigned mode
                                if let Some(detection) =
                                    Self::check_defense_unit_detection(unit, missile, config, Some(radar_mode), band, current_sim_time)
                                {
                                    self.active_detections.push(detection);
                                }
                            } else {
                                // No mode assignment yet - use default Search mode
                                let band = config.detection.get_band_for_mode(RadarMode::Search);
                                if let Some(detection) =
                                    Self::check_defense_unit_detection(unit, missile, config, None, band, current_sim_time)
                                {
                                    self.active_detections.push(detection);
                                }
                            }
                        }

                        // Generate false alarms (clutter/noise)
                        let false_alarms = Self::generate_false_alarms(
                            unit.id,
                            SensorKind::DefenseUnitRadar,
                            unit.position,
                            config.detection.detection_range_km,
                            BASE_FALSE_ALARM_RATE,
                        );
                        self.active_detections.extend(false_alarms);
                    }
                }
            }
        }

        // Process radar stations with scan timing
        for station in radar_stations {
            let config = sensor_configs.get_by_name(&station.sensor_config_name);
            track_limits.insert(station.id, config.tracking.max_simultaneous_tracks);

            // Update radar modes for phased arrays BEFORE scanning
            self.update_radar_modes(
                station.id,
                station.position,
                config.tracking.max_simultaneous_tracks,
                config.tracking.max_fire_control_tracks,
                interceptors,
                config.detection.radar_type,
                &config.tracking,
                config,
            );

            if self.should_scan(station.id, config.tracking.track_update_rate_hz, dt) {
                // Clear old detections for this sensor
                self.active_detections.retain(|d| d.sensor_id != station.id);

                // Process new detections
                for missile in &active_missiles {
                    if station.affiliation == missile.affiliation {
                        continue;
                    }

                    // Calculate bearing for mode-specific coverage check
                    let bearing = calculate_bearing(station.position, missile.position);

                    // Get mode and band assignment for this target
                    let mode_and_band = self
                        .radar_mode_states
                        .get(&station.id)
                        .and_then(|state| state.target_assignments.get(&missile.id))
                        .copied();

                    // Check detection based on radar type and mode assignment
                    if let Some((radar_mode, band)) = mode_and_band {
                        // Target has mode assignment - use mode-specific coverage for phased arrays
                        if config.detection.radar_type == RadarType::PhasedArray {
                            let effective_coverage = config.detection.get_effective_azimuth_coverage(radar_mode);

                            // RadarStation uses facing_deg and handles wraparound at 0/360
                            let half_coverage = effective_coverage / 2.0;
                            let min_bearing = (station.facing_deg - half_coverage).rem_euclid(360.0);
                            let max_bearing = (station.facing_deg + half_coverage).rem_euclid(360.0);

                            let in_coverage = if min_bearing <= max_bearing {
                                bearing >= min_bearing && bearing <= max_bearing
                            } else {
                                // Coverage wraps around 0/360
                                bearing >= min_bearing || bearing <= max_bearing
                            };

                            if !in_coverage {
                                continue;  // Target outside mode-specific coverage
                            }
                        }

                        if let Some(detection) =
                            Self::check_radar_station_detection(station, missile, config, Some(radar_mode), band, current_sim_time)
                        {
                            self.active_detections.push(detection);
                        }
                    } else {
                        // No mode assignment yet - use default Search mode
                        let band = config.detection.get_band_for_mode(RadarMode::Search);

                        // For phased arrays, check Search mode azimuth coverage
                        if config.detection.radar_type == RadarType::PhasedArray {
                            let effective_coverage = config.detection.get_effective_azimuth_coverage(RadarMode::Search);
                            let half_coverage = effective_coverage / 2.0;
                            let min_bearing = (station.facing_deg - half_coverage).rem_euclid(360.0);
                            let max_bearing = (station.facing_deg + half_coverage).rem_euclid(360.0);

                            let in_coverage = if min_bearing <= max_bearing {
                                bearing >= min_bearing && bearing <= max_bearing
                            } else {
                                // Coverage wraps around 0/360
                                bearing >= min_bearing || bearing <= max_bearing
                            };

                            if !in_coverage {
                                continue;  // Target outside Search coverage
                            }
                        }

                        if let Some(detection) =
                            Self::check_radar_station_detection(station, missile, config, None, band, current_sim_time)
                        {
                            self.active_detections.push(detection);
                        }
                    }
                }

                // Generate false alarms (clutter/noise)
                let false_alarms = Self::generate_false_alarms(
                    station.id,
                    SensorKind::GroundRadar,
                    station.position,
                    station.detection_range_km,
                    BASE_FALSE_ALARM_RATE,
                );
                self.active_detections.extend(false_alarms);
            }
        }

        // Process satellites (simplified timing - assume continuous coverage)
        // Satellites in GEO have continuous view, LEO satellites would need orbit timing
        for satellite in satellites {
            // Satellites have high track capacity (100 default)
            track_limits.insert(satellite.id, 100);

            // Use a default update rate for satellites (5 Hz)
            if self.should_scan(satellite.id, 5.0, dt) {
                self.active_detections
                    .retain(|d| d.sensor_id != satellite.id);

                for missile in &active_missiles {
                    if satellite.affiliation == missile.affiliation {
                        continue;
                    }
                    if let Some(detection) = Self::check_satellite_detection(satellite, missile) {
                        self.active_detections.push(detection);
                    }
                }
            }
        }

        // Update tracking states with track limits
        self.update_tracks(&track_limits, dt, current_sim_time, defense_units, radar_stations, satellites);
    }

    /// Check if a sensor should perform a scan this tick
    /// Returns true and resets accumulator if scan period has elapsed
    fn should_scan(&mut self, sensor_id: EntityId, update_rate_hz: f64, dt: f64) -> bool {
        let scan_period = 1.0 / update_rate_hz.max(0.1); // Avoid division by zero

        let accumulator = self.scan_accumulators.entry(sensor_id).or_insert(0.0);
        *accumulator += dt;

        if *accumulator >= scan_period {
            *accumulator %= scan_period; // Reset keeping remainder for accuracy
            true
        } else {
            false
        }
    }

    /// Generate false alarm detections (clutter/noise)
    /// Returns 0-2 false alarms based on probability
    fn generate_false_alarms(
        sensor_id: EntityId,
        sensor_type: SensorKind,
        _sensor_position: GeoCoord,
        detection_range_km: f64,
        false_alarm_rate: f64, // Probability per scan (0.0 to 1.0)
    ) -> Vec<Detection> {
        let mut rng = rand::thread_rng();
        let mut false_alarms = Vec::new();

        // Check if we generate any false alarms this scan
        if rng.gen::<f64>() > false_alarm_rate {
            return false_alarms;
        }

        // Generate 1-2 false alarms
        let num_alarms = if rng.gen::<f64>() < 0.7 { 1 } else { 2 };

        for _ in 0..num_alarms {
            // Random bearing and range within detection envelope
            let bearing = rng.gen_range(0.0..360.0);
            let range = rng.gen_range(detection_range_km * 0.3..detection_range_km * 0.9);
            // False alarms typically appear at medium altitudes
            let altitude = rng.gen_range(50.0..300.0);
            // Low quality - these are noise
            let quality = rng.gen_range(0.1..0.3);

            false_alarms.push(Detection {
                sensor_id,
                sensor_type,
                target_id: u64::MAX, // Special ID for false alarms
                detection_quality: quality,
                bearing_deg: bearing,
                range_km: range,
                altitude_km: altitude,
                is_false_alarm: true,
                radar_band: None,
                radar_measurement: None,
            });
        }

        false_alarms
    }

    /// Check if a defense unit can detect a missile (probabilistic)
    fn check_defense_unit_detection(
        unit: &DefenseUnit,
        missile: &Missile,
        config: &SensorConfig,
        radar_mode: Option<RadarMode>,
        band: RadarBand,
        timestamp: f64,
    ) -> Option<Detection> {
        // Determine effective mode (default to Search for mechanical radars)
        let mode = radar_mode.unwrap_or(RadarMode::Search);

        // Calculate effective range based on mode (using radar-specific multipliers if configured)
        let base_range = config.detection.detection_range_km;
        let effective_range = base_range * config.detection.get_range_multiplier(mode);

        // Calculate range based on mode (slant vs ground)
        let range_km = if mode.uses_slant_range() {
            calculate_slant_range(unit.position, 0.0, missile.position, missile.altitude_km)
        } else {
            haversine_distance(unit.position, missile.position)
        };

        // Early exit if way out of range
        if range_km > effective_range * 1.5 {
            return None;
        }

        // Calculate bearing (always use ground distance for bearing)
        let bearing = calculate_bearing(unit.position, missile.position);

        // Check altitude constraints (ground radars have horizon limits)
        let ground_range = haversine_distance(unit.position, missile.position);
        let horizon_angle = calculate_horizon_angle(ground_range, missile.altitude_km);
        if horizon_angle < config.detection.elevation_min_deg.max(2.0) {
            return None;
        }

        // Calculate detection probability using RCS and atmospheric factors
        let mut p_detect = calculate_detection_probability(
            range_km,
            effective_range,  // Use effective range, not base range
            missile.current_rcs_dbsm(),
            config.tracking.minimum_rcs_dbsm,
            missile.altitude_km,
            band.attenuation_coefficient(),
            band.quality_multiplier(),
        );

        // Apply EW/jamming effect from countermeasures
        // Deployed decoys create noise that degrades radar tracking
        if missile.decoys_deployed > 0 {
            let jamming_factor = 0.7_f64.powi(missile.decoys_deployed as i32);
            p_detect *= jamming_factor;
        }

        // Probabilistic detection roll
        let mut rng = rand::thread_rng();
        if rng.gen::<f64>() > p_detect {
            return None; // Detection failed this scan
        }

        // Detection successful - use probability as quality indicator
        // Create radar measurement for EKF (always use slant range)
        let slant_range = calculate_slant_range(unit.position, 0.0, missile.position, missile.altitude_km);
        let radar_measurement = Some(Self::create_radar_measurement(
            unit.position,
            0.0, // Ground-based unit
            slant_range,
            bearing,
            missile.altitude_km,
            timestamp,
            p_detect.max(0.1),
        ));

        Some(Detection {
            sensor_id: unit.id,
            sensor_type: SensorKind::DefenseUnitRadar,
            target_id: missile.id,
            detection_quality: p_detect.max(0.1),
            bearing_deg: bearing,
            range_km,
            altitude_km: missile.altitude_km,
            is_false_alarm: false,
            radar_band: Some(band),
            radar_measurement,
        })
    }

    /// Check if a radar station can detect a missile (probabilistic)
    fn check_radar_station_detection(
        station: &RadarStation,
        missile: &Missile,
        config: &SensorConfig,
        radar_mode: Option<RadarMode>,
        band: RadarBand,
        timestamp: f64,
    ) -> Option<Detection> {
        // Determine effective mode (default to Search for mechanical radars)
        let mode = radar_mode.unwrap_or(RadarMode::Search);

        // Calculate effective range based on mode (using radar-specific multipliers if configured)
        let base_range = config.detection.detection_range_km;
        let effective_range = base_range * config.detection.get_range_multiplier(mode);

        // Calculate range based on mode (slant vs ground)
        let range_km = if mode.uses_slant_range() {
            calculate_slant_range(station.position, 0.0, missile.position, missile.altitude_km)
        } else {
            haversine_distance(station.position, missile.position)
        };

        // Early exit if way out of range
        if range_km > effective_range * 1.5 {
            return None;
        }

        // Calculate bearing (always use ground distance for bearing)
        let bearing = calculate_bearing(station.position, missile.position);

        // Check if target is within radar's azimuth coverage
        if !station.is_bearing_in_coverage(bearing) {
            return None;
        }

        // Check elevation constraints (use ground range for angles)
        let ground_range = haversine_distance(station.position, missile.position);
        let elevation = calculate_elevation_angle(ground_range, missile.altitude_km);
        if elevation < config.detection.elevation_min_deg
            || elevation > config.detection.elevation_max_deg
        {
            return None;
        }

        // Check horizon
        let horizon_angle = calculate_horizon_angle(ground_range, missile.altitude_km);
        if horizon_angle < config.detection.elevation_min_deg {
            return None;
        }

        // Calculate detection probability using RCS and atmospheric factors
        let mut p_detect = calculate_detection_probability(
            range_km,
            effective_range,  // Use effective range, not base range
            missile.current_rcs_dbsm(),
            config.tracking.minimum_rcs_dbsm,
            missile.altitude_km,
            band.attenuation_coefficient(),
            band.quality_multiplier(),
        );

        // Apply EW/jamming effect from countermeasures
        if missile.decoys_deployed > 0 {
            let jamming_factor = 0.7_f64.powi(missile.decoys_deployed as i32);
            p_detect *= jamming_factor;
        }

        // Probabilistic detection roll
        let mut rng = rand::thread_rng();
        if rng.gen::<f64>() > p_detect {
            return None; // Detection failed this scan
        }

        // Create radar measurement for EKF (always use slant range)
        let slant_range = calculate_slant_range(station.position, 0.0, missile.position, missile.altitude_km);
        let radar_measurement = Some(Self::create_radar_measurement(
            station.position,
            0.0, // Ground-based station
            slant_range,
            bearing,
            missile.altitude_km,
            timestamp,
            p_detect.max(0.1),
        ));

        Some(Detection {
            sensor_id: station.id,
            sensor_type: SensorKind::GroundRadar,
            target_id: missile.id,
            detection_quality: p_detect.max(0.1),
            bearing_deg: bearing,
            range_km,
            altitude_km: missile.altitude_km,
            is_false_alarm: false,
            radar_band: Some(band),
            radar_measurement,
        })
    }

    /// Check if a satellite can detect a missile (probabilistic)
    /// Satellites use IR or radar - IR is better during boost phase
    fn check_satellite_detection(satellite: &Satellite, missile: &Missile) -> Option<Detection> {
        let ground_range = haversine_distance(satellite.position, missile.position);
        let coverage_radius = satellite.coverage_radius_km();

        if ground_range > coverage_radius * 1.2 {
            return None;
        }

        // Base detection probability from range
        let range_ratio = ground_range / coverage_radius;
        let base_p = if range_ratio <= 1.0 {
            1.0 - range_ratio.powi(2)
        } else {
            0.0
        };

        // IR satellites are particularly good at detecting boost phase
        // (hot exhaust plume is very visible in IR)
        let phase_bonus = match (satellite.sensor_type, missile.status) {
            (SensorType::Infrared | SensorType::Both, MissileStatus::Boost) => 0.4,
            (SensorType::Infrared | SensorType::Both, MissileStatus::Midcourse) => 0.0,
            (SensorType::Infrared | SensorType::Both, MissileStatus::Terminal) => 0.1, // Reentry heating
            (SensorType::Radar, _) => {
                // Radar satellites can use RCS - smaller targets harder to see
                let rcs_factor = 10.0_f64.powf(missile.current_rcs_dbsm() / 30.0).clamp(0.3, 1.0);
                rcs_factor - 1.0 // Convert to bonus/penalty
            }
            _ => 0.0,
        };

        let p_detect = (base_p + phase_bonus).clamp(0.0, 1.0);

        // Probabilistic detection
        let mut rng = rand::thread_rng();
        if rng.gen::<f64>() > p_detect {
            return None;
        }

        let bearing = calculate_bearing(satellite.position, missile.position);

        Some(Detection {
            sensor_id: satellite.id,
            sensor_type: match satellite.sensor_type {
                SensorType::Infrared => SensorKind::SatelliteIR,
                SensorType::Radar => SensorKind::SatelliteRadar,
                SensorType::Both => SensorKind::SatelliteIR, // Primary is IR
            },
            target_id: missile.id,
            detection_quality: p_detect.max(0.1),
            bearing_deg: bearing,
            range_km: ground_range,
            altitude_km: missile.altitude_km,
            is_false_alarm: false,
            radar_band: None, // Satellites don't use multi-band switching
            radar_measurement: None,
        })
    }

    /// Get sensor position for a given sensor ID
    fn get_sensor_position(
        sensor_id: EntityId,
        defense_units: &[DefenseUnit],
        radar_stations: &[RadarStation],
        satellites: &[Satellite],
    ) -> Option<GeoCoord> {
        // Check defense units first (legacy single-sensor)
        if let Some(unit) = defense_units.iter().find(|u| u.id == sensor_id) {
            return Some(unit.position);
        }

        // Check defense unit sensors (multi-sensor platforms)
        for unit in defense_units {
            if unit.sensors.iter().any(|s| s.sensor_id == sensor_id) {
                return Some(unit.position);
            }
        }

        // Check radar stations
        if let Some(radar) = radar_stations.iter().find(|r| r.id == sensor_id) {
            return Some(radar.position);
        }

        // Check satellites
        if let Some(sat) = satellites.iter().find(|s| s.id == sensor_id) {
            return Some(sat.position);
        }

        None
    }

    /// Update tracking states - degrade quality over time and enforce track limits
    /// Implements track handoff: sensors can boost track quality when other sensors
    /// are already tracking the same target (network-aided tracking)
    fn update_tracks(
        &mut self,
        track_limits: &HashMap<EntityId, u32>,
        dt: f64,
        current_sim_time: f64,
        defense_units: &[DefenseUnit],
        radar_stations: &[RadarStation],
        satellites: &[Satellite],
    ) {
        // Update existing tracks - degrade quality over time
        for track in &mut self.active_tracks {
            track.time_since_update += dt;
            // Degrade quality over time (lose track after ~10 seconds without update)
            track.track_quality -= dt * 0.1;
        }

        // Remove dead tracks
        self.active_tracks.retain(|t| t.track_quality > 0.0);

        // Build network track consensus - for each target, find the best track quality
        // across all sensors. This enables track handoff between sensors.
        let network_track_quality: HashMap<EntityId, f64> = {
            let mut best_quality: HashMap<EntityId, f64> = HashMap::new();
            for track in &self.active_tracks {
                let entry = best_quality.entry(track.target_id).or_insert(0.0);
                *entry = entry.max(track.track_quality);
            }
            best_quality
        };

        // Group detections by sensor and sort by quality (highest first)
        // Skip false alarms - they shouldn't become tracks
        let mut detections_by_sensor: HashMap<EntityId, Vec<&Detection>> = HashMap::new();
        for detection in &self.active_detections {
            if detection.is_false_alarm {
                continue; // Don't promote false alarms to tracks
            }
            detections_by_sensor
                .entry(detection.sensor_id)
                .or_default()
                .push(detection);
        }

        // Sort each sensor's detections by quality (best first)
        // Prioritize targets already in the network track
        for detections in detections_by_sensor.values_mut() {
            detections.sort_by(|a, b| {
                // Boost priority for targets already being tracked network-wide
                let a_network_bonus = if network_track_quality.contains_key(&a.target_id) {
                    0.2
                } else {
                    0.0
                };
                let b_network_bonus = if network_track_quality.contains_key(&b.target_id) {
                    0.2
                } else {
                    0.0
                };

                let a_score = a.detection_quality + a_network_bonus;
                let b_score = b.detection_quality + b_network_bonus;

                b_score
                    .partial_cmp(&a_score)
                    .unwrap_or(std::cmp::Ordering::Equal)
            });
        }

        // Process detections per sensor with track limits
        for (sensor_id, detections) in detections_by_sensor {
            let max_tracks = *track_limits.get(&sensor_id).unwrap_or(&50) as usize;

            for detection in detections {
                // Check if we're already tracking this target with THIS sensor
                if let Some(track) = self.active_tracks.iter_mut().find(|t| {
                    t.tracker_id == detection.sensor_id && t.target_id == detection.target_id
                }) {
                    // Update existing track (doesn't count against limit)
                    track.track_quality =
                        (track.track_quality + detection.detection_quality * 0.5).min(1.0);
                    track.time_since_update = 0.0;

                    // Calculate position from sensor bearing/range
                    if let Some(sensor_pos) =
                        Self::get_sensor_position(detection.sensor_id, defense_units, radar_stations, satellites)
                    {
                        let measured_position = calculate_position_from_bearing_range(
                            sensor_pos,
                            detection.bearing_deg,
                            detection.range_km,
                        );

                        // Update predicted position
                        track.predicted_position = measured_position;
                        track.predicted_altitude = detection.altitude_km;

                        // Update filter based on filter type
                        match self.filter_type {
                            FilterType::ExtendedKalman => {
                                // Use EKF with raw radar measurement
                                if let Some(ref radar_meas) = detection.radar_measurement {
                                    track.add_radar_measurement(radar_meas);
                                } else {
                                    // Fallback to position measurement if no radar data
                                    track.add_position_measurement(
                                        current_sim_time,
                                        measured_position,
                                        detection.altitude_km,
                                        detection.detection_quality,
                                    );
                                }
                            }
                            FilterType::LinearKalman => {
                                // Use linear KF with converted position
                                track.add_position_measurement(
                                    current_sim_time,
                                    measured_position,
                                    detection.altitude_km,
                                    detection.detection_quality,
                                );
                            }
                        }

                        // Update velocity estimate
                        track.update_velocity_estimate(current_sim_time);
                    }
                } else {
                    // New detection for this sensor - check if we can start tracking

                    // Track handoff boost: if another sensor is tracking this target,
                    // we get a quality boost (network-aided acquisition)
                    let network_boost = network_track_quality
                        .get(&detection.target_id)
                        .map(|q| q * 0.3) // 30% of network track quality as bonus
                        .unwrap_or(0.0);

                    let effective_quality = detection.detection_quality + network_boost;

                    // Lower threshold for network-tracked targets (0.2 vs 0.3)
                    let quality_threshold = if network_boost > 0.0 { 0.2 } else { 0.3 };

                    if effective_quality > quality_threshold {
                        // Check track limit before creating new track
                        let current_track_count = self
                            .active_tracks
                            .iter()
                            .filter(|t| t.tracker_id == sensor_id)
                            .count();

                        if current_track_count < max_tracks {
                            // Calculate initial position from sensor bearing/range
                            let (init_position, init_history) =
                                if let Some(sensor_pos) = Self::get_sensor_position(
                                    detection.sensor_id,
                                    defense_units,
                                    radar_stations,
                                    satellites,
                                ) {
                                    let measured_position = calculate_position_from_bearing_range(
                                        sensor_pos,
                                        detection.bearing_deg,
                                        detection.range_km,
                                    );
                                    let history = vec![PositionMeasurement {
                                        timestamp: current_sim_time,
                                        position: measured_position,
                                        altitude_km: detection.altitude_km,
                                        measurement_quality: detection.detection_quality,
                                    }];
                                    (measured_position, history)
                                } else {
                                    (GeoCoord::default(), Vec::new())
                                };

                            // Initialize filter based on filter type
                            let (kalman_filter, ekf_state) = match self.filter_type {
                                FilterType::ExtendedKalman => {
                                    // Initialize EKF with radar measurement if available
                                    let ekf = detection.radar_measurement.as_ref().map(|meas| {
                                        crate::simulation::ekf::EKFState::new(meas)
                                    });
                                    (None, ekf)
                                }
                                FilterType::LinearKalman => {
                                    // Initialize linear KF from position measurement
                                    let kf = init_history.first().map(|meas| {
                                        BallisticState::new(meas)
                                    });
                                    (kf, None)
                                }
                            };

                            // Create new track with network boost applied
                            self.active_tracks.push(TrackingState {
                                tracker_id: detection.sensor_id,
                                target_id: detection.target_id,
                                track_quality: effective_quality.min(1.0),
                                time_since_update: 0.0,
                                predicted_position: init_position,
                                predicted_altitude: detection.altitude_km,
                                position_history: init_history,
                                estimated_velocity: None,
                                estimated_heading_deg: None,
                                kalman_filter,
                                ekf_state,
                            });
                        }
                        // If at limit, detection is dropped (sensor saturated)
                    }
                }
            }
        }
    }

    /// Get all detections for a specific sensor
    pub fn detections_for_sensor(&self, sensor_id: EntityId) -> Vec<&Detection> {
        self.active_detections
            .iter()
            .filter(|d| d.sensor_id == sensor_id)
            .collect()
    }

    /// Get all detections of a specific target
    pub fn detections_of_target(&self, target_id: EntityId) -> Vec<&Detection> {
        self.active_detections
            .iter()
            .filter(|d| d.target_id == target_id)
            .collect()
    }

    /// Check if a target is being tracked by any friendly sensor
    pub fn is_target_tracked(&self, target_id: EntityId) -> bool {
        self.active_tracks.iter().any(|t| t.target_id == target_id)
    }

    /// Get the best track quality for a target
    pub fn best_track_quality(&self, target_id: EntityId) -> f64 {
        self.active_tracks
            .iter()
            .filter(|t| t.target_id == target_id)
            .map(|t| t.track_quality)
            .fold(0.0, f64::max)
    }

    /// Get the best Kalman state for a target (from track with most measurements)
    /// Used for filtered intercept calculations
    pub fn get_kalman_state(&self, target_id: EntityId) -> Option<&BallisticState> {
        self.active_tracks
            .iter()
            .filter(|t| t.target_id == target_id && t.kalman_filter.is_some())
            .max_by_key(|t| t.position_history.len())
            .and_then(|t| t.kalman_filter.as_ref())
    }

    /// Get best EKF state for a target (from highest-quality track)
    pub fn get_ekf_state(&self, target_id: EntityId) -> Option<&crate::simulation::ekf::EKFState> {
        self.active_tracks
            .iter()
            .filter(|t| t.target_id == target_id && t.ekf_state.is_some())
            .max_by_key(|t| t.position_history.len())
            .and_then(|t| t.ekf_state.as_ref())
    }

    /// Get a fused track for a target, combining all sensor tracks
    /// Returns None if no sensors are tracking this target
    /// defense_unit_ids: IDs of defense units (which have fire control radars)
    pub fn get_fused_track(&self, target_id: EntityId, defense_unit_ids: &HashSet<EntityId>) -> Option<FusedTrack> {
        let tracks: Vec<&TrackingState> = self
            .active_tracks
            .iter()
            .filter(|t| t.target_id == target_id)
            .collect();

        if tracks.is_empty() {
            return None;
        }

        // Check if any defense unit (fire control radar) is tracking this target
        let has_fire_control_lock = tracks.iter().any(|track| {
            defense_unit_ids.contains(&track.tracker_id)
        });

        // Weighted position fusion based on track quality
        let total_weight: f64 = tracks.iter().map(|t| t.track_quality).sum();
        if total_weight <= 0.0 {
            return None;
        }

        let mut lat_sum = 0.0;
        let mut lon_sum = 0.0;
        let mut alt_sum = 0.0;

        for track in &tracks {
            let weight = track.track_quality / total_weight;
            lat_sum += track.predicted_position.lat * weight;
            lon_sum += track.predicted_position.lon * weight;
            alt_sum += track.predicted_altitude * weight;
        }

        // Calculate fused quality (multiple sensors improve confidence)
        let fused_quality = Self::calculate_fused_quality(&tracks);

        // Calculate uncertainty radius based on quality and staleness
        let avg_staleness =
            tracks.iter().map(|t| t.time_since_update).sum::<f64>() / tracks.len() as f64;
        let uncertainty_radius_km =
            Self::calculate_uncertainty_radius(fused_quality, avg_staleness, tracks.len());

        // Fuse velocity estimates from all tracks
        let velocity_estimates: Vec<&VelocityEstimate> = tracks
            .iter()
            .filter_map(|t| t.estimated_velocity.as_ref())
            .collect();

        let (fused_velocity, predicted_heading) = if !velocity_estimates.is_empty() {
            // Weighted average by confidence
            let total_confidence: f64 = velocity_estimates.iter().map(|v| v.confidence).sum();
            if total_confidence > 0.0 {
                let avg_speed = velocity_estimates
                    .iter()
                    .map(|v| v.ground_speed_km_s * v.confidence)
                    .sum::<f64>()
                    / total_confidence;

                let avg_heading_x = velocity_estimates
                    .iter()
                    .map(|v| v.heading_deg.to_radians().cos() * v.confidence)
                    .sum::<f64>()
                    / total_confidence;
                let avg_heading_y = velocity_estimates
                    .iter()
                    .map(|v| v.heading_deg.to_radians().sin() * v.confidence)
                    .sum::<f64>()
                    / total_confidence;
                let avg_heading = avg_heading_y
                    .atan2(avg_heading_x)
                    .to_degrees()
                    .rem_euclid(360.0);

                let avg_vertical = velocity_estimates
                    .iter()
                    .map(|v| v.vertical_rate_km_s * v.confidence)
                    .sum::<f64>()
                    / total_confidence;

                let fused_vel = Some(VelocityEstimate {
                    ground_speed_km_s: avg_speed,
                    heading_deg: avg_heading,
                    vertical_rate_km_s: avg_vertical,
                    confidence: total_confidence / velocity_estimates.len() as f64,
                    staleness: avg_staleness,
                });

                (fused_vel, avg_heading)
            } else {
                (None, 0.0)
            }
        } else {
            (None, 0.0)
        };

        // Find best track for measurement count and Kalman uncertainty
        let best_track = tracks.iter()
            .max_by_key(|t| t.position_history.len())
            .unwrap();

        let measurement_count = best_track.position_history.len();
        let kalman_position_uncertainty_km = best_track.kalman_filter.as_ref()
            .map(|kf| kf.get_position_uncertainty());

        Some(FusedTrack {
            target_id,
            estimated_position: GeoCoord::new(lat_sum, lon_sum),
            estimated_altitude: alt_sum,
            fused_quality,
            uncertainty_radius_km,
            sensor_count: tracks.len(),
            staleness_seconds: avg_staleness,
            predicted_heading_deg: predicted_heading,
            estimated_velocity: fused_velocity,
            measurement_count,
            kalman_position_uncertainty_km,
            has_fire_control_lock,
        })
    }

    /// Get all targets currently being tracked as fused tracks
    pub fn get_all_fused_tracks(&self, defense_unit_ids: &HashSet<EntityId>) -> Vec<FusedTrack> {
        // Get unique target IDs
        let target_ids: std::collections::HashSet<EntityId> = self
            .active_tracks
            .iter()
            .map(|t| t.target_id)
            .collect();

        target_ids
            .into_iter()
            .filter_map(|id| self.get_fused_track(id, defense_unit_ids))
            .collect()
    }

    /// Calculate fused quality from multiple tracks
    /// Multiple sensors tracking same target improves confidence
    fn calculate_fused_quality(tracks: &[&TrackingState]) -> f64 {
        if tracks.is_empty() {
            return 0.0;
        }

        // Best single track quality
        let best_quality = tracks.iter().map(|t| t.track_quality).fold(0.0, f64::max);

        // Bonus for multiple sensors (up to 20% bonus for 3+ sensors)
        let multi_sensor_bonus = match tracks.len() {
            1 => 0.0,
            2 => 0.10,
            _ => 0.20,
        };

        (best_quality + multi_sensor_bonus).min(1.0)
    }

    /// Calculate position uncertainty radius based on track quality and staleness
    /// Returns uncertainty in kilometers
    fn calculate_uncertainty_radius(quality: f64, staleness_seconds: f64, sensor_count: usize) -> f64 {
        // Base uncertainty: 5km at quality 1.0, 50km at quality 0.0
        let base_uncertainty = 5.0 + (1.0 - quality) * 45.0;

        // Uncertainty grows with staleness (missiles move ~7km/s, uncertainty grows)
        // Add ~2km per second of staleness at low quality
        let staleness_factor = staleness_seconds * (1.0 - quality) * 2.0;

        // Multi-sensor bonus reduces uncertainty
        let multi_sensor_factor = match sensor_count {
            1 => 1.0,
            2 => 0.9,
            _ => 0.8,
        };

        (base_uncertainty + staleness_factor) * multi_sensor_factor
    }

    /// Calculate threat priority score for a tracked target
    /// Higher score = higher threat (closer, faster, better track quality)
    fn calculate_threat_score(
        &self,
        track: &TrackingState,
        sensor_position: GeoCoord,
        _sensor_id: EntityId,
    ) -> f64 {
        let mut score = 0.0;

        // Factor 1: Range (closer = higher threat, inverse relationship)
        if let Some(latest) = track.position_history.first() {
            let range_km = haversine_distance(sensor_position, latest.position);
            // Closer targets score higher: 1000km range = 1.0, 100km = 10.0
            score += 1000.0 / range_km.max(10.0);
        }

        // Factor 2: Closing velocity (faster approach = higher threat)
        if let Some(vel) = &track.estimated_velocity {
            if let Some(latest) = track.position_history.first() {
                // Calculate bearing from sensor to target
                let target_bearing = calculate_bearing(sensor_position, latest.position);
                // Velocity toward sensor: 0° difference = max closing, 180° = opening
                let bearing_diff = (vel.heading_deg - target_bearing).abs();
                let closing_factor = (180.0 - bearing_diff) / 180.0;  // 1.0 = directly approaching
                let closing_speed = vel.ground_speed_km_s * closing_factor;
                score += closing_speed.max(0.0) * 100.0;
            }
        }

        // Factor 3: Track quality (confident tracks prioritized)
        score += track.track_quality * 50.0;

        // Factor 4: Velocity confidence (stable velocity = reliable threat assessment)
        if let Some(vel) = &track.estimated_velocity {
            score += vel.confidence * 30.0;
        }

        score
    }

    /// Determine what mode a radar should use for a specific target
    fn determine_radar_mode(
        &self,
        _target_id: EntityId,
        has_interceptors_inflight: bool,
        track_quality: f64,
        sensor_role: SensorRole,
    ) -> RadarMode {
        // Priority 1: Fire control for targets with interceptors
        if has_interceptors_inflight {
            return RadarMode::FireControl;
        }

        // Priority 2: Role-based preference
        match sensor_role {
            SensorRole::FireControl => RadarMode::FireControl,
            SensorRole::Tracking if track_quality >= 0.6 => RadarMode::Track,
            SensorRole::Surveillance => RadarMode::Search,
            SensorRole::MultiRole if track_quality >= 0.6 => RadarMode::Track,
            _ => RadarMode::Search,
        }
    }

    /// Update radar mode assignments for a phased array sensor
    /// Must be called BEFORE detection checks each frame
    /// Enforces both count limits AND time budget constraints
    fn update_radar_modes(
        &mut self,
        sensor_id: EntityId,
        sensor_position: GeoCoord,
        max_simultaneous_tracks: u32,
        max_fire_control_tracks: u32,
        interceptors: &[Interceptor],
        radar_type: RadarType,
        tracking_config: &SensorTrackingConfig,
        config: &SensorConfig,
    ) {
        use std::collections::HashSet;

        // Only phased arrays support multi-mode
        if radar_type != RadarType::PhasedArray {
            return;
        }

        // Build set of targets with in-flight interceptors
        let targets_with_interceptors: HashSet<EntityId> = interceptors
            .iter()
            .filter(|i| i.status == InterceptorStatus::InFlight)
            .map(|i| i.target_id)
            .collect();

        // Get all tracks for this sensor
        let sensor_tracks: Vec<&TrackingState> = self.active_tracks
            .iter()
            .filter(|t| t.tracker_id == sensor_id)
            .collect();

        // Calculate threat scores for priority sorting
        let mut threats: Vec<(EntityId, f64, bool)> = sensor_tracks
            .iter()
            .map(|track| {
                let threat_score = self.calculate_threat_score(track, sensor_position, sensor_id);
                let has_interceptor = targets_with_interceptors.contains(&track.target_id);
                (track.target_id, threat_score, has_interceptor)
            })
            .collect();

        // Sort by threat score descending (highest threat first)
        threats.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap_or(std::cmp::Ordering::Equal));

        // Calculate time budget for this scan cycle (seconds)
        let scan_period_sec = 1.0 / tracking_config.track_update_rate_hz.max(0.1);
        let mut time_budget_remaining = scan_period_sec;

        // Get or create mode state (AFTER all threat calculations to avoid borrow conflicts)
        let mode_state = self.radar_mode_states
            .entry(sensor_id)
            .or_insert_with(|| RadarModeState::new(sensor_id));

        // Store previous assignments for statistics tracking
        let previous_assignments = mode_state.target_assignments.clone();

        // Clear old assignments
        mode_state.target_assignments.clear();

        // Track assignments by mode for debugging
        let mut fc_count = 0;
        let mut track_count = 0;
        let mut search_count = 0;

        // Assign modes based on priority, respecting BOTH count limits AND time budget
        for (target_id, _, has_interceptor) in threats.iter() {
            // Check total track limit
            if mode_state.target_assignments.len() >= max_simultaneous_tracks as usize {
                break;  // Hard limit on total tracks
            }

            // Determine desired mode based on priority
            let desired_mode = if *has_interceptor {
                // Priority 1: Fire control for targets with interceptors
                if fc_count >= max_fire_control_tracks {
                    continue;  // FC capacity exhausted, skip this target
                }
                RadarMode::FireControl
            } else {
                // Priority 2/3: Track or Search based on quality
                let track_quality = self.active_tracks
                    .iter()
                    .find(|t| t.target_id == *target_id && t.tracker_id == sensor_id)
                    .map(|t| t.track_quality)
                    .unwrap_or(0.0);

                if track_quality >= 0.6 {
                    RadarMode::Track
                } else {
                    RadarMode::Search
                }
            };

            // Check if we have time budget for this mode
            let dwell_time = tracking_config.get_dwell_time_sec(desired_mode);
            if dwell_time > time_budget_remaining {
                // Not enough time left - try degrading to lower mode
                if desired_mode == RadarMode::Track {
                    // Try Search instead
                    let search_dwell = tracking_config.get_dwell_time_sec(RadarMode::Search);
                    if search_dwell <= time_budget_remaining {
                        // Can fit in Search mode
                        let band = config.detection.get_band_for_mode(RadarMode::Search);
                        mode_state.target_assignments.insert(*target_id, (RadarMode::Search, band));
                        time_budget_remaining -= search_dwell;
                        search_count += 1;
                    }
                    // else: can't fit at all, skip
                } else if desired_mode == RadarMode::FireControl {
                    // FireControl is critical - try to fit by degrading to Track
                    let track_dwell = tracking_config.get_dwell_time_sec(RadarMode::Track);
                    if track_dwell <= time_budget_remaining {
                        let band = config.detection.get_band_for_mode(RadarMode::Track);
                        mode_state.target_assignments.insert(*target_id, (RadarMode::Track, band));
                        time_budget_remaining -= track_dwell;
                        track_count += 1;
                    } else {
                        // Last resort: try Search
                        let search_dwell = tracking_config.get_dwell_time_sec(RadarMode::Search);
                        if search_dwell <= time_budget_remaining {
                            let band = config.detection.get_band_for_mode(RadarMode::Search);
                            mode_state.target_assignments.insert(*target_id, (RadarMode::Search, band));
                            time_budget_remaining -= search_dwell;
                            search_count += 1;
                        }
                    }
                    // else: can't fit at all, critical target dropped!
                }
                // Search mode can't be degraded further - skip if no time
            } else {
                // Sufficient time budget - assign desired mode
                let band = config.detection.get_band_for_mode(desired_mode);
                mode_state.target_assignments.insert(*target_id, (desired_mode, band));
                time_budget_remaining -= dwell_time;

                match desired_mode {
                    RadarMode::FireControl => fc_count += 1,
                    RadarMode::Track => track_count += 1,
                    RadarMode::Search => search_count += 1,
                }
            }
        }

        // Targets not in target_modes are dropped (beyond capacity or time budget)

        // Update statistics
        // Count dropped targets
        let targets_assigned = mode_state.target_assignments.len();
        let targets_total = sensor_tracks.len();
        let dropped_this_scan = targets_total.saturating_sub(targets_assigned);
        mode_state.stats.targets_dropped_count = mode_state.stats.targets_dropped_count.saturating_add(dropped_this_scan as u32);

        // Track mode switches
        for (target_id, (new_mode, _new_band)) in &mode_state.target_assignments {
            if let Some(&(old_mode, _old_band)) = previous_assignments.get(target_id) {
                if old_mode != *new_mode {
                    mode_state.stats.mode_switch_count += 1;
                }
            }
        }

        // Calculate time budget utilization
        let total_budget = scan_period_sec;
        let used_budget = total_budget - time_budget_remaining;
        mode_state.stats.last_time_budget_utilization = used_budget / total_budget;

        // Update mode counts
        mode_state.stats.last_fc_count = fc_count;
        mode_state.stats.last_track_count = track_count;
        mode_state.stats.last_search_count = search_count;
    }
}

/// Calculate bearing from one point to another (degrees, 0 = North)
/// Calculate bearing from point A to point B in degrees (0 = North, 90 = East)
pub fn calculate_bearing(from: GeoCoord, to: GeoCoord) -> f64 {
    let lat1 = from.lat.to_radians();
    let lat2 = to.lat.to_radians();
    let delta_lon = (to.lon - from.lon).to_radians();

    let y = delta_lon.sin() * lat2.cos();
    let x = lat1.cos() * lat2.sin() - lat1.sin() * lat2.cos() * delta_lon.cos();

    y.atan2(x).to_degrees().rem_euclid(360.0)
}

/// Calculate the elevation angle to a target at given range and altitude
fn calculate_elevation_angle(range_km: f64, altitude_km: f64) -> f64 {
    if range_km <= 0.0 {
        return 90.0;
    }
    (altitude_km / range_km).atan().to_degrees()
}

/// Calculate the angle above the horizon for a target
/// Takes Earth curvature into account
fn calculate_horizon_angle(range_km: f64, altitude_km: f64) -> f64 {
    const EARTH_RADIUS_KM: f64 = 6371.0;

    if range_km <= 0.0 {
        return 90.0;
    }

    // Angle that the ground dips below horizontal at this range
    let earth_dip = (range_km / EARTH_RADIUS_KM).asin().to_degrees();

    // Simple elevation angle
    let elevation = (altitude_km / range_km).atan().to_degrees();

    // The apparent elevation above horizon
    elevation + earth_dip
}

/// Calculate line-of-sight distance considering Earth curvature
pub fn line_of_sight_range(observer_altitude_km: f64, target_altitude_km: f64) -> f64 {
    const EARTH_RADIUS_KM: f64 = 6371.0;

    // Distance to horizon for observer
    let d1 = (2.0 * EARTH_RADIUS_KM * observer_altitude_km + observer_altitude_km.powi(2)).sqrt();

    // Distance to horizon for target
    let d2 = (2.0 * EARTH_RADIUS_KM * target_altitude_km + target_altitude_km.powi(2)).sqrt();

    d1 + d2
}

/// Calculate atmospheric attenuation factor for radar signals
/// Returns 1.0 for no attenuation, approaches 0.0 for heavy attenuation
/// Takes radar band into account - higher frequencies attenuate more
fn calculate_atmospheric_attenuation(range_km: f64, target_altitude_km: f64, attenuation_coeff: f64) -> f64 {
    // Atmospheric scale height (km) - density halves every ~8.5 km
    const SCALE_HEIGHT_KM: f64 = 8.5;

    // Calculate average altitude of the radar-to-target path
    // Simplified: assume straight line, average altitude is half target altitude
    let avg_path_altitude = target_altitude_km / 2.0;

    // Atmospheric density falls off exponentially with altitude
    let density_factor = (-avg_path_altitude / SCALE_HEIGHT_KM).exp();

    // Effective attenuation coefficient at average path altitude
    let effective_coeff = attenuation_coeff * density_factor;

    // Path length through atmosphere (limited by target altitude)
    let atmospheric_path_km = range_km.min(target_altitude_km * 10.0);

    // Total attenuation in dB
    let attenuation_db = effective_coeff * atmospheric_path_km;

    // Convert to linear factor (0 dB = 1.0, -3 dB ≈ 0.5)
    10.0_f64.powf(-attenuation_db / 10.0).clamp(0.0, 1.0)
}

/// Calculate slant range (3D distance) between sensor and target
/// slant_range = sqrt(ground_range² + altitude_diff²)
pub fn calculate_slant_range(
    sensor_pos: GeoCoord,
    sensor_altitude_km: f64,
    target_pos: GeoCoord,
    target_altitude_km: f64,
) -> f64 {
    let ground_range = haversine_distance(sensor_pos, target_pos);
    let altitude_diff = target_altitude_km - sensor_altitude_km;
    (ground_range.powi(2) + altitude_diff.powi(2)).sqrt()
}

/// Calculate detection probability based on radar equation factors
/// Returns probability 0.0 to 1.0
fn calculate_detection_probability(
    range_km: f64,
    max_range_km: f64,
    target_rcs_dbsm: f64,
    sensor_min_rcs_dbsm: f64,
    altitude_km: f64,
    attenuation_coeff: f64,
    quality_multiplier: f64,
) -> f64 {
    // Early exit if way out of range
    if range_km > max_range_km * 1.5 {
        return 0.0;
    }

    // 1. Range factor: P decreases with R^4 (radar equation)
    // At nominal max_range, detection probability should be ~50%
    // Quality degrades smoothly from 100% at close range to 50% at max range
    let range_ratio = range_km / max_range_km;
    let range_factor = if range_ratio <= 1.0 {
        // Smooth degradation: 100% at 0 range, 50% at max_range
        // Using inverse power law: quality = 1 / (1 + range_ratio^2)
        1.0 / (1.0 + range_ratio.powi(2))
    } else {
        // Beyond max range: exponential falloff
        let excess_ratio = range_ratio - 1.0;
        0.5 * (-3.0 * excess_ratio).exp()
    };

    // 2. RCS factor: Higher RCS = easier detection
    // Each 10 dB increase in RCS roughly doubles detection range
    let rcs_advantage_db = target_rcs_dbsm - sensor_min_rcs_dbsm;
    let rcs_factor = if rcs_advantage_db >= 0.0 {
        // Target is above minimum RCS - high detection probability
        1.0
    } else {
        // Target below minimum RCS - probability reduced
        // Each -10 dB halves probability
        10.0_f64.powf(rcs_advantage_db / 20.0).clamp(0.0, 1.0)
    };

    // 3. Atmospheric attenuation (frequency-dependent)
    let attenuation_factor = calculate_atmospheric_attenuation(range_km, altitude_km, attenuation_coeff);

    // 4. Radar band quality multiplier (higher frequency = better resolution/quality)
    // Combined probability (independent factors multiply)
    (range_factor * rcs_factor * attenuation_factor * quality_multiplier).clamp(0.0, 1.0)
}

/// Calculate a position given an origin, bearing (degrees), and range (km)
pub fn calculate_position_from_bearing_range(
    origin: GeoCoord,
    bearing_deg: f64,
    range_km: f64,
) -> GeoCoord {
    const EARTH_RADIUS_KM: f64 = 6371.0;

    let lat1 = origin.lat.to_radians();
    let lon1 = origin.lon.to_radians();
    let bearing = bearing_deg.to_radians();
    let angular_distance = range_km / EARTH_RADIUS_KM;

    let lat2 = (lat1.sin() * angular_distance.cos()
        + lat1.cos() * angular_distance.sin() * bearing.cos())
    .asin();

    let lon2 = lon1
        + (bearing.sin() * angular_distance.sin() * lat1.cos())
            .atan2(angular_distance.cos() - lat1.sin() * lat2.sin());

    GeoCoord::new(lat2.to_degrees(), lon2.to_degrees())
}
