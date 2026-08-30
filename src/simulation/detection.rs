use std::cell::RefCell;
use std::collections::{HashMap, HashSet};

use rand::Rng;
use rayon::prelude::*;

use crate::simulation::config::{
    RadarBand, RadarMode, RadarType, SensorRole, SensorTrackingConfig,
};
use crate::simulation::kalman::BallisticState;
use crate::simulation::{
    bearing, haversine_distance, normalize_angle_diff, DefenseUnit, EntityId, Interceptor,
    InterceptorStatus, Missile, MissileStatus, RadarStation, Satellite, SensorConfig,
    SensorConfigRegistry, SensorType,
};
use crate::types::GeoCoord;

/// Type of Kalman filter to use for tracking
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum FilterType {
    /// Linear Kalman Filter in local ENU coordinates (simpler, faster)
    #[default]
    LinearKalman,
    /// Extended Kalman Filter in geodetic coordinates (more accurate for long range)
    /// NOTE: Enabled by default - EKF stability issues have been resolved
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

/// Maximum position measurements to retain for track history visualization
/// At 10 Hz update rate, 100 measurements = 10 seconds of track history
const MAX_POSITION_HISTORY: usize = 100;

/// Tracking state for a defense unit
#[derive(Clone, Debug)]
pub struct TrackingState {
    pub tracker_id: EntityId,
    pub target_id: EntityId,
    pub track_quality: f64, // 0.0 to 1.0, degrades without updates
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
    /// Whether EKF velocity has been initialized from position history
    pub ekf_velocity_initialized: bool,
    /// Measurements since last EKF reset (for grace period after reset)
    pub measurements_since_reset: u32,
}

/// Threshold for consecutive rejections before resetting track
/// Real radar systems drop diverged tracks after ~10-20 missed updates
const TRACK_RESET_THRESHOLD: u32 = 15;

/// Number of measurements after reset before applying strict validation
/// Allows EKF and velocity estimates to stabilize after track re-acquisition
/// Longer period needed for crossing/maneuvering targets where EKF may struggle
const RESET_GRACE_PERIOD: u32 = 25;

impl TrackingState {
    /// Reset the EKF state to recover from divergence
    /// This mimics real radar system behavior of dropping and re-acquiring tracks
    pub fn reset_ekf(&mut self) {
        self.ekf_state = None;
        self.ekf_velocity_initialized = false;
        self.kalman_filter = None;
        self.estimated_velocity = None;
        self.estimated_heading_deg = None;
        // Clear ALL position history to prevent acceleration checks using stale data
        // The track will rebuild from scratch with new measurements
        self.position_history.clear();
        // Reset measurement counter for grace period
        self.measurements_since_reset = 0;
        // Quality degrades but track isn't dropped entirely
        self.track_quality *= 0.5;
    }

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

        // Convert radar measurement to position first (needed for both branches)
        let (pos, alt) = crate::simulation::ekf::radar_to_geodetic(
            measurement.sensor_position,
            measurement.sensor_altitude_km,
            measurement.range_km,
            measurement.azimuth_rad,
            measurement.elevation_rad,
        );

        // Update EKF
        if let Some(ref mut ekf) = self.ekf_state {
            // Initialize EKF velocity from position history if not yet done
            // Must happen BEFORE predict/update cycle to avoid corrupted velocity inference
            let just_initialized =
                if !self.ekf_velocity_initialized && self.position_history.len() >= 1 {
                    // Find a measurement with sufficient time gap from current measurement
                    if let Some(older_meas) = self
                        .position_history
                        .iter()
                        .find(|m| (measurement.timestamp - m.timestamp).abs() > 0.5)
                    {
                        ekf.initialize_velocity_from_positions(
                            older_meas.position,
                            older_meas.altitude_km,
                            older_meas.timestamp,
                            pos,
                            alt,
                            measurement.timestamp,
                        );
                        // Update EKF position to match current measurement
                        ekf.x[0] = pos.lat.to_radians();
                        ekf.x[1] = pos.lon.to_radians();
                        ekf.x[2] = alt;
                        ekf.timestamp = measurement.timestamp;
                        self.ekf_velocity_initialized = true;
                        true
                    } else {
                        false
                    }
                } else {
                    false
                };

            // Skip predict/update if we just initialized - the velocity is already
            // computed from the position difference, running update would corrupt it
            if !just_initialized {
                // Predict forward to measurement time
                let dt = measurement.timestamp - ekf.timestamp;
                if dt > 0.0 {
                    ekf.predict(dt);
                }
                // Update with radar measurement
                ekf.update(measurement);
            }
        } else {
            // Initialize EKF with first measurement
            self.ekf_state = Some(EKFState::new(measurement));
        }

        // Also update position history for visualization/backup
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

        // Track measurements for grace period after reset
        self.measurements_since_reset = self.measurements_since_reset.saturating_add(1);
    }

    /// Calculate velocity estimate from EKF state or position history
    /// Prefers EKF velocity when available (more accurate, especially for crossing trajectories)
    /// Falls back to weighted least-squares over position measurements
    pub fn update_velocity_estimate(&mut self, current_time: f64) {
        // Prefer EKF velocity when available - it's properly filtered and handles
        // crossing trajectories better (where position-based heading is unstable)
        if let Some(ref ekf) = self.ekf_state {
            let (ground_speed, heading, vertical_rate) = ekf.get_velocity();

            // Sanity check: EKF velocity should be reasonable for ballistic missiles
            if ground_speed > 0.1 && ground_speed < 10.0 {
                // Confidence based on number of measurements and EKF uncertainty
                let measurement_factor =
                    (self.position_history.len() as f64 * 0.15 + 0.3).min(0.95);
                let uncertainty = ekf.get_position_uncertainty();
                // Lower uncertainty = higher confidence
                let uncertainty_factor = 1.0 / (1.0 + uncertainty * 0.1);
                let confidence = (measurement_factor * uncertainty_factor).clamp(0.3, 0.95);

                let staleness = if let Some(latest) = self.position_history.first() {
                    current_time - latest.timestamp
                } else {
                    0.0
                };

                self.estimated_velocity = Some(VelocityEstimate {
                    ground_speed_km_s: ground_speed,
                    heading_deg: heading,
                    vertical_rate_km_s: vertical_rate,
                    confidence,
                    staleness,
                });
                self.estimated_heading_deg = Some(heading);
                return;
            }
        }

        // Fallback: Use position-based velocity estimation
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

            // Require minimum 0.1 second gap for meaningful velocity calculation
            // Shorter intervals amplify noise in position measurements
            if dt < 0.1 {
                continue;
            }

            // Ground distance and direction
            let ground_distance = haversine_distance(older.position, newer.position);
            let heading = bearing(older.position, newer.position);
            let ground_speed = ground_distance / dt;

            // Sanity check: ballistic missiles don't exceed ~7 km/s horizontal
            // (ICBMs reach ~7 km/s, MRBMs ~3-4 km/s)
            if ground_speed > 8.0 {
                continue; // Unrealistic velocity, likely measurement error
            }

            // Vertical rate
            let altitude_change = newer.altitude_km - older.altitude_km;
            let vertical_rate = altitude_change / dt;

            // Sanity check: vertical rate shouldn't exceed ~5 km/s (reentry is ~2-3 km/s)
            if vertical_rate.abs() > 6.0 {
                continue; // Unrealistic vertical rate, likely measurement error
            }

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

/// A converged trajectory estimate that persists and is refined over time
/// Once established, this represents the stable "what we think the full trajectory is"
#[derive(Clone, Debug)]
pub struct ConvergedTrajectory {
    /// Estimated launch origin
    pub origin: GeoCoord,
    /// Estimated impact point
    pub target: GeoCoord,
    /// Estimated maximum altitude (apogee)
    pub apogee_km: f64,
    /// Estimated total range
    pub range_km: f64,
    /// Estimated total flight time
    pub flight_time_sec: f64,
    /// Number of measurements used to establish this trajectory
    pub measurements_at_establishment: usize,
    /// Confidence in the trajectory (0.0 to 1.0)
    pub confidence: f64,
    /// Timestamp when trajectory was established
    pub established_at_sim_time: f64,
    /// Origin uncertainty radius (km) - shrinks with more measurements
    pub origin_uncertainty_km: f64,
    /// Target uncertainty radius (km) - shrinks with more measurements
    pub target_uncertainty_km: f64,
    /// Sum of weights of all estimate samples blended into this trajectory
    pub total_weight: f64,
    /// Number of estimate samples blended so far
    pub sample_count: u32,
    /// Timestamp of the newest measurement consumed by the last refinement.
    /// Refinement is gated on this: no new measurement, no update.
    pub last_meas_ts: f64,
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
    /// Geometric heading computed directly from position history (more stable than velocity heading)
    pub geometric_heading_deg: Option<f64>,
    /// Position history from best track (for stable heading computation)
    pub position_history: Vec<PositionMeasurement>,
    /// Converged trajectory estimate - established once track has sufficient measurements
    /// This trajectory is REFINED (not replaced) as new measurements come in
    pub converged_trajectory: Option<ConvergedTrajectory>,
    /// Total radar detections received (including rejected)
    pub total_detections: u64,
    /// Number of detections rejected by measurement validation
    pub rejected_detections: u64,
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

            let projected_altitude =
                self.estimated_altitude + velocity.vertical_rate_km_s * dt_seconds;

            // Uncertainty grows: ~5km/s * dt * (1 - confidence)
            let uncertainty_growth = 5.0 * dt_seconds * (1.0 - velocity.confidence);
            let total_uncertainty = self.uncertainty_radius_km + uncertainty_growth;

            (projected_position, projected_altitude, total_uncertainty)
        } else {
            // No velocity - rapid uncertainty growth
            let stale_uncertainty = self.uncertainty_radius_km + 10.0 * dt_seconds;
            (
                self.estimated_position,
                self.estimated_altitude,
                stale_uncertainty,
            )
        }
    }

    /// Estimate current flight progress (0.0-1.0) from altitude and vertical rate
    pub fn estimate_flight_progress(&self, velocity: &VelocityEstimate) -> f64 {
        const G: f64 = 0.00981; // km/s² gravity

        // Estimate apogee
        let estimated_apogee = if velocity.vertical_rate_km_s >= 0.0 {
            let v_up = velocity.vertical_rate_km_s;
            self.estimated_altitude + (v_up * v_up) / (2.0 * G)
        } else {
            self.estimated_altitude * 2.0 // rough estimate for descending
        }
        .max(self.estimated_altitude);

        // Use parabolic model: h(t) = 4 * max_h * t * (1-t)
        let altitude_ratio = self.estimated_altitude / estimated_apogee.max(1.0);
        if altitude_ratio >= 0.99 {
            return 0.5; // At apogee
        }

        let discriminant = 0.25 - altitude_ratio / 4.0;
        if discriminant < 0.0 {
            // Log anomaly for debugging
            eprintln!(
                "Track {}: discriminant negative, altitude_ratio={:.3}, apogee={:.1}",
                self.target_id, altitude_ratio, estimated_apogee
            );
            return 0.5;
        }

        let sqrt_term = discriminant.sqrt();
        let mut progress = if velocity.vertical_rate_km_s >= 0.0 {
            (0.5 - sqrt_term).max(0.0) // Ascending
        } else {
            (0.5 + sqrt_term).min(1.0) // Descending
        };

        // Clamp progress to avoid near-zero/near-one instability
        progress = progress.clamp(0.01, 0.99);

        progress
    }
}

// ============================================================================
// MEASUREMENT NORMALIZATION
// ============================================================================

/// Result of measurement normalization
#[derive(Clone, Debug)]
pub struct NormalizedMeasurement {
    pub position: GeoCoord,
    pub altitude_km: f64,
    pub timestamp: f64,
    /// Quality weight (0.0 to 1.0) based on range, SNR, and consistency
    pub quality_weight: f64,
    /// Adjusted measurement noise standard deviations [range, azimuth, elevation]
    pub noise_std: [f64; 3],
    /// Whether this measurement passed all validation checks
    pub is_valid: bool,
    /// Reason for rejection if not valid
    pub rejection_reason: Option<MeasurementRejectionReason>,
}

/// Reasons why a measurement might be rejected
#[derive(Clone, Debug, PartialEq)]
pub enum MeasurementRejectionReason {
    /// Measurement is too far from predicted position (statistical outlier)
    StatisticalOutlier {
        mahalanobis_distance: f64,
        threshold: f64,
    },
    /// Implied velocity exceeds physical limits
    VelocityExceedsLimit {
        implied_velocity_km_s: f64,
        max_velocity_km_s: f64,
    },
    /// Implied acceleration exceeds physical limits
    AccelerationExceedsLimit {
        implied_accel_g: f64,
        max_accel_g: f64,
    },
    /// Altitude is outside valid range
    AltitudeOutOfRange {
        altitude_km: f64,
        min_km: f64,
        max_km: f64,
    },
    /// Measurement is duplicate (too close to previous in time/space)
    DuplicateMeasurement,
    /// Range exceeds sensor capability
    RangeExceedsSensorLimit { range_km: f64, max_range_km: f64 },
}

/// Configuration for measurement normalization
#[derive(Clone, Debug)]
pub struct NormalizerConfig {
    /// Statistical gating threshold in sigma (e.g., 3.0 = 3-sigma gate)
    pub gating_threshold_sigma: f64,
    /// Maximum valid velocity for ballistic missiles (km/s)
    pub max_velocity_km_s: f64,
    /// Maximum valid acceleration (g's) - structural limits
    pub max_acceleration_g: f64,
    /// Minimum valid altitude (km) - usually 0 or slightly negative for terrain
    pub min_altitude_km: f64,
    /// Maximum valid altitude (km) - above this is space debris territory
    pub max_altitude_km: f64,
    /// Minimum time between measurements to avoid duplicates (seconds)
    pub min_measurement_interval_sec: f64,
    /// Minimum position change to avoid duplicates (km)
    pub min_position_change_km: f64,
    /// Base noise standard deviations [range_km, azimuth_rad, elevation_rad]
    pub base_noise_std: [f64; 3],
    /// Range at which noise is measured (reference range for scaling)
    pub reference_range_km: f64,
    /// Whether to apply range-dependent noise scaling
    pub scale_noise_with_range: bool,
}

impl Default for NormalizerConfig {
    fn default() -> Self {
        Self {
            gating_threshold_sigma: 7.0, // 7-sigma gate - lenient for model mismatch with crossing trajectories
            max_velocity_km_s: 8.0,      // ~Mach 24, covers ICBMs
            max_acceleration_g: 5000.0, // Very high - acceleration check compares EKF vs position velocity, not actual acceleration
            min_altitude_km: -0.5,      // Allow slight terrain variation
            max_altitude_km: 2000.0,    // Well above GEO
            min_measurement_interval_sec: 0.05, // 20 Hz max update rate
            min_position_change_km: 0.01, // 10 meter minimum movement
            base_noise_std: [0.05, 0.001, 0.001], // 50m range, 0.057° angles
            reference_range_km: 100.0,  // Base noise at 100km
            scale_noise_with_range: true,
        }
    }
}

/// Measurement normalizer for radar tracking data
///
/// Performs validation, outlier rejection, and quality weighting on incoming
/// radar measurements before they are used for track updates.
#[derive(Clone, Debug)]
pub struct MeasurementNormalizer {
    pub config: NormalizerConfig,
    /// Rolling statistics for innovation monitoring (per track)
    innovation_stats: HashMap<EntityId, InnovationStatistics>,
}

/// Rolling statistics for tracking filter health
#[derive(Clone, Debug, Default)]
struct InnovationStatistics {
    /// Number of measurements processed
    count: u64,
    /// Rolling mean of innovations (should be ~0 if unbiased)
    mean_innovation: [f64; 3],
    /// Rolling variance of innovations
    variance_innovation: [f64; 3],
    /// Count of rejected measurements
    rejected_count: u64,
    /// Count of consecutive rejections (for track health monitoring)
    consecutive_rejections: u32,
}

impl MeasurementNormalizer {
    pub fn new() -> Self {
        Self {
            config: NormalizerConfig::default(),
            innovation_stats: HashMap::new(),
        }
    }

    pub fn with_config(config: NormalizerConfig) -> Self {
        Self {
            config,
            innovation_stats: HashMap::new(),
        }
    }

    /// Normalize a radar measurement against an existing track
    ///
    /// Returns a NormalizedMeasurement with quality weight and validation status.
    /// If the measurement fails validation, is_valid will be false and
    /// rejection_reason will explain why.
    pub fn normalize(
        &mut self,
        measurement: &crate::simulation::ekf::RadarMeasurement,
        track: &TrackingState,
        sensor_max_range_km: f64,
    ) -> NormalizedMeasurement {
        // Convert radar measurement to geodetic position
        let (pos, alt) = crate::simulation::ekf::radar_to_geodetic(
            measurement.sensor_position,
            measurement.sensor_altitude_km,
            measurement.range_km,
            measurement.azimuth_rad,
            measurement.elevation_rad,
        );

        // Estimate quality from noise - lower noise means higher quality
        // Use range noise as primary indicator (index 0)
        let estimated_quality = (1.0 / (1.0 + measurement.noise_std[0] * 10.0)).clamp(0.5, 1.0);

        // Start with base result
        let mut result = NormalizedMeasurement {
            position: pos,
            altitude_km: alt,
            timestamp: measurement.timestamp,
            quality_weight: estimated_quality,
            noise_std: self.compute_adjusted_noise(measurement.range_km, estimated_quality),
            is_valid: true,
            rejection_reason: None,
        };

        // 1. Check range limits
        if measurement.range_km > sensor_max_range_km {
            result.is_valid = false;
            result.rejection_reason = Some(MeasurementRejectionReason::RangeExceedsSensorLimit {
                range_km: measurement.range_km,
                max_range_km: sensor_max_range_km,
            });
            self.record_rejection(track.target_id, result.rejection_reason.as_ref().unwrap());
            return result;
        }

        // 2. Check altitude limits
        if alt < self.config.min_altitude_km || alt > self.config.max_altitude_km {
            result.is_valid = false;
            result.rejection_reason = Some(MeasurementRejectionReason::AltitudeOutOfRange {
                altitude_km: alt,
                min_km: self.config.min_altitude_km,
                max_km: self.config.max_altitude_km,
            });
            self.record_rejection(track.target_id, result.rejection_reason.as_ref().unwrap());
            return result;
        }

        // 3. Check for duplicate measurement
        if let Some(last) = track.position_history.first() {
            let dt = measurement.timestamp - last.timestamp;
            let distance = haversine_distance(pos, last.position);

            if dt < self.config.min_measurement_interval_sec
                && distance < self.config.min_position_change_km
            {
                result.is_valid = false;
                result.rejection_reason = Some(MeasurementRejectionReason::DuplicateMeasurement);
                // Don't count duplicates as rejections for track health
                return result;
            }

            // 4 & 5. Check velocity and acceleration feasibility
            // Skip for new tracks (< 3 measurements) since position calculation methods
            // may differ between initial track creation and radar measurements
            // Also skip during grace period after reset - measurements need time to stabilize
            // Require at least 0.1s time delta to avoid noise-induced extreme accelerations
            if dt > 0.1
                && track.position_history.len() >= 3
                && track.measurements_since_reset >= RESET_GRACE_PERIOD
            {
                let implied_velocity = distance / dt;

                // Velocity check
                if implied_velocity > self.config.max_velocity_km_s {
                    result.is_valid = false;
                    result.rejection_reason =
                        Some(MeasurementRejectionReason::VelocityExceedsLimit {
                            implied_velocity_km_s: implied_velocity,
                            max_velocity_km_s: self.config.max_velocity_km_s,
                        });
                    self.record_rejection(
                        track.target_id,
                        result.rejection_reason.as_ref().unwrap(),
                    );
                    return result;
                }

                // Acceleration check (if we have velocity history)
                if let Some(ref velocity) = track.estimated_velocity {
                    let current_speed = velocity.ground_speed_km_s;
                    let speed_change = (implied_velocity - current_speed).abs();
                    let implied_accel_km_s2 = speed_change / dt;
                    let implied_accel_g = implied_accel_km_s2 / 0.00981; // Convert to g's

                    if implied_accel_g > self.config.max_acceleration_g {
                        result.is_valid = false;
                        result.rejection_reason =
                            Some(MeasurementRejectionReason::AccelerationExceedsLimit {
                                implied_accel_g,
                                max_accel_g: self.config.max_acceleration_g,
                            });
                        self.record_rejection(
                            track.target_id,
                            result.rejection_reason.as_ref().unwrap(),
                        );
                        return result;
                    }
                }
            }
        }

        // 6. Statistical gating (if we have EKF state for prediction)
        // Only apply after EKF velocity is initialized and grace period has passed
        // Without velocity, the prediction stays stationary while the target moves
        // After reset, the EKF needs time to stabilize before gating is meaningful
        if let Some(ref ekf) = track.ekf_state {
            // Check grace period and velocity initialization
            let in_grace_period = track.measurements_since_reset < RESET_GRACE_PERIOD;
            if in_grace_period {
                // In grace period - skip all gating
            } else if !track.ekf_velocity_initialized {
                // Velocity not initialized - skip gating
            } else {
                // Use more lenient threshold for newer tracks
                let effective_threshold = if track.position_history.len() < 5 {
                    self.config.gating_threshold_sigma * 2.0 // 7-sigma for new tracks
                } else {
                    self.config.gating_threshold_sigma
                };

                let mahalanobis_dist = self.compute_mahalanobis_distance(measurement, ekf);

                if mahalanobis_dist > effective_threshold {
                    result.is_valid = false;
                    result.rejection_reason =
                        Some(MeasurementRejectionReason::StatisticalOutlier {
                            mahalanobis_distance: mahalanobis_dist,
                            threshold: effective_threshold,
                        });
                    self.record_rejection(
                        track.target_id,
                        result.rejection_reason.as_ref().unwrap(),
                    );
                    return result;
                }

                // Update innovation statistics
                self.update_innovation_stats(track.target_id, measurement, ekf);

                // Adjust quality weight based on innovation consistency
                let consistency_factor = self.compute_consistency_factor(track.target_id);
                result.quality_weight *= consistency_factor;
            }
        }

        // 7. Apply range-based quality degradation
        if self.config.scale_noise_with_range {
            let range_factor = (measurement.range_km / self.config.reference_range_km).sqrt();
            // Quality degrades with range (inverse relationship)
            result.quality_weight *= (1.0 / range_factor).clamp(0.5, 1.0);
        }

        // Count this as an accepted measurement and reset consecutive rejections
        // This ensures ALL valid measurements are counted, not just those with EKF
        let stats = self.innovation_stats.entry(track.target_id).or_default();
        stats.count += 1;
        stats.consecutive_rejections = 0;

        result
    }

    /// Compute range-adjusted noise standard deviations
    fn compute_adjusted_noise(&self, range_km: f64, detection_quality: f64) -> [f64; 3] {
        let mut noise = self.config.base_noise_std;

        if self.config.scale_noise_with_range {
            // Range noise scales with sqrt of range (radar equation)
            let range_factor = (range_km / self.config.reference_range_km).sqrt();
            noise[0] *= range_factor;

            // Angle noise scales linearly with range (cross-range error = range × angle)
            // Using sqrt scaling for the noise itself since position error = range × angle_noise
            let angle_factor = range_factor;
            noise[1] *= angle_factor;
            noise[2] *= angle_factor;
        }

        // Scale inversely with detection quality (better detection = less noise)
        let quality_factor = (2.0 - detection_quality).max(1.0);
        noise[0] *= quality_factor;
        noise[1] *= quality_factor;
        noise[2] *= quality_factor;

        noise
    }

    /// Compute Mahalanobis distance between measurement and EKF prediction
    fn compute_mahalanobis_distance(
        &self,
        measurement: &crate::simulation::ekf::RadarMeasurement,
        ekf: &crate::simulation::ekf::EKFState,
    ) -> f64 {
        // IMPORTANT: Propagate EKF state to measurement time before comparing
        // The EKF hasn't been updated yet, so we need to predict where the target
        // should be at the measurement timestamp
        let dt = measurement.timestamp - ekf.timestamp;
        let predicted = if dt > 0.001 {
            // Clone and propagate to measurement time
            let mut ekf_at_meas_time = ekf.clone();
            ekf_at_meas_time.predict(dt);
            ekf_at_meas_time
                .predict_measurement(measurement.sensor_position, measurement.sensor_altitude_km)
        } else {
            // No time delta, use current state
            ekf.predict_measurement(measurement.sensor_position, measurement.sensor_altitude_km)
        };

        // Innovation (measurement - prediction)
        let innovation = [
            measurement.range_km - predicted[0],
            angle_diff_rad(measurement.azimuth_rad, predicted[1]),
            measurement.elevation_rad - predicted[2],
        ];

        // Use measurement noise as covariance approximation
        // For proper Mahalanobis, we'd need S = H*P*H' + R
        let noise = &measurement.noise_std;

        // Compute squared Mahalanobis distance: d² = Σ(innovation_i² / variance_i)
        let d_squared = (innovation[0] / noise[0]).powi(2)
            + (innovation[1] / noise[1]).powi(2)
            + (innovation[2] / noise[2]).powi(2);

        d_squared.sqrt()
    }

    /// Update rolling innovation statistics for a track
    fn update_innovation_stats(
        &mut self,
        target_id: EntityId,
        measurement: &crate::simulation::ekf::RadarMeasurement,
        ekf: &crate::simulation::ekf::EKFState,
    ) {
        // Propagate EKF to measurement time before computing innovation
        let dt = measurement.timestamp - ekf.timestamp;
        let predicted = if dt > 0.001 {
            let mut ekf_at_meas_time = ekf.clone();
            ekf_at_meas_time.predict(dt);
            ekf_at_meas_time
                .predict_measurement(measurement.sensor_position, measurement.sensor_altitude_km)
        } else {
            ekf.predict_measurement(measurement.sensor_position, measurement.sensor_altitude_km)
        };

        let innovation = [
            measurement.range_km - predicted[0],
            angle_diff_rad(measurement.azimuth_rad, predicted[1]),
            measurement.elevation_rad - predicted[2],
        ];

        let stats = self.innovation_stats.entry(target_id).or_default();
        // Note: stats.count is now incremented in normalize() for ALL valid measurements

        // Exponential moving average for mean and variance
        let alpha = 0.1; // Smoothing factor

        for i in 0..3 {
            let old_mean = stats.mean_innovation[i];
            stats.mean_innovation[i] = (1.0 - alpha) * old_mean + alpha * innovation[i];

            let deviation = innovation[i] - stats.mean_innovation[i];
            stats.variance_innovation[i] =
                (1.0 - alpha) * stats.variance_innovation[i] + alpha * deviation.powi(2);
        }
    }

    /// Compute consistency factor based on innovation statistics
    /// Returns 1.0 for consistent tracks, lower for inconsistent tracks
    fn compute_consistency_factor(&self, target_id: EntityId) -> f64 {
        if let Some(stats) = self.innovation_stats.get(&target_id) {
            // Need enough measurements for statistics to be meaningful
            if stats.count < 5 {
                return 1.0;
            }

            // Check if mean innovation is biased (should be ~0)
            let bias_penalty: f64 = stats
                .mean_innovation
                .iter()
                .map(|m| (m.abs() * 10.0).min(0.2))
                .sum();

            // Check consecutive rejection rate
            let rejection_penalty = (stats.consecutive_rejections as f64 * 0.1).min(0.3);

            (1.0 - bias_penalty - rejection_penalty).max(0.3)
        } else {
            1.0
        }
    }

    /// Record a rejection for track health monitoring
    fn record_rejection(&mut self, target_id: EntityId, reason: &MeasurementRejectionReason) {
        let stats = self.innovation_stats.entry(target_id).or_default();
        stats.rejected_count += 1;
        stats.consecutive_rejections += 1;

        // Debug: Log rejection reasons periodically
        if stats.rejected_count % 100 == 1 {
            println!(
                "DEBUG rejection #{} for target {}: {:?}",
                stats.rejected_count, target_id, reason
            );
        }
    }

    /// Get track health status
    pub fn get_track_health(&self, target_id: EntityId) -> TrackHealth {
        if let Some(stats) = self.innovation_stats.get(&target_id) {
            let total = stats.count + stats.rejected_count;
            let rejection_rate = if total > 0 {
                stats.rejected_count as f64 / total as f64
            } else {
                0.0
            };

            // Check for bias in innovations
            let max_bias = stats
                .mean_innovation
                .iter()
                .map(|m| m.abs())
                .fold(0.0, f64::max);

            TrackHealth {
                measurement_count: stats.count,
                rejection_count: stats.rejected_count,
                rejection_rate,
                consecutive_rejections: stats.consecutive_rejections,
                innovation_bias: max_bias,
                is_healthy: rejection_rate < 0.3 && stats.consecutive_rejections < 5,
            }
        } else {
            TrackHealth::default()
        }
    }

    /// Clear statistics for a track (call when track is dropped)
    pub fn clear_track_stats(&mut self, target_id: EntityId) {
        self.innovation_stats.remove(&target_id);
    }
}

impl Default for MeasurementNormalizer {
    fn default() -> Self {
        Self::new()
    }
}

/// Track health status
#[derive(Clone, Debug, Default)]
pub struct TrackHealth {
    pub measurement_count: u64,
    pub rejection_count: u64,
    pub rejection_rate: f64,
    pub consecutive_rejections: u32,
    pub innovation_bias: f64,
    pub is_healthy: bool,
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
    /// Cueing data: maps sensor_id -> set of target_ids that sensor has been cued to track
    /// A phased array can only use extended range (beyond search range) for cued targets
    cued_targets: HashMap<EntityId, HashSet<EntityId>>,
    /// Tracks which targets have been detected by any sensor (for cueing propagation)
    /// Maps target_id -> set of sensor_ids that have detected it
    detected_by: HashMap<EntityId, HashSet<EntityId>>,
    /// Measurement normalizer for validation and quality weighting
    pub normalizer: MeasurementNormalizer,
    /// Persisted converged trajectories per target (survives across frames)
    /// Uses RefCell for interior mutability since rendering needs &self but must update trajectories
    converged_trajectories: RefCell<HashMap<EntityId, ConvergedTrajectory>>,
    /// Per-target altitude history (sim_time, altitude_km) for parabolic fitting.
    /// Cleared alongside converged trajectories on track reset.
    alt_histories: RefCell<HashMap<EntityId, Vec<(f64, f64)>>>,
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
            cued_targets: HashMap::new(),
            detected_by: HashMap::new(),
            normalizer: MeasurementNormalizer::new(),
            converged_trajectories: RefCell::new(HashMap::new()),
            alt_histories: RefCell::new(HashMap::new()),
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
            cued_targets: HashMap::new(),
            detected_by: HashMap::new(),
            normalizer: MeasurementNormalizer::new(),
            converged_trajectories: RefCell::new(HashMap::new()),
            alt_histories: RefCell::new(HashMap::new()),
        }
    }

    /// Check if a sensor has been cued to a target (can use extended range)
    pub fn is_cued_to_target(&self, sensor_id: EntityId, target_id: EntityId) -> bool {
        self.cued_targets
            .get(&sensor_id)
            .map(|targets| targets.contains(&target_id))
            .unwrap_or(false)
    }

    /// Cue a sensor to track a specific target (enables extended range tracking)
    pub fn cue_sensor_to_target(&mut self, sensor_id: EntityId, target_id: EntityId) {
        self.cued_targets
            .entry(sensor_id)
            .or_insert_with(HashSet::new)
            .insert(target_id);
    }

    /// Record that a sensor has detected a target
    fn record_detection(&mut self, sensor_id: EntityId, target_id: EntityId) {
        self.detected_by
            .entry(target_id)
            .or_insert_with(HashSet::new)
            .insert(sensor_id);
    }

    /// Propagate cueing from sensors that have detected targets to other sensors
    /// This allows engagement radars to track at extended range when cued by early warning sensors
    pub fn propagate_cueing(&mut self, all_sensor_ids: &[EntityId]) {
        // For each target that has been detected by at least one sensor,
        // cue all other sensors to that target
        for (target_id, detecting_sensors) in &self.detected_by {
            for sensor_id in all_sensor_ids {
                // If this sensor hasn't detected the target itself, but another sensor has,
                // then cue this sensor to the target
                if !detecting_sensors.contains(sensor_id) && !detecting_sensors.is_empty() {
                    self.cued_targets
                        .entry(*sensor_id)
                        .or_insert_with(HashSet::new)
                        .insert(*target_id);
                }
            }
        }
    }

    /// Clear cueing for targets that are no longer active
    pub fn clear_stale_cueing(&mut self, active_target_ids: &HashSet<EntityId>) {
        // Remove targets from cued_targets that are no longer active
        for targets in self.cued_targets.values_mut() {
            targets.retain(|id| active_target_ids.contains(id));
        }
        // Remove targets from detected_by that are no longer active
        self.detected_by
            .retain(|id, _| active_target_ids.contains(id));
    }

    /// Create a RadarMeasurement from detection data for EKF
    fn create_radar_measurement(
        sensor_position: GeoCoord,
        sensor_altitude_km: f64,
        slant_range_km: f64,
        bearing_deg: f64,
        target_altitude_km: f64,
        timestamp: f64,
        detection_quality: f64,
    ) -> crate::simulation::ekf::RadarMeasurement {
        // Calculate elevation angle from slant range and altitude difference
        // elevation = asin(altitude_diff / slant_range)
        // Note: We use slant_range directly here since that's what the radar measures
        let altitude_diff = target_altitude_km - sensor_altitude_km;
        let elevation_rad = if slant_range_km > 0.01 {
            (altitude_diff / slant_range_km).clamp(-1.0, 1.0).asin()
        } else {
            0.0
        };

        // Measurement noise depends on detection quality
        // Higher quality = lower noise
        let quality_factor = (1.0 - detection_quality).max(0.1);
        let range_noise = 0.1 * quality_factor; // 0.01 - 0.1 km noise
        let angle_noise = (0.5_f64).to_radians() * quality_factor; // 0.05 - 0.5 deg noise

        crate::simulation::ekf::RadarMeasurement {
            range_km: slant_range_km,
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

        // Clear stale cueing for targets that are no longer active
        let active_target_ids: HashSet<EntityId> = active_missiles.iter().map(|m| m.id).collect();
        self.clear_stale_cueing(&active_target_ids);

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

                    if self.should_scan(sensor.sensor_id, config.tracking.track_update_rate_hz, dt)
                    {
                        // Clear old detections for this sensor
                        self.active_detections
                            .retain(|d| d.sensor_id != sensor.sensor_id);

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

                            // Check if this sensor has been cued to this target
                            // Cueing enables extended range tracking for phased arrays
                            let is_cued = self.is_cued_to_target(sensor.sensor_id, missile.id);

                            // Check detection based on radar type and mode assignment
                            if let Some((radar_mode, band)) = mode_and_band {
                                // Target has mode assignment - use mode-specific coverage for phased arrays
                                if config.detection.radar_type == RadarType::PhasedArray {
                                    let effective_coverage =
                                        config.detection.get_effective_azimuth_coverage(radar_mode);
                                    let half_coverage = effective_coverage / 2.0;
                                    let relative_bearing =
                                        normalize_angle_diff(bearing - sensor.azimuth_center_deg);
                                    if relative_bearing.abs() > half_coverage {
                                        continue; // Target outside mode-specific coverage
                                    }
                                }

                                if let Some(detection) = Self::check_defense_unit_detection(
                                    unit,
                                    missile,
                                    config,
                                    Some(radar_mode),
                                    band,
                                    current_sim_time,
                                    is_cued,
                                ) {
                                    // Use unique sensor ID for detection
                                    let mut detection = detection;
                                    detection.sensor_id = sensor.sensor_id;
                                    self.active_detections.push(detection);
                                    // Record this detection for cueing propagation
                                    self.record_detection(sensor.sensor_id, missile.id);
                                }
                            } else {
                                // No mode assignment yet - use default Search mode
                                // This allows initial detection to establish tracks
                                let band = config.detection.get_band_for_mode(RadarMode::Search);

                                // For phased arrays, check Search mode azimuth coverage
                                if config.detection.radar_type == RadarType::PhasedArray {
                                    let effective_coverage = config
                                        .detection
                                        .get_effective_azimuth_coverage(RadarMode::Search);
                                    let half_coverage = effective_coverage / 2.0;
                                    let relative_bearing =
                                        normalize_angle_diff(bearing - sensor.azimuth_center_deg);
                                    if relative_bearing.abs() > half_coverage {
                                        continue; // Target outside Search coverage
                                    }
                                }

                                if let Some(detection) = Self::check_defense_unit_detection(
                                    unit,
                                    missile,
                                    config,
                                    None,
                                    band,
                                    current_sim_time,
                                    is_cued,
                                ) {
                                    let mut detection = detection;
                                    detection.sensor_id = sensor.sensor_id;
                                    self.active_detections.push(detection);
                                    // Record this detection for cueing propagation
                                    self.record_detection(sensor.sensor_id, missile.id);
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

                            // Check if this sensor has been cued to this target
                            let is_cued = self.is_cued_to_target(unit.id, missile.id);

                            // Check detection based on radar type and mode assignment
                            if let Some((radar_mode, band)) = mode_and_band {
                                // Target has mode assignment - use assigned mode
                                if let Some(detection) = Self::check_defense_unit_detection(
                                    unit,
                                    missile,
                                    config,
                                    Some(radar_mode),
                                    band,
                                    current_sim_time,
                                    is_cued,
                                ) {
                                    self.active_detections.push(detection);
                                    self.record_detection(unit.id, missile.id);
                                }
                            } else {
                                // No mode assignment yet - use default Search mode
                                let band = config.detection.get_band_for_mode(RadarMode::Search);
                                if let Some(detection) = Self::check_defense_unit_detection(
                                    unit,
                                    missile,
                                    config,
                                    None,
                                    band,
                                    current_sim_time,
                                    is_cued,
                                ) {
                                    self.active_detections.push(detection);
                                    self.record_detection(unit.id, missile.id);
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

        // Propagate cueing from sensors that have detected targets to other sensors
        // Collect all sensor IDs for cueing propagation
        let all_sensor_ids: Vec<EntityId> = defense_units
            .iter()
            .flat_map(|u| {
                if !u.sensors.is_empty() {
                    u.sensors.iter().map(|s| s.sensor_id).collect::<Vec<_>>()
                } else {
                    vec![u.id]
                }
            })
            .chain(radar_stations.iter().map(|s| s.id))
            .collect();
        self.propagate_cueing(&all_sensor_ids);

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
                            let effective_coverage =
                                config.detection.get_effective_azimuth_coverage(radar_mode);

                            // RadarStation uses facing_deg and handles wraparound at 0/360
                            let half_coverage = effective_coverage / 2.0;
                            let min_bearing =
                                (station.facing_deg - half_coverage).rem_euclid(360.0);
                            let max_bearing =
                                (station.facing_deg + half_coverage).rem_euclid(360.0);

                            let in_coverage = if min_bearing <= max_bearing {
                                bearing >= min_bearing && bearing <= max_bearing
                            } else {
                                // Coverage wraps around 0/360
                                bearing >= min_bearing || bearing <= max_bearing
                            };

                            if !in_coverage {
                                continue; // Target outside mode-specific coverage
                            }
                        }

                        if let Some(detection) = Self::check_radar_station_detection(
                            station,
                            missile,
                            config,
                            Some(radar_mode),
                            band,
                            current_sim_time,
                        ) {
                            self.active_detections.push(detection);
                            // Early warning radar detected a target - record for cueing propagation
                            self.record_detection(station.id, missile.id);
                        }
                    } else {
                        // No mode assignment yet - use default Search mode
                        let band = config.detection.get_band_for_mode(RadarMode::Search);

                        // For phased arrays, check Search mode azimuth coverage
                        if config.detection.radar_type == RadarType::PhasedArray {
                            let effective_coverage = config
                                .detection
                                .get_effective_azimuth_coverage(RadarMode::Search);
                            let half_coverage = effective_coverage / 2.0;
                            let min_bearing =
                                (station.facing_deg - half_coverage).rem_euclid(360.0);
                            let max_bearing =
                                (station.facing_deg + half_coverage).rem_euclid(360.0);

                            let in_coverage = if min_bearing <= max_bearing {
                                bearing >= min_bearing && bearing <= max_bearing
                            } else {
                                // Coverage wraps around 0/360
                                bearing >= min_bearing || bearing <= max_bearing
                            };

                            if !in_coverage {
                                continue; // Target outside Search coverage
                            }
                        }

                        if let Some(detection) = Self::check_radar_station_detection(
                            station,
                            missile,
                            config,
                            None,
                            band,
                            current_sim_time,
                        ) {
                            self.active_detections.push(detection);
                            // Early warning radar detected a target - record for cueing propagation
                            self.record_detection(station.id, missile.id);
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
        self.update_tracks(
            &track_limits,
            dt,
            current_sim_time,
            defense_units,
            radar_stations,
            satellites,
        );
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
    ///
    /// For phased array radars:
    /// - Without cueing: can only detect within base search range
    /// - With cueing: can use concentrated beam for extended range (track/fire control multipliers)
    ///
    /// Cueing comes from other sensors (early warning radars, satellites, other defense units)
    /// that have already detected and shared track data for this target.
    fn check_defense_unit_detection(
        unit: &DefenseUnit,
        missile: &Missile,
        config: &SensorConfig,
        radar_mode: Option<RadarMode>,
        band: RadarBand,
        timestamp: f64,
        is_cued: bool, // Whether this sensor has been cued to this target
    ) -> Option<Detection> {
        // Determine effective mode (default to Search for mechanical radars)
        let mode = radar_mode.unwrap_or(RadarMode::Search);

        let base_range = config.detection.detection_range_km;

        // For phased arrays: extended range requires cueing
        // The radar needs to know WHERE to concentrate its beam
        let effective_range = if config.detection.radar_type == RadarType::PhasedArray {
            if is_cued {
                // Cued: can use full mode-specific range (concentrated beam)
                base_range * config.detection.get_range_multiplier(mode)
            } else {
                // Not cued: limited to search range (broad scan)
                // Search multiplier is 1.0, so this is just the base range
                base_range * config.detection.get_range_multiplier(RadarMode::Search)
            }
        } else {
            // Mechanical radars: can't concentrate beam, always use mode-based range
            base_range * config.detection.get_range_multiplier(mode)
        };

        // Calculate range based on mode (slant vs ground)
        let range_km = if mode.uses_slant_range() {
            calculate_slant_range(unit.position, 0.0, missile.position, missile.altitude_km)
        } else {
            haversine_distance(unit.position, missile.position)
        };

        // Early exit if out of range (with 1.5x margin for probabilistic detection)
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

        // Calculate detection probability using RCS with aspect-angle variation
        // RCS varies based on viewing angle (nose-on vs broadside vs tail-on)
        let rcs_dbsm = missile.rcs_with_aspect(unit.position);
        let mut p_detect = calculate_detection_probability(
            range_km,
            effective_range, // Use effective range, not base range
            rcs_dbsm,
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
        let slant_range =
            calculate_slant_range(unit.position, 0.0, missile.position, missile.altitude_km);
        let radar_measurement = Some(Self::create_radar_measurement(
            unit.position,
            0.0, // Ground-based unit
            slant_range,
            bearing,
            missile.altitude_km,
            timestamp,
            p_detect.max(0.1),
        ));

        // Measurement quality is separate from detection probability
        // Modern phased array radars have high measurement accuracy once target is detected
        // Quality only degrades slightly at extreme range or for very stealthy targets
        let range_ratio = range_km / effective_range;
        let measurement_quality = if range_ratio <= 0.8 {
            // Within 80% of max range: excellent quality (90-95%)
            0.90 + (1.0 - range_ratio) * 0.0625 // 95% at close, 90% at 80% range
        } else {
            // Beyond 80%: gradual degradation (90% -> 75% at max range)
            0.90 - (range_ratio - 0.8) * 0.75 // Linear falloff
        }
        .clamp(0.7, 0.95);

        Some(Detection {
            sensor_id: unit.id,
            sensor_type: SensorKind::DefenseUnitRadar,
            target_id: missile.id,
            detection_quality: measurement_quality,
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

        // Calculate detection probability using RCS with aspect-angle variation
        let rcs_dbsm = missile.rcs_with_aspect(station.position);
        let mut p_detect = calculate_detection_probability(
            range_km,
            effective_range, // Use effective range, not base range
            rcs_dbsm,
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

        // Measurement quality for ground radar stations
        let range_ratio = range_km / effective_range;
        let measurement_quality = if range_ratio <= 0.8 {
            0.90 + (1.0 - range_ratio) * 0.0625
        } else {
            0.90 - (range_ratio - 0.8) * 0.75
        }
        .clamp(0.7, 0.95);

        // Create radar measurement for EKF (always use slant range)
        let slant_range =
            calculate_slant_range(station.position, 0.0, missile.position, missile.altitude_km);
        let radar_measurement = Some(Self::create_radar_measurement(
            station.position,
            0.0, // Ground-based station
            slant_range,
            bearing,
            missile.altitude_km,
            timestamp,
            measurement_quality,
        ));

        Some(Detection {
            sensor_id: station.id,
            sensor_type: SensorKind::GroundRadar,
            target_id: missile.id,
            detection_quality: measurement_quality,
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
                // Radar satellites can use RCS with aspect-angle variation
                let rcs_dbsm = missile.rcs_with_aspect(satellite.position);
                let rcs_factor = 10.0_f64.powf(rcs_dbsm / 30.0).clamp(0.3, 1.0);
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

        // Satellite measurement quality - good for early warning but not as precise as ground radar
        let measurement_quality = match satellite.sensor_type {
            SensorType::Radar => 0.85,    // Satellite radar has good quality
            SensorType::Infrared => 0.75, // IR has moderate quality for tracking
            SensorType::Both => 0.80,
        };

        Some(Detection {
            sensor_id: satellite.id,
            sensor_type: match satellite.sensor_type {
                SensorType::Infrared => SensorKind::SatelliteIR,
                SensorType::Radar => SensorKind::SatelliteRadar,
                SensorType::Both => SensorKind::SatelliteIR, // Primary is IR
            },
            target_id: missile.id,
            detection_quality: measurement_quality,
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
                // Use index-based access to allow separate borrows of normalizer and tracks
                let track_idx = self.active_tracks.iter().position(|t| {
                    t.tracker_id == detection.sensor_id && t.target_id == detection.target_id
                });

                if let Some(idx) = track_idx {
                    // Update existing track (doesn't count against limit)
                    self.active_tracks[idx].track_quality = (self.active_tracks[idx].track_quality
                        + detection.detection_quality * 0.5)
                        .min(1.0);
                    self.active_tracks[idx].time_since_update = 0.0;

                    // Calculate position from sensor bearing/range
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

                        // Update predicted position
                        self.active_tracks[idx].predicted_position = measured_position;
                        self.active_tracks[idx].predicted_altitude = detection.altitude_km;

                        // Update filter based on filter type
                        match self.filter_type {
                            FilterType::ExtendedKalman => {
                                // Use EKF with raw radar measurement
                                if let Some(ref radar_meas) = detection.radar_measurement {
                                    // Normalize measurement before adding to track
                                    // Use a high max range since detection already passed sensor range check
                                    let normalized = self.normalizer.normalize(
                                        radar_meas,
                                        &self.active_tracks[idx],
                                        10000.0, // Generous max range - already validated by detection
                                    );

                                    if normalized.is_valid {
                                        self.active_tracks[idx].add_radar_measurement(radar_meas);
                                    } else {
                                        // Measurement rejected - check for EKF divergence
                                        let target_id = self.active_tracks[idx].target_id;
                                        let health = self.normalizer.get_track_health(target_id);

                                        if health.consecutive_rejections >= TRACK_RESET_THRESHOLD {
                                            // EKF has diverged - reset ALL tracks for this target
                                            // (multiple sensors may be tracking the same target)
                                            // This mimics real radar behavior of dropping diverged tracks
                                            println!(
                                                "DEBUG: Track {} diverged ({} consecutive rejections) - resetting all sensors' EKFs",
                                                target_id, health.consecutive_rejections
                                            );
                                            for track in &mut self.active_tracks {
                                                if track.target_id == target_id {
                                                    track.reset_ekf();
                                                }
                                            }
                                            // Clear the normalizer's stats for fresh start
                                            self.normalizer.clear_track_stats(target_id);
                                            // Clear persisted converged trajectory
                                            self.converged_trajectories
                                                .borrow_mut()
                                                .remove(&target_id);
                                            self.alt_histories.borrow_mut().remove(&target_id);
                                        }
                                    }
                                } else {
                                    // Fallback to position measurement if no radar data
                                    self.active_tracks[idx].add_position_measurement(
                                        current_sim_time,
                                        measured_position,
                                        detection.altitude_km,
                                        detection.detection_quality,
                                    );
                                }
                            }
                            FilterType::LinearKalman => {
                                // Use linear KF with converted position
                                self.active_tracks[idx].add_position_measurement(
                                    current_sim_time,
                                    measured_position,
                                    detection.altitude_km,
                                    detection.detection_quality,
                                );
                            }
                        }

                        // Update velocity estimate
                        self.active_tracks[idx].update_velocity_estimate(current_sim_time);
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
                            let (init_position, init_history) = if let Some(sensor_pos) =
                                Self::get_sensor_position(
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
                                    let ekf = detection
                                        .radar_measurement
                                        .as_ref()
                                        .map(|meas| crate::simulation::ekf::EKFState::new(meas));
                                    (None, ekf)
                                }
                                FilterType::LinearKalman => {
                                    // Initialize linear KF from position measurement
                                    let kf =
                                        init_history.first().map(|meas| BallisticState::new(meas));
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
                                ekf_velocity_initialized: false,
                                measurements_since_reset: 1, // First measurement counts
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
    /// sim_time: current simulation time (for trajectory convergence tracking)
    pub fn get_fused_track(
        &self,
        target_id: EntityId,
        defense_unit_ids: &HashSet<EntityId>,
        sim_time: f64,
    ) -> Option<FusedTrack> {
        let tracks: Vec<&TrackingState> = self
            .active_tracks
            .iter()
            .filter(|t| t.target_id == target_id)
            .collect();

        if tracks.is_empty() {
            return None;
        }

        // Check if any defense unit (fire control radar) is tracking this target
        let has_fire_control_lock = tracks
            .iter()
            .any(|track| defense_unit_ids.contains(&track.tracker_id));

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
        let best_track = tracks
            .iter()
            .max_by_key(|t| t.position_history.len())
            .unwrap();

        let measurement_count = best_track.position_history.len();
        // Get position uncertainty from linear KF or EKF (whichever is in use)
        let kalman_position_uncertainty_km = best_track
            .kalman_filter
            .as_ref()
            .map(|kf| kf.get_position_uncertainty())
            .or_else(|| {
                best_track
                    .ekf_state
                    .as_ref()
                    .map(|ekf| ekf.get_position_uncertainty())
            });

        // Compute geometric heading directly from position history (most stable method)
        // Find two well-separated positions and compute bearing between them
        let geometric_heading_deg = {
            let pos_history = &best_track.position_history;
            if pos_history.len() >= 2 {
                // Find two positions at least 0.3 seconds apart AND at least 0.5km apart for stable heading
                let newest = &pos_history[0];
                let older_pos = pos_history.iter().skip(1).find(|m| {
                    let dt = (newest.timestamp - m.timestamp).abs();
                    let dist = haversine_distance(newest.position, m.position);
                    dt > 0.3 && dist > 0.5 // At least 0.3s and 0.5km apart
                });

                if let Some(older) = older_pos {
                    // Compute bearing from older to newer position
                    let bearing = calculate_bearing(older.position, newest.position);
                    Some(bearing)
                } else {
                    // Fallback: look for any position with enough distance, ignoring time
                    let distant_pos = pos_history
                        .iter()
                        .skip(1)
                        .find(|m| haversine_distance(newest.position, m.position) > 0.5);

                    if let Some(distant) = distant_pos {
                        Some(calculate_bearing(distant.position, newest.position))
                    } else {
                        None // Not enough position separation for stable heading
                    }
                }
            } else {
                None
            }
        };

        // Clone position history from best track
        let position_history = best_track.position_history.clone();

        // Get track health stats (total detections vs rejected)
        let track_health = self.normalizer.get_track_health(target_id);

        let mut fused_track = FusedTrack {
            target_id,
            estimated_position: GeoCoord::new(lat_sum, lon_sum),
            estimated_altitude: alt_sum,
            fused_quality,
            uncertainty_radius_km,
            sensor_count: tracks.len(),
            staleness_seconds: avg_staleness,
            predicted_heading_deg: predicted_heading,
            estimated_velocity: fused_velocity.clone(),
            measurement_count,
            kalman_position_uncertainty_km,
            has_fire_control_lock,
            geometric_heading_deg,
            position_history,
            // Start with persisted converged trajectory (if any)
            converged_trajectory: self
                .converged_trajectories
                .borrow()
                .get(&target_id)
                .cloned(),
            total_detections: track_health.measurement_count + track_health.rejection_count,
            rejected_detections: track_health.rejection_count,
        };

        // Refine converged trajectory ONLY when new measurements have arrived
        if let Some(velocity) = &fused_velocity {
            let newest_meas_ts = tracks
                .iter()
                .filter_map(|t| t.position_history.first().map(|m| m.timestamp))
                .fold(0.0_f64, f64::max);
            if newest_meas_ts > 0.0 {
                self.refine_converged_trajectory(
                    target_id,
                    &mut fused_track,
                    velocity,
                    newest_meas_ts,
                    sim_time,
                );
            }
        }

        Some(fused_track)
    }

    /// Get all targets currently being tracked as fused tracks
    pub fn get_all_fused_tracks(
        &self,
        defense_unit_ids: &HashSet<EntityId>,
        sim_time: f64,
    ) -> Vec<FusedTrack> {
        // Get unique target IDs
        let target_ids: std::collections::HashSet<EntityId> =
            self.active_tracks.iter().map(|t| t.target_id).collect();

        target_ids
            .into_iter()
            .filter_map(|id| self.get_fused_track(id, defense_unit_ids, sim_time))
            .collect()
    }

    /// Get the sensor-derived converged trajectory estimate for a target,
    /// for use by fire control (intercept solutions, mid-course guidance).
    ///
    /// Returns (trajectory, estimated_total_flight_time_sec) or None if no
    /// converged estimate exists yet. This is the ONLY projection fire control
    /// may use — engaging on ground-truth data is prohibited by the project's
    /// realism requirements (undetected/unmodeled = unengaged).
    pub fn get_converged_trajectory(
        &self,
        target_id: EntityId,
    ) -> Option<(ConvergedTrajectory, f64)> {
        let traj = self
            .converged_trajectories
            .borrow()
            .get(&target_id)
            .cloned()?;
        let flight_time = traj.flight_time_sec;
        Some((traj, flight_time))
    }

    /// Establish or refine the converged trajectory estimate for a target.
    ///
    /// GATED: only runs when a new measurement timestamp has arrived since the
    /// last refinement — render-rate calls with no new data do nothing. This
    /// prevents the impact marker from churning between sensor updates.
    ///
    /// Primary estimator: least-squares quadratic fit of the altitude history.
    /// The simulation flies missiles with h(t) = 4A·τ(1−τ) (a quadratic in time)
    /// and constant ground speed along a great circle, so the fit recovers
    /// launch/impact times and apogee exactly. Fallback: model-based consistent
    /// (apogee, progress) solver with low weight.
    ///
    /// Blending: running weighted mean of per-measurement estimates, so marker
    /// steps shrink ~1/n and the estimate converges instead of oscillating.
    fn refine_converged_trajectory(
        &self,
        target_id: EntityId,
        fused: &mut FusedTrack,
        velocity: &VelocityEstimate,
        newest_meas_ts: f64,
        sim_time: f64,
    ) {
        use crate::simulation::physics::{estimate_flight_time, estimate_range_from_apogee};

        const MIN_MEASUREMENTS: usize = 3;
        const MIN_VEL_CONFIDENCE: f64 = 0.3;

        // ---- Gate on new measurements ----
        let existing = self
            .converged_trajectories
            .borrow()
            .get(&target_id)
            .cloned();
        if let Some(ref ex) = existing {
            if newest_meas_ts <= ex.last_meas_ts {
                return;
            }
        }
        if fused.measurement_count < MIN_MEASUREMENTS {
            return;
        }
        if velocity.confidence < MIN_VEL_CONFIDENCE {
            return;
        }

        // ---- Record altitude sample for the quadratic fit ----
        // Fused tracks can be queried multiple times per tick; skip duplicates.
        {
            let mut histories = self.alt_histories.borrow_mut();
            let hist = histories.entry(target_id).or_default();
            if hist.last().map(|&(t, _)| t) != Some(sim_time) {
                hist.push((sim_time, fused.estimated_altitude));
            }
            // Cap history (10 minutes at 10 Hz is ample for a global fit)
            if hist.len() > 6000 {
                hist.drain(0..hist.len() - 6000);
            }
        }

        let heading = fused.geometric_heading_deg.unwrap_or(velocity.heading_deg);
        let alt = fused.estimated_altitude;

        // ---- Primary estimate: exact quadratic fit of the altitude profile ----
        let fit_sample = {
            let histories = self.alt_histories.borrow();
            histories
                .get(&target_id)
                .and_then(|hist| fit_altitude_quadratic(hist))
                .and_then(|fit| {
                    let (t_launch, t_impact) = fit.t_launch_impact()?;
                    let flight_time = t_impact - t_launch;
                    let t_remaining = t_impact - sim_time;
                    let t_elapsed = sim_time - t_launch;
                    let apogee = fit.apogee_km();
                    let v_g = velocity.ground_speed_km_s;
                    // Sanity: ballistic missile with positive apogee, live flight,
                    // physically plausible speeds (SRBM..ICBM)
                    if !(60.0..=5400.0).contains(&flight_time) {
                        return None;
                    }
                    if !(1.0..=5400.0).contains(&t_remaining) {
                        return None;
                    }
                    if !(0.1..=8.0).contains(&v_g) {
                        return None;
                    }
                    if apogee < alt * 0.9 || apogee < 10.0 {
                        return None;
                    }
                    let range = v_g * flight_time;
                    if !(80.0..=16000.0).contains(&range) {
                        return None;
                    }

                    let reverse = (heading + 180.0).rem_euclid(360.0);
                    let origin = calculate_position_from_bearing_range(
                        fused.estimated_position,
                        reverse,
                        v_g * t_elapsed,
                    );
                    let target = calculate_position_from_bearing_range(
                        fused.estimated_position,
                        heading,
                        v_g * t_remaining,
                    );

                    // Weight grows with fit span (conditioning) and residual quality.
                    // Early short-span fits are noisy extrapolations — weight down.
                    let span_factor = (fit.span / 60.0).min(1.0);
                    let weight = velocity.confidence
                        * (1.0 / (1.0 + fit.rms_km))
                        * span_factor
                        * span_factor;
                    Some((origin, target, apogee, range, flight_time, weight))
                })
        };

        // ---- Fallback estimate: model-based consistent (apogee, progress) pair ----
        let (origin, target, apogee, range, flight_time, weight) = match fit_sample {
            Some(s) => s,
            None => {
                let (ap, pr) = match parabola_state_from_altitude(alt, velocity.vertical_rate_km_s)
                {
                    Some(v) => v,
                    None => return,
                };
                let range = estimate_range_from_apogee(ap);
                if !(50.0..=15000.0).contains(&range) {
                    return;
                }
                let t_flight = estimate_flight_time(range);
                let remaining = range * (1.0 - pr);
                let reverse = (heading + 180.0).rem_euclid(360.0);
                let origin = calculate_position_from_bearing_range(
                    fused.estimated_position,
                    reverse,
                    range * pr,
                );
                let target = calculate_position_from_bearing_range(
                    fused.estimated_position,
                    heading,
                    remaining,
                );
                // Low weight: model assumptions unreliable, especially during boost
                let boost_factor = if velocity.vertical_rate_km_s > 0.0 && alt < 50.0 {
                    0.3
                } else {
                    1.0
                };
                (
                    origin,
                    target,
                    ap,
                    range,
                    t_flight,
                    0.05 * velocity.confidence * boost_factor,
                )
            }
        };

        // ---- Running weighted mean: marker steps shrink ~1/n and converge ----
        match existing {
            Some(mut ex) => {
                let mut w = weight;
                // Outlier dampening: a single wild measurement can't yank the estimate
                if ex.sample_count >= 5 && haversine_distance(ex.target, target) > 400.0 {
                    w *= 0.1;
                }
                let tw = ex.total_weight + w;
                if tw <= 0.0 {
                    return;
                }
                let blend = |old: f64, new: f64| (old * ex.total_weight + new * w) / tw;
                ex.origin = GeoCoord::new(
                    blend(ex.origin.lat, origin.lat),
                    blend(ex.origin.lon, origin.lon),
                );
                ex.target = GeoCoord::new(
                    blend(ex.target.lat, target.lat),
                    blend(ex.target.lon, target.lon),
                );
                ex.apogee_km = blend(ex.apogee_km, apogee);
                ex.range_km = blend(ex.range_km, range);
                ex.flight_time_sec = blend(ex.flight_time_sec, flight_time);
                ex.total_weight = tw;
                ex.sample_count += 1;
                ex.last_meas_ts = newest_meas_ts;
                ex.confidence = (ex.confidence * 0.85 + velocity.confidence * 0.15).min(0.95);
                // Uncertainty shrinks monotonically with accumulated evidence
                ex.target_uncertainty_km = (50.0 / (1.0 + ex.total_weight)).max(10.0);
                ex.origin_uncertainty_km = (25.0 / (1.0 + ex.total_weight)).max(5.0);
                self.converged_trajectories
                    .borrow_mut()
                    .insert(target_id, ex);
            }
            None => {
                let traj = ConvergedTrajectory {
                    origin,
                    target,
                    apogee_km: apogee,
                    range_km: range,
                    flight_time_sec: flight_time,
                    measurements_at_establishment: fused.measurement_count,
                    confidence: velocity.confidence.min(0.95),
                    established_at_sim_time: sim_time,
                    origin_uncertainty_km: 25.0,
                    target_uncertainty_km: 50.0,
                    total_weight: weight,
                    sample_count: 1,
                    last_meas_ts: newest_meas_ts,
                };
                self.converged_trajectories
                    .borrow_mut()
                    .insert(target_id, traj);
            }
        }
        fused.converged_trajectory = self
            .converged_trajectories
            .borrow()
            .get(&target_id)
            .cloned();
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
    fn calculate_uncertainty_radius(
        quality: f64,
        staleness_seconds: f64,
        sensor_count: usize,
    ) -> f64 {
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
                let closing_factor = (180.0 - bearing_diff) / 180.0; // 1.0 = directly approaching
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
        let sensor_tracks: Vec<&TrackingState> = self
            .active_tracks
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
        let mode_state = self
            .radar_mode_states
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
                break; // Hard limit on total tracks
            }

            // Determine desired mode based on priority
            let desired_mode = if *has_interceptor {
                // Priority 1: Fire control for targets with interceptors
                if fc_count >= max_fire_control_tracks {
                    continue; // FC capacity exhausted, skip this target
                }
                RadarMode::FireControl
            } else {
                // Priority 2/3: Track or Search based on quality
                let track_quality = self
                    .active_tracks
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
                        mode_state
                            .target_assignments
                            .insert(*target_id, (RadarMode::Search, band));
                        time_budget_remaining -= search_dwell;
                        search_count += 1;
                    }
                    // else: can't fit at all, skip
                } else if desired_mode == RadarMode::FireControl {
                    // FireControl is critical - try to fit by degrading to Track
                    let track_dwell = tracking_config.get_dwell_time_sec(RadarMode::Track);
                    if track_dwell <= time_budget_remaining {
                        let band = config.detection.get_band_for_mode(RadarMode::Track);
                        mode_state
                            .target_assignments
                            .insert(*target_id, (RadarMode::Track, band));
                        time_budget_remaining -= track_dwell;
                        track_count += 1;
                    } else {
                        // Last resort: try Search
                        let search_dwell = tracking_config.get_dwell_time_sec(RadarMode::Search);
                        if search_dwell <= time_budget_remaining {
                            let band = config.detection.get_band_for_mode(RadarMode::Search);
                            mode_state
                                .target_assignments
                                .insert(*target_id, (RadarMode::Search, band));
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
                mode_state
                    .target_assignments
                    .insert(*target_id, (desired_mode, band));
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
        mode_state.stats.targets_dropped_count = mode_state
            .stats
            .targets_dropped_count
            .saturating_add(dropped_this_scan as u32);

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

/// Normalize angle difference to [-pi, pi] (radians)
fn angle_diff_rad(a: f64, b: f64) -> f64 {
    let mut diff = a - b;
    while diff > std::f64::consts::PI {
        diff -= 2.0 * std::f64::consts::PI;
    }
    while diff < -std::f64::consts::PI {
        diff += 2.0 * std::f64::consts::PI;
    }
    diff
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
fn calculate_atmospheric_attenuation(
    range_km: f64,
    target_altitude_km: f64,
    attenuation_coeff: f64,
) -> f64 {
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
    let attenuation_factor =
        calculate_atmospheric_attenuation(range_km, altitude_km, attenuation_coeff);

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

// ============================================================================
// TRAJECTORY ESTIMATION HELPERS
// ============================================================================

/// Result of a least-squares quadratic fit of altitude vs time.
/// Models the simulation's parabolic altitude profile h(t) = 4A·τ(1−τ),
/// which is exactly a quadratic in absolute time.
struct AltitudeFit {
    /// h(u) = qa·u² + qb·u + c with normalized u = (t − t_mid) / span
    qa: f64,
    qb: f64,
    qc: f64,
    t_mid: f64,
    span: f64,
    rms_km: f64,
}

impl AltitudeFit {
    /// Solve h(u) = 0 for the parabola's two roots; convert back to absolute time.
    /// Returns (t_launch, t_impact), or None if the parabola doesn't open downward
    /// or has no real roots (happens when the fit is degenerate).
    fn t_launch_impact(&self) -> Option<(f64, f64)> {
        // Parabola must open downward (missile altitude profile)
        if self.qa >= -1e-12 {
            return None;
        }
        let disc = self.qb * self.qb - 4.0 * self.qa * self.qc;
        if disc <= 0.0 {
            return None;
        }
        let sq = disc.sqrt();
        let u1 = (-self.qb + sq) / (2.0 * self.qa);
        let u2 = (-self.qb - sq) / (2.0 * self.qa);
        let (ul, ui) = if u1 < u2 { (u1, u2) } else { (u2, u1) };
        Some((self.t_mid + ul * self.span, self.t_mid + ui * self.span))
    }

    /// Apex height of the fitted parabola (km)
    fn apogee_km(&self) -> f64 {
        self.qc - self.qb * self.qb / (4.0 * self.qa)
    }
}

/// Least-squares fit of h(t) = at² + bt + c to (timestamp, altitude) history.
/// History must be ordered oldest-first. Time is centered and scaled to
/// u ∈ [−0.5, 0.5] for numerical conditioning. The 3×3 symmetric normal
/// equations are solved by Cramer's rule.
fn fit_altitude_quadratic(history: &[(f64, f64)]) -> Option<AltitudeFit> {
    const MIN_POINTS: usize = 6;
    const MIN_SPAN_SEC: f64 = 4.0;
    if history.len() < MIN_POINTS {
        return None;
    }
    let t_first = history[0].0;
    let t_last = history[history.len() - 1].0;
    let span = t_last - t_first;
    if span < MIN_SPAN_SEC {
        return None;
    }
    let t_mid = (t_first + t_last) / 2.0;

    // Accumulate normal equations for h(u) = qa·u² + qb·u + qc
    let (mut s0, mut s1, mut s2, mut s3, mut s4) = (0.0, 0.0, 0.0, 0.0, 0.0);
    let (mut sh, mut suh, mut su2h) = (0.0, 0.0, 0.0);
    for &(t, h) in history {
        let u = (t - t_mid) / span;
        let u2 = u * u;
        s0 += 1.0;
        s1 += u;
        s2 += u2;
        s3 += u2 * u;
        s4 += u2 * u2;
        sh += h;
        suh += u * h;
        su2h += u2 * h;
    }

    // Solve [s4 s3 s2; s3 s2 s1; s2 s1 s0]·[qa qb qc]ᵀ = [su2h suh sh]ᵀ
    let det = s4 * (s2 * s0 - s1 * s1) - s3 * (s3 * s0 - s1 * s2) + s2 * (s3 * s1 - s2 * s2);
    if det.abs() < 1e-12 {
        return None;
    }

    // Cramer's rule: replace column k with the RHS vector
    let det_a = su2h * (s2 * s0 - s1 * s1) - s3 * (suh * s0 - s1 * sh) + s2 * (suh * s1 - s2 * sh);
    let det_b = s4 * (suh * s0 - s1 * sh) - su2h * (s3 * s0 - s1 * s2) + s2 * (s3 * sh - suh * s2);
    let det_c = s4 * (s2 * sh - suh * s1) - s3 * (s3 * sh - suh * s2) + su2h * (s3 * s1 - s2 * s2);

    let qa = det_a / det;
    let qb = det_b / det;
    let qc = det_c / det;

    // RMS residual (km) - measures fit quality against measurement noise
    let mut sum_sq = 0.0;
    for &(t, h) in history {
        let u = (t - t_mid) / span;
        let model = qa * u * u + qb * u + qc;
        let resid = h - model;
        sum_sq += resid * resid;
    }
    let rms_km = (sum_sq / history.len() as f64).sqrt();

    Some(AltitudeFit {
        qa,
        qb,
        qc,
        t_mid,
        span,
        rms_km,
    })
}

/// Solve apogee and flight progress consistently from altitude and vertical rate,
/// assuming the parabolic altitude profile h(τ) = 4A·τ(1−τ) and the physics.rs
/// range/time models. Given vz·T/h ≡ k, progress solves k·τ² − (k+2)·τ + 1 = 0;
/// the root in (0,1) is always τ = [(k+2) − √(k²+4)] / (2k) (verified for both
/// ascent and descent, i.e. any sign of k).
/// Returns (apogee_km, progress) or None if inputs are out of model range.
fn parabola_state_from_altitude(alt_km: f64, vz_km_s: f64) -> Option<(f64, f64)> {
    use crate::simulation::physics::{estimate_flight_time, estimate_range_from_apogee};

    const G: f64 = 0.00981; // km/s² standard gravity
    if !(0.5..=2000.0).contains(&alt_km) {
        return None;
    }

    // Initial apogee guess from energy conservation (ascent) or midcourse rule (descent)
    let mut apogee = if vz_km_s >= 0.0 {
        alt_km + vz_km_s * vz_km_s / (2.0 * G)
    } else {
        alt_km * 2.0
    }
    .clamp(alt_km, 2000.0);

    // Two refinement iterations to make (apogee, progress) mutually consistent
    let mut progress = 0.5;
    for _ in 0..2 {
        let range = estimate_range_from_apogee(apogee);
        if !(50.0..=15000.0).contains(&range) {
            return None;
        }
        let t_flight = estimate_flight_time(range);
        let k = vz_km_s * t_flight / alt_km;
        progress = if k.abs() < 1e-4 {
            0.5 // near apogee: vertical rate ~0
        } else {
            (((k + 2.0) - (k * k + 4.0).sqrt()) / (2.0 * k)).clamp(0.02, 0.98)
        };
        apogee = (alt_km / (4.0 * progress * (1.0 - progress))).clamp(alt_km, 2000.0);
    }

    Some((apogee, progress))
}
