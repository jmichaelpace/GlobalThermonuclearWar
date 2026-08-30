use crate::simulation::detection::FusedTrack;
use crate::simulation::kalman::BallisticState;
use crate::types::GeoCoord;

const EARTH_RADIUS_KM: f64 = 6371.0;

/// Calculate the great-circle distance between two coordinates in kilometers
pub fn haversine_distance(from: GeoCoord, to: GeoCoord) -> f64 {
    let lat1 = from.lat.to_radians();
    let lat2 = to.lat.to_radians();
    let delta_lat = (to.lat - from.lat).to_radians();
    let delta_lon = (to.lon - from.lon).to_radians();

    let a =
        (delta_lat / 2.0).sin().powi(2) + lat1.cos() * lat2.cos() * (delta_lon / 2.0).sin().powi(2);
    let c = 2.0 * a.sqrt().asin();

    EARTH_RADIUS_KM * c
}

/// Calculate the bearing from one coordinate to another (in degrees)
pub fn bearing(from: GeoCoord, to: GeoCoord) -> f64 {
    let lat1 = from.lat.to_radians();
    let lat2 = to.lat.to_radians();
    let delta_lon = (to.lon - from.lon).to_radians();

    let y = delta_lon.sin() * lat2.cos();
    let x = lat1.cos() * lat2.sin() - lat1.sin() * lat2.cos() * delta_lon.cos();

    y.atan2(x).to_degrees().rem_euclid(360.0)
}

/// Interpolate a position along the great-circle path between two points
/// `t` is the fraction of the journey (0.0 = start, 1.0 = end)
pub fn interpolate_great_circle(from: GeoCoord, to: GeoCoord, t: f64) -> GeoCoord {
    let lat1 = from.lat.to_radians();
    let lon1 = from.lon.to_radians();
    let lat2 = to.lat.to_radians();
    let lon2 = to.lon.to_radians();

    // Calculate angular distance
    let delta = haversine_distance(from, to) / EARTH_RADIUS_KM;

    if delta.abs() < 1e-10 {
        return from;
    }

    let a = ((1.0 - t) * delta).sin() / delta.sin();
    let b = (t * delta).sin() / delta.sin();

    let x = a * lat1.cos() * lon1.cos() + b * lat2.cos() * lon2.cos();
    let y = a * lat1.cos() * lon1.sin() + b * lat2.cos() * lon2.sin();
    let z = a * lat1.sin() + b * lat2.sin();

    let lat = z.atan2((x * x + y * y).sqrt());
    let lon = y.atan2(x);

    GeoCoord::new(lat.to_degrees(), lon.to_degrees())
}

/// Ballistic trajectory calculator
/// Models a simplified ballistic missile flight profile
#[derive(Clone, Debug)]
pub struct BallisticTrajectory {
    pub origin: GeoCoord,
    pub target: GeoCoord,
    pub range_km: f64,
    pub max_altitude_km: f64,
    pub flight_time_sec: f64,
    /// Uncertainty in estimated origin (km) - None for perfect trajectories
    pub origin_uncertainty_km: Option<f64>,
    /// Uncertainty in estimated target (km) - None for perfect trajectories
    pub target_uncertainty_km: Option<f64>,
    /// Uncertainty in estimated apogee (km) - None for perfect trajectories
    pub apogee_uncertainty_km: Option<f64>,
    /// True if this trajectory was reconstructed from sensor data
    pub is_sensor_derived: bool,
}

impl BallisticTrajectory {
    /// Create a new ballistic trajectory with auto-calculated parameters
    /// `range_km` is auto-calculated, `max_altitude_km` is estimated based on range
    pub fn new(origin: GeoCoord, target: GeoCoord) -> Self {
        let range_km = haversine_distance(origin, target);

        // Estimate max altitude based on range
        // ICBMs typically reach 1000-1500km apogee for 10000km range
        // This is a simplified model
        let max_altitude_km = estimate_apogee(range_km);

        // Estimate flight time (simplified)
        // ICBMs typically take 25-35 minutes for intercontinental range
        let flight_time_sec = estimate_flight_time(range_km);

        Self {
            origin,
            target,
            range_km,
            max_altitude_km,
            flight_time_sec,
            origin_uncertainty_km: None,
            target_uncertainty_km: None,
            apogee_uncertainty_km: None,
            is_sensor_derived: false,
        }
    }

    /// Create a new ballistic trajectory with custom parameters from config
    pub fn with_params(
        origin: GeoCoord,
        target: GeoCoord,
        max_altitude_km: f64,
        flight_time_sec: f64,
    ) -> Self {
        let range_km = haversine_distance(origin, target);

        Self {
            origin,
            target,
            range_km,
            max_altitude_km,
            flight_time_sec,
            origin_uncertainty_km: None,
            target_uncertainty_km: None,
            apogee_uncertainty_km: None,
            is_sensor_derived: false,
        }
    }

    /// Get position and altitude at a given progress (0.0 to 1.0)
    pub fn position_at(&self, progress: f64) -> (GeoCoord, f64) {
        let t = progress.clamp(0.0, 1.0);

        // Ground position along great circle
        let ground_pos = interpolate_great_circle(self.origin, self.target, t);

        // Altitude follows a parabolic profile
        // Peak at t = 0.5 (midcourse)
        let altitude = self.altitude_at_progress(t);

        (ground_pos, altitude)
    }

    /// Calculate altitude at a given progress using parabolic approximation
    fn altitude_at_progress(&self, t: f64) -> f64 {
        // Parabola: h(t) = 4 * max_h * t * (1 - t)
        // This gives 0 at t=0 and t=1, max at t=0.5
        4.0 * self.max_altitude_km * t * (1.0 - t)
    }

    /// Reconstruct a ballistic trajectory from sensor observations
    /// Uses current position, velocity vector, and altitude to estimate trajectory parameters
    /// Returns None if insufficient data or unrealistic trajectory
    pub fn from_sensor_track(
        current_position: GeoCoord,
        current_altitude_km: f64,
        velocity: &crate::simulation::VelocityEstimate,
        current_flight_progress_estimate: f64,
    ) -> Option<Self> {
        let t = current_flight_progress_estimate;

        // 1. FIRST estimate apogee from current altitude and flight progress
        // Using parabolic trajectory model: h(t) = 4 * max_h * t * (1-t)
        // Solving for max_h: max_h = h / (4 * t * (1-t))
        let denominator = 4.0 * t * (1.0 - t);
        let estimated_apogee = if denominator > 0.01 {
            // Use parabolic formula
            (current_altitude_km / denominator)
                .clamp(current_altitude_km, current_altitude_km * 5.0)
        } else {
            // Near launch or impact - use altitude-based estimate
            // For early flight (t < 0.1), estimate apogee as higher
            if t < 0.1 {
                current_altitude_km * 4.0 // Early in flight, apogee will be much higher
            } else {
                current_altitude_km * 1.5 // Near impact
            }
        };

        // 2. Now estimate total range from the estimated APOGEE (not current altitude!)
        let estimated_total_range_km = estimate_range_from_apogee(estimated_apogee);

        // 3. Project backward and forward along heading to estimate origin/target
        let heading = velocity.heading_deg;

        // Distance traveled so far (rough estimate)
        let distance_traveled = estimated_total_range_km * t;
        let distance_remaining = estimated_total_range_km * (1.0 - t);

        // Project backward to estimate origin
        let reverse_heading = (heading + 180.0).rem_euclid(360.0);
        let estimated_origin = crate::simulation::calculate_position_from_bearing_range(
            current_position,
            reverse_heading,
            distance_traveled,
        );

        // Project forward to estimate target
        let estimated_target = crate::simulation::calculate_position_from_bearing_range(
            current_position,
            heading,
            distance_remaining,
        );

        // 4. Estimate total flight time from range
        let estimated_flight_time = estimate_flight_time(estimated_total_range_km);

        // 5. Calculate uncertainty based on velocity confidence
        let origin_uncertainty = 50.0 * (1.0 - velocity.confidence);
        let target_uncertainty = 50.0 * (1.0 - velocity.confidence);
        let apogee_uncertainty = current_altitude_km * 0.3;

        // 6. Construct trajectory with estimated parameters
        Some(BallisticTrajectory {
            origin: estimated_origin,
            target: estimated_target,
            range_km: estimated_total_range_km,
            max_altitude_km: estimated_apogee,
            flight_time_sec: estimated_flight_time,
            origin_uncertainty_km: Some(origin_uncertainty),
            target_uncertainty_km: Some(target_uncertainty),
            apogee_uncertainty_km: Some(apogee_uncertainty),
            is_sensor_derived: true,
        })
    }

    /// Estimate current flight progress (0.0-1.0) from altitude and vertical rate
    pub fn estimate_flight_progress_from_altitude(
        altitude_km: f64,
        vertical_rate_km_s: f64,
        max_altitude_estimate: f64,
    ) -> f64 {
        // Parabola: h(t) = 4 * max_h * t * (1-t)
        // Solve for t given h: t = 0.5 ± sqrt(0.25 - h / (4 * max_h))

        let altitude_ratio = altitude_km / max_altitude_estimate.max(1.0);
        if altitude_ratio >= 0.99 {
            return 0.5; // At apogee
        }

        let discriminant = 0.25 - altitude_ratio / 4.0;
        if discriminant < 0.0 {
            return 0.5; // Math error, default to midcourse
        }

        let sqrt_term = discriminant.sqrt();

        // Two solutions: ascending (t < 0.5) or descending (t > 0.5)
        // Use vertical_rate sign to determine which
        if vertical_rate_km_s >= 0.0 {
            // Ascending - use smaller t
            (0.5 - sqrt_term).max(0.0)
        } else {
            // Descending - use larger t
            (0.5 + sqrt_term).min(1.0)
        }
    }

    /// Determine flight phase based on progress
    pub fn phase_at(&self, progress: f64) -> FlightPhase {
        match progress {
            p if p < 0.0 => FlightPhase::PreLaunch,
            p if p < 0.1 => FlightPhase::Boost,
            p if p < 0.85 => FlightPhase::Midcourse,
            p if p < 1.0 => FlightPhase::Terminal,
            _ => FlightPhase::Impact,
        }
    }
}

/// Flight phase of a ballistic missile
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FlightPhase {
    PreLaunch,
    Boost,
    Midcourse,
    Terminal,
    Impact,
}

/// Estimate apogee (max altitude) based on range using orbital mechanics
///
/// Uses minimum-energy trajectory assumption for ballistic missiles.
/// Reference: Bate, Mueller, White, "Fundamentals of Astrodynamics"
///
/// # Physics
/// For a minimum-energy transfer over range R:
/// - Central angle: θ = R / R_earth
/// - For a ballistic trajectory on a non-rotating Earth:
///   - Apogee occurs at θ/2 from launch
///   - Semi-latus rectum: p = r₀ × sin²(θ/2) / sin(θ/2) (simplified)
///
/// For practical ballistic missiles, we use energy-based derivation:
/// - Burnout altitude varies with range
/// - Burnout velocity determines apogee via energy conservation
/// - E = v²/2 - μ/r  (specific orbital energy)
/// - At apogee: E = v_horizontal²/2 - μ/r_apogee
pub fn estimate_apogee(range_km: f64) -> f64 {
    if range_km <= 0.0 {
        return 0.0;
    }

    // Estimate burnout conditions based on range
    let (burnout_alt_km, burnout_velocity_km_s, launch_angle_deg) = burnout_conditions(range_km);

    // Calculate apogee from burnout conditions using energy conservation
    apogee_from_burnout(burnout_alt_km, burnout_velocity_km_s, launch_angle_deg)
}

/// Estimate burnout conditions (altitude, velocity, angle) based on range
///
/// Returns (burnout_altitude_km, burnout_velocity_km_s, launch_angle_deg)
fn burnout_conditions(range_km: f64) -> (f64, f64, f64) {
    // Based on typical ballistic missile parameters
    // Reference: Various open-source missile data

    if range_km < 300.0 {
        // Tactical/SRBM: Low burnout, steep trajectory
        (15.0, 1.5 + range_km * 0.003, 55.0)
    } else if range_km < 1000.0 {
        // SRBM: ~50km burnout, 2-3 km/s
        (30.0 + range_km * 0.02, 2.0 + range_km * 0.001, 50.0)
    } else if range_km < 3000.0 {
        // MRBM: ~100km burnout, 3-5 km/s
        (80.0 + range_km * 0.01, 3.0 + range_km * 0.0007, 45.0)
    } else if range_km < 5500.0 {
        // IRBM: ~150km burnout, 5-6 km/s
        (120.0 + range_km * 0.01, 4.5 + range_km * 0.0003, 40.0)
    } else {
        // ICBM: ~200-300km burnout, 6.5-7.5 km/s
        (200.0 + range_km * 0.005, 6.0 + range_km * 0.0001, 35.0)
    }
}

/// Calculate apogee from burnout conditions using energy conservation
///
/// # Arguments
/// * `burnout_alt_km` - Altitude at burnout (km)
/// * `burnout_velocity_km_s` - Velocity at burnout (km/s)
/// * `launch_angle_deg` - Flight path angle at burnout (degrees from horizontal)
///
/// # Physics
/// Specific orbital energy: E = v²/2 - μ/r
/// At apogee, radial velocity = 0, so: E = v_horizontal²/2 - μ/r_apogee
/// Solving: r_apogee = μ / (μ/r - v²/2 + v_horizontal²/2)
fn apogee_from_burnout(
    burnout_alt_km: f64,
    burnout_velocity_km_s: f64,
    launch_angle_deg: f64,
) -> f64 {
    let r_burnout = EARTH_RADIUS_KM + burnout_alt_km;
    let v = burnout_velocity_km_s;
    let gamma = launch_angle_deg.to_radians();

    // Velocity components
    let v_radial = v * gamma.sin();
    let v_horizontal = v * gamma.cos();

    // Specific orbital energy
    let mu = MU; // km³/s² (gravitational parameter)
    let energy = (v * v) / 2.0 - mu / r_burnout;

    // For suborbital trajectory, energy < 0
    if energy >= 0.0 {
        // Escape trajectory - use simplified model
        return burnout_alt_km + v_radial * v_radial / (2.0 * 0.00981);
    }

    // Semi-major axis from energy: a = -μ/(2E)
    let semi_major_axis = -mu / (2.0 * energy);

    // Specific angular momentum: h = r × v_horizontal
    let angular_momentum = r_burnout * v_horizontal;

    // Semi-latus rectum: p = h²/μ
    let semi_latus_rectum = (angular_momentum * angular_momentum) / mu;

    // Eccentricity from p = a(1-e²)
    let e_squared = 1.0 - semi_latus_rectum / semi_major_axis;
    let eccentricity = if e_squared > 0.0 {
        e_squared.sqrt()
    } else {
        0.0
    };

    // Apogee radius: r_a = a(1+e)
    let apogee_radius = semi_major_axis * (1.0 + eccentricity);

    // Apogee altitude
    let apogee_altitude = apogee_radius - EARTH_RADIUS_KM;

    // Sanity check: apogee should be above burnout
    apogee_altitude.max(burnout_alt_km)
}

/// Estimate range from apogee (inverse of estimate_apogee)
/// Used for trajectory reconstruction from sensor data
///
/// Uses bisection search since the physics-based apogee function
/// is not analytically invertible.
pub fn estimate_range_from_apogee(apogee_km: f64) -> f64 {
    if apogee_km <= 0.0 {
        return 0.0;
    }

    // Use bisection to find range that produces the given apogee
    let mut low = 50.0; // Minimum range (km)
    let mut high = 15000.0; // Maximum range (km) - beyond ICBM range

    // Quick bounds check
    if estimate_apogee(low) > apogee_km {
        return low;
    }
    if estimate_apogee(high) < apogee_km {
        return high;
    }

    // Bisection search
    for _ in 0..50 {
        let mid = (low + high) / 2.0;
        let mid_apogee = estimate_apogee(mid);

        if (mid_apogee - apogee_km).abs() < 1.0 {
            return mid; // Close enough
        }

        if mid_apogee < apogee_km {
            low = mid;
        } else {
            high = mid;
        }
    }

    (low + high) / 2.0
}

/// Estimate flight time based on range (in seconds)
pub fn estimate_flight_time(range_km: f64) -> f64 {
    // Rough approximation
    // Short range: ~5-10 minutes
    // Medium range: ~10-15 minutes
    // ICBM: ~25-35 minutes

    if range_km < 500.0 {
        300.0 + range_km * 0.3 // 5-7.5 minutes
    } else if range_km < 3000.0 {
        450.0 + range_km * 0.2 // 7.5-17.5 minutes
    } else {
        900.0 + range_km * 0.12 // 15-35 minutes
    }
}

/// Convert kilometers to degrees (approximate, at equator)
pub fn km_to_degrees(km: f64) -> f64 {
    km / 111.32 // 1 degree ≈ 111.32 km at equator
}

/// Convert degrees to kilometers (approximate, at equator)
pub fn degrees_to_km(degrees: f64) -> f64 {
    degrees * 111.32
}

/// Uncertainty envelope for a predicted trajectory
/// Used when calculating intercept solutions from filtered estimates
#[derive(Clone, Debug)]
pub struct TrajectoryUncertainty {
    /// Current position uncertainty (standard deviation in km)
    pub position_uncertainty_km: f64,
    /// Rate at which uncertainty grows per second (km/s)
    /// This accounts for velocity uncertainty propagating to position
    pub grows_per_second: f64,
    /// Velocity uncertainty (km/s)
    pub velocity_uncertainty_km_s: f64,
}

impl TrajectoryUncertainty {
    /// Get position uncertainty at a future time
    pub fn uncertainty_at_time(&self, dt: f64) -> f64 {
        // Uncertainty grows linearly with time due to velocity uncertainty
        self.position_uncertainty_km + self.grows_per_second * dt.abs()
    }
}

/// Result of trajectory prediction from track data
#[derive(Clone, Debug)]
pub struct TrackPrediction {
    /// Predicted trajectory (may have larger uncertainty than ground truth)
    pub trajectory: BallisticTrajectory,
    /// Uncertainty information
    pub uncertainty: TrajectoryUncertainty,
    /// Estimated current progress along trajectory (0.0-1.0)
    pub current_progress: f64,
    /// Estimated time remaining to impact (seconds)
    pub time_to_impact_sec: f64,
}

/// Predict future trajectory from fused track using Kalman state
///
/// This is used for intercept calculations when we want to use filtered
/// estimates instead of ground truth missile state.
///
/// Returns None if insufficient data for prediction (no velocity estimate or Kalman state)
pub fn predict_trajectory_from_track(
    fused_track: &FusedTrack,
    kalman_state: Option<&BallisticState>,
    _prediction_horizon_sec: f64,
) -> Option<TrackPrediction> {
    // Need either velocity estimate or Kalman state
    let velocity = fused_track.estimated_velocity.as_ref()?;

    // Calculate uncertainty from available data
    let (position_uncertainty, velocity_uncertainty, grows_per_second) =
        if let Some(kf) = kalman_state {
            let pos_unc = kf.get_position_uncertainty();
            let vel_unc = kf.get_velocity_uncertainty();
            // Uncertainty growth rate is the velocity uncertainty
            (pos_unc, vel_unc, vel_unc)
        } else {
            // Fall back to fused track uncertainty
            let pos_unc = fused_track
                .kalman_position_uncertainty_km
                .unwrap_or(fused_track.uncertainty_radius_km);
            // Estimate velocity uncertainty from confidence
            let vel_unc = 0.5 * (1.0 - velocity.confidence);
            (pos_unc, vel_unc, vel_unc)
        };

    // Estimate current apogee (max altitude so far or estimated)
    // Use current altitude and vertical rate to estimate
    let estimated_apogee = if velocity.vertical_rate_km_s >= 0.0 {
        // Still ascending - apogee not yet reached
        // Estimate using ballistic kinematics: h_max = h + v²/(2g)
        let g = 0.00981; // km/s²
        let v_up = velocity.vertical_rate_km_s;
        fused_track.estimated_altitude + (v_up * v_up) / (2.0 * g)
    } else {
        // Descending - estimate apogee from current altitude assuming midcourse
        fused_track.estimated_altitude * 2.0 // rough estimate
    }
    .max(fused_track.estimated_altitude);

    // Estimate flight progress from altitude and vertical rate
    let current_progress = BallisticTrajectory::estimate_flight_progress_from_altitude(
        fused_track.estimated_altitude,
        velocity.vertical_rate_km_s,
        estimated_apogee,
    );

    // Build trajectory from sensor data
    let trajectory = BallisticTrajectory::from_sensor_track(
        fused_track.estimated_position,
        fused_track.estimated_altitude,
        velocity,
        current_progress,
    )?;

    // Estimate time to impact
    let time_to_impact_sec = trajectory.flight_time_sec * (1.0 - current_progress);

    Some(TrackPrediction {
        trajectory,
        uncertainty: TrajectoryUncertainty {
            position_uncertainty_km: position_uncertainty,
            grows_per_second,
            velocity_uncertainty_km_s: velocity_uncertainty,
        },
        current_progress,
        time_to_impact_sec,
    })
}

/// Predict position at a future time using EKF state (preferred), BallisticState, or trajectory
/// EKF is preferred because it uses geodetic coordinates with proper ballistic physics
pub fn predict_position_at_time_with_ekf(
    fused_track: &FusedTrack,
    ekf_state: Option<&crate::simulation::ekf::EKFState>,
    kalman_state: Option<&BallisticState>,
    dt: f64,
) -> Option<(GeoCoord, f64, f64)> {
    // Prefer EKF state prediction (geodetic coordinates, most accurate)
    if let Some(ekf) = ekf_state {
        let mut ekf_clone = ekf.clone();
        ekf_clone.predict(dt);
        let (pos, alt) = ekf_clone.get_position();
        let uncertainty = ekf_clone.get_position_uncertainty();
        return Some((pos, alt, uncertainty));
    }

    // Fall back to BallisticState (ENU coordinates)
    if let Some(kf) = kalman_state {
        return Some(kf.predict_at_time(dt));
    }

    // Fall back to trajectory-based prediction
    let prediction = predict_trajectory_from_track(fused_track, None, dt)?;
    let future_progress =
        (prediction.current_progress + dt / prediction.trajectory.flight_time_sec).clamp(0.0, 1.0);
    let (pos, alt) = prediction.trajectory.position_at(future_progress);
    let uncertainty = prediction.uncertainty.uncertainty_at_time(dt);

    Some((pos, alt, uncertainty))
}

/// Predict position at a future time using Kalman state if available,
/// otherwise fall back to trajectory-based prediction
pub fn predict_position_at_time(
    fused_track: &FusedTrack,
    kalman_state: Option<&BallisticState>,
    dt: f64,
) -> Option<(GeoCoord, f64, f64)> {
    // Prefer Kalman state prediction if available
    if let Some(kf) = kalman_state {
        return Some(kf.predict_at_time(dt));
    }

    // Fall back to trajectory-based prediction
    let prediction = predict_trajectory_from_track(fused_track, None, dt)?;
    let future_progress =
        (prediction.current_progress + dt / prediction.trajectory.flight_time_sec).clamp(0.0, 1.0);
    let (pos, alt) = prediction.trajectory.position_at(future_progress);
    let uncertainty = prediction.uncertainty.uncertainty_at_time(dt);

    Some((pos, alt, uncertainty))
}

// ============================================================================
// LAMBERT GUIDANCE
// Reference: Vallado, "Fundamentals of Astrodynamics and Applications"
// Used for exo-atmospheric (above ~100km) hit-to-kill intercepts
// ============================================================================

/// Gravitational parameter (km³/s²)
const MU: f64 = 398600.4418;

/// Result of Lambert solver
#[derive(Debug, Clone, Copy)]
pub struct LambertSolution {
    /// Required velocity vector at departure (km/s) in ECEF frame
    pub v1: [f64; 3],
    /// Arrival velocity vector (km/s) in ECEF frame
    pub v2: [f64; 3],
    /// Whether solution converged
    pub converged: bool,
}

/// Convert geodetic coordinates to ECEF (Earth-Centered Earth-Fixed)
/// Returns position vector [x, y, z] in km
pub fn geodetic_to_ecef(coord: GeoCoord, altitude_km: f64) -> [f64; 3] {
    let lat = coord.lat.to_radians();
    let lon = coord.lon.to_radians();
    let r = EARTH_RADIUS_KM + altitude_km;

    [
        r * lat.cos() * lon.cos(),
        r * lat.cos() * lon.sin(),
        r * lat.sin(),
    ]
}

/// Convert ECEF position to geodetic coordinates
pub fn ecef_to_geodetic(pos: [f64; 3]) -> (GeoCoord, f64) {
    let x = pos[0];
    let y = pos[1];
    let z = pos[2];

    let r = (x * x + y * y + z * z).sqrt();
    let lat = (z / r).asin().to_degrees();
    let lon = y.atan2(x).to_degrees();
    let altitude_km = r - EARTH_RADIUS_KM;

    (GeoCoord { lat, lon }, altitude_km)
}

/// Vector magnitude
fn vec_mag(v: [f64; 3]) -> f64 {
    (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt()
}

/// Dot product
fn vec_dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

/// Cross product
fn vec_cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

/// Stumpff function C(z)
fn stumpff_c(z: f64) -> f64 {
    if z.abs() < 1e-6 {
        0.5 - z / 24.0 + z * z / 720.0
    } else if z > 0.0 {
        (1.0 - z.sqrt().cos()) / z
    } else {
        ((-z).sqrt().cosh() - 1.0) / (-z)
    }
}

/// Stumpff function S(z)
fn stumpff_s(z: f64) -> f64 {
    if z.abs() < 1e-6 {
        1.0 / 6.0 - z / 120.0 + z * z / 5040.0
    } else if z > 0.0 {
        (z.sqrt() - z.sqrt().sin()) / z.powf(1.5)
    } else {
        ((-z).sqrt().sinh() - (-z).sqrt()) / (-z).powf(1.5)
    }
}

/// Solve Lambert's problem using universal variable method
/// Given two positions and time of flight, find the required velocities
///
/// # Arguments
/// * `r1` - Initial position vector (km) in ECEF
/// * `r2` - Final position vector (km) in ECEF
/// * `tof` - Time of flight (seconds)
/// * `prograde` - True for prograde (short way), false for retrograde
///
/// # Returns
/// Lambert solution with velocity vectors, or None if no solution found
pub fn solve_lambert(
    r1: [f64; 3],
    r2: [f64; 3],
    tof: f64,
    prograde: bool,
) -> Option<LambertSolution> {
    let r1_mag = vec_mag(r1);
    let r2_mag = vec_mag(r2);

    // Determine transfer angle
    let cross = vec_cross(r1, r2);
    let cos_dnu = vec_dot(r1, r2) / (r1_mag * r2_mag);
    let cos_dnu = cos_dnu.clamp(-1.0, 1.0);

    // Determine direction based on prograde/retrograde
    let sin_dnu = if prograde == (cross[2] >= 0.0) {
        (1.0 - cos_dnu * cos_dnu).sqrt()
    } else {
        -(1.0 - cos_dnu * cos_dnu).sqrt()
    };

    // Variable A for Lambert's problem
    let a_lambert = sin_dnu * (r1_mag * r2_mag / (1.0 - cos_dnu)).sqrt();

    if a_lambert.abs() < 1e-10 {
        return None; // Degenerate case
    }

    // Newton-Raphson iteration to find universal variable z
    let mut z = 0.0; // Initial guess (parabolic)

    // Better initial guess based on transfer angle
    if cos_dnu < 0.0 {
        z = 1.0; // Elliptical
    }

    let max_iter = 50;
    let tol = 1e-8;

    for _ in 0..max_iter {
        let c = stumpff_c(z);
        let s = stumpff_s(z);

        // y function
        let y = r1_mag + r2_mag + a_lambert * (z * s - 1.0) / c.sqrt();

        if y < 0.0 {
            z += 0.5; // Adjust if y becomes negative
            continue;
        }

        // x = sqrt(y/C(z))
        let x = (y / c).sqrt();

        // Time of flight for current z
        let t = (x * x * x * s + a_lambert * y.sqrt()) / MU.sqrt();

        // Derivative dt/dz
        let dt_dz = if z.abs() < 1e-6 {
            (y.sqrt() / (2.0 * MU)).powf(1.5) * (s / c + 1.0 / (4.0 * c))
                + a_lambert / (4.0 * MU.sqrt()) * (3.0 * s * y.sqrt() / c + a_lambert / y.sqrt())
        } else {
            let dc_dz = (1.0 - z * s - 2.0 * c) / (2.0 * z);
            let ds_dz = (c - 3.0 * s) / (2.0 * z);
            (x * x * x * (ds_dz - 3.0 * s * dc_dz / (2.0 * c))
                + a_lambert * (3.0 * s * y.sqrt() / c + a_lambert / y.sqrt()) / 4.0)
                / MU.sqrt()
        };

        // Newton step
        let dz = (tof - t) / dt_dz;
        z += dz;

        if dz.abs() < tol {
            // Converged - compute velocities
            let c = stumpff_c(z);
            let s = stumpff_s(z);
            let y = r1_mag + r2_mag + a_lambert * (z * s - 1.0) / c.sqrt();
            let y = y.max(0.0);

            let f = 1.0 - y / r1_mag;
            let g = a_lambert * (y / MU).sqrt();
            let g_dot = 1.0 - y / r2_mag;

            // v1 = (r2 - f*r1) / g
            let v1 = [
                (r2[0] - f * r1[0]) / g,
                (r2[1] - f * r1[1]) / g,
                (r2[2] - f * r1[2]) / g,
            ];

            // v2 = (g_dot*r2 - r1) / g
            let v2 = [
                (g_dot * r2[0] - r1[0]) / g,
                (g_dot * r2[1] - r1[1]) / g,
                (g_dot * r2[2] - r1[2]) / g,
            ];

            return Some(LambertSolution {
                v1,
                v2,
                converged: true,
            });
        }
    }

    // Did not converge
    None
}

/// Compute Lambert guidance for exo-atmospheric intercept
/// Returns the required heading (degrees) and climb angle (degrees) for the interceptor
///
/// # Arguments
/// * `interceptor_pos` - Interceptor position
/// * `interceptor_alt` - Interceptor altitude (km)
/// * `target_pos` - Predicted target position at intercept
/// * `target_alt` - Predicted target altitude at intercept (km)
/// * `time_to_intercept` - Time to reach intercept point (seconds)
///
/// # Returns
/// (heading_deg, climb_angle_deg, required_speed_km_s) or None if no solution
pub fn lambert_guidance(
    interceptor_pos: GeoCoord,
    interceptor_alt: f64,
    target_pos: GeoCoord,
    target_alt: f64,
    time_to_intercept: f64,
) -> Option<(f64, f64, f64)> {
    // Minimum altitude for Lambert guidance (Kármán line)
    const MIN_LAMBERT_ALT_KM: f64 = 100.0;

    // Only use Lambert for exo-atmospheric intercepts
    if interceptor_alt < MIN_LAMBERT_ALT_KM && target_alt < MIN_LAMBERT_ALT_KM {
        return None;
    }

    // Convert to ECEF
    let r1 = geodetic_to_ecef(interceptor_pos, interceptor_alt);
    let r2 = geodetic_to_ecef(target_pos, target_alt);

    // Solve Lambert's problem (prograde trajectory)
    let solution = solve_lambert(r1, r2, time_to_intercept, true)?;

    // Extract velocity components
    let v1 = solution.v1;
    let speed = vec_mag(v1);

    // Convert velocity to local ENU (East-North-Up) frame for heading/climb
    let lat = interceptor_pos.lat.to_radians();
    let lon = interceptor_pos.lon.to_radians();

    // Rotation matrix from ECEF to ENU
    let sin_lat = lat.sin();
    let cos_lat = lat.cos();
    let sin_lon = lon.sin();
    let cos_lon = lon.cos();

    // v_enu = R * v_ecef
    let v_east = -sin_lon * v1[0] + cos_lon * v1[1];
    let v_north = -sin_lat * cos_lon * v1[0] - sin_lat * sin_lon * v1[1] + cos_lat * v1[2];
    let v_up = cos_lat * cos_lon * v1[0] + cos_lat * sin_lon * v1[1] + sin_lat * v1[2];

    // Heading from velocity (degrees from north, clockwise)
    let heading_rad = v_east.atan2(v_north);
    let heading_deg = heading_rad.to_degrees().rem_euclid(360.0);

    // Climb angle (positive = climbing)
    let horizontal_speed = (v_east * v_east + v_north * v_north).sqrt();
    let climb_angle_rad = v_up.atan2(horizontal_speed);
    let climb_angle_deg = climb_angle_rad.to_degrees();

    Some((heading_deg, climb_angle_deg, speed))
}

// ============================================================================
// ATMOSPHERIC DRAG MODEL
// Reference: 1976 US Standard Atmosphere (NASA-TM-X-74335)
// ============================================================================

/// Sea-level atmospheric density (kg/m³)
const RHO_0: f64 = 1.225;

/// Scale height for exponential atmosphere model (km)
/// H ≈ 8.5 km is a good approximation for the lower atmosphere
const SCALE_HEIGHT_KM: f64 = 8.5;

/// Atmospheric density at altitude using exponential model
/// Based on 1976 US Standard Atmosphere
///
/// # Arguments
/// * `altitude_km` - Altitude above sea level (km)
///
/// # Returns
/// Atmospheric density (kg/m³)
///
/// # Notes
/// The exponential model ρ(h) = ρ₀ × exp(-h/H) is accurate to ~10% up to 100km.
/// Above 100km (Kármán line), density is effectively zero for drag purposes.
pub fn atmospheric_density(altitude_km: f64) -> f64 {
    if altitude_km >= 100.0 {
        return 0.0; // Above Kármán line - exoatmospheric
    }
    if altitude_km < 0.0 {
        return RHO_0; // Below sea level - use sea level density
    }

    // For more accuracy, use piecewise model based on US Standard Atmosphere
    // Reference: https://www.grc.nasa.gov/www/k-12/airplane/atmosmet.html
    if altitude_km < 11.0 {
        // Troposphere (0-11 km)
        let temp = 288.15 - 6.5 * altitude_km; // Temperature (K)
        let pressure = 101325.0 * (temp / 288.15).powf(5.2561); // Pressure (Pa)
        pressure / (287.05 * temp) // Ideal gas law: ρ = P / (R × T)
    } else if altitude_km < 20.0 {
        // Lower stratosphere (11-20 km) - isothermal
        let temp = 216.65; // Constant temperature (K)
        let pressure = 22632.0 * (-(altitude_km - 11.0) / 6.341).exp();
        pressure / (287.05 * temp)
    } else if altitude_km < 32.0 {
        // Upper stratosphere (20-32 km)
        let temp = 216.65 + (altitude_km - 20.0); // Temperature increases
        let pressure = 5474.9 * (temp / 216.65).powf(-34.163);
        pressure / (287.05 * temp)
    } else if altitude_km < 47.0 {
        // Stratosphere (32-47 km)
        let temp = 228.65 + 2.8 * (altitude_km - 32.0);
        let pressure = 868.02 * (temp / 228.65).powf(-12.201);
        pressure / (287.05 * temp)
    } else if altitude_km < 51.0 {
        // Mesosphere lower (47-51 km) - isothermal
        let temp = 270.65;
        let pressure = 110.91 * (-(altitude_km - 47.0) / 7.922).exp();
        pressure / (287.05 * temp)
    } else if altitude_km < 71.0 {
        // Mesosphere (51-71 km)
        let temp = 270.65 - 2.8 * (altitude_km - 51.0);
        let pressure = 66.939 * (temp / 270.65).powf(12.201);
        pressure / (287.05 * temp)
    } else {
        // Upper mesosphere/thermosphere (71-100 km)
        // Exponential decay approximation
        let temp = 214.65 - 2.0 * (altitude_km - 71.0);
        let temp = temp.max(180.0); // Minimum temperature bound
        let pressure = 3.9564 * (-(altitude_km - 71.0) / 5.0).exp();
        pressure / (287.05 * temp)
    }
}

/// Ballistic coefficient for different vehicle types
/// β = m / (Cd × A) in kg/m²
///
/// Higher ballistic coefficient = less affected by drag
/// - ICBMs/RVs: 10,000-20,000 kg/m² (very streamlined, dense)
/// - Interceptors: 1,000-5,000 kg/m² (lighter, more maneuverable)
/// - Aircraft: 100-500 kg/m²
#[derive(Debug, Clone, Copy)]
pub struct BallisticCoefficient {
    /// Mass (kg)
    pub mass_kg: f64,
    /// Drag coefficient (dimensionless, typically 0.2-0.5 for missiles)
    pub drag_coefficient: f64,
    /// Reference area (m²)
    pub reference_area_m2: f64,
}

impl BallisticCoefficient {
    /// Create a new ballistic coefficient
    pub fn new(mass_kg: f64, drag_coefficient: f64, reference_area_m2: f64) -> Self {
        Self {
            mass_kg,
            drag_coefficient,
            reference_area_m2,
        }
    }

    /// Get the ballistic coefficient value (kg/m²)
    pub fn value(&self) -> f64 {
        self.mass_kg / (self.drag_coefficient * self.reference_area_m2)
    }

    /// Default for ICBM reentry vehicle
    pub fn icbm_rv() -> Self {
        // Typical Mk21 RV: ~300kg, Cd≈0.15, A≈0.1m²
        Self::new(300.0, 0.15, 0.1) // β ≈ 20,000 kg/m²
    }

    /// Default for MRBM reentry vehicle
    pub fn mrbm_rv() -> Self {
        // Smaller, less optimized: ~200kg, Cd≈0.2, A≈0.15m²
        Self::new(200.0, 0.2, 0.15) // β ≈ 6,667 kg/m²
    }

    /// Default for SRBM
    pub fn srbm() -> Self {
        // Small tactical missile: ~100kg, Cd≈0.3, A≈0.1m²
        Self::new(100.0, 0.3, 0.1) // β ≈ 3,333 kg/m²
    }

    /// Default for exo-atmospheric interceptor (GBI, SM-3, Arrow-3)
    pub fn exo_interceptor() -> Self {
        // EKV or similar: ~70kg, Cd≈0.2, A≈0.05m²
        Self::new(70.0, 0.2, 0.05) // β ≈ 7,000 kg/m²
    }

    /// Default for endo-atmospheric interceptor (PAC-3, THAAD)
    pub fn endo_interceptor() -> Self {
        // Hit-to-kill vehicle: ~100kg, Cd≈0.3, A≈0.08m²
        Self::new(100.0, 0.3, 0.08) // β ≈ 4,167 kg/m²
    }
}

/// Calculate drag deceleration at given conditions
///
/// # Arguments
/// * `velocity_km_s` - Current velocity (km/s)
/// * `altitude_km` - Current altitude (km)
/// * `ballistic_coef` - Ballistic coefficient (kg/m²)
///
/// # Returns
/// Deceleration due to drag (km/s²)
///
/// # Formula
/// a_drag = (ρ × v² × Cd × A) / (2 × m) = (ρ × v²) / (2 × β)
/// where β = m / (Cd × A) is the ballistic coefficient
pub fn drag_deceleration(velocity_km_s: f64, altitude_km: f64, ballistic_coef: f64) -> f64 {
    let rho = atmospheric_density(altitude_km); // kg/m³

    // Convert velocity to m/s for consistent units
    let v_m_s = velocity_km_s * 1000.0;

    // Drag deceleration in m/s²
    let a_drag_m_s2 = (rho * v_m_s * v_m_s) / (2.0 * ballistic_coef);

    // Convert back to km/s²
    a_drag_m_s2 / 1000.0
}

/// Apply drag to velocity over a time step
///
/// # Arguments
/// * `velocity_km_s` - Current velocity (km/s)
/// * `altitude_km` - Current altitude (km)
/// * `ballistic_coef` - Ballistic coefficient (kg/m²)
/// * `dt` - Time step (seconds)
///
/// # Returns
/// New velocity after drag (km/s)
pub fn apply_drag(velocity_km_s: f64, altitude_km: f64, ballistic_coef: f64, dt: f64) -> f64 {
    if altitude_km >= 100.0 || velocity_km_s <= 0.0 {
        return velocity_km_s; // No drag above Kármán line
    }

    let decel = drag_deceleration(velocity_km_s, altitude_km, ballistic_coef);
    let new_velocity = velocity_km_s - decel * dt;

    new_velocity.max(0.0) // Velocity can't go negative
}

/// Calculate cumulative velocity loss due to drag over altitude descent
/// Used for trajectory prediction (approximate integral)
///
/// # Arguments
/// * `initial_velocity_km_s` - Velocity at start of descent
/// * `start_altitude_km` - Starting altitude (km)
/// * `end_altitude_km` - Ending altitude (km)
/// * `ballistic_coef` - Ballistic coefficient (kg/m²)
/// * `entry_angle_deg` - Entry angle from horizontal (degrees)
///
/// # Returns
/// Velocity at end altitude (km/s)
pub fn integrate_drag_descent(
    initial_velocity_km_s: f64,
    start_altitude_km: f64,
    end_altitude_km: f64,
    ballistic_coef: f64,
    entry_angle_deg: f64,
) -> f64 {
    if start_altitude_km <= end_altitude_km {
        return initial_velocity_km_s;
    }

    // Use numerical integration with small altitude steps
    const STEP_KM: f64 = 1.0;
    let entry_angle_rad = entry_angle_deg.to_radians();
    let sin_gamma = entry_angle_rad.sin().abs().max(0.1); // Avoid division by zero

    let mut v = initial_velocity_km_s;
    let mut alt = start_altitude_km;

    while alt > end_altitude_km && v > 0.1 {
        let step = STEP_KM.min(alt - end_altitude_km);
        let rho = atmospheric_density(alt);

        // Convert to consistent units
        let v_m_s = v * 1000.0;

        // Drag along path: dv/ds = -ρv/(2β), ds = dh/sin(γ)
        let dv_dh = (rho * v_m_s) / (2.0 * ballistic_coef * sin_gamma);
        let dv = dv_dh * step * 1000.0; // Convert km to m

        v -= dv / 1000.0; // Back to km/s
        alt -= step;
    }

    v.max(0.0)
}
