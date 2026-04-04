use crate::map::GeoCoord;

const EARTH_RADIUS_KM: f64 = 6371.0;

/// Calculate the great-circle distance between two coordinates in kilometers
pub fn haversine_distance(from: GeoCoord, to: GeoCoord) -> f64 {
    let lat1 = from.lat.to_radians();
    let lat2 = to.lat.to_radians();
    let delta_lat = (to.lat - from.lat).to_radians();
    let delta_lon = (to.lon - from.lon).to_radians();

    let a = (delta_lat / 2.0).sin().powi(2)
        + lat1.cos() * lat2.cos() * (delta_lon / 2.0).sin().powi(2);
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
#[derive(Clone)]
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
        // 1. Estimate total range from altitude (inverse of estimate_apogee)
        let estimated_total_range_km = estimate_range_from_apogee(current_altitude_km);

        // 2. Project backward and forward along heading to estimate origin/target
        let heading = velocity.heading_deg;

        // Distance traveled so far (rough estimate)
        let distance_traveled = estimated_total_range_km * current_flight_progress_estimate;
        let distance_remaining = estimated_total_range_km * (1.0 - current_flight_progress_estimate);

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

        // 3. Estimate apogee from current altitude and flight phase
        // Parabola: h(t) = 4 * max_h * t * (1-t) => max_h = h(t) / (4 * t * (1-t))
        let t = current_flight_progress_estimate;
        let denominator = 4.0 * t * (1.0 - t);
        let estimated_apogee = if denominator > 0.001 {
            (current_altitude_km / denominator).clamp(current_altitude_km, current_altitude_km * 5.0)
        } else {
            // Near launch or impact, use range-based estimate
            estimate_apogee(estimated_total_range_km)
        };

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

/// Estimate apogee (max altitude) based on range
pub fn estimate_apogee(range_km: f64) -> f64 {
    // Rough approximation based on typical ICBM profiles
    // Short range (< 1000km): ~150-300km apogee
    // Medium range (1000-3000km): ~300-800km apogee
    // ICBM (> 5000km): ~1000-1500km apogee

    if range_km < 500.0 {
        range_km * 0.3  // Short range missiles
    } else if range_km < 3000.0 {
        150.0 + range_km * 0.2  // Medium range
    } else {
        600.0 + range_km * 0.1  // ICBMs
    }
}

/// Estimate range from apogee (inverse of estimate_apogee)
/// Used for trajectory reconstruction from sensor data
pub fn estimate_range_from_apogee(apogee_km: f64) -> f64 {
    // Inverse the piecewise function from estimate_apogee()
    if apogee_km < 150.0 {
        // range * 0.3 = apogee => range = apogee / 0.3
        apogee_km / 0.3
    } else if apogee_km < 750.0 {
        // 150 + range * 0.2 = apogee => range = (apogee - 150) / 0.2
        (apogee_km - 150.0) / 0.2
    } else {
        // 600 + range * 0.1 = apogee => range = (apogee - 600) / 0.1
        (apogee_km - 600.0) / 0.1
    }
}

/// Estimate flight time based on range (in seconds)
fn estimate_flight_time(range_km: f64) -> f64 {
    // Rough approximation
    // Short range: ~5-10 minutes
    // Medium range: ~10-15 minutes
    // ICBM: ~25-35 minutes

    if range_km < 500.0 {
        300.0 + range_km * 0.3  // 5-7.5 minutes
    } else if range_km < 3000.0 {
        450.0 + range_km * 0.2  // 7.5-17.5 minutes
    } else {
        900.0 + range_km * 0.12  // 15-35 minutes
    }
}

/// Convert kilometers to degrees (approximate, at equator)
pub fn km_to_degrees(km: f64) -> f64 {
    km / 111.32  // 1 degree ≈ 111.32 km at equator
}

/// Convert degrees to kilometers (approximate, at equator)
pub fn degrees_to_km(degrees: f64) -> f64 {
    degrees * 111.32
}
