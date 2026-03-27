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
fn estimate_apogee(range_km: f64) -> f64 {
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
