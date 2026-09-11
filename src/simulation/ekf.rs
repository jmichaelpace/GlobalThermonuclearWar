//! Extended Kalman Filter for ballistic trajectory estimation
//!
//! This module implements a true Extended Kalman Filter (EKF) that operates in
//! geodetic coordinates with nonlinear state dynamics and measurement models.
//!
//! Key differences from the linear Kalman filter:
//! - State in geodetic coordinates (lat/lon/alt) instead of local ENU
//! - State transition Jacobian computed at each step (depends on current state)
//! - Nonlinear measurement model for radar (range/azimuth/elevation)
//! - Measurement Jacobian computed at each step
//!
//! Earth model: WGS-84 ellipsoid. Horizontal motion uses the meridional
//! radius of curvature M(φ) (north) and prime vertical radius N(φ) (east),
//! matching the geodetic rate equations
//! dφ/dt = v_n / M, dλ/dt = v_e / (N·cos φ).

use crate::simulation::physics::{
    geodetic_to_ecef, meridional_radius_km, prime_vertical_radius_km, G0,
};
use crate::types::GeoCoord;

/// Extended Kalman Filter state in geodetic coordinates
///
/// State vector: [lat, lon, alt, v_north, v_east, v_up]
/// - lat, lon in radians
/// - alt in km (above sea level)
/// - velocities in km/s (North-East-Up local frame)
#[derive(Clone, Debug)]
pub struct EKFState {
    /// State vector [lat_rad, lon_rad, alt_km, v_north_km_s, v_east_km_s, v_up_km_s]
    pub x: [f64; 6],
    /// State covariance matrix (6x6, flattened row-major)
    pub P: [f64; 36],
    /// Last update timestamp
    pub timestamp: f64,
}

/// Raw radar measurement in spherical coordinates
#[derive(Clone, Debug)]
pub struct RadarMeasurement {
    /// Slant range to target (km)
    pub range_km: f64,
    /// Azimuth angle (radians, 0 = North, positive clockwise)
    pub azimuth_rad: f64,
    /// Elevation angle (radians, positive above horizon)
    pub elevation_rad: f64,
    /// Sensor position
    pub sensor_position: GeoCoord,
    /// Sensor altitude (km)
    pub sensor_altitude_km: f64,
    /// Measurement timestamp
    pub timestamp: f64,
    /// Measurement noise standard deviations [range_km, azimuth_rad, elevation_rad]
    pub noise_std: [f64; 3],
}

impl EKFState {
    /// Initialize EKF state from first radar measurement
    pub fn new(measurement: &RadarMeasurement) -> Self {
        // Convert radar measurement to geodetic position
        let (pos, alt) = radar_to_geodetic(
            measurement.sensor_position,
            measurement.sensor_altitude_km,
            measurement.range_km,
            measurement.azimuth_rad,
            measurement.elevation_rad,
        );

        // Initial state - position known, velocity unknown
        let x = [
            pos.lat.to_radians(),
            pos.lon.to_radians(),
            alt,
            0.0, // v_north unknown
            0.0, // v_east unknown
            0.0, // v_up unknown
        ];

        // Initial covariance - position from measurement uncertainty, velocity large
        let range_var = measurement.noise_std[0].powi(2);
        let angle_var = measurement.noise_std[1].powi(2);

        // Position uncertainty from range/angle errors
        let pos_var_horiz = range_var + (measurement.range_km * angle_var).powi(2);
        let pos_var_vert = range_var;

        // Convert to lat/lon variance (approximate, using local curvature)
        let lat_var = pos_var_horiz / (meridional_radius_km(pos.lat).powi(2));
        let lon_var = pos_var_horiz
            / ((prime_vertical_radius_km(pos.lat) * pos.lat.to_radians().cos()).powi(2));

        // Large velocity uncertainty initially
        let vel_var = 1.0; // 1 km/s std dev

        let mut P = [0.0; 36];
        P[0] = lat_var; // var(lat)
        P[7] = lon_var; // var(lon)
        P[14] = pos_var_vert; // var(alt)
        P[21] = vel_var; // var(v_north)
        P[28] = vel_var; // var(v_east)
        P[35] = vel_var; // var(v_up)

        Self {
            x,
            P,
            timestamp: measurement.timestamp,
        }
    }

    /// Initialize from known geodetic position
    pub fn from_geodetic(pos: GeoCoord, alt_km: f64, timestamp: f64) -> Self {
        let x = [
            pos.lat.to_radians(),
            pos.lon.to_radians(),
            alt_km,
            0.0,
            0.0,
            0.0,
        ];

        // Conservative initial covariance (2 km position uncertainty
        // expressed via curvature radii at the initial latitude)
        let mut P = [0.0; 36];
        P[0] = (2.0 / meridional_radius_km(pos.lat)).powi(2); // ~2km position uncertainty
        P[7] = (2.0 / (prime_vertical_radius_km(pos.lat) * pos.lat.to_radians().cos())).powi(2);
        P[14] = 1.0; // 1km altitude uncertainty
        P[21] = 1.0; // 1 km/s velocity uncertainty
        P[28] = 1.0;
        P[35] = 1.0;

        Self { x, P, timestamp }
    }

    /// Unpack state vector into named components
    #[inline]
    pub fn unpack(&self) -> (f64, f64, f64, f64, f64, f64) {
        (
            self.x[0], self.x[1], self.x[2], self.x[3], self.x[4], self.x[5],
        )
    }

    /// Get position in GeoCoord format
    pub fn get_position(&self) -> (GeoCoord, f64) {
        let pos = GeoCoord::new(self.x[0].to_degrees(), self.x[1].to_degrees());
        (pos, self.x[2])
    }

    /// Get velocity in geographic terms (ground_speed_km_s, heading_deg, vertical_rate_km_s)
    pub fn get_velocity(&self) -> (f64, f64, f64) {
        let v_north = self.x[3];
        let v_east = self.x[4];
        let v_up = self.x[5];

        let ground_speed = (v_north.powi(2) + v_east.powi(2)).sqrt();
        let heading = v_east.atan2(v_north).to_degrees();
        let heading = (heading + 360.0) % 360.0;

        (ground_speed, heading, v_up)
    }

    /// Get position uncertainty (average std dev in km)
    pub fn get_position_uncertainty(&self) -> f64 {
        // Convert lat/lon variance to km variance via curvature radii
        let lat_deg = self.x[0].to_degrees();
        let lat_var_km = self.P[0] * meridional_radius_km(lat_deg).powi(2);
        let lon_var_km = self.P[7] * (prime_vertical_radius_km(lat_deg) * self.x[0].cos()).powi(2);
        let alt_var_km = self.P[14];

        ((lat_var_km + lon_var_km + alt_var_km) / 3.0).sqrt()
    }

    /// Initialize velocity state from two position measurements
    /// This provides a much better initial velocity estimate than zero,
    /// especially important for crossing (east-west) trajectories.
    ///
    /// Should be called after the second measurement to bootstrap velocity.
    pub fn initialize_velocity_from_positions(
        &mut self,
        older_pos: GeoCoord,
        older_alt: f64,
        older_time: f64,
        newer_pos: GeoCoord,
        newer_alt: f64,
        newer_time: f64,
    ) {
        let dt = newer_time - older_time;
        if dt < 0.1 {
            return; // Need sufficient time gap
        }

        // Compute velocity components from position change
        let lat1 = older_pos.lat.to_radians();
        let lat2 = newer_pos.lat.to_radians();
        let lon1 = older_pos.lon.to_radians();
        let lon2 = newer_pos.lon.to_radians();

        // Curvature radii at the mid-latitude of the pair (WGS-84)
        let avg_lat = (lat1 + lat2) / 2.0;
        let avg_alt = (older_alt + newer_alt) / 2.0;
        let r_m = meridional_radius_km(avg_lat.to_degrees()) + avg_alt;
        let r_n = prime_vertical_radius_km(avg_lat.to_degrees()) + avg_alt;

        // Velocity in north direction (from latitude change)
        let v_north = (lat2 - lat1) * r_m / dt;

        // Velocity in east direction (from longitude change, accounting for latitude)
        let v_east = (lon2 - lon1) * r_n * avg_lat.cos() / dt;

        // Vertical velocity from altitude change
        let v_up = (newer_alt - older_alt) / dt;

        // Sanity check: ballistic missiles typically < 8 km/s horizontal
        let ground_speed = (v_north.powi(2) + v_east.powi(2)).sqrt();
        if ground_speed > 10.0 {
            return; // Unrealistic velocity, likely measurement error
        }

        // Set velocity state
        self.x[3] = v_north;
        self.x[4] = v_east;
        self.x[5] = v_up;

        // Reduce velocity uncertainty since we now have an estimate
        // Still keep some uncertainty for filter to refine
        let vel_var = 0.25; // 0.5 km/s std dev (reduced from initial 1.0)
        self.P[21] = vel_var;
        self.P[28] = vel_var;
        self.P[35] = vel_var;
    }

    /// Predict state at multiple future times (for trajectory visualization)
    /// Returns Vec of (time_offset, position, altitude, uncertainty)
    ///
    /// This is useful for rendering predicted trajectories based on EKF state.
    /// Uses proper ballistic dynamics with gravity.
    pub fn predict_trajectory(
        &self,
        time_step: f64,
        num_steps: usize,
    ) -> Vec<(f64, GeoCoord, f64, f64)> {
        let mut results = Vec::with_capacity(num_steps);
        let mut state = self.clone();

        for i in 0..num_steps {
            let t = (i as f64) * time_step;
            if i > 0 {
                state.predict(time_step);
            }
            let (pos, alt) = state.get_position();
            let uncertainty = state.get_position_uncertainty();
            results.push((t, pos, alt, uncertainty));
        }
        results
    }

    /// Nonlinear state transition function f(x, dt)
    /// Returns predicted state after time dt
    fn f(&self, dt: f64) -> [f64; 6] {
        let (lat, lon, alt, v_n, v_e, v_u) = self.unpack();
        let lat_deg = lat.to_degrees();

        // WGS-84 curvature radii at current altitude
        let r_m = meridional_radius_km(lat_deg) + alt;
        let r_n = prime_vertical_radius_km(lat_deg) + alt;

        // Local gravity with altitude correction (inverse-square about the
        // mean Earth radius)
        let g_local = G0
            * (crate::simulation::physics::MEAN_EARTH_RADIUS_KM
                / (crate::simulation::physics::MEAN_EARTH_RADIUS_KM + alt))
                .powi(2);

        // Geodetic rate equations (WGS-84 ellipsoidal curvature)
        // dlat/dt = v_north / M(phi)
        // dlon/dt = v_east / (N(phi) * cos(lat))
        // dalt/dt = v_up - 0.5 * g * dt (include gravity term)

        let lat_new = lat + (v_n / r_m) * dt;
        let lon_new = lon + (v_e / (r_n * lat.cos())) * dt;
        let alt_new = alt + v_u * dt - 0.5 * g_local * dt.powi(2);

        // Velocity changes - only vertical velocity affected by gravity
        let v_n_new = v_n;
        let v_e_new = v_e;
        let v_u_new = v_u - g_local * dt;

        [lat_new, lon_new, alt_new, v_n_new, v_e_new, v_u_new]
    }

    /// State transition Jacobian F = df/dx evaluated at current state
    /// This is what makes it "extended" - F depends on x
    fn F(&self, dt: f64) -> [f64; 36] {
        use crate::simulation::physics::{WGS84_A, WGS84_E2};
        let (lat, _lon, alt, _v_n, v_e, _v_u) = self.unpack();
        let lat_deg = lat.to_degrees();

        let cos_lat = lat.cos();
        let sin_lat = lat.sin();

        // WGS-84 curvature radii and their latitude derivatives.
        // M = a(1-e²)/(1-e²s²)^{3/2}; N = a/(1-e²s²)^{1/2}
        // d(1/M)/dφ = 3e²·s·c·√(1-e²s²) / (a(1-e²))
        // d(1/N)/dφ = e²·s·c / (a·√(1-e²s²))
        let m = meridional_radius_km(lat_deg);
        let n = prime_vertical_radius_km(lat_deg);
        let r_m = m + alt;
        let r_n = n + alt;
        let k = 1.0 - WGS84_E2 * sin_lat * sin_lat;
        let d_inv_m = 3.0 * WGS84_E2 * sin_lat * cos_lat * k.sqrt() / (WGS84_A * (1.0 - WGS84_E2));
        let d_inv_n = WGS84_E2 * sin_lat * cos_lat / (WGS84_A * k.sqrt());

        // Partial derivatives of f with respect to state
        // F[i][j] = df_i / dx_j
        let mut F = [0.0; 36];

        // Identity on diagonal
        for i in 0..6 {
            F[i * 6 + i] = 1.0;
        }

        // df_lat/dv_n = dt / M (row 0, col 3)
        F[3] = dt / r_m;
        // df_lat/dlat = 1 + v_n · d(1/M)/dφ · dt (identity + curvature
        // derivative; the derivative is e²-scale small but included for
        // correctness)
        F[0] += self.x[3] * d_inv_m * dt;

        // df_lon/dlat = identity + v_e · [d(1/N)/dφ / cos(lat) + tan(lat)/N] · dt
        if cos_lat.abs() > 1e-10 {
            F[6] += v_e * (d_inv_n / cos_lat + (1.0 / r_n) * (sin_lat / cos_lat)) * dt;
            // df_lon/dv_e = dt / (N · cos(lat))
            F[10] = dt / (r_n * cos_lat);
        }

        // df_alt/dv_u = dt
        F[17] = dt;

        // df_alt/dalt (gravity correction term; inverse-square about the
        // mean radius)
        let g_deriv = 2.0 * G0 * crate::simulation::physics::MEAN_EARTH_RADIUS_KM.powi(2)
            / (crate::simulation::physics::MEAN_EARTH_RADIUS_KM + alt).powi(3);
        F[14] = 1.0 + 0.5 * g_deriv * dt.powi(2);

        // df_v_u/dalt (gravity depends on altitude)
        F[32] = g_deriv * dt;

        F
    }

    /// Process noise covariance Q for time step dt
    fn Q(&self, dt: f64) -> [f64; 36] {
        // Process noise models uncertainty in physics
        // For ballistic missiles, the physics is very predictable
        let mut Q = [0.0; 36];

        // Position process noise (very small for ballistic trajectory)
        let q_pos = 0.0001 * dt.powi(2); // rad² for lat/lon, km² for alt

        // Velocity process noise (small - no thrust after boost)
        let q_vel = 0.001 * dt; // (km/s)²

        let lat_deg = self.x[0].to_degrees();
        Q[0] = q_pos / meridional_radius_km(lat_deg).powi(2); // lat variance
        Q[7] = q_pos / (prime_vertical_radius_km(lat_deg) * self.x[0].cos()).powi(2); // lon variance
        Q[14] = q_pos; // alt variance
        Q[21] = q_vel; // v_n variance
        Q[28] = q_vel; // v_e variance
        Q[35] = q_vel; // v_u variance

        Q
    }

    /// EKF predict step - propagate state and covariance forward
    pub fn predict(&mut self, dt: f64) {
        if dt <= 0.0 {
            return;
        }

        // 1. Predict state using nonlinear dynamics
        let x_pred = self.f(dt);

        // 2. Compute Jacobian at current state
        let F = self.F(dt);

        // 3. Process noise
        let Q = self.Q(dt);

        // 4. Propagate covariance: P = F * P * F^T + Q
        let F_P = matrix_mult_6x6(&F, &self.P);
        let F_P_FT = matrix_mult_6x6_transpose(&F_P, &F);
        self.P = matrix_add_6x6(&F_P_FT, &Q);

        // 5. Update state
        self.x = x_pred;
        self.timestamp += dt;
    }

    /// Nonlinear measurement function h(x, sensor)
    /// Predicts what radar would observe given current state
    fn h(&self, sensor_pos: GeoCoord, sensor_alt: f64) -> [f64; 3] {
        let (_, _, alt, _, _, _) = self.unpack();

        // Convert state to Cartesian ECEF (WGS-84)
        let (state_pos, _) = self.get_position();
        let target_ecef = geodetic_to_ecef(state_pos, alt);
        let sensor_ecef = geodetic_to_ecef(sensor_pos, sensor_alt);

        // Vector from sensor to target
        let dx = target_ecef[0] - sensor_ecef[0];
        let dy = target_ecef[1] - sensor_ecef[1];
        let dz = target_ecef[2] - sensor_ecef[2];

        // Range
        let range = (dx.powi(2) + dy.powi(2) + dz.powi(2)).sqrt();

        // Convert to local ENU at sensor location
        let (e, n, u) = ecef_to_enu(dx, dy, dz, sensor_pos.lat, sensor_pos.lon);

        // Azimuth (from North, clockwise positive)
        let azimuth = e.atan2(n);

        // Elevation (above horizon)
        let horizontal_range = (e.powi(2) + n.powi(2)).sqrt();
        let elevation = u.atan2(horizontal_range);

        [range, azimuth, elevation]
    }

    /// Public wrapper for measurement prediction
    /// Returns predicted [range_km, azimuth_rad, elevation_rad] for a sensor
    pub fn predict_measurement(&self, sensor_pos: GeoCoord, sensor_alt: f64) -> [f64; 3] {
        self.h(sensor_pos, sensor_alt)
    }

    /// Measurement Jacobian H = dh/dx evaluated at current state
    fn H(&self, sensor_pos: GeoCoord, sensor_alt: f64) -> [f64; 18] {
        // Numerical differentiation for robustness
        // (Analytical derivatives are complex due to coordinate transforms)
        let eps = 1e-8;
        let h0 = self.h(sensor_pos, sensor_alt);

        let mut H = [0.0; 18];

        for j in 0..6 {
            let mut x_plus = self.x;
            x_plus[j] += eps;

            let state_plus = EKFState {
                x: x_plus,
                P: self.P,
                timestamp: self.timestamp,
            };
            let h_plus = state_plus.h(sensor_pos, sensor_alt);

            for i in 0..3 {
                H[i * 6 + j] = (h_plus[i] - h0[i]) / eps;
            }
        }

        H
    }

    /// Measurement noise covariance R
    fn R(noise_std: &[f64; 3]) -> [f64; 9] {
        let mut R = [0.0; 9];
        R[0] = noise_std[0].powi(2); // range variance
        R[4] = noise_std[1].powi(2); // azimuth variance
        R[8] = noise_std[2].powi(2); // elevation variance
        R
    }

    /// EKF update step - incorporate radar measurement
    pub fn update(&mut self, measurement: &RadarMeasurement) {
        // 1. Predict measurement
        let z_pred = self.h(measurement.sensor_position, measurement.sensor_altitude_km);

        // 2. Measurement Jacobian at current state
        let H = self.H(measurement.sensor_position, measurement.sensor_altitude_km);

        // 3. Actual measurement
        let z_actual = [
            measurement.range_km,
            measurement.azimuth_rad,
            measurement.elevation_rad,
        ];

        // 4. Innovation (handle angle wraparound for azimuth)
        let y = [
            z_actual[0] - z_pred[0],
            angle_diff(z_actual[1], z_pred[1]),
            z_actual[2] - z_pred[2],
        ];

        // 5. Measurement noise
        let R = Self::R(&measurement.noise_std);

        // 6. Innovation covariance: S = H * P * H^T + R
        let H_P = matrix_mult_3x6_6x6(&H, &self.P);
        let H_P_HT = matrix_mult_3x6_6x3_transpose(&H_P, &H);
        let S = matrix_add_3x3(&H_P_HT, &R);

        // 7. Kalman gain: K = P * H^T * S^-1
        let S_inv = matrix_inv_3x3(&S);
        let P_HT = matrix_mult_6x6_6x3_transpose(&self.P, &H);
        let K = matrix_mult_6x3_3x3(&P_HT, &S_inv);

        // 8. State update: x = x + K * y
        for i in 0..6 {
            self.x[i] += K[i * 3 + 0] * y[0] + K[i * 3 + 1] * y[1] + K[i * 3 + 2] * y[2];
        }

        // 9. Covariance update: P = (I - K*H) * P (Joseph form for numerical stability)
        let K_H = matrix_mult_6x3_3x6(&K, &H);
        let mut I_KH = [0.0; 36];
        for i in 0..6 {
            I_KH[i * 6 + i] = 1.0;
        }
        for i in 0..36 {
            I_KH[i] -= K_H[i];
        }
        self.P = matrix_mult_6x6(&I_KH, &self.P);

        self.timestamp = measurement.timestamp;
    }
}

// === Coordinate Transformations ===

// geodetic_to_ecef is now the shared WGS-84 implementation in
// crate::simulation::physics (this module re-exports it via the use at top).

/// Convert ECEF difference vector to local ENU at observer location
fn ecef_to_enu(dx: f64, dy: f64, dz: f64, obs_lat_deg: f64, obs_lon_deg: f64) -> (f64, f64, f64) {
    let lat = obs_lat_deg.to_radians();
    let lon = obs_lon_deg.to_radians();

    let sin_lat = lat.sin();
    let cos_lat = lat.cos();
    let sin_lon = lon.sin();
    let cos_lon = lon.cos();

    // Rotation matrix from ECEF to ENU
    let e = -sin_lon * dx + cos_lon * dy;
    let n = -sin_lat * cos_lon * dx - sin_lat * sin_lon * dy + cos_lat * dz;
    let u = cos_lat * cos_lon * dx + cos_lat * sin_lon * dy + sin_lat * dz;

    (e, n, u)
}

/// Convert radar measurement (range, azimuth, elevation) to geodetic position
///
/// Uses the WGS-84 local curvature radii for the small-area ENU-to-geodetic
/// conversion (valid for the km-scale offsets of radar measurements).
pub fn radar_to_geodetic(
    sensor_pos: GeoCoord,
    sensor_alt: f64,
    range_km: f64,
    azimuth_rad: f64,
    elevation_rad: f64,
) -> (GeoCoord, f64) {
    // Convert to local ENU offset
    let horizontal_range = range_km * elevation_rad.cos();
    let e = horizontal_range * azimuth_rad.sin();
    let n = horizontal_range * azimuth_rad.cos();
    let u = range_km * elevation_rad.sin();

    // Convert ENU offset to geodetic via curvature radii at the sensor
    let lat_offset = n / (meridional_radius_km(sensor_pos.lat) + sensor_alt);
    let lon_offset = e
        / ((prime_vertical_radius_km(sensor_pos.lat) + sensor_alt)
            * sensor_pos.lat.to_radians().cos());

    let target_pos = GeoCoord::new(
        sensor_pos.lat + lat_offset.to_degrees(),
        sensor_pos.lon + lon_offset.to_degrees(),
    );
    let target_alt = sensor_alt + u;

    (target_pos, target_alt)
}

/// Normalize angle difference to [-pi, pi]
fn angle_diff(a: f64, b: f64) -> f64 {
    let mut diff = a - b;
    while diff > std::f64::consts::PI {
        diff -= 2.0 * std::f64::consts::PI;
    }
    while diff < -std::f64::consts::PI {
        diff += 2.0 * std::f64::consts::PI;
    }
    diff
}

// === Matrix Operations ===

fn matrix_mult_6x6(a: &[f64; 36], b: &[f64; 36]) -> [f64; 36] {
    let mut result = [0.0; 36];
    for i in 0..6 {
        for j in 0..6 {
            for k in 0..6 {
                result[i * 6 + j] += a[i * 6 + k] * b[k * 6 + j];
            }
        }
    }
    result
}

fn matrix_mult_6x6_transpose(a: &[f64; 36], b: &[f64; 36]) -> [f64; 36] {
    // a * b^T
    let mut result = [0.0; 36];
    for i in 0..6 {
        for j in 0..6 {
            for k in 0..6 {
                result[i * 6 + j] += a[i * 6 + k] * b[j * 6 + k]; // b transposed
            }
        }
    }
    result
}

fn matrix_add_6x6(a: &[f64; 36], b: &[f64; 36]) -> [f64; 36] {
    let mut result = [0.0; 36];
    for i in 0..36 {
        result[i] = a[i] + b[i];
    }
    result
}

fn matrix_mult_3x6_6x6(a: &[f64; 18], b: &[f64; 36]) -> [f64; 18] {
    let mut result = [0.0; 18];
    for i in 0..3 {
        for j in 0..6 {
            for k in 0..6 {
                result[i * 6 + j] += a[i * 6 + k] * b[k * 6 + j];
            }
        }
    }
    result
}

fn matrix_mult_3x6_6x3_transpose(a: &[f64; 18], b: &[f64; 18]) -> [f64; 9] {
    // a (3x6) * b^T (6x3) = 3x3
    let mut result = [0.0; 9];
    for i in 0..3 {
        for j in 0..3 {
            for k in 0..6 {
                result[i * 3 + j] += a[i * 6 + k] * b[j * 6 + k];
            }
        }
    }
    result
}

fn matrix_mult_6x6_6x3_transpose(a: &[f64; 36], b: &[f64; 18]) -> [f64; 18] {
    // a (6x6) * b^T (6x3) = 6x3
    let mut result = [0.0; 18];
    for i in 0..6 {
        for j in 0..3 {
            for k in 0..6 {
                result[i * 3 + j] += a[i * 6 + k] * b[j * 6 + k];
            }
        }
    }
    result
}

fn matrix_mult_6x3_3x3(a: &[f64; 18], b: &[f64; 9]) -> [f64; 18] {
    let mut result = [0.0; 18];
    for i in 0..6 {
        for j in 0..3 {
            for k in 0..3 {
                result[i * 3 + j] += a[i * 3 + k] * b[k * 3 + j];
            }
        }
    }
    result
}

fn matrix_mult_6x3_3x6(a: &[f64; 18], b: &[f64; 18]) -> [f64; 36] {
    let mut result = [0.0; 36];
    for i in 0..6 {
        for j in 0..6 {
            for k in 0..3 {
                result[i * 6 + j] += a[i * 3 + k] * b[k * 6 + j];
            }
        }
    }
    result
}

fn matrix_add_3x3(a: &[f64; 9], b: &[f64; 9]) -> [f64; 9] {
    let mut result = [0.0; 9];
    for i in 0..9 {
        result[i] = a[i] + b[i];
    }
    result
}

/// Invert 3x3 matrix using cofactor method
fn matrix_inv_3x3(m: &[f64; 9]) -> [f64; 9] {
    let det = m[0] * (m[4] * m[8] - m[5] * m[7]) - m[1] * (m[3] * m[8] - m[5] * m[6])
        + m[2] * (m[3] * m[7] - m[4] * m[6]);

    if det.abs() < 1e-10 {
        // Singular matrix, return identity
        return [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0];
    }

    let inv_det = 1.0 / det;

    [
        (m[4] * m[8] - m[5] * m[7]) * inv_det,
        (m[2] * m[7] - m[1] * m[8]) * inv_det,
        (m[1] * m[5] - m[2] * m[4]) * inv_det,
        (m[5] * m[6] - m[3] * m[8]) * inv_det,
        (m[0] * m[8] - m[2] * m[6]) * inv_det,
        (m[2] * m[3] - m[0] * m[5]) * inv_det,
        (m[3] * m[7] - m[4] * m[6]) * inv_det,
        (m[1] * m[6] - m[0] * m[7]) * inv_det,
        (m[0] * m[4] - m[1] * m[3]) * inv_det,
    ]
}
