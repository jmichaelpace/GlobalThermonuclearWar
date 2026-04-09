/// Kalman filter for ballistic trajectory estimation
///
/// Implements an Extended Kalman Filter (EKF) to estimate missile state from noisy sensor measurements.
/// This is the approach used by real missile defense systems.

use crate::map::GeoCoord;
use crate::simulation::detection::PositionMeasurement;

/// State vector for ballistic trajectory tracking
/// Uses East-North-Up (ENU) local coordinates relative to first detection
#[derive(Clone, Debug)]
pub struct BallisticState {
    /// Position in ENU coordinates (km): [east, north, up]
    pub position: [f64; 3],
    /// Velocity in ENU coordinates (km/s): [v_east, v_north, v_up]
    pub velocity: [f64; 3],
    /// State covariance matrix (6x6, flattened row-major)
    /// Upper-left 3x3: position covariance
    /// Lower-right 3x3: velocity covariance
    pub covariance: [f64; 36],
    /// Reference point for ENU coordinate system
    pub reference: GeoCoord,
    /// Last update timestamp
    pub timestamp: f64,
}

impl BallisticState {
    /// Initialize state from first measurement
    pub fn new(measurement: &PositionMeasurement) -> Self {
        // Initial position uncertainty: 2km horizontal, 1km vertical (conservative start)
        let pos_var_horiz = 4.0;  // Variance = std_dev²
        let pos_var_vert = 1.0;

        // Initial velocity uncertainty: 1 km/s (completely unknown)
        let vel_var = 1.0;

        // Initialize covariance as diagonal matrix
        let mut covariance = [0.0; 36];
        covariance[0] = pos_var_horiz;  // var(east)
        covariance[7] = pos_var_horiz;  // var(north)
        covariance[14] = pos_var_vert;  // var(up)
        covariance[21] = vel_var;       // var(v_east)
        covariance[28] = vel_var;       // var(v_north)
        covariance[35] = vel_var;       // var(v_up)

        Self {
            position: [0.0, 0.0, measurement.altitude_km],
            velocity: [0.0, 0.0, 0.0],  // Unknown initially
            covariance,
            reference: measurement.position,
            timestamp: measurement.timestamp,
        }
    }

    /// Predict state forward in time using ballistic motion model
    pub fn predict(&mut self, dt: f64) {
        if dt <= 0.0 {
            return;
        }

        // Ballistic motion: constant horizontal velocity, vertical acceleration due to gravity
        const G: f64 = 0.00981; // km/s² (Earth gravity)

        // State transition: x_new = F * x_old
        // position += velocity * dt
        // velocity_horizontal += 0 (no horizontal forces in ballistic flight)
        // velocity_vertical += -g * dt (gravity)

        self.position[0] += self.velocity[0] * dt;  // east
        self.position[1] += self.velocity[1] * dt;  // north
        self.position[2] += self.velocity[2] * dt - 0.5 * G * dt * dt;  // up (with gravity)

        self.velocity[2] -= G * dt;  // vertical velocity decreases due to gravity

        // State transition matrix F (6x6)
        let mut f = [0.0; 36];
        // Identity for positions (diagonal)
        f[0] = 1.0; f[7] = 1.0; f[14] = 1.0;
        // Velocity contribution to position
        f[3] = dt; f[10] = dt; f[17] = dt;
        // Identity for velocities (diagonal)
        f[21] = 1.0; f[28] = 1.0; f[35] = 1.0;

        // Process noise covariance Q
        // Models uncertainty in the physics model
        // For ballistic missiles, physics is highly predictable - use small process noise
        let mut q = [0.0; 36];
        let q_pos = 0.01 * dt * dt;  // Small position process noise (ballistic physics is predictable)
        let q_vel = 0.001 * dt;       // Very small velocity process noise (no thrust after boost)

        q[0] = q_pos; q[7] = q_pos; q[14] = q_pos;
        q[21] = q_vel; q[28] = q_vel; q[35] = q_vel;

        // Update covariance: P = F * P * F^T + Q
        let p_old = self.covariance;
        self.covariance = matrix_add_6x6(
            &matrix_mult_6x6(&matrix_mult_6x6(&f, &p_old), &matrix_transpose_6x6(&f)),
            &q
        );

        self.timestamp += dt;
    }

    /// Update state with new measurement (Kalman update step)
    pub fn update(&mut self, measurement: &PositionMeasurement) {
        // Convert measurement to ENU coordinates
        let meas_enu = geo_to_enu(measurement.position, measurement.altitude_km, self.reference);

        // Measurement vector z = [east, north, up]
        let z = meas_enu;

        // Predicted measurement: H * x (we measure position directly)
        let h_x = self.position;

        // Innovation (measurement residual): y = z - H*x
        let y = [
            z[0] - h_x[0],
            z[1] - h_x[1],
            z[2] - h_x[2],
        ];

        // Measurement noise covariance R (3x3)
        // Depends on measurement quality
        let quality = measurement.measurement_quality;
        let r_horiz = (2.0 / quality).max(0.5);  // 0.5-2km horizontal uncertainty
        let r_vert = (1.0 / quality).max(0.25);   // 0.25-1km vertical uncertainty

        let mut r = [0.0; 9];
        r[0] = r_horiz; r[4] = r_horiz; r[8] = r_vert;

        // Measurement matrix H (3x6) - we observe position only
        let mut h = [0.0; 18];
        h[0] = 1.0; h[7] = 1.0; h[14] = 1.0;  // First 3 columns are identity

        // Innovation covariance: S = H * P * H^T + R
        let p = self.covariance;
        let h_p = matrix_mult_3x6_6x6(&h, &p);
        let h_p_ht = matrix_mult_3x6_6x3(&h_p, &matrix_transpose_3x6(&h));
        let s = matrix_add_3x3(&h_p_ht, &r);

        // Kalman gain: K = P * H^T * S^-1
        let s_inv = matrix_inv_3x3(&s);
        let p_ht = matrix_mult_6x6_6x3(&p, &matrix_transpose_3x6(&h));
        let k = matrix_mult_6x3_3x3(&p_ht, &s_inv);

        // State update: x = x + K * y
        self.position[0] += k[0] * y[0] + k[1] * y[1] + k[2] * y[2];
        self.position[1] += k[6] * y[0] + k[7] * y[1] + k[8] * y[2];
        self.position[2] += k[12] * y[0] + k[13] * y[1] + k[14] * y[2];

        self.velocity[0] += k[3] * y[0] + k[4] * y[1] + k[5] * y[2];
        self.velocity[1] += k[9] * y[0] + k[10] * y[1] + k[11] * y[2];
        self.velocity[2] += k[15] * y[0] + k[16] * y[1] + k[17] * y[2];

        // Covariance update: P = (I - K*H) * P
        let mut i_kh = [0.0; 36];
        // Start with identity
        for i in 0..6 {
            i_kh[i * 6 + i] = 1.0;
        }
        // Subtract K*H
        let kh = matrix_mult_6x3_3x6(&k, &h);
        for i in 0..36 {
            i_kh[i] -= kh[i];
        }

        self.covariance = matrix_mult_6x6(&i_kh, &p);
        self.timestamp = measurement.timestamp;
    }

    /// Get current position in geographic coordinates
    pub fn get_position(&self) -> (GeoCoord, f64) {
        enu_to_geo(self.position, self.reference)
    }

    /// Get velocity magnitude and direction
    pub fn get_velocity(&self) -> (f64, f64, f64) {
        let v_horiz = (self.velocity[0] * self.velocity[0] + self.velocity[1] * self.velocity[1]).sqrt();
        let heading = self.velocity[1].atan2(self.velocity[0]).to_degrees();
        let heading = (90.0 - heading + 360.0) % 360.0;  // Convert from math to geographic (0=North)
        (v_horiz, heading, self.velocity[2])
    }

    /// Get position uncertainty (standard deviation in km)
    pub fn get_position_uncertainty(&self) -> f64 {
        // Return sqrt of average position variance
        let var_avg = (self.covariance[0] + self.covariance[7] + self.covariance[14]) / 3.0;
        var_avg.sqrt()
    }

    /// Get velocity uncertainty (standard deviation in km/s)
    pub fn get_velocity_uncertainty(&self) -> f64 {
        let var_avg = (self.covariance[21] + self.covariance[28] + self.covariance[35]) / 3.0;
        var_avg.sqrt()
    }

    /// Get 3D position uncertainty from covariance diagonal
    /// Returns (east_uncertainty_km, north_uncertainty_km, up_uncertainty_km)
    pub fn get_3d_position_uncertainty(&self) -> (f64, f64, f64) {
        (
            self.covariance[0].sqrt(),   // east variance -> std dev
            self.covariance[7].sqrt(),   // north variance -> std dev
            self.covariance[14].sqrt(),  // up variance -> std dev
        )
    }

    /// Get 3D velocity uncertainty from covariance diagonal
    /// Returns (v_east_uncertainty, v_north_uncertainty, v_up_uncertainty) in km/s
    pub fn get_3d_velocity_uncertainty(&self) -> (f64, f64, f64) {
        (
            self.covariance[21].sqrt(),  // v_east variance -> std dev
            self.covariance[28].sqrt(),  // v_north variance -> std dev
            self.covariance[35].sqrt(),  // v_up variance -> std dev
        )
    }

    /// Predict position and uncertainty at a future time without modifying state
    /// Returns (position: GeoCoord, altitude_km: f64, position_uncertainty_km: f64)
    ///
    /// This is useful for intercept calculations where we need to predict
    /// where the target will be at a future time.
    pub fn predict_at_time(&self, dt: f64) -> (GeoCoord, f64, f64) {
        if dt <= 0.0 {
            let (pos, alt) = self.get_position();
            return (pos, alt, self.get_position_uncertainty());
        }

        // Clone state and predict forward
        let mut predicted = self.clone();
        predicted.predict(dt);

        let (pos, alt) = predicted.get_position();
        let uncertainty = predicted.get_position_uncertainty();

        (pos, alt, uncertainty)
    }

    /// Predict state at multiple future times (for trajectory visualization)
    /// Returns Vec of (time_offset, position, altitude, uncertainty)
    pub fn predict_trajectory(&self, time_step: f64, num_steps: usize) -> Vec<(f64, GeoCoord, f64, f64)> {
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

    /// Get velocity in geographic terms (ground_speed_km_s, heading_deg, vertical_rate_km_s)
    /// with uncertainties
    pub fn get_velocity_with_uncertainty(&self) -> ((f64, f64, f64), (f64, f64, f64)) {
        let velocity = self.get_velocity();
        let uncertainty = self.get_3d_velocity_uncertainty();
        (velocity, uncertainty)
    }
}

// === Coordinate Transformations ===

/// Convert geographic coordinates to local East-North-Up (ENU) frame
fn geo_to_enu(point: GeoCoord, altitude_km: f64, reference: GeoCoord) -> [f64; 3] {
    const EARTH_RADIUS_KM: f64 = 6371.0;

    let lat_diff = (point.lat - reference.lat).to_radians();
    let lon_diff = (point.lon - reference.lon).to_radians();
    let ref_lat = reference.lat.to_radians();

    // Approximate ENU for small distances
    let east = EARTH_RADIUS_KM * lon_diff * ref_lat.cos();
    let north = EARTH_RADIUS_KM * lat_diff;
    let up = altitude_km;

    [east, north, up]
}

/// Convert local ENU coordinates back to geographic
fn enu_to_geo(enu: [f64; 3], reference: GeoCoord) -> (GeoCoord, f64) {
    const EARTH_RADIUS_KM: f64 = 6371.0;

    let ref_lat = reference.lat.to_radians();

    let lat_diff = (enu[1] / EARTH_RADIUS_KM).to_degrees();
    let lon_diff = (enu[0] / (EARTH_RADIUS_KM * ref_lat.cos())).to_degrees();

    let position = GeoCoord::new(
        reference.lat + lat_diff,
        reference.lon + lon_diff,
    );

    (position, enu[2])
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

fn matrix_transpose_6x6(a: &[f64; 36]) -> [f64; 36] {
    let mut result = [0.0; 36];
    for i in 0..6 {
        for j in 0..6 {
            result[i * 6 + j] = a[j * 6 + i];
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

fn matrix_mult_3x6_6x3(a: &[f64; 18], b: &[f64; 18]) -> [f64; 9] {
    let mut result = [0.0; 9];
    for i in 0..3 {
        for j in 0..3 {
            for k in 0..6 {
                result[i * 3 + j] += a[i * 6 + k] * b[j * 6 + k];  // b is transposed 6x3, stored as 3x6
            }
        }
    }
    result
}

fn matrix_mult_6x6_6x3(a: &[f64; 36], b: &[f64; 18]) -> [f64; 18] {
    let mut result = [0.0; 18];
    for i in 0..6 {
        for j in 0..3 {
            for k in 0..6 {
                result[i * 3 + j] += a[i * 6 + k] * b[j * 6 + k];  // b is transposed, stored as 3x6
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

fn matrix_transpose_3x6(a: &[f64; 18]) -> [f64; 18] {
    let mut result = [0.0; 18];
    for i in 0..3 {
        for j in 0..6 {
            result[j * 3 + i] = a[i * 6 + j];
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
    // Calculate determinant
    let det = m[0] * (m[4] * m[8] - m[5] * m[7])
            - m[1] * (m[3] * m[8] - m[5] * m[6])
            + m[2] * (m[3] * m[7] - m[4] * m[6]);

    if det.abs() < 1e-10 {
        // Singular matrix, return identity
        return [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0];
    }

    let inv_det = 1.0 / det;

    // Calculate cofactor matrix and transpose
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
