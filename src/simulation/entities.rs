use crate::simulation::config::{MissileType, SensorRole};
use crate::simulation::physics::{
    atmospheric_density, bearing, integrate_drag_descent, BallisticCoefficient,
};
use crate::types::GeoCoord;
use serde::{Deserialize, Serialize};

/// Calculate bearing from one position to another (degrees, 0=North, 90=East)
fn bearing_deg(from: GeoCoord, to: GeoCoord) -> f64 {
    bearing(from, to)
}

/// Unique identifier for entities
pub type EntityId = u64;

/// Affiliation of an entity
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum Affiliation {
    Friendly,
    Hostile,
    Neutral,
}

/// Status of a missile
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum MissileStatus {
    PreLaunch,
    Boost,
    Midcourse,
    Terminal,
    Intercepted,
    Impacted,
}

/// Status of a defense unit
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum UnitStatus {
    Idle,
    Tracking,
    Engaged,
    Disabled,
}

/// Type of defense system
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum DefenseType {
    Patriot,
    THAAD,
    Aegis,
    GBI, // Ground-Based Interceptor
    S400,
    IronDome,
    DavidsSling, // Israeli mid-tier defense (between Iron Dome and Arrow)
    Arrow3,      // Israeli exo-atmospheric interceptor
}

impl DefenseType {
    /// Display name
    pub fn name(&self) -> &'static str {
        match self {
            DefenseType::Patriot => "Patriot",
            DefenseType::THAAD => "THAAD",
            DefenseType::Aegis => "Aegis BMD",
            DefenseType::GBI => "GBI",
            DefenseType::S400 => "S-400",
            DefenseType::IronDome => "Iron Dome",
            DefenseType::DavidsSling => "David's Sling",
            DefenseType::Arrow3 => "Arrow 3",
        }
    }
}

/// Sensor type for detection
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum SensorType {
    Radar,
    Infrared,
    Both,
}

/// A ballistic missile
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Missile {
    pub id: EntityId,
    pub name: String,
    pub affiliation: Affiliation,
    pub missile_type: MissileType,
    pub origin: GeoCoord,
    pub target: GeoCoord,
    pub position: GeoCoord,
    pub altitude_km: f64,
    pub status: MissileStatus,
    pub launch_time: f64,         // Simulation time when launched
    pub flight_time: f64,         // Total expected flight time in seconds
    pub current_flight_time: f64, // Time since launch
    /// Current velocity in km/s (affected by boost acceleration and atmospheric drag)
    pub current_velocity_km_s: f64,
    /// Whether this missile has decoys/countermeasures
    pub has_countermeasures: bool,
    /// Number of decoys deployed (reduces hit probability)
    pub decoys_deployed: u32,
    /// Maximum number of decoys
    pub max_decoys: u32,
    /// Radar cross-section during boost phase (dBsm)
    pub rcs_boost_dbsm: f64,
    /// Radar cross-section during midcourse phase (dBsm)
    pub rcs_midcourse_dbsm: f64,
    /// Radar cross-section during terminal phase (dBsm)
    pub rcs_terminal_dbsm: f64,
}

impl Missile {
    pub fn new(
        id: EntityId,
        name: String,
        affiliation: Affiliation,
        origin: GeoCoord,
        target: GeoCoord,
        flight_time: f64,
    ) -> Self {
        Self {
            id,
            name,
            affiliation,
            missile_type: MissileType::ICBM, // Default, will be overridden by engine
            origin,
            target,
            position: origin,
            altitude_km: 0.0,
            status: MissileStatus::PreLaunch,
            launch_time: 0.0,
            flight_time,
            current_flight_time: 0.0,
            current_velocity_km_s: 0.0,
            has_countermeasures: false,
            decoys_deployed: 0,
            max_decoys: 0,
            rcs_boost_dbsm: 5.0,      // Default: large due to exhaust plume
            rcs_midcourse_dbsm: -5.0, // Default: small RV in space
            rcs_terminal_dbsm: -10.0, // Default: smallest signature
        }
    }

    /// Create a missile with countermeasures capability
    pub fn with_countermeasures(mut self, max_decoys: u32) -> Self {
        self.has_countermeasures = true;
        self.max_decoys = max_decoys;
        self
    }

    /// Deploy a decoy if available (reduces hit probability of interceptors)
    pub fn deploy_decoy(&mut self) -> bool {
        if self.has_countermeasures && self.decoys_deployed < self.max_decoys {
            self.decoys_deployed += 1;
            true
        } else {
            false
        }
    }

    /// Get the hit probability reduction factor based on deployed decoys
    /// Each decoy reduces hit probability by ~15%
    pub fn decoy_effectiveness(&self) -> f64 {
        if self.decoys_deployed == 0 {
            1.0
        } else {
            // Each decoy reduces effectiveness multiplicatively
            // 1 decoy: 0.85, 2 decoys: 0.72, 3 decoys: 0.61, etc.
            0.85_f64.powi(self.decoys_deployed as i32)
        }
    }

    /// Get the progress through the flight (0.0 to 1.0)
    pub fn flight_progress(&self) -> f64 {
        if self.flight_time <= 0.0 {
            return 0.0;
        }
        (self.current_flight_time / self.flight_time).clamp(0.0, 1.0)
    }

    /// Get current radar cross-section based on flight phase (dBsm)
    pub fn current_rcs_dbsm(&self) -> f64 {
        match self.status {
            MissileStatus::Boost => self.rcs_boost_dbsm,
            MissileStatus::Midcourse => self.rcs_midcourse_dbsm,
            MissileStatus::Terminal => self.rcs_terminal_dbsm,
            _ => self.rcs_midcourse_dbsm, // Default for PreLaunch, Destroyed, Intercepted
        }
    }

    /// Get RCS with aspect-angle variation based on radar viewing angle
    ///
    /// RCS varies significantly with aspect angle for conical reentry vehicles:
    /// - Nose-on: Smaller RCS (pointed end toward radar)
    /// - Broadside: Larger RCS (full body visible)
    /// - Tail-on: Medium RCS
    ///
    /// # Arguments
    /// * `radar_position` - Position of the radar
    ///
    /// # Returns
    /// RCS in dBsm adjusted for aspect angle
    pub fn rcs_with_aspect(&self, radar_position: GeoCoord) -> f64 {
        let base_rcs = self.current_rcs_dbsm();

        // Calculate aspect angle: angle between radar LOS and missile velocity vector
        // Missile heading is from current position toward target
        let missile_heading = self.heading_deg();

        // Radar-to-missile bearing
        let radar_to_missile_bearing = bearing_deg(radar_position, self.position);

        // Aspect angle: difference between missile heading and radar bearing
        // 0° = nose-on (radar looking at missile front)
        // 90° = broadside
        // 180° = tail-on (radar looking at missile rear)
        let aspect_diff = (missile_heading - radar_to_missile_bearing).rem_euclid(360.0);
        let aspect_angle = if aspect_diff > 180.0 {
            360.0 - aspect_diff
        } else {
            aspect_diff
        };

        // RCS modifier based on aspect angle (typical for conical RV)
        // Reference: Skolnik, "Introduction to Radar Systems"
        let aspect_factor_db = if aspect_angle < 30.0 {
            -1.5 // Nose-on: -1.5 dB (smaller target)
        } else if aspect_angle < 60.0 {
            -0.7 // Forward quarter: -0.7 dB
        } else if aspect_angle < 120.0 {
            1.2 // Broadside: +1.2 dB (larger target)
        } else if aspect_angle < 150.0 {
            -0.5 // Rear quarter: -0.5 dB
        } else {
            -1.0 // Tail-on: -1.0 dB
        };

        base_rcs + aspect_factor_db
    }

    /// Get missile heading in degrees (0 = North, 90 = East)
    fn heading_deg(&self) -> f64 {
        bearing_deg(self.position, self.target)
    }

    /// Update velocity based on flight phase with altitude-based atmospheric drag
    /// - Boost: Accelerates from 0 to burnout velocity
    /// - Post-boost: Atmospheric drag based on current altitude affects velocity
    ///   - Above 100km (Karman line): No significant drag
    ///   - Below 100km: Drag increases exponentially as altitude decreases
    pub fn update_velocity(&mut self, range_km: f64) {
        let progress = self.flight_progress();

        // Estimate burnout velocity based on range (longer range = higher burnout velocity)
        // ICBMs: ~7 km/s, IRBMs: ~4-5 km/s, SRBMs: ~2-3 km/s
        let burnout_velocity_km_s = if range_km > 5500.0 {
            7.0 // ICBM
        } else if range_km > 3000.0 {
            5.5 // IRBM
        } else if range_km > 1000.0 {
            4.0 // MRBM
        } else {
            2.5 // SRBM
        };

        // Boost phase ends at ~15% of flight
        const BOOST_END: f64 = 0.15;

        // Calculate base velocity (before drag)
        let base_velocity = match self.status {
            MissileStatus::PreLaunch => 0.0,
            MissileStatus::Boost => {
                // Linear acceleration during boost
                let boost_progress = (progress / BOOST_END).clamp(0.0, 1.0);
                burnout_velocity_km_s * boost_progress
            }
            MissileStatus::Midcourse | MissileStatus::Terminal => burnout_velocity_km_s,
            MissileStatus::Intercepted | MissileStatus::Impacted => 0.0,
        };

        // Apply altitude-based atmospheric drag for any unboosted trajectory
        // Drag only applies after boost phase ends
        if self.status == MissileStatus::Midcourse || self.status == MissileStatus::Terminal {
            let drag_factor = Self::atmospheric_drag_factor(self.altitude_km, base_velocity);
            self.current_velocity_km_s = base_velocity * drag_factor;
        } else {
            self.current_velocity_km_s = base_velocity;
        }
    }

    /// Calculate velocity at current altitude accounting for atmospheric drag
    /// Uses 1976 US Standard Atmosphere model with physics-based drag integration
    ///
    /// # Arguments
    /// * `altitude_km` - Current altitude (km)
    /// * `burnout_velocity_km_s` - Velocity at end of boost phase (km/s)
    /// * `range_km` - Missile range for ballistic coefficient selection
    ///
    /// # Returns
    /// Factor between 0.35 (heavy drag at sea level) and 1.0 (no drag)
    ///
    /// # Physics
    /// Reference: 1976 US Standard Atmosphere (NASA-TM-X-74335)
    /// Integrates drag equation: dv/ds = -ρv/(2β) along descent path
    /// Real RV terminal velocities: 2-3 km/s from 7 km/s burnout (~35-40% retention)
    pub fn atmospheric_drag_factor(altitude_km: f64, burnout_velocity_km_s: f64) -> f64 {
        // Above 100km (Kármán line): no significant atmosphere
        if altitude_km >= 100.0 {
            return 1.0;
        }

        if burnout_velocity_km_s <= 0.0 {
            return 1.0;
        }

        // Select ballistic coefficient based on velocity (proxy for missile class)
        // Higher velocity = longer range = higher β (ICBM RVs are more streamlined)
        let ballistic_coef = if burnout_velocity_km_s > 6.0 {
            BallisticCoefficient::icbm_rv().value() // ~20,000 kg/m²
        } else if burnout_velocity_km_s > 4.0 {
            BallisticCoefficient::mrbm_rv().value() // ~6,667 kg/m²
        } else {
            BallisticCoefficient::srbm().value() // ~3,333 kg/m²
        };

        // Typical reentry angle for ballistic missiles: 20-30 degrees from horizontal
        let entry_angle_deg = 25.0;

        // Integrate drag from Kármán line (100km) to current altitude
        let velocity_at_alt = integrate_drag_descent(
            burnout_velocity_km_s,
            100.0, // Start integration at Kármán line
            altitude_km,
            ballistic_coef,
            entry_angle_deg,
        );

        // Return factor (ratio of current velocity to burnout velocity)
        (velocity_at_alt / burnout_velocity_km_s).clamp(0.35, 1.0)
    }

    /// Estimate burnout velocity based on missile range
    /// Longer range missiles require higher burnout velocities
    pub fn estimate_burnout_velocity(range_km: f64) -> f64 {
        if range_km > 5500.0 {
            7.0 // ICBM
        } else if range_km > 3000.0 {
            5.5 // IRBM
        } else if range_km > 1000.0 {
            4.0 // MRBM
        } else {
            2.5 // SRBM
        }
    }

    /// Predict velocity at a given altitude accounting for atmospheric drag
    /// Used for intercept solution calculations
    pub fn predict_velocity_at_altitude(range_km: f64, altitude_km: f64) -> f64 {
        let burnout_velocity = Self::estimate_burnout_velocity(range_km);
        let drag_factor = Self::atmospheric_drag_factor(altitude_km, burnout_velocity);
        burnout_velocity * drag_factor
    }
}

/// Normalize angle difference to [-180, 180]
pub fn normalize_angle_diff(angle: f64) -> f64 {
    let mut diff = angle % 360.0;
    if diff > 180.0 {
        diff -= 360.0;
    } else if diff < -180.0 {
        diff += 360.0;
    }
    diff
}

/// Runtime sensor instance on a defense unit
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DefenseUnitSensor {
    /// Unique identifier for this sensor instance
    pub sensor_id: EntityId,

    /// Reference to sensor configuration
    pub config_name: String,

    /// Role of this sensor
    pub role: SensorRole,

    /// Azimuth center direction in degrees
    pub azimuth_center_deg: f64,

    /// Effective azimuth coverage in degrees
    pub azimuth_coverage_deg: f64,
}

impl DefenseUnitSensor {
    /// Check if a bearing falls within this sensor's coverage
    pub fn is_bearing_in_coverage(&self, bearing_deg: f64) -> bool {
        let half_coverage = self.azimuth_coverage_deg / 2.0;
        let relative_bearing = normalize_angle_diff(bearing_deg - self.azimuth_center_deg);
        relative_bearing.abs() <= half_coverage
    }
}

/// A missile defense unit
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct DefenseUnit {
    pub id: EntityId,
    pub name: String,
    pub affiliation: Affiliation,
    pub position: GeoCoord,
    pub defense_type: DefenseType,
    pub status: UnitStatus,
    pub interceptors_remaining: u32,
    pub max_interceptors: u32,
    pub sensor_type: SensorType,
    /// Legacy single sensor field (for backward compatibility)
    /// Name of sensor configuration to use (maps to SensorConfigRegistry)
    pub sensor_config_name: String,
    /// NEW: Multi-sensor support
    #[serde(default)]
    pub sensors: Vec<DefenseUnitSensor>,
}

impl DefenseUnit {
    pub fn new(
        id: EntityId,
        name: String,
        affiliation: Affiliation,
        position: GeoCoord,
        defense_type: DefenseType,
        interceptors: u32,
    ) -> Self {
        // Default sensor config name based on defense type
        let sensor_config_name = defense_type.name().to_lowercase().replace(' ', "_");
        Self {
            id,
            name,
            affiliation,
            position,
            defense_type,
            status: UnitStatus::Idle,
            interceptors_remaining: interceptors,
            max_interceptors: interceptors,
            sensor_type: SensorType::Radar,
            sensor_config_name,
            sensors: vec![], // Multi-sensor support, populated by add_defense_unit()
        }
    }
}

/// A detection/tracking satellite
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Satellite {
    pub id: EntityId,
    pub name: String,
    pub affiliation: Affiliation,
    pub position: GeoCoord, // Sub-satellite point
    pub altitude_km: f64,
    pub sensor_type: SensorType,
    pub coverage_angle_deg: f64, // Half-angle of sensor cone
    pub orbital_period_hours: f64,
}

impl Satellite {
    pub fn new(
        id: EntityId,
        name: String,
        affiliation: Affiliation,
        position: GeoCoord,
        altitude_km: f64,
        sensor_type: SensorType,
    ) -> Self {
        Self {
            id,
            name,
            affiliation,
            position,
            altitude_km,
            sensor_type,
            coverage_angle_deg: 10.0,
            orbital_period_hours: 24.0, // Geostationary by default
        }
    }

    /// Calculate coverage radius on Earth's surface in km
    pub fn coverage_radius_km(&self) -> f64 {
        let earth_radius_km = 6371.0;
        let angle_rad = self.coverage_angle_deg.to_radians();
        // Simplified calculation
        (self.altitude_km + earth_radius_km) * angle_rad.tan()
    }
}

/// A ground-based radar station
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct RadarStation {
    pub id: EntityId,
    pub name: String,
    pub affiliation: Affiliation,
    pub position: GeoCoord,
    pub detection_range_km: f64,
    pub azimuth_coverage_deg: f64, // How wide the radar scans (360 for full coverage)
    pub facing_deg: f64,           // Direction the radar faces (0 = North, 90 = East)
    pub elevation_min_deg: f64,
    pub elevation_max_deg: f64,
    pub sensor_type: SensorType,
    /// Name of sensor configuration to use (maps to SensorConfigRegistry)
    pub sensor_config_name: String,
}

impl RadarStation {
    pub fn new(
        id: EntityId,
        name: String,
        affiliation: Affiliation,
        position: GeoCoord,
        detection_range_km: f64,
    ) -> Self {
        // Normalize name for config lookup
        let sensor_config_name = name.to_lowercase().replace([' ', '-'], "_");
        Self {
            id,
            name,
            affiliation,
            position,
            detection_range_km,
            azimuth_coverage_deg: 360.0,
            facing_deg: 0.0, // Default: facing North
            elevation_min_deg: 3.0,
            elevation_max_deg: 85.0,
            sensor_type: SensorType::Radar,
            sensor_config_name,
        }
    }

    /// Create a radar station facing a specific direction
    pub fn with_facing(mut self, facing_deg: f64) -> Self {
        self.facing_deg = facing_deg;
        self
    }

    /// Check if a bearing is within the radar's azimuth coverage
    pub fn is_bearing_in_coverage(&self, bearing_deg: f64) -> bool {
        if self.azimuth_coverage_deg >= 360.0 {
            return true;
        }

        let half_coverage = self.azimuth_coverage_deg / 2.0;
        let min_bearing = (self.facing_deg - half_coverage).rem_euclid(360.0);
        let max_bearing = (self.facing_deg + half_coverage).rem_euclid(360.0);

        if min_bearing <= max_bearing {
            bearing_deg >= min_bearing && bearing_deg <= max_bearing
        } else {
            // Coverage wraps around 0/360
            bearing_deg >= min_bearing || bearing_deg <= max_bearing
        }
    }
}

/// Status of an interceptor
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum InterceptorStatus {
    Pending, // Scheduled but not yet launched (for salvo delay)
    InFlight,
    Hit,          // Successfully intercepted target
    Miss,         // Failed to intercept
    SelfDestruct, // Lost track, self-destructed
}

/// Flight phase of an interceptor
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum InterceptorPhase {
    Boost,    // Initial acceleration phase (high thrust)
    Coast,    // Ballistic coast phase (no thrust, constant velocity)
    Terminal, // Terminal homing phase (maneuvering toward target)
}

/// Reason for intercept miss
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum MissReason {
    None,         // Not a miss (or miss reason not yet determined)
    PkRoll,       // Probability kill roll failed
    DebrisDamage, // Damaged by debris cloud
    OffCourse,    // Flew past intercept point without close approach
    SeekerLost,   // Seeker lost track of target
}

/// Kinematics profile for an interceptor type
#[derive(Clone, Copy, Debug)]
pub struct InterceptorKinematics {
    pub boost_duration_sec: f64,   // Duration of boost phase
    pub boost_acceleration_g: f64, // Acceleration during boost (in g's)
    pub max_velocity_km_s: f64,    // Maximum velocity achieved
    pub terminal_maneuver_g: f64,  // Max g's available for terminal maneuver
    pub burnout_altitude_km: f64,  // Typical altitude at end of boost
}

impl InterceptorKinematics {
    /// Get kinematics profile for a defense type
    pub fn for_defense_type(defense_type: DefenseType) -> Self {
        match defense_type {
            // GBI - three-stage solid rocket, reaches ~8 km/s
            DefenseType::GBI => Self {
                boost_duration_sec: 170.0, // ~3 minute boost
                boost_acceleration_g: 5.0,
                max_velocity_km_s: 8.0,
                terminal_maneuver_g: 20.0, // EKV can maneuver hard
                burnout_altitude_km: 200.0,
            },
            // SM-3 Block IIA - three-stage, ~4.5 km/s
            DefenseType::Aegis => Self {
                boost_duration_sec: 30.0,
                boost_acceleration_g: 15.0,
                max_velocity_km_s: 4.5,
                terminal_maneuver_g: 25.0,
                burnout_altitude_km: 100.0,
            },
            // Arrow 3 - two-stage, exoatmospheric
            DefenseType::Arrow3 => Self {
                boost_duration_sec: 25.0,
                boost_acceleration_g: 12.0,
                max_velocity_km_s: 2.5,
                terminal_maneuver_g: 20.0,
                burnout_altitude_km: 50.0,
            },
            // THAAD - single stage solid rocket
            DefenseType::THAAD => Self {
                boost_duration_sec: 12.0,
                boost_acceleration_g: 20.0,
                max_velocity_km_s: 2.8,
                terminal_maneuver_g: 30.0,
                burnout_altitude_km: 40.0,
            },
            // PAC-3 MSE - single stage, high maneuverability
            DefenseType::Patriot => Self {
                boost_duration_sec: 8.0,
                boost_acceleration_g: 25.0,
                max_velocity_km_s: 1.7,
                terminal_maneuver_g: 50.0, // Very agile
                burnout_altitude_km: 15.0,
            },
            // David's Sling Stunner - two-stage
            DefenseType::DavidsSling => Self {
                boost_duration_sec: 10.0,
                boost_acceleration_g: 15.0,
                max_velocity_km_s: 2.0,
                terminal_maneuver_g: 40.0,
                burnout_altitude_km: 20.0,
            },
            // S-400 40N6 missile
            DefenseType::S400 => Self {
                boost_duration_sec: 15.0,
                boost_acceleration_g: 18.0,
                // 2.1 km/s per config/interceptors/40n6.toml (published data)
                max_velocity_km_s: 2.1,
                terminal_maneuver_g: 25.0,
                burnout_altitude_km: 30.0,
            },
            // Iron Dome Tamir - small, fast boost
            DefenseType::IronDome => Self {
                boost_duration_sec: 3.0,
                boost_acceleration_g: 30.0,
                max_velocity_km_s: 0.7,
                terminal_maneuver_g: 35.0,
                burnout_altitude_km: 5.0,
            },
        }
    }

    /// Calculate velocity at a given time since launch
    pub fn velocity_at_time(&self, time_sec: f64) -> f64 {
        if time_sec <= 0.0 {
            return 0.0;
        }

        let g_to_km_s2 = 0.00981; // Convert g to km/s²

        if time_sec <= self.boost_duration_sec {
            // Boost phase: accelerating
            let acceleration = self.boost_acceleration_g * g_to_km_s2;
            (acceleration * time_sec).min(self.max_velocity_km_s)
        } else {
            // Coast/terminal phase: constant max velocity
            self.max_velocity_km_s
        }
    }

    /// Calculate distance traveled at a given time since launch
    pub fn distance_at_time(&self, time_sec: f64) -> f64 {
        if time_sec <= 0.0 {
            return 0.0;
        }

        let g_to_km_s2 = 0.00981;
        let acceleration = self.boost_acceleration_g * g_to_km_s2;

        if time_sec <= self.boost_duration_sec {
            // Boost phase: d = 0.5 * a * t²
            0.5 * acceleration * time_sec * time_sec
        } else {
            // Distance during boost
            let boost_distance =
                0.5 * acceleration * self.boost_duration_sec * self.boost_duration_sec;
            // Plus distance during coast at max velocity
            let coast_time = time_sec - self.boost_duration_sec;
            boost_distance + self.max_velocity_km_s * coast_time
        }
    }

    /// Time to cover a 3D distance from a point `t_already_in_flight` seconds
    /// into the flight, accounting for the boost phase already elapsed and
    /// (for endo-atmospheric flight) a conservative drag allowance.
    ///
    /// This is the SINGLE SOURCE OF TRUTH for interceptor arrival-time math.
    /// It must be used by launch solutions, mid-course guidance re-solves, and
    /// intercept-time recalculations so all three agree. Previously three
    /// separate formulas existed (constant-max-velocity, boost-quadratic, and
    /// config-driven), producing systematic timing errors of 1-81 s depending
    /// on system — the primary cause of timeout misses.
    ///
    /// Returns None if the distance is unreachable (insufficient velocity).
    pub fn time_to_cover_distance(
        &self,
        distance_km: f64,
        t_already_in_flight: f64,
        endo_atmospheric: bool,
    ) -> Option<f64> {
        if distance_km <= 0.0 {
            return Some(0.0);
        }

        let g_to_km_s2 = 0.00981;
        let acceleration = self.boost_acceleration_g * g_to_km_s2;
        let boost_duration = self.boost_duration_sec;

        // Current velocity at start of the segment
        let v0 = if t_already_in_flight <= 0.0 {
            0.0
        } else if t_already_in_flight < boost_duration {
            (acceleration * t_already_in_flight).min(self.max_velocity_km_s)
        } else {
            self.max_velocity_km_s
        };

        // Remaining boost time and the distance still coverable during it
        let remaining_boost = (boost_duration - t_already_in_flight).max(0.0);
        // v(t) = v0 + a*t; d(t) = v0*t + 0.5*a*t²
        let boost_dist =
            v0 * remaining_boost + 0.5 * acceleration * remaining_boost * remaining_boost;
        let v_end_boost = (v0 + acceleration * remaining_boost).min(self.max_velocity_km_s);

        if distance_km <= boost_dist {
            // Segment completes during boost: solve v0*t + 0.5*a*t² = d
            let disc = v0 * v0 + 2.0 * acceleration * distance_km;
            let t = if acceleration > 1e-9 {
                (-v0 + disc.sqrt()) / acceleration
            } else if v0 > 1e-9 {
                distance_km / v0
            } else {
                return None;
            };
            return Some(t.max(0.0));
        }

        // Coast after burnout at v_end_boost
        if v_end_boost <= 1e-6 {
            return None;
        }

        // Conservative drag allowance for endo-atmospheric coast: integrates
        // a = ρ(h)v²/(2β) with a fixed mid-coast altitude approximation. The
        // runtime integration in Interceptor::update_kinematics is exact;
        // this estimate is used for arrival-time planning and deliberately
        // errs on the side of arriving slightly early (seeker absorbs ~1s).
        let effective_coast_velocity = if endo_atmospheric {
            // Typical endo intercept altitudes 10-40 km; use ~12% speed loss
            // as a planning margin (β≈4167 kg/m², 1976 US Std Atmosphere)
            v_end_boost * 0.88
        } else {
            v_end_boost
        };

        let coast_time = (distance_km - boost_dist) / effective_coast_velocity;
        Some(remaining_boost + coast_time)
    }

    /// Get the flight phase at a given progress (0.0 to 1.0)
    pub fn phase_at_progress(&self, progress: f64, flight_duration: f64) -> InterceptorPhase {
        let time_sec = progress * flight_duration;

        if time_sec <= self.boost_duration_sec {
            InterceptorPhase::Boost
        } else if progress < 0.7 {
            InterceptorPhase::Coast
        } else {
            InterceptorPhase::Terminal
        }
    }
}

/// An interceptor missile launched by a defense unit
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Interceptor {
    pub id: EntityId,
    pub launcher_id: EntityId, // Defense unit that launched this
    pub target_id: EntityId,   // Missile being targeted
    pub affiliation: Affiliation,
    pub defense_type: DefenseType, // Type of interceptor (matches launcher)
    pub launch_position: GeoCoord,
    pub position: GeoCoord,
    pub altitude_km: f64,
    pub target_position: GeoCoord, // Predicted intercept point (updated by mid-course guidance)
    pub target_altitude_km: f64,
    pub status: InterceptorStatus,
    pub miss_reason: MissReason, // Reason for miss (if status is Miss)
    pub phase: InterceptorPhase, // Current flight phase
    pub launch_time: f64,
    pub intercept_time: f64, // Predicted time to intercept
    pub current_flight_time: f64,
    pub current_velocity_km_s: f64, // Current velocity
    /// Actual path length traveled (km) — integrated from current_velocity,
    /// reflects cumulative drag (unlike kin.distance_at_time's ideal profile)
    pub distance_traveled_km: f64,
    pub hit_probability: f64, // Probability of successful intercept (0.0-1.0)
    // Mid-course guidance tracking
    pub original_target_position: GeoCoord, // Initial intercept point at launch
    pub original_target_altitude_km: f64,   // Initial target altitude at launch
    pub divert_budget_remaining_km: f64,    // Remaining divert capability
    pub total_divert_used_km: f64,          // Cumulative divert consumed
    pub last_guidance_update_time: f64,     // Timestamp of last mid-course update
    pub guidance_updates_count: u32,        // Number of guidance updates received
    // Skid maneuver tracking (for timing synchronization)
    pub is_skidding: bool, // Whether performing skid maneuver to bleed speed
    pub skid_factor: f64,  // Velocity reduction factor (0.0 = full speed, 1.0 = stopped)
    pub target_arrival_time: f64, // When target is expected at intercept point
    // Seeker and guidance state
    pub previous_position: GeoCoord, // Previous position for velocity vector calculation
    pub seeker_acquired: bool,       // Whether seeker has acquired the target
    pub seeker_acquisition_time: f64, // When seeker first acquired target (for acquisition delay)
    pub off_boresight_angle_deg: f64, // Current angle to target from interceptor boresight
    pub seeker_gimbal_limit_deg: f64, // Maximum seeker look angle from boresight
    // Energy and maneuverability state
    pub energy_state: f64, // Normalized energy (1.0 = full, 0.0 = depleted)
    pub total_maneuver_delta_v_used: f64, // Total delta-V used for maneuvers (km/s)
    pub max_maneuver_delta_v: f64, // Maximum available delta-V for maneuvers
    // Proportional Navigation state
    pub last_los_angle_rad: f64, // Last line-of-sight angle for PN guidance
    pub los_rate_rad_s: f64,     // Rate of change of LOS angle
    // Closest Point of Approach (CPA) tracking
    pub previous_distance_to_target_km: f64, // Distance to target last frame (for CPA detection)
    pub cpa_increasing_frames: u32, // Consecutive frames where distance is increasing (hysteresis)
    pub passed_cpa: bool,           // True once CPA is detected - stops guidance from turning back
    pub heading_at_cpa: f64, // Heading (degrees) when CPA was detected - maintain this heading
    // Final intercept result (snapshot at time of resolution)
    pub final_miss_distance_km: Option<f64>, // 3D miss distance at intercept (None if not resolved)
    pub final_pk: Option<f64>, // Pk at moment of intercept attempt (None if not resolved)
}

impl Interceptor {
    pub fn new(
        id: EntityId,
        launcher_id: EntityId,
        target_id: EntityId,
        affiliation: Affiliation,
        defense_type: DefenseType,
        launch_position: GeoCoord,
        target_position: GeoCoord,
        target_altitude_km: f64,
        launch_time: f64,
        intercept_time: f64,
        divert_budget_km: f64,
    ) -> Self {
        // Hit probability based on defense system type
        let hit_probability = match defense_type {
            DefenseType::GBI => 0.56,         // ~56% for GBI
            DefenseType::THAAD => 0.80,       // ~80% for THAAD
            DefenseType::Aegis => 0.85,       // ~85% for SM-3
            DefenseType::Patriot => 0.70,     // ~70% for PAC-3
            DefenseType::S400 => 0.75,        // ~75% estimate
            DefenseType::IronDome => 0.90,    // ~90% for Iron Dome
            DefenseType::DavidsSling => 0.85, // ~85% for Stunner missile
            DefenseType::Arrow3 => 0.80,      // ~80% for Arrow 3
        };

        // Seeker gimbal limits vary by system
        let seeker_gimbal_limit_deg = match defense_type {
            DefenseType::GBI => 25.0,     // EKV has moderate gimbal
            DefenseType::THAAD => 30.0,   // THAAD has good gimbal range
            DefenseType::Aegis => 20.0,   // SM-3 KW has limited gimbal
            DefenseType::Patriot => 40.0, // PAC-3 is very agile
            DefenseType::S400 => 25.0,
            DefenseType::IronDome => 45.0, // Tamir is highly maneuverable
            DefenseType::DavidsSling => 35.0,
            DefenseType::Arrow3 => 25.0,
        };

        // Maximum divert delta-V (km/s) for terminal maneuvers
        let max_maneuver_delta_v = match defense_type {
            DefenseType::GBI => 0.8, // EKV has good divert
            DefenseType::THAAD => 0.5,
            DefenseType::Aegis => 0.4,   // SM-3 KW limited divert
            DefenseType::Patriot => 0.6, // PAC-3 is agile
            DefenseType::S400 => 0.4,
            DefenseType::IronDome => 0.3,
            DefenseType::DavidsSling => 0.5,
            DefenseType::Arrow3 => 0.5,
        };

        Self {
            id,
            launcher_id,
            target_id,
            affiliation,
            defense_type,
            launch_position,
            position: launch_position,
            altitude_km: 0.0,
            target_position,
            target_altitude_km,
            status: InterceptorStatus::Pending, // Starts pending until launch_time
            miss_reason: MissReason::None,
            phase: InterceptorPhase::Boost,
            launch_time,
            intercept_time,
            current_flight_time: 0.0,
            current_velocity_km_s: 0.0,
            distance_traveled_km: 0.0,
            hit_probability,
            // Mid-course guidance initialization
            original_target_position: target_position,
            original_target_altitude_km: target_altitude_km,
            divert_budget_remaining_km: divert_budget_km,
            total_divert_used_km: 0.0,
            last_guidance_update_time: launch_time,
            guidance_updates_count: 0,
            // Skid maneuver initialization
            is_skidding: false,
            skid_factor: 0.0,
            target_arrival_time: intercept_time,
            // Seeker and guidance state
            previous_position: launch_position,
            seeker_acquired: false,
            seeker_acquisition_time: 0.0,
            off_boresight_angle_deg: 0.0,
            seeker_gimbal_limit_deg,
            // Energy and maneuverability
            energy_state: 1.0,
            total_maneuver_delta_v_used: 0.0,
            max_maneuver_delta_v,
            // Proportional Navigation state
            last_los_angle_rad: 0.0,
            los_rate_rad_s: 0.0,
            // CPA tracking - start with large value
            previous_distance_to_target_km: f64::MAX,
            cpa_increasing_frames: 0,
            passed_cpa: false,
            heading_at_cpa: 0.0,
            // Final intercept result
            final_miss_distance_km: None,
            final_pk: None,
        }
    }

    /// Get the progress through the flight (0.0 to 1.0)
    pub fn flight_progress(&self) -> f64 {
        let flight_duration = self.intercept_time - self.launch_time;
        if flight_duration <= 0.0 {
            return 1.0;
        }
        (self.current_flight_time / flight_duration).clamp(0.0, 1.0)
    }

    /// Get kinematics profile for this interceptor
    pub fn kinematics(&self) -> InterceptorKinematics {
        InterceptorKinematics::for_defense_type(self.defense_type)
    }

    /// Get max interceptor speed in km/s based on type
    pub fn max_speed_km_s(&self) -> f64 {
        self.kinematics().max_velocity_km_s
    }

    /// Update kinematics based on current flight time
    /// Includes CUMULATIVE atmospheric drag for endo-atmospheric interceptors.
    ///
    /// Drag is stateful: each sub-step integrates v ← v − a_drag(v,h)·dt from
    /// the previous ACTUAL velocity, not the ideal boost profile. Previously
    /// the velocity was re-derived from the ideal profile each frame with a
    /// single ≤10% decrement, so drag never accumulated and the interceptor
    /// flew faster than every arrival-time calculation predicted.
    pub fn update_kinematics(&mut self, dt: f64) {
        let kin = self.kinematics();
        let flight_duration = self.intercept_time - self.launch_time;

        // Endo-atmospheric systems subject to drag below 100 km (post-boost)
        let is_endo_system = matches!(
            self.defense_type,
            DefenseType::Patriot
                | DefenseType::THAAD
                | DefenseType::IronDome
                | DefenseType::DavidsSling
        );

        let ideal_velocity = kin.velocity_at_time(self.current_flight_time);

        self.current_velocity_km_s = if is_endo_system
            && self.altitude_km < 100.0
            && self.phase != InterceptorPhase::Boost
        {
            // Cumulative drag integration from previous actual velocity.
            // Reference: 1976 US Standard Atmosphere; a = ρv²/(2β)
            let ballistic_coef = BallisticCoefficient::endo_interceptor().value();
            let rho = atmospheric_density(self.altitude_km);

            // Start from previous actual velocity, but never below the ideal
            // profile (boost may have raised it since last step)
            let v_prev = self.current_velocity_km_s.max(ideal_velocity * 0.999);

            if rho > 0.0 && v_prev > 0.0 {
                let v_m_s = v_prev * 1000.0;
                let drag_decel_m_s2 = (rho * v_m_s * v_m_s) / (2.0 * ballistic_coef);
                let dv = (drag_decel_m_s2 * dt.max(0.0)).min(v_m_s * 0.5); // clamp per-step loss
                let v_new = ((v_m_s - dv) / 1000.0).max(0.3);
                v_new.min(ideal_velocity) // drag never speeds us up
            } else {
                ideal_velocity
            }
        } else {
            // Exo-atmospheric or boost: follow ideal profile exactly
            ideal_velocity
        };

        // Track actual path length (used by guidance for arrival-time math)
        self.distance_traveled_km += self.current_velocity_km_s * dt.max(0.0);

        // Update phase
        self.phase = kin.phase_at_progress(self.flight_progress(), flight_duration);
    }
}
