use crate::map::GeoCoord;
use crate::simulation::config::{MissileType, SensorRole};
use serde::{Deserialize, Serialize};

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
    GBI,          // Ground-Based Interceptor
    S400,
    IronDome,
    DavidsSling,  // Israeli mid-tier defense (between Iron Dome and Arrow)
    Arrow3,       // Israeli exo-atmospheric interceptor
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
    pub launch_time: f64,       // Simulation time when launched
    pub flight_time: f64,       // Total expected flight time in seconds
    pub current_flight_time: f64, // Time since launch
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
    pub position: GeoCoord,  // Sub-satellite point
    pub altitude_km: f64,
    pub sensor_type: SensorType,
    pub coverage_angle_deg: f64,  // Half-angle of sensor cone
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
    pub azimuth_coverage_deg: f64,  // How wide the radar scans (360 for full coverage)
    pub facing_deg: f64,            // Direction the radar faces (0 = North, 90 = East)
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
    Pending,   // Scheduled but not yet launched (for salvo delay)
    InFlight,
    Hit,       // Successfully intercepted target
    Miss,      // Failed to intercept
    SelfDestruct, // Lost track, self-destructed
}

/// Flight phase of an interceptor
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum InterceptorPhase {
    Boost,      // Initial acceleration phase (high thrust)
    Coast,      // Ballistic coast phase (no thrust, constant velocity)
    Terminal,   // Terminal homing phase (maneuvering toward target)
}

/// Kinematics profile for an interceptor type
#[derive(Clone, Copy, Debug)]
pub struct InterceptorKinematics {
    pub boost_duration_sec: f64,      // Duration of boost phase
    pub boost_acceleration_g: f64,     // Acceleration during boost (in g's)
    pub max_velocity_km_s: f64,        // Maximum velocity achieved
    pub terminal_maneuver_g: f64,      // Max g's available for terminal maneuver
    pub burnout_altitude_km: f64,      // Typical altitude at end of boost
}

impl InterceptorKinematics {
    /// Get kinematics profile for a defense type
    pub fn for_defense_type(defense_type: DefenseType) -> Self {
        match defense_type {
            // GBI - three-stage solid rocket, reaches ~8 km/s
            DefenseType::GBI => Self {
                boost_duration_sec: 170.0,      // ~3 minute boost
                boost_acceleration_g: 5.0,
                max_velocity_km_s: 8.0,
                terminal_maneuver_g: 20.0,      // EKV can maneuver hard
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
                terminal_maneuver_g: 50.0,      // Very agile
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
                max_velocity_km_s: 2.0,
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
            let boost_distance = 0.5 * acceleration * self.boost_duration_sec * self.boost_duration_sec;
            // Plus distance during coast at max velocity
            let coast_time = time_sec - self.boost_duration_sec;
            boost_distance + self.max_velocity_km_s * coast_time
        }
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
    pub launcher_id: EntityId,     // Defense unit that launched this
    pub target_id: EntityId,       // Missile being targeted
    pub affiliation: Affiliation,
    pub defense_type: DefenseType, // Type of interceptor (matches launcher)
    pub launch_position: GeoCoord,
    pub position: GeoCoord,
    pub altitude_km: f64,
    pub target_position: GeoCoord, // Predicted intercept point
    pub target_altitude_km: f64,
    pub status: InterceptorStatus,
    pub phase: InterceptorPhase,   // Current flight phase
    pub launch_time: f64,
    pub intercept_time: f64,       // Predicted time to intercept
    pub current_flight_time: f64,
    pub current_velocity_km_s: f64, // Current velocity
    pub hit_probability: f64,      // Probability of successful intercept (0.0-1.0)
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
    ) -> Self {
        // Hit probability based on defense system type
        let hit_probability = match defense_type {
            DefenseType::GBI => 0.56,        // ~56% for GBI
            DefenseType::THAAD => 0.80,      // ~80% for THAAD
            DefenseType::Aegis => 0.85,      // ~85% for SM-3
            DefenseType::Patriot => 0.70,    // ~70% for PAC-3
            DefenseType::S400 => 0.75,       // ~75% estimate
            DefenseType::IronDome => 0.90,   // ~90% for Iron Dome
            DefenseType::DavidsSling => 0.85, // ~85% for Stunner missile
            DefenseType::Arrow3 => 0.80,     // ~80% for Arrow 3
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
            phase: InterceptorPhase::Boost,
            launch_time,
            intercept_time,
            current_flight_time: 0.0,
            current_velocity_km_s: 0.0,
            hit_probability,
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
    pub fn update_kinematics(&mut self) {
        let kin = self.kinematics();
        let flight_duration = self.intercept_time - self.launch_time;

        // Update velocity based on flight phase
        self.current_velocity_km_s = kin.velocity_at_time(self.current_flight_time);

        // Update phase
        self.phase = kin.phase_at_progress(self.flight_progress(), flight_duration);
    }
}
