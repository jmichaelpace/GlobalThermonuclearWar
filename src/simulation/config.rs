use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::fs;
use std::path::Path;

use super::entities::DefenseType;

/// System identification info
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SystemInfo {
    pub name: String,
    #[serde(default)]
    pub description: String,
    #[serde(default)]
    pub country: Option<String>,
    #[serde(default)]
    pub nato_designation: Option<String>,
}

/// Detection capabilities
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DetectionConfig {
    pub detection_range_km: f64,
    pub engagement_range_km: f64,
}

/// Altitude engagement envelope
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AltitudeEnvelopeConfig {
    pub min_engagement_altitude_km: f64,
    pub max_engagement_altitude_km: f64,
}

/// Kinematic parameters
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct KinematicsConfig {
    pub boost_duration_sec: f64,
    pub boost_acceleration_g: f64,
    pub max_velocity_km_s: f64,
    pub terminal_maneuver_g: f64,
    pub burnout_altitude_km: f64,
    pub average_speed_km_s: f64,
}

/// Engagement parameters
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EngagementConfig {
    pub hit_probability: f64,
    pub terminal_blend_factor: f64,
}

/// Kill envelope parameters
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct KillEnvelopeConfig {
    pub seeker_range_km: f64,
    pub kill_radius_km: f64,
    pub base_pk: f64,
}

/// Complete defense system configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DefenseConfig {
    pub system: SystemInfo,
    pub detection: DetectionConfig,
    pub altitude_envelope: AltitudeEnvelopeConfig,
    pub kinematics: KinematicsConfig,
    pub engagement: EngagementConfig,
    pub kill_envelope: KillEnvelopeConfig,
}

/// Registry holding all defense configurations
pub struct DefenseConfigRegistry {
    configs: HashMap<DefenseType, DefenseConfig>,
}

impl DefenseConfigRegistry {
    /// Load configurations from a directory
    pub fn load(config_dir: &Path) -> Result<Self, ConfigError> {
        let mut configs = HashMap::new();

        let defense_types = [
            (DefenseType::GBI, "gbi.toml"),
            (DefenseType::Aegis, "aegis.toml"),
            (DefenseType::THAAD, "thaad.toml"),
            (DefenseType::Arrow3, "arrow3.toml"),
            (DefenseType::DavidsSling, "davids_sling.toml"),
            (DefenseType::Patriot, "patriot.toml"),
            (DefenseType::S400, "s400.toml"),
            (DefenseType::IronDome, "iron_dome.toml"),
        ];

        for (defense_type, filename) in defense_types {
            let file_path = config_dir.join("platform").join(filename);

            let config = if file_path.exists() {
                match Self::load_config_file(&file_path) {
                    Ok(cfg) => cfg,
                    Err(e) => {
                        eprintln!("Warning: Failed to load {}: {}, using defaults", filename, e);
                        Self::default_config(defense_type)
                    }
                }
            } else {
                eprintln!("Warning: Config file {} not found, using defaults", filename);
                Self::default_config(defense_type)
            };

            configs.insert(defense_type, config);
        }

        Ok(Self { configs })
    }

    /// Load a single config file
    fn load_config_file(path: &Path) -> Result<DefenseConfig, ConfigError> {
        let contents = fs::read_to_string(path)
            .map_err(|e| ConfigError::IoError(e.to_string()))?;

        toml::from_str(&contents)
            .map_err(|e| ConfigError::ParseError(e.to_string()))
    }

    /// Get configuration for a defense type
    pub fn get(&self, defense_type: DefenseType) -> &DefenseConfig {
        self.configs.get(&defense_type).unwrap_or_else(|| {
            panic!("No config for defense type {:?}", defense_type)
        })
    }

    /// Create default configuration using hardcoded values
    pub fn default_config(defense_type: DefenseType) -> DefenseConfig {
        match defense_type {
            DefenseType::GBI => DefenseConfig {
                system: SystemInfo {
                    name: "GBI".to_string(),
                    description: "Ground-Based Interceptor - midcourse defense".to_string(),
                    country: None,
                    nato_designation: None,
                },
                detection: DetectionConfig {
                    detection_range_km: 2000.0,
                    engagement_range_km: 2000.0,
                },
                altitude_envelope: AltitudeEnvelopeConfig {
                    min_engagement_altitude_km: 200.0,
                    max_engagement_altitude_km: 2000.0,
                },
                kinematics: KinematicsConfig {
                    boost_duration_sec: 170.0,
                    boost_acceleration_g: 5.0,
                    max_velocity_km_s: 8.0,
                    terminal_maneuver_g: 20.0,
                    burnout_altitude_km: 200.0,
                    average_speed_km_s: 7.0,
                },
                engagement: EngagementConfig {
                    hit_probability: 0.56,
                    terminal_blend_factor: 0.15,
                },
                kill_envelope: KillEnvelopeConfig {
                    seeker_range_km: 100.0,
                    kill_radius_km: 8.0,
                    base_pk: 0.56,
                },
            },
            DefenseType::Aegis => DefenseConfig {
                system: SystemInfo {
                    name: "Aegis BMD".to_string(),
                    description: "SM-3 Block IIA ship-based exoatmospheric".to_string(),
                    country: None,
                    nato_designation: None,
                },
                detection: DetectionConfig {
                    detection_range_km: 500.0,
                    engagement_range_km: 500.0,
                },
                altitude_envelope: AltitudeEnvelopeConfig {
                    min_engagement_altitude_km: 80.0,
                    max_engagement_altitude_km: 600.0,
                },
                kinematics: KinematicsConfig {
                    boost_duration_sec: 30.0,
                    boost_acceleration_g: 15.0,
                    max_velocity_km_s: 4.5,
                    terminal_maneuver_g: 25.0,
                    burnout_altitude_km: 100.0,
                    average_speed_km_s: 3.5,
                },
                engagement: EngagementConfig {
                    hit_probability: 0.85,
                    terminal_blend_factor: 0.15,
                },
                kill_envelope: KillEnvelopeConfig {
                    seeker_range_km: 80.0,
                    kill_radius_km: 5.0,
                    base_pk: 0.80,
                },
            },
            DefenseType::THAAD => DefenseConfig {
                system: SystemInfo {
                    name: "THAAD".to_string(),
                    description: "Terminal High Altitude Area Defense".to_string(),
                    country: None,
                    nato_designation: None,
                },
                detection: DetectionConfig {
                    detection_range_km: 200.0,
                    engagement_range_km: 200.0,
                },
                altitude_envelope: AltitudeEnvelopeConfig {
                    min_engagement_altitude_km: 40.0,
                    max_engagement_altitude_km: 150.0,
                },
                kinematics: KinematicsConfig {
                    boost_duration_sec: 12.0,
                    boost_acceleration_g: 20.0,
                    max_velocity_km_s: 2.8,
                    terminal_maneuver_g: 30.0,
                    burnout_altitude_km: 40.0,
                    average_speed_km_s: 2.5,
                },
                engagement: EngagementConfig {
                    hit_probability: 0.80,
                    terminal_blend_factor: 0.40,
                },
                kill_envelope: KillEnvelopeConfig {
                    seeker_range_km: 50.0,
                    kill_radius_km: 3.0,
                    base_pk: 0.90,
                },
            },
            DefenseType::Arrow3 => DefenseConfig {
                system: SystemInfo {
                    name: "Arrow 3".to_string(),
                    description: "Israeli exo-atmospheric interceptor".to_string(),
                    country: None,
                    nato_designation: None,
                },
                detection: DetectionConfig {
                    detection_range_km: 400.0,
                    engagement_range_km: 400.0,
                },
                altitude_envelope: AltitudeEnvelopeConfig {
                    min_engagement_altitude_km: 50.0,
                    max_engagement_altitude_km: 100.0,
                },
                kinematics: KinematicsConfig {
                    boost_duration_sec: 25.0,
                    boost_acceleration_g: 12.0,
                    max_velocity_km_s: 2.5,
                    terminal_maneuver_g: 20.0,
                    burnout_altitude_km: 50.0,
                    average_speed_km_s: 2.5,
                },
                engagement: EngagementConfig {
                    hit_probability: 0.80,
                    terminal_blend_factor: 0.20,
                },
                kill_envelope: KillEnvelopeConfig {
                    seeker_range_km: 80.0,
                    kill_radius_km: 5.0,
                    base_pk: 0.80,
                },
            },
            DefenseType::DavidsSling => DefenseConfig {
                system: SystemInfo {
                    name: "David's Sling".to_string(),
                    description: "Israeli mid-tier defense with Stunner missile".to_string(),
                    country: None,
                    nato_designation: None,
                },
                detection: DetectionConfig {
                    detection_range_km: 160.0,
                    engagement_range_km: 160.0,
                },
                altitude_envelope: AltitudeEnvelopeConfig {
                    min_engagement_altitude_km: 15.0,
                    max_engagement_altitude_km: 70.0,
                },
                kinematics: KinematicsConfig {
                    boost_duration_sec: 10.0,
                    boost_acceleration_g: 15.0,
                    max_velocity_km_s: 2.0,
                    terminal_maneuver_g: 40.0,
                    burnout_altitude_km: 20.0,
                    average_speed_km_s: 1.0,
                },
                engagement: EngagementConfig {
                    hit_probability: 0.85,
                    terminal_blend_factor: 0.50,
                },
                kill_envelope: KillEnvelopeConfig {
                    seeker_range_km: 40.0,
                    kill_radius_km: 15.0,
                    base_pk: 0.85,
                },
            },
            DefenseType::Patriot => DefenseConfig {
                system: SystemInfo {
                    name: "Patriot".to_string(),
                    description: "PAC-3 MSE terminal defense".to_string(),
                    country: None,
                    nato_designation: None,
                },
                detection: DetectionConfig {
                    detection_range_km: 150.0,
                    engagement_range_km: 70.0,
                },
                altitude_envelope: AltitudeEnvelopeConfig {
                    min_engagement_altitude_km: 0.5,
                    max_engagement_altitude_km: 40.0,
                },
                kinematics: KinematicsConfig {
                    boost_duration_sec: 8.0,
                    boost_acceleration_g: 25.0,
                    max_velocity_km_s: 1.7,
                    terminal_maneuver_g: 50.0,
                    burnout_altitude_km: 15.0,
                    average_speed_km_s: 1.7,
                },
                engagement: EngagementConfig {
                    hit_probability: 0.70,
                    terminal_blend_factor: 0.60,
                },
                kill_envelope: KillEnvelopeConfig {
                    seeker_range_km: 30.0,
                    kill_radius_km: 2.0,
                    base_pk: 0.90,
                },
            },
            DefenseType::S400 => DefenseConfig {
                system: SystemInfo {
                    name: "S-400".to_string(),
                    description: "Russian long-range SAM with 40N6 missile".to_string(),
                    country: None,
                    nato_designation: None,
                },
                detection: DetectionConfig {
                    detection_range_km: 400.0,
                    engagement_range_km: 400.0,
                },
                altitude_envelope: AltitudeEnvelopeConfig {
                    min_engagement_altitude_km: 0.01,
                    max_engagement_altitude_km: 185.0,
                },
                kinematics: KinematicsConfig {
                    boost_duration_sec: 15.0,
                    boost_acceleration_g: 18.0,
                    max_velocity_km_s: 2.0,
                    terminal_maneuver_g: 25.0,
                    burnout_altitude_km: 30.0,
                    average_speed_km_s: 2.0,
                },
                engagement: EngagementConfig {
                    hit_probability: 0.75,
                    terminal_blend_factor: 0.50,
                },
                kill_envelope: KillEnvelopeConfig {
                    seeker_range_km: 50.0,
                    kill_radius_km: 20.0,
                    base_pk: 0.80,
                },
            },
            DefenseType::IronDome => DefenseConfig {
                system: SystemInfo {
                    name: "Iron Dome".to_string(),
                    description: "Israeli short-range defense with Tamir missile".to_string(),
                    country: None,
                    nato_designation: None,
                },
                detection: DetectionConfig {
                    detection_range_km: 70.0,
                    engagement_range_km: 70.0,
                },
                altitude_envelope: AltitudeEnvelopeConfig {
                    min_engagement_altitude_km: 0.0,
                    max_engagement_altitude_km: 10.0,
                },
                kinematics: KinematicsConfig {
                    boost_duration_sec: 3.0,
                    boost_acceleration_g: 30.0,
                    max_velocity_km_s: 0.7,
                    terminal_maneuver_g: 35.0,
                    burnout_altitude_km: 5.0,
                    average_speed_km_s: 0.3,
                },
                engagement: EngagementConfig {
                    hit_probability: 0.90,
                    terminal_blend_factor: 0.70,
                },
                kill_envelope: KillEnvelopeConfig {
                    seeker_range_km: 20.0,
                    kill_radius_km: 10.0,
                    base_pk: 0.90,
                },
            },
        }
    }

    /// Create registry with all defaults (no file loading)
    pub fn with_defaults() -> Self {
        let mut configs = HashMap::new();

        let defense_types = [
            DefenseType::GBI,
            DefenseType::Aegis,
            DefenseType::THAAD,
            DefenseType::Arrow3,
            DefenseType::DavidsSling,
            DefenseType::Patriot,
            DefenseType::S400,
            DefenseType::IronDome,
        ];

        for defense_type in defense_types {
            configs.insert(defense_type, Self::default_config(defense_type));
        }

        Self { configs }
    }
}

// ============================================================================
// Sensor Configuration
// ============================================================================

/// Radar frequency bands with different characteristics
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub enum RadarBand {
    #[serde(rename = "L")]
    L,  // 1-2 GHz: Long range, low attenuation, lower resolution
    #[serde(rename = "S")]
    S,  // 2-4 GHz: Good range, moderate attenuation, good resolution
    #[serde(rename = "C")]
    C,  // 4-8 GHz: Medium range, moderate-high attenuation
    #[serde(rename = "X")]
    X,  // 8-12 GHz: High resolution, higher attenuation, fire control
    #[serde(rename = "Ku")]
    Ku, // 12-18 GHz: Very high resolution, high attenuation
}

impl RadarBand {
    /// Atmospheric attenuation coefficient (dB/km at sea level)
    pub fn attenuation_coefficient(&self) -> f64 {
        match self {
            RadarBand::L => 0.005,   // Very low attenuation
            RadarBand::S => 0.010,   // Low attenuation
            RadarBand::C => 0.015,   // Moderate attenuation
            RadarBand::X => 0.020,   // Higher attenuation
            RadarBand::Ku => 0.030,  // High attenuation
        }
    }

    /// Resolution/quality multiplier (higher frequency = better resolution)
    pub fn quality_multiplier(&self) -> f64 {
        match self {
            RadarBand::L => 0.85,    // Lower resolution
            RadarBand::S => 0.92,    // Good resolution
            RadarBand::C => 0.96,    // Better resolution
            RadarBand::X => 1.00,    // Excellent resolution (baseline)
            RadarBand::Ku => 1.05,   // Outstanding resolution
        }
    }
}

impl Default for RadarBand {
    fn default() -> Self {
        RadarBand::X  // Default to X-band (most common for fire control)
    }
}

/// Sensor detection parameters
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SensorDetectionConfig {
    pub detection_range_km: f64,
    pub azimuth_coverage_deg: f64,
    pub elevation_min_deg: f64,
    pub elevation_max_deg: f64,
    #[serde(default)]
    pub radar_band: RadarBand,
}

/// Sensor tracking parameters
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SensorTrackingConfig {
    pub max_simultaneous_tracks: u32,
    pub track_update_rate_hz: f64,
    pub minimum_rcs_dbsm: f64,
}

/// Complete sensor configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SensorConfig {
    pub system: SystemInfo,
    pub detection: SensorDetectionConfig,
    pub tracking: SensorTrackingConfig,
}

/// Registry holding all sensor configurations
pub struct SensorConfigRegistry {
    configs: HashMap<String, SensorConfig>,
    default_config: SensorConfig,
}

impl SensorConfigRegistry {
    /// Load configurations from a directory
    pub fn load(config_dir: &Path) -> Result<Self, ConfigError> {
        let mut configs = HashMap::new();
        let sensors_dir = config_dir.join("sensors");

        // Load default config first
        let default_path = sensors_dir.join("default.toml");
        let default_config = if default_path.exists() {
            match Self::load_config_file(&default_path) {
                Ok(cfg) => cfg,
                Err(e) => {
                    eprintln!("Warning: Failed to load sensors/default.toml: {}, using hardcoded defaults", e);
                    Self::hardcoded_default()
                }
            }
        } else {
            eprintln!("Warning: sensors/default.toml not found, using hardcoded defaults");
            Self::hardcoded_default()
        };

        // Load all .toml files from sensors directory
        if let Ok(entries) = fs::read_dir(&sensors_dir) {
            for entry in entries.flatten() {
                let path = entry.path();
                if path.extension().map_or(false, |ext| ext == "toml") {
                    if path.file_name().map_or(false, |n| n == "default.toml") {
                        continue;
                    }

                    match Self::load_config_file(&path) {
                        Ok(cfg) => {
                            let normalized_name = Self::normalize_name(&cfg.system.name);
                            configs.insert(normalized_name, cfg);
                        }
                        Err(e) => {
                            eprintln!("Warning: Failed to load {:?}: {}", path, e);
                        }
                    }
                }
            }
        }

        Ok(Self { configs, default_config })
    }

    fn load_config_file(path: &Path) -> Result<SensorConfig, ConfigError> {
        let contents = fs::read_to_string(path)
            .map_err(|e| ConfigError::IoError(e.to_string()))?;
        toml::from_str(&contents)
            .map_err(|e| ConfigError::ParseError(e.to_string()))
    }

    fn normalize_name(name: &str) -> String {
        name.to_lowercase().replace([' ', '-', '/'], "_")
    }

    /// Get configuration for a sensor by name
    pub fn get_by_name(&self, sensor_name: &str) -> &SensorConfig {
        let normalized = Self::normalize_name(sensor_name);
        self.configs.get(&normalized).unwrap_or(&self.default_config)
    }

    fn hardcoded_default() -> SensorConfig {
        SensorConfig {
            system: SystemInfo {
                name: "Generic Radar".to_string(),
                description: "Default radar configuration".to_string(),
                country: None,
                nato_designation: None,
            },
            detection: SensorDetectionConfig {
                detection_range_km: 200.0,
                azimuth_coverage_deg: 360.0,
                elevation_min_deg: 0.0,
                elevation_max_deg: 90.0,
                radar_band: RadarBand::X,
            },
            tracking: SensorTrackingConfig {
                max_simultaneous_tracks: 20,
                track_update_rate_hz: 5.0,
                minimum_rcs_dbsm: 0.0,
            },
        }
    }

    pub fn with_defaults() -> Self {
        Self {
            configs: HashMap::new(),
            default_config: Self::hardcoded_default(),
        }
    }
}

// ============================================================================
// Interceptor Configuration
// ============================================================================

/// Complete interceptor configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct InterceptorConfig {
    pub system: SystemInfo,
    pub altitude_envelope: AltitudeEnvelopeConfig,
    pub kinematics: KinematicsConfig,
    pub engagement: EngagementConfig,
    pub kill_envelope: KillEnvelopeConfig,
}

/// Registry holding all interceptor configurations
pub struct InterceptorConfigRegistry {
    configs: HashMap<String, InterceptorConfig>,
    default_config: InterceptorConfig,
}

impl InterceptorConfigRegistry {
    /// Load configurations from a directory
    pub fn load(config_dir: &Path) -> Result<Self, ConfigError> {
        let mut configs = HashMap::new();
        let interceptors_dir = config_dir.join("interceptors");

        // Load default config first
        let default_path = interceptors_dir.join("default.toml");
        let default_config = if default_path.exists() {
            match Self::load_config_file(&default_path) {
                Ok(cfg) => cfg,
                Err(e) => {
                    eprintln!("Warning: Failed to load interceptors/default.toml: {}, using hardcoded defaults", e);
                    Self::hardcoded_default()
                }
            }
        } else {
            eprintln!("Warning: interceptors/default.toml not found, using hardcoded defaults");
            Self::hardcoded_default()
        };

        // Load all .toml files from interceptors directory
        if let Ok(entries) = fs::read_dir(&interceptors_dir) {
            for entry in entries.flatten() {
                let path = entry.path();
                if path.extension().map_or(false, |ext| ext == "toml") {
                    if path.file_name().map_or(false, |n| n == "default.toml") {
                        continue;
                    }

                    match Self::load_config_file(&path) {
                        Ok(cfg) => {
                            let normalized_name = Self::normalize_name(&cfg.system.name);
                            configs.insert(normalized_name, cfg);
                        }
                        Err(e) => {
                            eprintln!("Warning: Failed to load {:?}: {}", path, e);
                        }
                    }
                }
            }
        }

        Ok(Self { configs, default_config })
    }

    fn load_config_file(path: &Path) -> Result<InterceptorConfig, ConfigError> {
        let contents = fs::read_to_string(path)
            .map_err(|e| ConfigError::IoError(e.to_string()))?;
        toml::from_str(&contents)
            .map_err(|e| ConfigError::ParseError(e.to_string()))
    }

    fn normalize_name(name: &str) -> String {
        name.to_lowercase().replace([' ', '-'], "_")
    }

    /// Get configuration for an interceptor by name
    pub fn get_by_name(&self, interceptor_name: &str) -> &InterceptorConfig {
        let normalized = Self::normalize_name(interceptor_name);
        self.configs.get(&normalized).unwrap_or(&self.default_config)
    }

    fn hardcoded_default() -> InterceptorConfig {
        InterceptorConfig {
            system: SystemInfo {
                name: "Generic Interceptor".to_string(),
                description: "Default interceptor configuration".to_string(),
                country: None,
                nato_designation: None,
            },
            altitude_envelope: AltitudeEnvelopeConfig {
                min_engagement_altitude_km: 10.0,
                max_engagement_altitude_km: 100.0,
            },
            kinematics: KinematicsConfig {
                boost_duration_sec: 10.0,
                boost_acceleration_g: 15.0,
                max_velocity_km_s: 2.0,
                terminal_maneuver_g: 25.0,
                burnout_altitude_km: 20.0,
                average_speed_km_s: 1.5,
            },
            engagement: EngagementConfig {
                hit_probability: 0.70,
                terminal_blend_factor: 0.30,
            },
            kill_envelope: KillEnvelopeConfig {
                seeker_range_km: 30.0,
                kill_radius_km: 5.0,
                base_pk: 0.70,
            },
        }
    }

    pub fn with_defaults() -> Self {
        Self {
            configs: HashMap::new(),
            default_config: Self::hardcoded_default(),
        }
    }
}

// ============================================================================
// Satellite Configuration
// ============================================================================

/// Orbit type for satellites
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum OrbitType {
    GEO,  // Geosynchronous
    HEO,  // Highly Elliptical Orbit
    LEO,  // Low Earth Orbit
    MEO,  // Medium Earth Orbit
}

/// Satellite orbit parameters
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SatelliteOrbitConfig {
    pub altitude_km: f64,
    pub orbital_period_hours: f64,
    pub orbit_type: OrbitType,
    #[serde(default)]
    pub inclination_deg: f64,
}

/// Satellite sensor parameters
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SatelliteSensorConfig {
    pub sensor_type: String,
    pub coverage_angle_deg: f64,
    pub detection_range_km: f64,
    #[serde(default)]
    pub can_detect_boost_phase: bool,
    #[serde(default)]
    pub can_track_midcourse: bool,
    #[serde(default = "default_revisit_time")]
    pub revisit_time_sec: f64,
}

fn default_revisit_time() -> f64 {
    5.0
}

/// Complete satellite configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SatelliteConfig {
    pub system: SystemInfo,
    pub orbit: SatelliteOrbitConfig,
    pub sensor: SatelliteSensorConfig,
}

/// Registry holding all satellite configurations
pub struct SatelliteConfigRegistry {
    configs: HashMap<String, SatelliteConfig>,
    default_config: SatelliteConfig,
}

impl SatelliteConfigRegistry {
    /// Load configurations from a directory
    pub fn load(config_dir: &Path) -> Result<Self, ConfigError> {
        let mut configs = HashMap::new();
        let satellites_dir = config_dir.join("satellites");

        // Load default config first
        let default_path = satellites_dir.join("default.toml");
        let default_config = if default_path.exists() {
            match Self::load_config_file(&default_path) {
                Ok(cfg) => cfg,
                Err(e) => {
                    eprintln!("Warning: Failed to load satellites/default.toml: {}, using hardcoded defaults", e);
                    Self::hardcoded_default()
                }
            }
        } else {
            eprintln!("Warning: satellites/default.toml not found, using hardcoded defaults");
            Self::hardcoded_default()
        };

        // Load all .toml files from satellites directory
        if let Ok(entries) = fs::read_dir(&satellites_dir) {
            for entry in entries.flatten() {
                let path = entry.path();
                if path.extension().map_or(false, |ext| ext == "toml") {
                    if path.file_name().map_or(false, |n| n == "default.toml") {
                        continue;
                    }

                    match Self::load_config_file(&path) {
                        Ok(cfg) => {
                            let normalized_name = Self::normalize_name(&cfg.system.name);
                            configs.insert(normalized_name, cfg);
                        }
                        Err(e) => {
                            eprintln!("Warning: Failed to load {:?}: {}", path, e);
                        }
                    }
                }
            }
        }

        Ok(Self { configs, default_config })
    }

    fn load_config_file(path: &Path) -> Result<SatelliteConfig, ConfigError> {
        let contents = fs::read_to_string(path)
            .map_err(|e| ConfigError::IoError(e.to_string()))?;
        toml::from_str(&contents)
            .map_err(|e| ConfigError::ParseError(e.to_string()))
    }

    fn normalize_name(name: &str) -> String {
        name.to_lowercase().replace([' ', '-'], "_")
    }

    /// Get configuration for a satellite by name
    pub fn get_by_name(&self, satellite_name: &str) -> &SatelliteConfig {
        let normalized = Self::normalize_name(satellite_name);
        self.configs.get(&normalized).unwrap_or(&self.default_config)
    }

    fn hardcoded_default() -> SatelliteConfig {
        SatelliteConfig {
            system: SystemInfo {
                name: "Generic Satellite".to_string(),
                description: "Default early warning satellite".to_string(),
                country: None,
                nato_designation: None,
            },
            orbit: SatelliteOrbitConfig {
                altitude_km: 35786.0,
                orbital_period_hours: 24.0,
                orbit_type: OrbitType::GEO,
                inclination_deg: 0.0,
            },
            sensor: SatelliteSensorConfig {
                sensor_type: "Infrared".to_string(),
                coverage_angle_deg: 10.0,
                detection_range_km: 4000.0,
                can_detect_boost_phase: true,
                can_track_midcourse: false,
                revisit_time_sec: 5.0,
            },
        }
    }

    pub fn with_defaults() -> Self {
        Self {
            configs: HashMap::new(),
            default_config: Self::hardcoded_default(),
        }
    }
}

// ============================================================================
// Missile Configuration
// ============================================================================

/// Type of ballistic missile
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum MissileType {
    ICBM,   // Intercontinental Ballistic Missile (>5,500 km)
    SLBM,   // Submarine-Launched Ballistic Missile
    IRBM,   // Intermediate-Range Ballistic Missile (3,000-5,500 km)
    MRBM,   // Medium-Range Ballistic Missile (1,000-3,000 km)
    SRBM,   // Short-Range Ballistic Missile (<1,000 km)
}

impl MissileType {
    pub fn name(&self) -> &'static str {
        match self {
            MissileType::ICBM => "ICBM",
            MissileType::SLBM => "SLBM",
            MissileType::IRBM => "IRBM",
            MissileType::MRBM => "MRBM",
            MissileType::SRBM => "SRBM",
        }
    }
}

/// Classification for missile type
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MissileClassificationConfig {
    #[serde(rename = "type")]
    pub missile_type: MissileType,
}

/// Range parameters for missile type
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MissileRangeConfig {
    pub min_range_km: f64,
    pub max_range_km: f64,
}

/// Trajectory parameters for missile type
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MissileTrajectoryConfig {
    /// Apogee calculation: base_km + range_km * range_factor
    pub apogee_base_km: f64,
    pub apogee_range_factor: f64,
    /// Flight time calculation: base_sec + range_km * range_factor
    pub flight_time_base_sec: f64,
    pub flight_time_range_factor: f64,
}

/// Boost phase configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MissileBoostConfig {
    /// Boost phase as fraction of total flight (0.0 to 1.0)
    pub boost_phase_fraction: f64,
    /// Approximate boost velocity in km/s
    pub boost_velocity_km_s: f64,
}

/// Countermeasures configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MissileCountermeasuresConfig {
    /// Whether this missile type typically has countermeasures
    pub has_countermeasures: bool,
    /// Default number of decoys
    pub default_decoys: u32,
    /// Maximum number of decoys
    pub max_decoys: u32,
}

/// Radar cross-section configuration (phase-dependent)
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MissileRcsConfig {
    /// RCS during boost phase in dBsm (typically large due to exhaust plume)
    pub rcs_boost_dbsm: f64,
    /// RCS during midcourse phase in dBsm (reentry vehicle in space)
    pub rcs_midcourse_dbsm: f64,
    /// RCS during terminal phase in dBsm (small, descending RV)
    pub rcs_terminal_dbsm: f64,
}

impl Default for MissileRcsConfig {
    fn default() -> Self {
        Self {
            rcs_boost_dbsm: 5.0,      // Large due to exhaust plume
            rcs_midcourse_dbsm: -5.0, // Small RV in space
            rcs_terminal_dbsm: -10.0, // Smallest signature
        }
    }
}

/// Complete missile type configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MissileConfig {
    pub system: SystemInfo,
    pub classification: MissileClassificationConfig,
    pub range: MissileRangeConfig,
    pub trajectory: MissileTrajectoryConfig,
    pub boost: MissileBoostConfig,
    pub countermeasures: MissileCountermeasuresConfig,
    #[serde(default)]
    pub radar_signature: MissileRcsConfig,
}

/// Registry holding all missile configurations by variant name
pub struct MissileConfigRegistry {
    /// Configs indexed by normalized name (lowercase, spaces to underscores)
    configs: HashMap<String, MissileConfig>,
    /// Default fallback config
    default_config: MissileConfig,
}

impl MissileConfigRegistry {
    /// Load configurations from a directory
    pub fn load(config_dir: &Path) -> Result<Self, ConfigError> {
        let mut configs = HashMap::new();
        let missiles_dir = config_dir.join("missiles");

        // Load default config first
        let default_path = missiles_dir.join("default.toml");
        let default_config = if default_path.exists() {
            match Self::load_config_file(&default_path) {
                Ok(cfg) => cfg,
                Err(e) => {
                    eprintln!("Warning: Failed to load default.toml: {}, using hardcoded defaults", e);
                    Self::hardcoded_default()
                }
            }
        } else {
            eprintln!("Warning: default.toml not found, using hardcoded defaults");
            Self::hardcoded_default()
        };

        // Recursively load all .toml files from missiles directory and subdirectories
        Self::load_configs_recursive(&missiles_dir, &mut configs);

        Ok(Self { configs, default_config })
    }

    /// Recursively load config files from a directory and its subdirectories
    fn load_configs_recursive(dir: &Path, configs: &mut HashMap<String, MissileConfig>) {
        if let Ok(entries) = fs::read_dir(dir) {
            for entry in entries.flatten() {
                let path = entry.path();

                if path.is_dir() {
                    // Recurse into subdirectories
                    Self::load_configs_recursive(&path, configs);
                } else if path.extension().map_or(false, |ext| ext == "toml") {
                    // Skip default.toml, it's handled separately
                    if path.file_name().map_or(false, |n| n == "default.toml") {
                        continue;
                    }

                    match Self::load_config_file(&path) {
                        Ok(cfg) => {
                            let normalized_name = Self::normalize_name(&cfg.system.name);
                            configs.insert(normalized_name, cfg);
                        }
                        Err(e) => {
                            eprintln!("Warning: Failed to load {:?}: {}", path, e);
                        }
                    }
                }
            }
        }
    }

    /// Load a single config file
    fn load_config_file(path: &Path) -> Result<MissileConfig, ConfigError> {
        let contents = fs::read_to_string(path)
            .map_err(|e| ConfigError::IoError(e.to_string()))?;

        toml::from_str(&contents)
            .map_err(|e| ConfigError::ParseError(e.to_string()))
    }

    /// Normalize a missile name for lookup (lowercase, replace spaces/hyphens with underscores)
    fn normalize_name(name: &str) -> String {
        name.to_lowercase()
            .replace([' ', '-'], "_")
    }

    /// Get configuration for a missile by name
    /// Matches missile names like "Shahab-3 #1" to config "Shahab-3"
    pub fn get_by_name(&self, missile_name: &str) -> &MissileConfig {
        // Strip any numbering suffix like " #1", " #2", etc.
        let base_name = missile_name
            .split(" #")
            .next()
            .unwrap_or(missile_name);

        let normalized = Self::normalize_name(base_name);

        self.configs.get(&normalized).unwrap_or(&self.default_config)
    }

    /// Get configuration for a missile type (returns first matching config or default)
    pub fn get(&self, missile_type: MissileType) -> &MissileConfig {
        self.configs
            .values()
            .find(|cfg| cfg.classification.missile_type == missile_type)
            .unwrap_or(&self.default_config)
    }

    /// Calculate apogee for a given range using a missile's config
    pub fn calculate_apogee_by_name(&self, missile_name: &str, range_km: f64) -> f64 {
        let config = self.get_by_name(missile_name);
        config.trajectory.apogee_base_km + range_km * config.trajectory.apogee_range_factor
    }

    /// Calculate flight time for a given range using a missile's config
    pub fn calculate_flight_time_by_name(&self, missile_name: &str, range_km: f64) -> f64 {
        let config = self.get_by_name(missile_name);
        config.trajectory.flight_time_base_sec + range_km * config.trajectory.flight_time_range_factor
    }

    /// Calculate apogee for a given range using the missile type's config
    pub fn calculate_apogee(&self, missile_type: MissileType, range_km: f64) -> f64 {
        let config = self.get(missile_type);
        config.trajectory.apogee_base_km + range_km * config.trajectory.apogee_range_factor
    }

    /// Calculate flight time for a given range using the missile type's config
    pub fn calculate_flight_time(&self, missile_type: MissileType, range_km: f64) -> f64 {
        let config = self.get(missile_type);
        config.trajectory.flight_time_base_sec + range_km * config.trajectory.flight_time_range_factor
    }

    /// Hardcoded default configuration
    fn hardcoded_default() -> MissileConfig {
        MissileConfig {
            system: SystemInfo {
                name: "Generic Ballistic Missile".to_string(),
                description: "Default configuration for unspecified missiles".to_string(),
                country: None,
                nato_designation: None,
            },
            classification: MissileClassificationConfig {
                missile_type: MissileType::MRBM,
            },
            range: MissileRangeConfig {
                min_range_km: 500.0,
                max_range_km: 5000.0,
            },
            trajectory: MissileTrajectoryConfig {
                apogee_base_km: 150.0,
                apogee_range_factor: 0.15,
                flight_time_base_sec: 400.0,
                flight_time_range_factor: 0.2,
            },
            boost: MissileBoostConfig {
                boost_phase_fraction: 0.15,
                boost_velocity_km_s: 3.5,
            },
            countermeasures: MissileCountermeasuresConfig {
                has_countermeasures: false,
                default_decoys: 0,
                max_decoys: 3,
            },
            radar_signature: MissileRcsConfig::default(),
        }
    }

    /// Create registry with hardcoded defaults (no file loading)
    pub fn with_defaults() -> Self {
        Self {
            configs: HashMap::new(),
            default_config: Self::hardcoded_default(),
        }
    }

    /// Determine missile type from range
    pub fn type_for_range(&self, range_km: f64) -> MissileType {
        if range_km >= 5500.0 {
            MissileType::ICBM
        } else if range_km >= 3000.0 {
            MissileType::IRBM
        } else if range_km >= 1000.0 {
            MissileType::MRBM
        } else {
            MissileType::SRBM
        }
    }

    /// Get the default configuration
    pub fn default(&self) -> &MissileConfig {
        &self.default_config
    }
}

/// Configuration loading errors
#[derive(Debug)]
pub enum ConfigError {
    IoError(String),
    ParseError(String),
}

impl std::fmt::Display for ConfigError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ConfigError::IoError(msg) => write!(f, "IO error: {}", msg),
            ConfigError::ParseError(msg) => write!(f, "Parse error: {}", msg),
        }
    }
}

impl std::error::Error for ConfigError {}
