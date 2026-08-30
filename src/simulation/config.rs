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

/// Interceptor guidance type
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
pub enum GuidanceType {
    /// Active homing - interceptor has onboard radar (SM-3, THAAD, Arrow-3)
    #[serde(rename = "active")]
    Active,
    /// Semi-active homing - requires continuous radar illumination (SM-2, Patriot PAC-2, 40N6)
    #[serde(rename = "semi_active")]
    SemiActive,
    /// Command guidance - radar provides steering commands (older systems)
    #[serde(rename = "command")]
    Command,
}

impl Default for GuidanceType {
    fn default() -> Self {
        GuidanceType::Active // Default to active for backward compatibility
    }
}

/// Engagement parameters
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EngagementConfig {
    pub hit_probability: f64,
    pub terminal_blend_factor: f64,
    /// Guidance type - determines if continuous illumination is required
    #[serde(default)]
    pub guidance_type: GuidanceType,
    /// Mid-course guidance parameters
    #[serde(default)]
    pub midcourse_guidance: MidcourseGuidanceConfig,
}

/// Mid-course guidance configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MidcourseGuidanceConfig {
    /// Total divert capability in km (fuel budget for course corrections)
    pub divert_budget_km: f64,
    /// How often guidance updates are sent (seconds)
    pub update_interval_sec: f64,
    /// Minimum correction threshold - don't waste divert on tiny adjustments (km)
    pub min_correction_km: f64,
    /// Whether mid-course updates are enabled
    pub enabled: bool,
    /// Proportional Navigation constant (N) for terminal guidance
    /// Typical values: 3-5 for missiles. Higher = more aggressive pursuit.
    /// Reference: Zarchan, "Tactical and Strategic Missile Guidance"
    #[serde(default = "default_navigation_constant")]
    pub navigation_constant: f64,
}

fn default_navigation_constant() -> f64 {
    4.0 // N=4 is a common choice for hit-to-kill interceptors
}

impl Default for MidcourseGuidanceConfig {
    fn default() -> Self {
        Self {
            divert_budget_km: 50.0,   // 50km total divert capability
            update_interval_sec: 5.0, // Update every 5 seconds
            min_correction_km: 1.0,   // Ignore corrections < 1km
            enabled: true,
            navigation_constant: default_navigation_constant(),
        }
    }
}

/// Kill envelope parameters
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct KillEnvelopeConfig {
    pub seeker_range_km: f64,
    pub kill_radius_km: f64,
    pub base_pk: f64,
}

/// Weights for Pk factor contributions using weighted log-odds calculation
/// Higher weight = factor has more impact on final Pk
/// Weights are relative - they get normalized during calculation
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PkWeights {
    // === Critical Factors (weight ~2.0) ===
    /// Timing synchronization - must arrive at intercept point when missile does
    pub timing_sync: f64,

    /// Sensor track quality - fire control accuracy determines guidance precision
    pub track_quality: f64,

    /// Prediction error - how well intercept point matches actual missile position
    pub prediction_error: f64,

    // === Moderate Factors (weight ~1.0) ===
    /// Countermeasures - decoys and jamming that confuse the seeker
    pub countermeasures: f64,

    /// Closure speed - combined approach velocity affects seeker acquisition time
    pub closure_speed: f64,

    /// Aspect angle - intercept geometry (head-on vs tail chase)
    pub aspect_angle: f64,

    // === Low Priority (weight ~0.3) ===
    /// Energy state - remaining fuel/thruster capacity for terminal corrections
    pub energy_state: f64,

    // === Scaling ===
    /// Overall penalty severity in logit space
    /// Higher = factors have more impact on final Pk
    pub severity_scale: f64,
}

impl Default for PkWeights {
    fn default() -> Self {
        Self {
            // Critical factors
            timing_sync: 2.0,
            track_quality: 2.0,
            prediction_error: 2.0,

            // Moderate factors
            countermeasures: 1.0,
            closure_speed: 1.0,
            aspect_angle: 0.8,

            // Low priority
            energy_state: 0.3,

            // Scaling
            severity_scale: 4.0,
        }
    }
}

impl PkWeights {
    /// Load PkWeights from config/simulation.toml
    /// Falls back to defaults if file doesn't exist or is invalid
    pub fn load(config_dir: &Path) -> Self {
        let config_path = config_dir.join("simulation.toml");

        if !config_path.exists() {
            return Self::default();
        }

        match fs::read_to_string(&config_path) {
            Ok(contents) => {
                // Parse the TOML file - look for [pk_weights] section
                match toml::from_str::<SimulationConfig>(&contents) {
                    Ok(config) => config.pk_weights.unwrap_or_default(),
                    Err(e) => {
                        eprintln!(
                            "Warning: Failed to parse simulation.toml: {}, using defaults",
                            e
                        );
                        Self::default()
                    }
                }
            }
            Err(e) => {
                eprintln!(
                    "Warning: Failed to read simulation.toml: {}, using defaults",
                    e
                );
                Self::default()
            }
        }
    }
}

/// Physics simulation settings
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PhysicsConfig {
    /// Number of physics sub-steps per frame for high-precision intercept calculations
    /// Higher values = more precise but slower. 100 gives ~0.16ms precision at 60fps.
    pub sub_steps: u32,

    /// Minimum sub-steps to use when no interceptors are in terminal phase
    /// Saves CPU when precision isn't needed
    pub min_sub_steps: u32,

    /// Distance threshold (km) for using full sub-steps
    /// When interceptor is within this distance of target, use full sub_steps
    pub precision_distance_km: f64,
}

impl Default for PhysicsConfig {
    fn default() -> Self {
        Self {
            sub_steps: 100,              // 100 sub-steps = ~0.16ms precision at 60fps
            min_sub_steps: 1,            // Normal precision when not needed
            precision_distance_km: 50.0, // Use full precision within 50km of target
        }
    }
}

impl PhysicsConfig {
    /// Load PhysicsConfig from config/simulation.toml
    pub fn load(config_dir: &Path) -> Self {
        let config_path = config_dir.join("simulation.toml");

        if !config_path.exists() {
            return Self::default();
        }

        match fs::read_to_string(&config_path) {
            Ok(contents) => match toml::from_str::<SimulationConfig>(&contents) {
                Ok(config) => config.physics.unwrap_or_default(),
                Err(e) => {
                    eprintln!("Warning: Failed to parse simulation.toml physics config: {}, using defaults", e);
                    Self::default()
                }
            },
            Err(e) => {
                eprintln!(
                    "Warning: Failed to read simulation.toml: {}, using defaults",
                    e
                );
                Self::default()
            }
        }
    }
}

/// Top-level simulation configuration (from simulation.toml)
#[derive(Debug, Clone, Deserialize)]
struct SimulationConfig {
    /// Pk calculation weights
    pk_weights: Option<PkWeights>,
    /// Physics simulation settings
    physics: Option<PhysicsConfig>,
}

// ============================================================================
// Sensor Configuration
// ============================================================================

/// Radar frequency bands with different characteristics
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum RadarBand {
    #[serde(rename = "L")]
    L, // 1-2 GHz: Long range, low attenuation, lower resolution
    #[serde(rename = "S")]
    S, // 2-4 GHz: Good range, moderate attenuation, good resolution
    #[serde(rename = "C")]
    C, // 4-8 GHz: Medium range, moderate-high attenuation
    #[serde(rename = "X")]
    X, // 8-12 GHz: High resolution, higher attenuation, fire control
    #[serde(rename = "Ku")]
    Ku, // 12-18 GHz: Very high resolution, high attenuation
}

impl RadarBand {
    /// Atmospheric attenuation coefficient (dB/km at sea level)
    pub fn attenuation_coefficient(&self) -> f64 {
        match self {
            RadarBand::L => 0.005,  // Very low attenuation
            RadarBand::S => 0.010,  // Low attenuation
            RadarBand::C => 0.015,  // Moderate attenuation
            RadarBand::X => 0.020,  // Higher attenuation
            RadarBand::Ku => 0.030, // High attenuation
        }
    }

    /// Resolution/quality multiplier (higher frequency = better resolution)
    pub fn quality_multiplier(&self) -> f64 {
        match self {
            RadarBand::L => 0.85,  // Lower resolution
            RadarBand::S => 0.92,  // Good resolution
            RadarBand::C => 0.96,  // Better resolution
            RadarBand::X => 1.00,  // Excellent resolution (baseline)
            RadarBand::Ku => 1.05, // Outstanding resolution
        }
    }
}

impl Default for RadarBand {
    fn default() -> Self {
        RadarBand::X // Default to X-band (most common for fire control)
    }
}

/// Multi-band radar configuration - defines which bands to use for each mode
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MultiBandConfig {
    /// Band to use in Search mode
    pub search_band: RadarBand,

    /// Band to use in Track mode
    pub track_band: RadarBand,

    /// Band to use in FireControl mode
    pub fire_control_band: RadarBand,
}

impl Default for MultiBandConfig {
    fn default() -> Self {
        // Conservative single X-band default
        Self {
            search_band: RadarBand::X,
            track_band: RadarBand::X,
            fire_control_band: RadarBand::X,
        }
    }
}

impl MultiBandConfig {
    /// Get the appropriate band for a given radar mode
    pub fn get_band_for_mode(&self, mode: RadarMode) -> RadarBand {
        match mode {
            RadarMode::Search => self.search_band,
            RadarMode::Track => self.track_band,
            RadarMode::FireControl => self.fire_control_band,
        }
    }

    /// Check if this configuration uses multiple bands
    pub fn is_multi_band(&self) -> bool {
        self.search_band != self.track_band || self.track_band != self.fire_control_band
    }
}

/// Radar scanning mechanism type
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum RadarType {
    #[serde(rename = "mechanical")]
    Mechanical, // Traditional rotating antenna
    #[serde(rename = "phased_array")]
    PhasedArray, // Electronically steered beam
}

/// Radar operating modes with different characteristics
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum RadarMode {
    Search,      // Wide area surveillance
    Track,       // Tracking specific targets
    FireControl, // Precision guidance for intercept
}

impl RadarMode {
    /// Whether this mode uses slant range (true) or ground range (false)
    pub fn uses_slant_range(&self) -> bool {
        match self {
            RadarMode::Search => false,     // Ground range for search
            RadarMode::Track => true,       // Slant range for tracking
            RadarMode::FireControl => true, // Slant range for fire control
        }
    }
}

/// Role that determines sensor behavior and mode preferences
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum SensorRole {
    #[serde(rename = "surveillance")]
    Surveillance, // Wide-area search, prefers Search mode
    #[serde(rename = "tracking")]
    Tracking, // Track maintenance, prefers Track mode
    #[serde(rename = "fire_control")]
    FireControl, // Terminal guidance, prefers FireControl mode
    #[serde(rename = "multi_role")]
    MultiRole, // Can perform any role
}

impl Default for SensorRole {
    fn default() -> Self {
        SensorRole::MultiRole
    }
}

/// Configuration for a single sensor on a multi-sensor platform
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PlatformSensorConfig {
    /// Reference to sensor configuration by name
    pub config_name: String,

    /// Role of this sensor on the platform
    #[serde(default)]
    pub role: SensorRole,

    /// Azimuth center direction in degrees (0 = North, 90 = East)
    #[serde(default)]
    pub azimuth_center_deg: f64,

    /// Optional override for azimuth coverage (uses sensor's detection config if not set)
    #[serde(default)]
    pub azimuth_coverage_override_deg: Option<f64>,
}

/// Sensor detection parameters
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SensorDetectionConfig {
    pub detection_range_km: f64,
    pub azimuth_coverage_deg: f64,
    pub elevation_min_deg: f64,
    pub elevation_max_deg: f64,
    /// Legacy single-band field (for backward compatibility)
    #[serde(default)]
    pub radar_band: RadarBand,
    /// NEW: Multi-band configuration (optional)
    /// If specified, radar_band is ignored and bands are selected by mode
    #[serde(default)]
    pub multi_band: Option<MultiBandConfig>,
    pub radar_type: RadarType,
    /// Track mode range multiplier (multiplier on base range when in Track mode)
    #[serde(default = "default_track_multiplier")]
    pub track_range_multiplier: f64,
    /// FireControl mode range multiplier (multiplier on base range when in FireControl mode)
    #[serde(default = "default_fc_multiplier")]
    pub fire_control_range_multiplier: f64,
    /// Track mode azimuth coverage multiplier (multiplier on base azimuth_coverage_deg)
    #[serde(default = "default_track_azimuth_multiplier")]
    pub track_azimuth_multiplier: f64,
    /// FireControl mode azimuth coverage multiplier (multiplier on base azimuth_coverage_deg)
    #[serde(default = "default_fc_azimuth_multiplier")]
    pub fire_control_azimuth_multiplier: f64,
}

fn default_track_multiplier() -> f64 {
    2.0 // Default 2× base range in Track mode
}

fn default_fc_multiplier() -> f64 {
    3.0 // Default 3× base range in FireControl mode
}

fn default_track_azimuth_multiplier() -> f64 {
    1.0 // Default: same as search mode (backward compatible)
}

fn default_fc_azimuth_multiplier() -> f64 {
    1.0 // Default: same as search mode (backward compatible)
}

impl SensorDetectionConfig {
    /// Get range multiplier for the given radar mode
    pub fn get_range_multiplier(&self, mode: RadarMode) -> f64 {
        match mode {
            RadarMode::Search => 1.0, // Base range
            RadarMode::Track => self.track_range_multiplier,
            RadarMode::FireControl => self.fire_control_range_multiplier,
        }
    }

    /// Get the radar band for a given mode, accounting for multi-band support
    pub fn get_band_for_mode(&self, mode: RadarMode) -> RadarBand {
        if let Some(multi_band) = &self.multi_band {
            multi_band.get_band_for_mode(mode)
        } else {
            // Legacy single-band behavior
            self.radar_band
        }
    }

    /// Check if this sensor supports multi-band operation
    pub fn is_multi_band(&self) -> bool {
        self.multi_band
            .as_ref()
            .map_or(false, |mb| mb.is_multi_band())
    }

    /// Get azimuth coverage multiplier for the given radar mode
    pub fn get_azimuth_multiplier(&self, mode: RadarMode) -> f64 {
        match mode {
            RadarMode::Search => 1.0, // Full base coverage
            RadarMode::Track => self.track_azimuth_multiplier,
            RadarMode::FireControl => self.fire_control_azimuth_multiplier,
        }
    }

    /// Get effective azimuth coverage (in degrees) for a given radar mode
    pub fn get_effective_azimuth_coverage(&self, mode: RadarMode) -> f64 {
        self.azimuth_coverage_deg * self.get_azimuth_multiplier(mode)
    }
}

/// Sensor tracking parameters
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SensorTrackingConfig {
    pub max_simultaneous_tracks: u32,
    pub track_update_rate_hz: f64,
    pub minimum_rcs_dbsm: f64,
    #[serde(default = "default_max_fc_tracks")]
    pub max_fire_control_tracks: u32,
    #[serde(default = "default_search_dwell_ms")]
    pub search_dwell_time_ms: f64,
    #[serde(default = "default_track_dwell_ms")]
    pub track_dwell_time_ms: f64,
    #[serde(default = "default_fc_dwell_ms")]
    pub fire_control_dwell_time_ms: f64,
}

fn default_max_fc_tracks() -> u32 {
    2 // Conservative default for fire control tracks
}

fn default_search_dwell_ms() -> f64 {
    20.0 // 20ms per target in search mode
}

fn default_track_dwell_ms() -> f64 {
    100.0 // 100ms per target in track mode
}

fn default_fc_dwell_ms() -> f64 {
    500.0 // 500ms per target in fire control mode
}

impl SensorTrackingConfig {
    pub fn get_dwell_time_sec(&self, mode: RadarMode) -> f64 {
        match mode {
            RadarMode::Search => self.search_dwell_time_ms / 1000.0,
            RadarMode::Track => self.track_dwell_time_ms / 1000.0,
            RadarMode::FireControl => self.fire_control_dwell_time_ms / 1000.0,
        }
    }
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

        Ok(Self {
            configs,
            default_config,
        })
    }

    fn load_config_file(path: &Path) -> Result<SensorConfig, ConfigError> {
        let contents = fs::read_to_string(path).map_err(|e| ConfigError::IoError(e.to_string()))?;
        toml::from_str(&contents).map_err(|e| ConfigError::ParseError(e.to_string()))
    }

    fn normalize_name(name: &str) -> String {
        name.to_lowercase().replace([' ', '-', '/'], "_")
    }

    /// Get configuration for a sensor by name
    pub fn get_by_name(&self, sensor_name: &str) -> &SensorConfig {
        let normalized = Self::normalize_name(sensor_name);
        self.configs
            .get(&normalized)
            .unwrap_or(&self.default_config)
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
                multi_band: None,
                radar_type: RadarType::Mechanical,
                track_range_multiplier: 2.0,
                fire_control_range_multiplier: 3.0,
                track_azimuth_multiplier: 1.0,
                fire_control_azimuth_multiplier: 1.0,
            },
            tracking: SensorTrackingConfig {
                max_simultaneous_tracks: 20,
                track_update_rate_hz: 5.0,
                minimum_rcs_dbsm: 0.0,
                max_fire_control_tracks: 2,
                search_dwell_time_ms: 20.0,
                track_dwell_time_ms: 100.0,
                fire_control_dwell_time_ms: 500.0,
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
// Platform Configuration
// ============================================================================

/// Launcher capabilities and magazine
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LauncherConfig {
    /// Name of interceptor config to use (e.g., "THAAD Interceptor", "PAC-3 MSE")
    pub interceptor_type: String,
    /// Legacy single-sensor field (for backward compatibility)
    /// Name of sensor config to use (e.g., "AN/TPY-2", "AN/MPQ-65")
    #[serde(default)]
    pub sensor_config_name: String,
    /// NEW: Multi-sensor support
    #[serde(default)]
    pub sensors: Vec<PlatformSensorConfig>,
    /// Maximum number of interceptors this platform can hold
    pub max_interceptors: u32,
    /// Time to reload magazine after depletion (minutes)
    #[serde(default)]
    pub reload_time_minutes: f64,
    /// Maximum number of interceptors that can be launched simultaneously
    #[serde(default = "default_max_salvo")]
    pub max_salvo_size: u32,
    /// Platform engagement range (may be limited by fire control quality, not just interceptor)
    pub engagement_range_km: f64,
}

impl LauncherConfig {
    /// Get sensors for this platform (handles legacy and new formats)
    pub fn get_sensors(&self) -> Vec<PlatformSensorConfig> {
        if !self.sensors.is_empty() {
            // New format: use sensors array
            self.sensors.clone()
        } else if !self.sensor_config_name.is_empty() {
            // Legacy format: convert single sensor to array
            vec![PlatformSensorConfig {
                config_name: self.sensor_config_name.clone(),
                role: SensorRole::MultiRole,
                azimuth_center_deg: 0.0,
                azimuth_coverage_override_deg: None,
            }]
        } else {
            // No sensors configured
            vec![]
        }
    }
}

fn default_max_salvo() -> u32 {
    2 // Conservative default
}

/// Complete platform/launcher configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PlatformConfig {
    pub system: SystemInfo,
    pub launcher: LauncherConfig,
}

/// Registry holding all platform configurations
pub struct PlatformConfigRegistry {
    configs: HashMap<String, PlatformConfig>,
    default_config: PlatformConfig,
}

impl PlatformConfigRegistry {
    /// Load configurations from a directory
    pub fn load(config_dir: &Path) -> Result<Self, ConfigError> {
        let mut configs = HashMap::new();
        let platform_dir = config_dir.join("platform");

        // Load default config first
        let default_path = platform_dir.join("default.toml");
        let default_config = if default_path.exists() {
            match Self::load_config_file(&default_path) {
                Ok(cfg) => cfg,
                Err(e) => {
                    eprintln!("Warning: Failed to load platform/default.toml: {}, using hardcoded defaults", e);
                    Self::hardcoded_default()
                }
            }
        } else {
            eprintln!("Warning: platform/default.toml not found, using hardcoded defaults");
            Self::hardcoded_default()
        };

        // Load all .toml files from platform directory
        if let Ok(entries) = fs::read_dir(&platform_dir) {
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

        Ok(Self {
            configs,
            default_config,
        })
    }

    fn load_config_file(path: &Path) -> Result<PlatformConfig, ConfigError> {
        let contents = fs::read_to_string(path)
            .map_err(|e| ConfigError::IoError(format!("Failed to read {:?}: {}", path, e)))?;

        toml::from_str(&contents)
            .map_err(|e| ConfigError::ParseError(format!("Failed to parse {:?}: {}", path, e)))
    }

    fn normalize_name(name: &str) -> String {
        name.to_lowercase().replace(' ', "_").replace('-', "_")
    }

    fn hardcoded_default() -> PlatformConfig {
        PlatformConfig {
            system: SystemInfo {
                name: "Generic Platform".to_string(),
                description: "Default platform configuration".to_string(),
                country: None,
                nato_designation: None,
            },
            launcher: LauncherConfig {
                interceptor_type: "Default Interceptor".to_string(),
                sensor_config_name: "default".to_string(),
                sensors: vec![],
                max_interceptors: 8,
                reload_time_minutes: 30.0,
                max_salvo_size: 2,
                engagement_range_km: 100.0,
            },
        }
    }

    pub fn with_defaults() -> Self {
        Self {
            configs: HashMap::new(),
            default_config: Self::hardcoded_default(),
        }
    }

    pub fn get_by_name(&self, platform_name: &str) -> &PlatformConfig {
        let normalized = Self::normalize_name(platform_name);
        self.configs
            .get(&normalized)
            .unwrap_or(&self.default_config)
    }

    pub fn get_by_defense_type(&self, defense_type: DefenseType) -> &PlatformConfig {
        self.get_by_name(defense_type.name())
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

        Ok(Self {
            configs,
            default_config,
        })
    }

    fn load_config_file(path: &Path) -> Result<InterceptorConfig, ConfigError> {
        let contents = fs::read_to_string(path).map_err(|e| ConfigError::IoError(e.to_string()))?;
        toml::from_str(&contents).map_err(|e| ConfigError::ParseError(e.to_string()))
    }

    fn normalize_name(name: &str) -> String {
        name.to_lowercase().replace([' ', '-'], "_")
    }

    /// Get configuration for an interceptor by name
    pub fn get_by_name(&self, interceptor_name: &str) -> &InterceptorConfig {
        let normalized = Self::normalize_name(interceptor_name);
        self.configs
            .get(&normalized)
            .unwrap_or(&self.default_config)
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
                guidance_type: GuidanceType::Active,
                midcourse_guidance: MidcourseGuidanceConfig::default(),
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
    GEO, // Geosynchronous
    HEO, // Highly Elliptical Orbit
    LEO, // Low Earth Orbit
    MEO, // Medium Earth Orbit
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

        Ok(Self {
            configs,
            default_config,
        })
    }

    fn load_config_file(path: &Path) -> Result<SatelliteConfig, ConfigError> {
        let contents = fs::read_to_string(path).map_err(|e| ConfigError::IoError(e.to_string()))?;
        toml::from_str(&contents).map_err(|e| ConfigError::ParseError(e.to_string()))
    }

    fn normalize_name(name: &str) -> String {
        name.to_lowercase().replace([' ', '-'], "_")
    }

    /// Get configuration for a satellite by name
    pub fn get_by_name(&self, satellite_name: &str) -> &SatelliteConfig {
        let normalized = Self::normalize_name(satellite_name);
        self.configs
            .get(&normalized)
            .unwrap_or(&self.default_config)
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
    ICBM, // Intercontinental Ballistic Missile (>5,500 km)
    SLBM, // Submarine-Launched Ballistic Missile
    IRBM, // Intermediate-Range Ballistic Missile (3,000-5,500 km)
    MRBM, // Medium-Range Ballistic Missile (1,000-3,000 km)
    SRBM, // Short-Range Ballistic Missile (<1,000 km)
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
                    eprintln!(
                        "Warning: Failed to load default.toml: {}, using hardcoded defaults",
                        e
                    );
                    Self::hardcoded_default()
                }
            }
        } else {
            eprintln!("Warning: default.toml not found, using hardcoded defaults");
            Self::hardcoded_default()
        };

        // Recursively load all .toml files from missiles directory and subdirectories
        Self::load_configs_recursive(&missiles_dir, &mut configs);

        Ok(Self {
            configs,
            default_config,
        })
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
        let contents = fs::read_to_string(path).map_err(|e| ConfigError::IoError(e.to_string()))?;

        toml::from_str(&contents).map_err(|e| ConfigError::ParseError(e.to_string()))
    }

    /// Normalize a missile name for lookup (lowercase, replace spaces/hyphens with underscores)
    fn normalize_name(name: &str) -> String {
        name.to_lowercase().replace([' ', '-'], "_")
    }

    /// Get configuration for a missile by name
    /// Matches missile names like "Shahab-3 #1" to config "Shahab-3"
    pub fn get_by_name(&self, missile_name: &str) -> &MissileConfig {
        // Strip any numbering suffix like " #1", " #2", etc.
        let base_name = missile_name.split(" #").next().unwrap_or(missile_name);

        let normalized = Self::normalize_name(base_name);

        self.configs
            .get(&normalized)
            .unwrap_or(&self.default_config)
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
        config.trajectory.flight_time_base_sec
            + range_km * config.trajectory.flight_time_range_factor
    }

    /// Calculate apogee for a given range using the missile type's config
    pub fn calculate_apogee(&self, missile_type: MissileType, range_km: f64) -> f64 {
        let config = self.get(missile_type);
        config.trajectory.apogee_base_km + range_km * config.trajectory.apogee_range_factor
    }

    /// Calculate flight time for a given range using the missile type's config
    pub fn calculate_flight_time(&self, missile_type: MissileType, range_km: f64) -> f64 {
        let config = self.get(missile_type);
        config.trajectory.flight_time_base_sec
            + range_km * config.trajectory.flight_time_range_factor
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
