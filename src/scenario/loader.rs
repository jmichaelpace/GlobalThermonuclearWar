use crate::simulation::{Affiliation, DefenseType, SensorType, SimulationEngine};
use crate::types::GeoCoord;
use serde::{Deserialize, Serialize};
use std::fs;
use std::path::Path;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ScenarioFile {
    pub metadata: ScenarioMetadata,
    #[serde(default)]
    pub defense_units: Vec<DefenseUnitConfig>,
    #[serde(default)]
    pub radar_stations: Vec<RadarStationConfig>,
    #[serde(default)]
    pub satellites: Vec<SatelliteConfig>,
    #[serde(default)]
    pub missiles: Vec<MissileConfig>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ScenarioMetadata {
    pub id: String,
    pub name: String,
    pub description: String,
    pub region: String,
    pub center_lat: f64,
    pub center_lon: f64,
    pub zoom: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DefenseUnitConfig {
    pub name: String,
    pub affiliation: String,
    pub lat: f64,
    pub lon: f64,
    #[serde(rename = "type")]
    pub defense_type: String,
    pub interceptors: u32,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RadarStationConfig {
    pub name: String,
    pub affiliation: String,
    pub lat: f64,
    pub lon: f64,
    pub range_km: f64,
    /// Sensor configuration name (references config/sensors/*.toml filename without extension)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub sensor_config: Option<String>,
    /// Azimuth direction the radar faces (degrees, 0=North, 90=East)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub facing_deg: Option<f64>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SatelliteConfig {
    pub name: String,
    pub affiliation: String,
    pub lat: f64,
    pub lon: f64,
    pub altitude_km: f64,
    pub sensor_type: String,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MissileConfig {
    pub name: String,
    pub affiliation: String,
    pub origin_lat: f64,
    pub origin_lon: f64,
    pub target_lat: f64,
    pub target_lon: f64,
    pub launch_delay_sec: f64,
}

impl ScenarioFile {
    /// Load a scenario from a TOML file
    pub fn load_from_file<P: AsRef<Path>>(path: P) -> Result<Self, Box<dyn std::error::Error>> {
        let contents = fs::read_to_string(path)?;
        let scenario: ScenarioFile = toml::from_str(&contents)?;
        Ok(scenario)
    }

    /// Load scenario into the simulation engine
    pub fn load_into_engine(&self, engine: &mut SimulationEngine) {
        engine.reset();

        // Load defense units
        for unit in &self.defense_units {
            let affiliation = parse_affiliation(&unit.affiliation);
            let defense_type = parse_defense_type(&unit.defense_type);
            let position = GeoCoord::new(unit.lat, unit.lon);

            engine.add_defense_unit(
                unit.name.clone(),
                affiliation,
                position,
                defense_type,
                unit.interceptors,
            );
        }

        // Load radar stations
        for radar in &self.radar_stations {
            let affiliation = parse_affiliation(&radar.affiliation);
            let position = GeoCoord::new(radar.lat, radar.lon);

            engine.add_radar_station_with_config(
                radar.name.clone(),
                affiliation,
                position,
                radar.range_km,
                radar.sensor_config.clone(),
                radar.facing_deg,
            );
        }

        // Load satellites
        for satellite in &self.satellites {
            let affiliation = parse_affiliation(&satellite.affiliation);
            let position = GeoCoord::new(satellite.lat, satellite.lon);
            let sensor_type = parse_sensor_type(&satellite.sensor_type);

            engine.add_satellite(
                satellite.name.clone(),
                affiliation,
                position,
                satellite.altitude_km,
                sensor_type,
            );
        }

        // Load missiles
        for missile in &self.missiles {
            let affiliation = parse_affiliation(&missile.affiliation);
            let origin = GeoCoord::new(missile.origin_lat, missile.origin_lon);
            let target = GeoCoord::new(missile.target_lat, missile.target_lon);

            engine.add_missile(
                missile.name.clone(),
                affiliation,
                origin,
                target,
                missile.launch_delay_sec,
            );
        }
    }
}

fn parse_affiliation(s: &str) -> Affiliation {
    match s.to_lowercase().as_str() {
        "friendly" => Affiliation::Friendly,
        "hostile" => Affiliation::Hostile,
        "neutral" => Affiliation::Neutral,
        _ => Affiliation::Neutral,
    }
}

fn parse_defense_type(s: &str) -> DefenseType {
    match s {
        "THAAD" => DefenseType::THAAD,
        "Aegis" => DefenseType::Aegis,
        "Patriot" => DefenseType::Patriot,
        "GBI" => DefenseType::GBI,
        "IronDome" => DefenseType::IronDome,
        "Arrow3" => DefenseType::Arrow3,
        "DavidsSling" => DefenseType::DavidsSling,
        "S400" => DefenseType::S400,
        _ => DefenseType::THAAD, // Default
    }
}

fn parse_sensor_type(s: &str) -> SensorType {
    match s {
        "Infrared" => SensorType::Infrared,
        "Radar" => SensorType::Radar,
        _ => SensorType::Infrared, // Default
    }
}

/// Load all scenarios from the scenarios directory
pub fn load_all_scenarios() -> Result<Vec<ScenarioFile>, Box<dyn std::error::Error>> {
    let mut scenarios = Vec::new();
    let scenarios_dir = Path::new("scenarios");

    if !scenarios_dir.exists() {
        eprintln!("Warning: scenarios directory not found");
        return Ok(scenarios);
    }

    for entry in fs::read_dir(scenarios_dir)? {
        let entry = entry?;
        let path = entry.path();

        if path.extension().and_then(|s| s.to_str()) == Some("toml") {
            match ScenarioFile::load_from_file(&path) {
                Ok(scenario) => {
                    println!("Loaded scenario: {}", scenario.metadata.name);
                    scenarios.push(scenario);
                }
                Err(e) => {
                    eprintln!("Failed to load scenario from {:?}: {}", path, e);
                }
            }
        }
    }

    Ok(scenarios)
}
