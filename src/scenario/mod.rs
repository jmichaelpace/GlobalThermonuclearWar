use crate::simulation::SimulationEngine;
use crate::types::GeoCoord;

pub mod builder;
pub mod loader;
pub use builder::{
    classify_missile_range, BuilderTool, DraftCategory, ScenarioDraft, AFFILIATIONS, DEFENSE_TYPES,
    SATELLITE_SENSOR_TYPES,
};
pub use loader::{load_all_scenarios, ScenarioFile};

/// A scenario definition loaded from a TOML file
#[derive(Clone)]
pub struct ScenarioDefinition {
    pub id: String,
    pub name: String,
    pub description: String,
    pub region: String,
    pub center: GeoCoord,
    pub zoom: f64,
    pub file: ScenarioFile,
}

impl ScenarioDefinition {
    /// Load this scenario into the simulation engine
    pub fn load(&self, engine: &mut SimulationEngine) {
        self.file.load_into_engine(engine);
    }
}

/// Get all available scenarios from TOML files in scenarios/ directory
pub fn get_scenarios() -> Vec<ScenarioDefinition> {
    let mut scenarios = Vec::new();

    match load_all_scenarios() {
        Ok(scenario_files) => {
            for file in scenario_files {
                scenarios.push(ScenarioDefinition {
                    id: file.metadata.id.clone(),
                    name: file.metadata.name.clone(),
                    description: file.metadata.description.clone(),
                    region: file.metadata.region.clone(),
                    center: GeoCoord::new(file.metadata.center_lat, file.metadata.center_lon),
                    zoom: file.metadata.zoom,
                    file,
                });
            }
        }
        Err(e) => {
            eprintln!("Error loading scenarios: {}", e);
            eprintln!("Make sure the 'scenarios/' directory exists with .toml files");
        }
    }

    if scenarios.is_empty() {
        eprintln!("Warning: No scenarios loaded! Create .toml files in scenarios/ directory");
    }

    scenarios
}
