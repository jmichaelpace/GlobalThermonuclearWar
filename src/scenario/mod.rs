use crate::map::GeoCoord;
use crate::simulation::{Affiliation, DefenseType, SensorType, SimulationEngine};

/// A predefined scenario
pub struct ScenarioDefinition {
    pub id: &'static str,
    pub name: &'static str,
    pub description: &'static str,
    pub region: &'static str,
    pub center: GeoCoord,
    pub zoom: f64,
}

impl ScenarioDefinition {
    /// Load this scenario into the simulation engine
    pub fn load(&self, engine: &mut SimulationEngine) {
        match self.id {
            "pacific_theater" => load_pacific_theater(engine),
            "middle_east" => load_middle_east(engine),
            "european_theater" => load_european_theater(engine),
            "north_atlantic" => load_north_atlantic(engine),
            "demo" => load_demo_scenario(engine),
            _ => load_demo_scenario(engine),
        }
    }
}

/// Get all available scenarios
pub fn get_scenarios() -> Vec<ScenarioDefinition> {
    vec![
        ScenarioDefinition {
            id: "pacific_theater",
            name: "Pacific Theater",
            description: "North Korean ICBM launches against US West Coast",
            region: "Pacific",
            center: GeoCoord::new(40.0, 180.0),
            zoom: 2.5,
        },
        ScenarioDefinition {
            id: "middle_east",
            name: "Middle East Crisis",
            description: "Iranian missile strikes against regional targets",
            region: "Middle East",
            center: GeoCoord::new(30.0, 45.0),
            zoom: 4.0,
        },
        ScenarioDefinition {
            id: "european_theater",
            name: "European Theater",
            description: "Russian strikes against NATO targets",
            region: "Europe",
            center: GeoCoord::new(52.0, 20.0),
            zoom: 3.5,
        },
        ScenarioDefinition {
            id: "north_atlantic",
            name: "North Atlantic",
            description: "Submarine-launched ballistic missiles",
            region: "Atlantic",
            center: GeoCoord::new(55.0, -30.0),
            zoom: 3.0,
        },
        ScenarioDefinition {
            id: "demo",
            name: "Demo Scenario",
            description: "Multi-region demonstration with various threats",
            region: "Global",
            center: GeoCoord::new(20.0, 0.0),
            zoom: 2.0,
        },
    ]
}

/// Pacific Theater - North Korea vs US
fn load_pacific_theater(engine: &mut SimulationEngine) {
    engine.reset();

    // US Defense Assets
    engine.add_defense_unit(
        "THAAD Battery Alpha".into(),
        Affiliation::Friendly,
        GeoCoord::new(37.5, -122.0), // California
        DefenseType::THAAD,
        48,
    );

    engine.add_defense_unit(
        "THAAD Battery Bravo".into(),
        Affiliation::Friendly,
        GeoCoord::new(47.6, -122.3), // Seattle area
        DefenseType::THAAD,
        48,
    );

    engine.add_defense_unit(
        "GBI Site Fort Greely".into(),
        Affiliation::Friendly,
        GeoCoord::new(64.0, -146.0), // Alaska
        DefenseType::GBI,
        44,
    );

    engine.add_defense_unit(
        "GBI Site Vandenberg".into(),
        Affiliation::Friendly,
        GeoCoord::new(34.7, -120.5), // California
        DefenseType::GBI,
        8,
    );

    // Naval Assets
    engine.add_defense_unit(
        "USS Shiloh (Aegis)".into(),
        Affiliation::Friendly,
        GeoCoord::new(38.0, 132.0), // Sea of Japan
        DefenseType::Aegis,
        96,
    );

    engine.add_defense_unit(
        "USS John Paul Jones (Aegis)".into(),
        Affiliation::Friendly,
        GeoCoord::new(35.0, -165.0), // Central Pacific
        DefenseType::Aegis,
        96,
    );

    // Early Warning
    engine.add_radar_station(
        "Cobra Dane".into(),
        Affiliation::Friendly,
        GeoCoord::new(52.7, 174.1), // Shemya, Alaska
        2500.0,
    );

    engine.add_radar_station(
        "Sea-Based X-Band".into(),
        Affiliation::Friendly,
        GeoCoord::new(45.0, 170.0), // Pacific
        2000.0,
    );

    // Satellites
    engine.add_satellite(
        "SBIRS GEO-2".into(),
        Affiliation::Friendly,
        GeoCoord::new(0.0, 120.0), // Western Pacific
        35786.0,
        SensorType::Infrared,
    );

    engine.add_satellite(
        "SBIRS GEO-4".into(),
        Affiliation::Friendly,
        GeoCoord::new(0.0, -135.0), // Eastern Pacific
        35786.0,
        SensorType::Infrared,
    );

    // North Korean Missiles
    engine.add_missile(
        "Hwasong-15 #1".into(),
        Affiliation::Hostile,
        GeoCoord::new(39.0, 125.5), // Pyongyang area
        GeoCoord::new(37.5, -122.0), // San Francisco
        30.0,
    );

    engine.add_missile(
        "Hwasong-15 #2".into(),
        Affiliation::Hostile,
        GeoCoord::new(39.0, 125.5),
        GeoCoord::new(47.6, -122.3), // Seattle
        60.0,
    );

    engine.add_missile(
        "Hwasong-15 #3".into(),
        Affiliation::Hostile,
        GeoCoord::new(40.0, 127.0), // Northern NK
        GeoCoord::new(33.7, -118.2), // Los Angeles
        90.0,
    );

    engine.add_missile(
        "Hwasong-14 #1".into(),
        Affiliation::Hostile,
        GeoCoord::new(39.5, 126.0),
        GeoCoord::new(21.3, -157.8), // Honolulu
        120.0,
    );
}

/// Middle East - Iran vs Israel/Gulf States
fn load_middle_east(engine: &mut SimulationEngine) {
    engine.reset();

    // Israeli Defense
    engine.add_defense_unit(
        "Iron Dome - Tel Aviv".into(),
        Affiliation::Friendly,
        GeoCoord::new(32.0, 34.8),
        DefenseType::IronDome,
        100,
    );

    engine.add_defense_unit(
        "Arrow 3 Battery".into(),
        Affiliation::Friendly,
        GeoCoord::new(31.5, 34.5),
        DefenseType::Arrow3,
        24,
    );

    engine.add_defense_unit(
        "David's Sling".into(),
        Affiliation::Friendly,
        GeoCoord::new(32.8, 35.0),
        DefenseType::DavidsSling,
        48,
    );

    // Saudi Defense
    engine.add_defense_unit(
        "Patriot - Riyadh".into(),
        Affiliation::Friendly,
        GeoCoord::new(24.7, 46.7),
        DefenseType::Patriot,
        64,
    );

    engine.add_defense_unit(
        "THAAD - UAE".into(),
        Affiliation::Friendly,
        GeoCoord::new(24.5, 54.5), // Abu Dhabi
        DefenseType::THAAD,
        48,
    );

    // US Naval Support
    engine.add_defense_unit(
        "USS Carney (Aegis)".into(),
        Affiliation::Friendly,
        GeoCoord::new(26.0, 52.0), // Persian Gulf
        DefenseType::Aegis,
        96,
    );

    // Radar
    engine.add_radar_station(
        "AN/TPY-2 Israel".into(),
        Affiliation::Friendly,
        GeoCoord::new(30.5, 34.9),
        1000.0,
    );

    // Iranian Missiles
    engine.add_missile(
        "Shahab-3 #1".into(),
        Affiliation::Hostile,
        GeoCoord::new(35.7, 51.4), // Tehran
        GeoCoord::new(32.0, 34.8), // Tel Aviv
        20.0,
    );

    engine.add_missile(
        "Shahab-3 #2".into(),
        Affiliation::Hostile,
        GeoCoord::new(34.3, 47.1), // Western Iran
        GeoCoord::new(32.0, 34.8),
        40.0,
    );

    engine.add_missile(
        "Emad #1".into(),
        Affiliation::Hostile,
        GeoCoord::new(35.7, 51.4),
        GeoCoord::new(24.7, 46.7), // Riyadh
        60.0,
    );

    engine.add_missile(
        "Sejjil #1".into(),
        Affiliation::Hostile,
        GeoCoord::new(36.0, 52.0),
        GeoCoord::new(24.5, 54.5), // Abu Dhabi
        80.0,
    );
}

/// European Theater - Russia vs NATO
fn load_european_theater(engine: &mut SimulationEngine) {
    engine.reset();

    // NATO Defense - Poland
    engine.add_defense_unit(
        "Aegis Ashore Poland".into(),
        Affiliation::Friendly,
        GeoCoord::new(53.8, 21.1), // Redzikowo
        DefenseType::Aegis,
        24,
    );

    engine.add_defense_unit(
        "Patriot Battery Poland".into(),
        Affiliation::Friendly,
        GeoCoord::new(52.2, 21.0), // Warsaw
        DefenseType::Patriot,
        64,
    );

    // Romania
    engine.add_defense_unit(
        "Aegis Ashore Romania".into(),
        Affiliation::Friendly,
        GeoCoord::new(43.9, 24.0), // Deveselu
        DefenseType::Aegis,
        24,
    );

    // Germany
    engine.add_defense_unit(
        "Patriot Battery Germany".into(),
        Affiliation::Friendly,
        GeoCoord::new(52.5, 13.4), // Berlin
        DefenseType::Patriot,
        64,
    );

    // UK
    engine.add_defense_unit(
        "Sky Sabre UK".into(),
        Affiliation::Friendly,
        GeoCoord::new(51.5, -0.1), // London
        DefenseType::Patriot,
        48,
    );

    // Radar
    engine.add_radar_station(
        "Fylingdales".into(),
        Affiliation::Friendly,
        GeoCoord::new(54.4, -0.7), // UK
        4800.0,
    );

    // Russian Missiles
    engine.add_missile(
        "Iskander-M #1".into(),
        Affiliation::Hostile,
        GeoCoord::new(54.7, 20.5), // Kaliningrad
        GeoCoord::new(52.2, 21.0), // Warsaw
        15.0,
    );

    engine.add_missile(
        "RS-26 #1".into(),
        Affiliation::Hostile,
        GeoCoord::new(56.0, 37.0), // Moscow area
        GeoCoord::new(52.5, 13.4), // Berlin
        60.0,
    );

    engine.add_missile(
        "RS-26 #2".into(),
        Affiliation::Hostile,
        GeoCoord::new(56.0, 37.0),
        GeoCoord::new(51.5, -0.1), // London
        90.0,
    );

    engine.add_missile(
        "Kalibr Cruise".into(),
        Affiliation::Hostile,
        GeoCoord::new(44.6, 33.5), // Crimea
        GeoCoord::new(43.9, 24.0), // Romania
        30.0,
    );
}

/// North Atlantic - SLBM Scenario
fn load_north_atlantic(engine: &mut SimulationEngine) {
    engine.reset();

    // US/NATO Defense
    engine.add_defense_unit(
        "GBI Fort Greely".into(),
        Affiliation::Friendly,
        GeoCoord::new(64.0, -146.0),
        DefenseType::GBI,
        44,
    );

    engine.add_defense_unit(
        "THAAD UK".into(),
        Affiliation::Friendly,
        GeoCoord::new(51.5, -0.1),
        DefenseType::THAAD,
        48,
    );

    engine.add_defense_unit(
        "USS Ross (Aegis)".into(),
        Affiliation::Friendly,
        GeoCoord::new(58.0, -20.0), // North Atlantic
        DefenseType::Aegis,
        96,
    );

    engine.add_defense_unit(
        "USS Porter (Aegis)".into(),
        Affiliation::Friendly,
        GeoCoord::new(42.0, -60.0), // Western Atlantic
        DefenseType::Aegis,
        96,
    );

    // Radar
    engine.add_radar_station(
        "Thule Greenland".into(),
        Affiliation::Friendly,
        GeoCoord::new(76.5, -68.7),
        4800.0,
    );

    engine.add_radar_station(
        "Fylingdales UK".into(),
        Affiliation::Friendly,
        GeoCoord::new(54.4, -0.7),
        4800.0,
    );

    // Satellite
    engine.add_satellite(
        "SBIRS GEO-1".into(),
        Affiliation::Friendly,
        GeoCoord::new(0.0, -30.0), // Atlantic
        35786.0,
        SensorType::Infrared,
    );

    // SLBM Launches (from submarine positions)
    engine.add_missile(
        "Bulava SLBM #1".into(),
        Affiliation::Hostile,
        GeoCoord::new(70.0, 30.0), // Barents Sea
        GeoCoord::new(38.9, -77.0), // Washington DC
        30.0,
    );

    engine.add_missile(
        "Bulava SLBM #2".into(),
        Affiliation::Hostile,
        GeoCoord::new(70.0, 30.0),
        GeoCoord::new(40.7, -74.0), // New York
        45.0,
    );

    engine.add_missile(
        "Sineva SLBM #1".into(),
        Affiliation::Hostile,
        GeoCoord::new(55.0, -25.0), // Mid-Atlantic
        GeoCoord::new(51.5, -0.1), // London
        20.0,
    );
}

/// Demo scenario with variety of threats
fn load_demo_scenario(engine: &mut SimulationEngine) {
    engine.reset();

    // Mix of defense from Pacific scenario
    engine.add_defense_unit(
        "THAAD California".into(),
        Affiliation::Friendly,
        GeoCoord::new(37.5, -122.0),
        DefenseType::THAAD,
        48,
    );

    engine.add_defense_unit(
        "Aegis Sea of Japan".into(),
        Affiliation::Friendly,
        GeoCoord::new(38.0, 132.0),
        DefenseType::Aegis,
        96,
    );

    engine.add_defense_unit(
        "GBI Alaska".into(),
        Affiliation::Friendly,
        GeoCoord::new(64.0, -146.0),
        DefenseType::GBI,
        44,
    );

    engine.add_radar_station(
        "Early Warning Alaska".into(),
        Affiliation::Friendly,
        GeoCoord::new(71.0, -156.0),
        2000.0,
    );

    engine.add_satellite(
        "SBIRS GEO-2".into(),
        Affiliation::Friendly,
        GeoCoord::new(0.0, 120.0),
        35786.0,
        SensorType::Infrared,
    );

    // Demo missiles
    engine.add_missile(
        "ICBM Demo 1".into(),
        Affiliation::Hostile,
        GeoCoord::new(39.0, 125.5), // North Korea
        GeoCoord::new(37.5, -122.0), // San Francisco
        60.0,
    );

    engine.add_missile(
        "ICBM Demo 2".into(),
        Affiliation::Hostile,
        GeoCoord::new(39.0, 125.5),
        GeoCoord::new(47.6, -122.3), // Seattle
        120.0,
    );

    engine.add_missile(
        "MRBM Demo".into(),
        Affiliation::Hostile,
        GeoCoord::new(35.0, 51.0), // Iran
        GeoCoord::new(32.0, 34.8), // Tel Aviv
        30.0,
    );
}
