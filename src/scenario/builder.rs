//! Scenario builder: pure draft model, validation, and serialization.
//!
//! This module holds the non-UI half of the in-app scenario builder. The UI
//! half lives in `src/ui/scenario_builder.rs` (binary target). Keeping the
//! logic egui-free makes it unit-testable headless and reusable.
//!
//! A `ScenarioDraft` wraps the same `ScenarioFile` the TOML loader uses, so
//! import ("edit existing scenario") is trivial — scenarios are already
//! `Clone` — and saving produces files byte-compatible with the loader's
//! expectations (proven by the round-trip tests in
//! `tests/scenario_builder_test.rs`).

use std::path::PathBuf;

use crate::scenario::loader::{
    DefenseUnitConfig, MissileConfig, RadarStationConfig, SatelliteConfig, ScenarioFile,
    ScenarioMetadata,
};
use crate::simulation::haversine_distance;
use crate::types::GeoCoord;

/// Valid defense unit type strings (case-sensitive, must match
/// `loader::parse_defense_type`). The UI presents these as a dropdown so
/// users can never type a wrong-case variant.
pub const DEFENSE_TYPES: [&str; 8] = [
    "THAAD",
    "Aegis",
    "Patriot",
    "GBI",
    "IronDome",
    "Arrow3",
    "DavidsSling",
    "S400",
];

/// Valid affiliation strings (`loader::parse_affiliation` matches lowercase).
pub const AFFILIATIONS: [&str; 3] = ["Friendly", "Hostile", "Neutral"];

/// Valid satellite sensor type strings (`loader::parse_sensor_type`).
pub const SATELLITE_SENSOR_TYPES: [&str; 2] = ["Infrared", "Radar"];

/// Which placement tool is active in the builder.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BuilderTool {
    Select,
    DefenseUnit,
    RadarStation,
    Satellite,
    Missile,
}

/// Which draft entity category a selection refers to (indices are stable
/// within a frame; the UI revalidates against current lengths).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DraftCategory {
    DefenseUnit(usize),
    RadarStation(usize),
    Satellite(usize),
    Missile(usize),
}

/// A scenario under construction.
#[derive(Clone, Debug)]
pub struct ScenarioDraft {
    pub file: ScenarioFile,
    /// Target filename (without extension) in scenarios/
    pub filename: String,
    /// Whether center/zoom auto-compute from placed entities is enabled
    pub auto_center: bool,
}

impl Default for ScenarioDraft {
    fn default() -> Self {
        Self::new()
    }
}

impl ScenarioDraft {
    /// A fresh, empty draft with placeholder metadata.
    pub fn new() -> Self {
        Self {
            file: ScenarioFile {
                metadata: ScenarioMetadata {
                    id: "my_scenario".to_string(),
                    name: "My Scenario".to_string(),
                    description: String::new(),
                    region: "Custom".to_string(),
                    center_lat: 20.0,
                    center_lon: 0.0,
                    zoom: 2.5,
                },
                defense_units: Vec::new(),
                radar_stations: Vec::new(),
                satellites: Vec::new(),
                missiles: Vec::new(),
            },
            filename: "my_scenario".to_string(),
            auto_center: true,
        }
    }

    /// Create a draft from an existing scenario (import/edit flow).
    /// The filename defaults to the scenario id; the metadata is preserved
    /// so saving produces a near-identical file.
    pub fn from_scenario(file: ScenarioFile, filename: String) -> Self {
        Self {
            file,
            filename,
            auto_center: false,
        }
    }

    // ------------------------------------------------------------------
    // Entity operations
    // ------------------------------------------------------------------

    /// Add a defense unit at a position, returning its index.
    pub fn add_defense_unit(&mut self, pos: GeoCoord, defense_type: &str) -> usize {
        let name = format!("{} {}", defense_type, self.file.defense_units.len() + 1);
        self.file.defense_units.push(DefenseUnitConfig {
            name,
            affiliation: "Friendly".to_string(),
            lat: pos.lat,
            lon: pos.lon,
            defense_type: defense_type.to_string(),
            interceptors: default_interceptors_for(defense_type),
        });
        self.file.defense_units.len() - 1
    }

    /// Add a radar station at a position, returning its index.
    pub fn add_radar_station(&mut self, pos: GeoCoord) -> usize {
        let name = format!("Radar {}", self.file.radar_stations.len() + 1);
        self.file.radar_stations.push(RadarStationConfig {
            name,
            affiliation: "Friendly".to_string(),
            lat: pos.lat,
            lon: pos.lon,
            range_km: 1000.0,
            sensor_config: None,
            facing_deg: None,
        });
        self.file.radar_stations.len() - 1
    }

    /// Add a satellite at a position, returning its index.
    pub fn add_satellite(&mut self, pos: GeoCoord) -> usize {
        let name = format!("Satellite {}", self.file.satellites.len() + 1);
        self.file.satellites.push(SatelliteConfig {
            name,
            affiliation: "Friendly".to_string(),
            lat: pos.lat,
            lon: pos.lon,
            altitude_km: 35786.0, // GEO by default
            sensor_type: "Infrared".to_string(),
        });
        self.file.satellites.len() - 1
    }

    /// Add a missile with an origin and target (two-click flow complete),
    /// returning its index.
    pub fn add_missile(&mut self, origin: GeoCoord, target: GeoCoord) -> usize {
        let name = format!("Missile {}", self.file.missiles.len() + 1);
        self.file.missiles.push(MissileConfig {
            name,
            affiliation: "Hostile".to_string(),
            origin_lat: origin.lat,
            origin_lon: origin.lon,
            target_lat: target.lat,
            target_lon: target.lon,
            launch_delay_sec: 0.0,
        });
        self.file.missiles.len() - 1
    }

    /// Remove the selected entity (no-op if the index is out of bounds).
    pub fn remove(&mut self, sel: DraftCategory) {
        match sel {
            DraftCategory::DefenseUnit(i) => {
                if i < self.file.defense_units.len() {
                    self.file.defense_units.remove(i);
                }
            }
            DraftCategory::RadarStation(i) => {
                if i < self.file.radar_stations.len() {
                    self.file.radar_stations.remove(i);
                }
            }
            DraftCategory::Satellite(i) => {
                if i < self.file.satellites.len() {
                    self.file.satellites.remove(i);
                }
            }
            DraftCategory::Missile(i) => {
                if i < self.file.missiles.len() {
                    self.file.missiles.remove(i);
                }
            }
        }
    }

    /// Duplicate the selected entity (inserted right after the original),
    /// returning the new selection (the duplicate's index).
    pub fn duplicate(&mut self, sel: DraftCategory) -> Option<DraftCategory> {
        match sel {
            DraftCategory::DefenseUnit(i) => self
                .file
                .defense_units
                .get(i)
                .cloned()
                .map(|mut u| {
                    u.name = format!("{} (copy)", u.name);
                    u
                })
                .map(|u| {
                    self.file.defense_units.insert(i + 1, u);
                    DraftCategory::DefenseUnit(i + 1)
                }),
            DraftCategory::RadarStation(i) => self
                .file
                .radar_stations
                .get(i)
                .cloned()
                .map(|mut r| {
                    r.name = format!("{} (copy)", r.name);
                    r
                })
                .map(|r| {
                    self.file.radar_stations.insert(i + 1, r);
                    DraftCategory::RadarStation(i + 1)
                }),
            DraftCategory::Satellite(i) => self
                .file
                .satellites
                .get(i)
                .cloned()
                .map(|mut s| {
                    s.name = format!("{} (copy)", s.name);
                    s
                })
                .map(|s| {
                    self.file.satellites.insert(i + 1, s);
                    DraftCategory::Satellite(i + 1)
                }),
            DraftCategory::Missile(i) => self
                .file
                .missiles
                .get(i)
                .cloned()
                .map(|mut m| {
                    m.name = format!("{} (copy)", m.name);
                    m
                })
                .map(|m| {
                    self.file.missiles.insert(i + 1, m);
                    DraftCategory::Missile(i + 1)
                }),
        }
    }

    // ------------------------------------------------------------------
    // Metadata helpers
    // ------------------------------------------------------------------

    /// Sanitize a user-entered filename: lowercase, [a-z0-9_] only.
    pub fn sanitize_filename(raw: &str) -> String {
        let mut out = String::with_capacity(raw.len());
        for ch in raw.trim().chars() {
            if ch.is_ascii_alphanumeric() {
                out.extend(ch.to_lowercase());
            } else if ch == '-' || ch == ' ' || ch == '_' {
                out.push('_');
            }
            // everything else dropped
        }
        let trimmed = out.trim_matches('_').to_string();
        if trimmed.is_empty() {
            "my_scenario".to_string()
        } else {
            trimmed
        }
    }

    /// Recompute metadata id and center/zoom from current contents.
    /// Called before save/test-run when `auto_center` is on.
    pub fn sync_metadata(&mut self) {
        self.file.metadata.id = Self::sanitize_filename(&self.filename);

        if self.auto_center {
            let mut count = 0usize;
            let (mut lat_sum, mut lon_sum) = (0.0f64, 0.0f64);
            let add =
                |lat: f64, lon: f64, count: &mut usize, lat_sum: &mut f64, lon_sum: &mut f64| {
                    *count += 1;
                    *lat_sum += lat;
                    *lon_sum += lon;
                };
            for u in &self.file.defense_units {
                add(u.lat, u.lon, &mut count, &mut lat_sum, &mut lon_sum);
            }
            for r in &self.file.radar_stations {
                add(r.lat, r.lon, &mut count, &mut lat_sum, &mut lon_sum);
            }
            for s in &self.file.satellites {
                add(s.lat, s.lon, &mut count, &mut lat_sum, &mut lon_sum);
            }
            for m in &self.file.missiles {
                add(
                    m.origin_lat,
                    m.origin_lon,
                    &mut count,
                    &mut lat_sum,
                    &mut lon_sum,
                );
                add(
                    m.target_lat,
                    m.target_lon,
                    &mut count,
                    &mut lat_sum,
                    &mut lon_sum,
                );
            }
            if count > 0 {
                self.file.metadata.center_lat = lat_sum / count as f64;
                self.file.metadata.center_lon = lon_sum / count as f64;
                self.file.metadata.zoom = 3.0;
            }
        }
    }

    // ------------------------------------------------------------------
    // Validation & serialization
    // ------------------------------------------------------------------

    /// Live validation. Errors block save/test-run; warnings don't.
    pub fn validate(&self) -> (Vec<String>, Vec<String>) {
        let mut errors = Vec::new();
        let mut warnings = Vec::new();

        // --- Metadata ---
        if self.file.metadata.name.trim().is_empty() {
            errors.push("Metadata: name is empty".to_string());
        }
        if self.filename.trim().is_empty() {
            errors.push("Metadata: filename is empty".to_string());
        }

        // --- Lat/lon bounds + duplicate names per category ---
        let check_latlon = |lat: f64, lon: f64, what: &str, errors: &mut Vec<String>| {
            if !(-90.0..=90.0).contains(&lat) {
                errors.push(format!("{what}: latitude {lat} out of bounds (-90..90)"));
            }
            if !(-180.0..=180.0).contains(&lon) {
                errors.push(format!("{what}: longitude {lon} out of bounds (-180..180)"));
            }
        };

        let mut unit_names = std::collections::HashSet::new();
        for (i, u) in self.file.defense_units.iter().enumerate() {
            let what = format!("Defense unit #{} ('{}')", i + 1, u.name);
            check_latlon(u.lat, u.lon, &what, &mut errors);
            if u.name.trim().is_empty() {
                errors.push(format!("Defense unit #{} has an empty name", i + 1));
            } else if !unit_names.insert(u.name.trim().to_string()) {
                errors.push(format!("Defense unit: duplicate name '{}'", u.name));
            }
            if !DEFENSE_TYPES.contains(&u.defense_type.as_str()) {
                errors.push(format!(
                    "{what}: unknown type '{}' (valid: {})",
                    u.defense_type,
                    DEFENSE_TYPES.join(", ")
                ));
            }
            if !AFFILIATIONS.contains(&u.affiliation.as_str()) {
                errors.push(format!("{what}: unknown affiliation '{}'", u.affiliation));
            }
        }

        let mut radar_names = std::collections::HashSet::new();
        for (i, r) in self.file.radar_stations.iter().enumerate() {
            let what = format!("Radar #{} ('{}')", i + 1, r.name);
            check_latlon(r.lat, r.lon, &what, &mut errors);
            if r.name.trim().is_empty() {
                errors.push(format!("Radar #{} has an empty name", i + 1));
            } else if !radar_names.insert(r.name.trim().to_string()) {
                errors.push(format!("Radar: duplicate name '{}'", r.name));
            }
            if r.range_km <= 0.0 {
                errors.push(format!("{what}: range must be > 0 km"));
            }
            if r.range_km > 6000.0 {
                warnings.push(format!(
                    "{what}: range {} km is unusually large",
                    r.range_km
                ));
            }
            if let Some(cfg) = &r.sensor_config {
                if !sensor_config_exists(cfg) {
                    warnings.push(format!(
                        "{what}: sensor_config '{}' has no matching config/sensors/{}.toml",
                        cfg,
                        cfg.to_lowercase().replace([' ', '-'], "_")
                    ));
                }
            }
        }

        let mut sat_names = std::collections::HashSet::new();
        for (i, s) in self.file.satellites.iter().enumerate() {
            let what = format!("Satellite #{} ('{}')", i + 1, s.name);
            check_latlon(s.lat, s.lon, &what, &mut errors);
            if s.name.trim().is_empty() {
                errors.push(format!("Satellite #{} has an empty name", i + 1));
            } else if !sat_names.insert(s.name.trim().to_string()) {
                errors.push(format!("Satellite: duplicate name '{}'", s.name));
            }
            if s.altitude_km <= 0.0 {
                errors.push(format!("{what}: altitude must be > 0 km"));
            }
            if !SATELLITE_SENSOR_TYPES.contains(&s.sensor_type.as_str()) {
                errors.push(format!("{what}: unknown sensor_type '{}'", s.sensor_type));
            }
        }

        let mut missile_names = std::collections::HashSet::new();
        for (i, m) in self.file.missiles.iter().enumerate() {
            let what = format!("Missile #{} ('{}')", i + 1, m.name);
            check_latlon(m.origin_lat, m.origin_lon, &what, &mut errors);
            check_latlon(m.target_lat, m.target_lon, &what, &mut errors);
            if m.name.trim().is_empty() {
                errors.push(format!("Missile #{} has an empty name", i + 1));
            } else if !missile_names.insert(m.name.trim().to_string()) {
                errors.push(format!("Missile: duplicate name '{}'", m.name));
            }
            let origin = GeoCoord::new(m.origin_lat, m.origin_lon);
            let target = GeoCoord::new(m.target_lat, m.target_lon);
            if haversine_distance(origin, target) < 1.0 {
                errors.push(format!("{what}: origin and target are the same point"));
            }
            if m.launch_delay_sec < 0.0 {
                errors.push(format!("{what}: launch delay must be >= 0"));
            }
        }

        // --- Composition warnings ---
        if self.file.missiles.is_empty() {
            warnings.push("No missiles: the scenario has no threats".to_string());
        }
        if self.file.defense_units.is_empty() {
            warnings.push("No defense units: nothing can engage the threats".to_string());
        } else if !self.file.missiles.is_empty() {
            // Every hostile missile impact point should be near SOME friendly
            // defender, else nothing can possibly engage it.
            for m in &self.file.missiles {
                if m.affiliation != "Hostile" {
                    continue;
                }
                let impact = GeoCoord::new(m.target_lat, m.target_lon);
                let nearest = self
                    .file
                    .defense_units
                    .iter()
                    .filter(|u| u.affiliation == "Friendly")
                    .map(|u| haversine_distance(GeoCoord::new(u.lat, u.lon), impact))
                    .min_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
                match nearest {
                    None => warnings.push(format!(
                        "Missile '{}': no Friendly defense units exist to engage it",
                        m.name
                    )),
                    Some(d) if d > 500.0 => warnings.push(format!(
                        "Missile '{}': nearest Friendly defense unit is {:.0} km from its impact point — likely out of engagement range",
                        m.name, d
                    )),
                    _ => {}
                }
            }
        }

        (errors, warnings)
    }

    /// Serialize the draft to TOML text. Fails if validation errors exist
    /// (the UI should gate the button, but the API enforces it too).
    pub fn to_toml(&self) -> Result<String, String> {
        let (errors, _) = self.validate();
        if !errors.is_empty() {
            return Err(format!(
                "cannot serialize: {} validation error(s): {}",
                errors.len(),
                errors.join("; ")
            ));
        }
        toml::to_string_pretty(&self.file).map_err(|e| format!("serialization failed: {e}"))
    }

    /// Save the draft to `scenarios/<filename>.toml`.
    /// Returns the path written. Fails on validation errors or IO errors.
    /// Overwrites existing files with the same name.
    pub fn save(&self) -> Result<PathBuf, String> {
        let text = self.to_toml()?;
        let path = PathBuf::from("scenarios").join(format!("{}.toml", self.filename));
        std::fs::write(&path, text)
            .map_err(|e| format!("failed to write {}: {e}", path.display()))?;
        Ok(path)
    }

    // ------------------------------------------------------------------
    // Read helpers for the UI
    // ------------------------------------------------------------------

    /// Total entity count (for the panel header).
    pub fn entity_count(&self) -> usize {
        self.file.defense_units.len()
            + self.file.radar_stations.len()
            + self.file.satellites.len()
            + self.file.missiles.len()
    }

    /// Positions of all placed entities for map hit-testing. Satellites are
    /// included (they render on the map plane at their sub-satellite point);
    /// missiles contribute both origin and target points.
    pub fn placed_positions(&self) -> Vec<(GeoCoord, DraftCategory)> {
        let mut out = Vec::new();
        for (i, u) in self.file.defense_units.iter().enumerate() {
            out.push((GeoCoord::new(u.lat, u.lon), DraftCategory::DefenseUnit(i)));
        }
        for (i, r) in self.file.radar_stations.iter().enumerate() {
            out.push((GeoCoord::new(r.lat, r.lon), DraftCategory::RadarStation(i)));
        }
        for (i, s) in self.file.satellites.iter().enumerate() {
            out.push((GeoCoord::new(s.lat, s.lon), DraftCategory::Satellite(i)));
        }
        for (i, m) in self.file.missiles.iter().enumerate() {
            out.push((
                GeoCoord::new(m.origin_lat, m.origin_lon),
                DraftCategory::Missile(i),
            ));
            out.push((
                GeoCoord::new(m.target_lat, m.target_lon),
                DraftCategory::Missile(i),
            ));
        }
        out
    }
}

/// Classify a missile by range, mirroring the engine's auto-classification
/// thresholds (`MissileConfigRegistry::type_for_range`, config.rs). Used for
/// display only — the engine still classifies at load time.
pub fn classify_missile_range(range_km: f64) -> &'static str {
    if range_km >= 5500.0 {
        "ICBM"
    } else if range_km >= 3000.0 {
        "IRBM"
    } else if range_km >= 1000.0 {
        "MRBM"
    } else {
        "SRBM"
    }
}

/// Default interceptor loadout per defense type (matches the demo scenario's
/// conventions; users can edit afterwards).
fn default_interceptors_for(defense_type: &str) -> u32 {
    match defense_type {
        "THAAD" => 48,
        "Aegis" => 96,
        "Patriot" => 32,
        "GBI" => 44,
        "IronDome" => 60,
        "Arrow3" => 24,
        "DavidsSling" => 36,
        "S400" => 48,
        _ => 24,
    }
}

/// Check that a sensor_config name has a matching config/sensors file.
/// Mirrors the naming convention used by the engine when no config is given
/// (lowercase, spaces/dashes to underscores).
fn sensor_config_exists(name: &str) -> bool {
    let normalized = name.to_lowercase().replace([' ', '-'], "_");
    PathBuf::from("config/sensors")
        .join(format!("{normalized}.toml"))
        .exists()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_filename_sanitization() {
        assert_eq!(
            ScenarioDraft::sanitize_filename("My Cool Scenario"),
            "my_cool_scenario"
        );
        assert_eq!(
            ScenarioDraft::sanitize_filename("Korean Pen. 2!"),
            "korean_pen_2"
        );
        assert_eq!(ScenarioDraft::sanitize_filename("---"), "my_scenario");
        assert_eq!(ScenarioDraft::sanitize_filename("  spaced  "), "spaced");
    }

    #[test]
    fn test_validation_catches_errors() {
        let mut draft = ScenarioDraft::new();

        // Empty draft: no errors (entities are optional), but warnings exist
        let (errors, warnings) = draft.validate();
        assert!(
            errors.is_empty(),
            "empty draft should have no errors: {errors:?}"
        );
        assert!(
            !warnings.is_empty(),
            "empty draft should warn about no missiles"
        );

        // Add a missile with origin == target
        let origin = GeoCoord::new(39.0, 125.5);
        draft.add_missile(origin, origin);
        let (errors, _) = draft.validate();
        assert!(
            errors.iter().any(|e| e.contains("same point")),
            "expected same-point error, got: {errors:?}"
        );

        // Add a unit with a bad type (simulating a hand-edited import)
        draft.file.defense_units.push(DefenseUnitConfig {
            name: "Bad Unit".into(),
            affiliation: "Friendly".into(),
            lat: 0.0,
            lon: 0.0,
            defense_type: "thaad".into(), // wrong case
            interceptors: 8,
        });
        let (errors, _) = draft.validate();
        assert!(
            errors.iter().any(|e| e.contains("unknown type")),
            "expected unknown-type error, got: {errors:?}"
        );

        // Out-of-bounds lat
        draft.file.defense_units.push(DefenseUnitConfig {
            name: "Far Unit".into(),
            affiliation: "Friendly".into(),
            lat: 120.0,
            lon: 0.0,
            defense_type: "THAAD".into(),
            interceptors: 8,
        });
        let (errors, _) = draft.validate();
        assert!(
            errors.iter().any(|e| e.contains("out of bounds")),
            "expected bounds error, got: {errors:?}"
        );
    }

    #[test]
    fn test_validation_duplicate_names() {
        let mut draft = ScenarioDraft::new();
        let pos = GeoCoord::new(37.0, 132.0);
        draft.add_defense_unit(pos, "Aegis");
        // Force the same name on a second unit
        draft.add_defense_unit(pos, "Aegis");
        draft.file.defense_units[1].name = draft.file.defense_units[0].name.clone();

        let (errors, _) = draft.validate();
        assert!(
            errors.iter().any(|e| e.contains("duplicate name")),
            "expected duplicate-name error, got: {errors:?}"
        );
    }

    #[test]
    fn test_add_and_duplicate() {
        let mut draft = ScenarioDraft::new();
        let pos = GeoCoord::new(37.0, 132.0);
        let idx = draft.add_defense_unit(pos, "Aegis");
        assert_eq!(idx, 0);
        assert_eq!(draft.file.defense_units.len(), 1);
        assert_eq!(draft.file.defense_units[0].defense_type, "Aegis");
        assert_eq!(draft.file.defense_units[0].interceptors, 96);

        let new_sel = draft.duplicate(DraftCategory::DefenseUnit(0)).unwrap();
        assert_eq!(new_sel, DraftCategory::DefenseUnit(1));
        assert_eq!(draft.file.defense_units.len(), 2);
        assert_eq!(draft.file.defense_units[1].name, "Aegis 1 (copy)");

        draft.remove(DraftCategory::DefenseUnit(0));
        assert_eq!(draft.file.defense_units.len(), 1);
        assert_eq!(draft.file.defense_units[0].name, "Aegis 1 (copy)");
    }

    #[test]
    fn test_auto_center() {
        let mut draft = ScenarioDraft::new();
        draft.add_defense_unit(GeoCoord::new(40.0, -100.0), "THAAD");
        draft.add_missile(GeoCoord::new(39.0, 125.5), GeoCoord::new(35.0, 139.0));
        draft.sync_metadata();

        // Center = mean of unit + missile origin + target = (40+39+35)/3, (-100+125.5+139)/3
        assert!((draft.file.metadata.center_lat - (40.0 + 39.0 + 35.0) / 3.0).abs() < 1e-9);
        assert!((draft.file.metadata.center_lon - (-100.0 + 125.5 + 139.0) / 3.0).abs() < 1e-9);
    }

    #[test]
    fn test_missile_classification() {
        assert_eq!(classify_missile_range(300.0), "SRBM");
        assert_eq!(classify_missile_range(1500.0), "MRBM");
        assert_eq!(classify_missile_range(4000.0), "IRBM");
        assert_eq!(classify_missile_range(9000.0), "ICBM");
    }

    #[test]
    fn test_to_toml_blocks_on_errors() {
        let mut draft = ScenarioDraft::new();
        let origin = GeoCoord::new(39.0, 125.5);
        draft.add_missile(origin, origin); // same-point error
        assert!(draft.to_toml().is_err());
        assert!(draft.save().is_err());
    }
}
