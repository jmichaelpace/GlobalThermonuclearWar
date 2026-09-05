//! Scenario wiring regression tests.
//!
//! The scenario audit found that scenarios could silently degrade: radar
//! stations whose display names don't match a sensor config fall back to the
//! 200 km Generic Radar, and missiles whose names don't match a missile
//! config fly the Generic Ballistic Missile profile (typed MRBM, apogee
//! 150 + 0.15*range) regardless of what the scenario comments claim.
//!
//! These tests make that entire failure class impossible to reintroduce
//! silently: every scenario in `scenarios/` must be explicitly wired to real
//! configs.

use std::path::Path;

use global_thermonuclear_war::scenario::builder::{ScenarioDraft, DEFENSE_TYPES};
use global_thermonuclear_war::scenario::loader::ScenarioFile;
use global_thermonuclear_war::simulation::{MissileConfigRegistry, SensorConfigRegistry};

fn load_all() -> Vec<(String, ScenarioFile)> {
    let dir = Path::new("scenarios");
    assert!(dir.exists(), "scenarios/ directory must exist");

    let mut out = Vec::new();
    for entry in std::fs::read_dir(dir).expect("read scenarios dir") {
        let path = entry.expect("dir entry").path();
        if path.extension().and_then(|e| e.to_str()) != Some("toml") {
            continue;
        }
        let name = path
            .file_name()
            .and_then(|n| n.to_str())
            .unwrap_or("<unnamed>")
            .to_string();
        let scenario =
            ScenarioFile::load_from_file(&path).unwrap_or_else(|e| panic!("{name}: parse: {e}"));
        out.push((name, scenario));
    }
    assert!(!out.is_empty(), "no scenario TOML files found");
    out
}

/// Every radar station in every scenario must declare an explicit
/// `sensor_config` that resolves (via the engine's own registry) to something
/// other than the Generic Radar default.
///
/// Without this, stations rely on name-derived lookup
/// (`RadarStation::new` lowercases the display name), which almost always
/// misses and silently falls back to a 200 km mechanical radar - the scenario
/// then tests nothing it claims to test.
#[test]
fn test_all_radar_stations_resolve_real_sensors() {
    let registry = SensorConfigRegistry::load(Path::new("config"))
        .expect("config/sensors must load (run from repo root)");

    let mut failures: Vec<String> = Vec::new();

    for (name, scenario) in load_all() {
        for radar in &scenario.radar_stations {
            let Some(config_name) = &radar.sensor_config else {
                failures.push(format!(
                    "{name}: radar '{}' ('{}') has no explicit sensor_config",
                    radar.name, radar.name
                ));
                continue;
            };

            let resolved = registry.get_by_name(config_name);
            if resolved.system.name == "Generic Radar" {
                failures.push(format!(
                    "{name}: radar '{}' sensor_config '{config_name}' resolves to Generic Radar",
                    radar.name
                ));
            }
        }
    }

    assert!(
        failures.is_empty(),
        "{} radar station(s) not wired to real sensors:\n{}",
        failures.len(),
        failures.join("\n")
    );
}

/// Every missile in every scenario must resolve (after stripping the " #N"
/// suffix, exactly as `MissileConfigRegistry::get_by_name` does) to something
/// other than the Generic Ballistic Missile default profile.
#[test]
fn test_all_missiles_resolve_real_configs() {
    let registry = MissileConfigRegistry::load(Path::new("config"))
        .expect("config/missiles must load (run from repo root)");

    let mut failures: Vec<String> = Vec::new();

    for (name, scenario) in load_all() {
        for missile in &scenario.missiles {
            // Same normalization as the registry: strip " #N", lowercase,
            // spaces/hyphens to underscores.
            let base = missile.name.split(" #").next().unwrap_or(&missile.name);
            let resolved = registry.get_by_name(base);
            if resolved.system.name == "Generic Ballistic Missile" {
                failures.push(format!(
                    "{name}: missile '{}' resolves to the Generic Ballistic Missile default",
                    missile.name
                ));
            }
        }
    }

    assert!(
        failures.is_empty(),
        "{} missile(s) not wired to real configs:\n{}",
        failures.len(),
        failures.join("\n")
    );
}

/// Every defense unit `type` string must be one of the 8 valid values.
/// A typo silently falls back to THAAD in `parse_defense_type`, changing
/// the platform's sensors, interceptor, and envelope.
#[test]
fn test_defense_types_are_known() {
    let mut failures: Vec<String> = Vec::new();

    for (name, scenario) in load_all() {
        for unit in &scenario.defense_units {
            if !DEFENSE_TYPES.contains(&unit.defense_type.as_str()) {
                failures.push(format!(
                    "{name}: defense unit '{}' has unknown type '{}' (valid: {DEFENSE_TYPES:?})",
                    unit.name, unit.defense_type
                ));
            }
        }
    }

    assert!(
        failures.is_empty(),
        "{} defense unit(s) with unknown types:\n{}",
        failures.len(),
        failures.join("\n")
    );
}

/// Every scenario must pass builder validation with zero errors - the same
/// gate the in-app Save/Test Run buttons enforce.
#[test]
fn test_all_scenarios_pass_builder_validation() {
    let mut failures: Vec<String> = Vec::new();

    for (name, scenario) in load_all() {
        let draft = ScenarioDraft::from_scenario(scenario.clone(), name.clone());
        let (errors, _) = draft.validate();
        if !errors.is_empty() {
            failures.push(format!("{name}: validation errors: {errors:?}"));
        }
    }

    assert!(
        failures.is_empty(),
        "{} scenario(s) failed builder validation:\n{}",
        failures.len(),
        failures.join("\n")
    );
}
