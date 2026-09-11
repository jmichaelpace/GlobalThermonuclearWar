//! Scenario builder foundation tests.
//!
//! Verifies that every scenario TOML in scenarios/ can be parsed, serialized,
//! and re-parsed into a structurally identical value. This is the foundation
//! for the in-app scenario builder: a draft built or imported in the builder
//! must survive a save/load cycle without losing or corrupting any field.

use std::path::Path;

use global_thermonuclear_war::scenario::loader::ScenarioFile;

/// Parse → serialize → re-parse must produce identical scenario structs.
///
/// Structural comparison uses the serialized TOML text itself: two structs
/// serialize identically if and only if every field matches (f64s are
/// compared via their TOML representation, which is exact round-trip
/// formatting in the `toml` crate).
#[test]
fn test_all_scenarios_round_trip() {
    let scenarios_dir = Path::new("scenarios");
    assert!(
        scenarios_dir.exists(),
        "scenarios/ directory must exist to run this test"
    );

    let mut files_checked = 0;
    let mut failures: Vec<String> = Vec::new();

    let entries = std::fs::read_dir(scenarios_dir).expect("failed to read scenarios directory");

    for entry in entries {
        let path = entry.expect("failed to read dir entry").path();
        if path.extension().and_then(|e| e.to_str()) != Some("toml") {
            continue;
        }

        let name = path
            .file_name()
            .and_then(|n| n.to_str())
            .unwrap_or("<unnamed>")
            .to_string();

        // --- Parse the original file ---
        let original = match ScenarioFile::load_from_file(&path) {
            Ok(s) => s,
            Err(e) => {
                failures.push(format!("{name}: failed to parse original: {e}"));
                continue;
            }
        };

        // --- Serialize it back to TOML ---
        let serialized = match toml::to_string_pretty(&original) {
            Ok(s) => s,
            Err(e) => {
                failures.push(format!("{name}: failed to serialize: {e}"));
                continue;
            }
        };

        // --- Re-parse the serialized text ---
        let reparsed: ScenarioFile = match toml::from_str(&serialized) {
            Ok(s) => s,
            Err(e) => {
                failures.push(format!(
                    "{name}: serialized output failed to re-parse: {e}\n---\n{serialized}---"
                ));
                continue;
            }
        };

        // --- Serialize the re-parsed value again; text must be identical ---
        // Comparing second-pass TOML text to first-pass text is a strict
        // structural equality check that catches any field loss, default
        // injection drift, or Option handling regression.
        let reserialized = match toml::to_string_pretty(&reparsed) {
            Ok(s) => s,
            Err(e) => {
                failures.push(format!("{name}: failed to re-serialize: {e}"));
                continue;
            }
        };

        if serialized != reserialized {
            failures.push(format!(
                "{name}: round-trip mismatch (first-pass and second-pass TOML differ)\n\
                 --- first ---\n{serialized}--- second ---\n{reserialized}---"
            ));
        }

        files_checked += 1;
    }

    assert!(
        files_checked > 0,
        "no scenario TOML files found in scenarios/ — test is vacuous"
    );
    assert!(
        failures.is_empty(),
        "{} of {} scenario files failed round-trip:\n{}",
        failures.len(),
        files_checked,
        failures.join("\n")
    );

    println!("Round-trip verified for {files_checked} scenario files");
}

/// Option fields must round-trip losslessly:
/// - `None` radar fields must serialize to ABSENT keys (not null) and
///   re-parse back to `None`
/// - `Some` fields must survive with their values intact
#[test]
fn test_option_fields_round_trip() {
    let radar_none = ScenarioFile::load_from_file(Path::new("scenarios/test_thaad.toml"))
        .expect("test_thaad.toml must parse");

    // Serialize and confirm no null/invalid radar keys appear
    let text = toml::to_string_pretty(&radar_none).expect("serialize must succeed");
    assert!(
        !text.contains("sensor_config ="),
        "'None' sensor_config must be absent from output, got:\n{text}"
    );

    // Re-parse: absent keys must deserialize back to None
    let reparsed: ScenarioFile = toml::from_str(&text).expect("re-parse must succeed");
    for radar in &reparsed.radar_stations {
        // Every radar in this fixture either has both fields or we verify the
        // ones that had None stay None: check that at least the count of
        // sensor_config values matches the original
        let _ = radar;
    }
    assert_eq!(
        reparsed.radar_stations.len(),
        radar_none.radar_stations.len(),
        "radar count must survive round-trip"
    );

    // Compare sensor_config presence pattern
    for (orig, new) in radar_none
        .radar_stations
        .iter()
        .zip(reparsed.radar_stations.iter())
    {
        assert_eq!(
            orig.sensor_config.is_some(),
            new.sensor_config.is_some(),
            "sensor_config presence changed for radar '{}'",
            orig.name
        );
        assert_eq!(
            orig.facing_deg.is_some(),
            new.facing_deg.is_some(),
            "facing_deg presence changed for radar '{}'",
            orig.name
        );
        if let (Some(a), Some(b)) = (&orig.sensor_config, &new.sensor_config) {
            assert_eq!(
                a, b,
                "sensor_config value changed for radar '{}'",
                orig.name
            );
        }
    }

    // Defense-unit facing_deg must round-trip the same way: absent keys must
    // deserialize back to None.
    for (orig, new) in radar_none
        .defense_units
        .iter()
        .zip(reparsed.defense_units.iter())
    {
        assert_eq!(
            orig.facing_deg.is_some(),
            new.facing_deg.is_some(),
            "facing_deg presence changed for defense unit '{}'",
            orig.name
        );
        if let (Some(a), Some(b)) = (orig.facing_deg, new.facing_deg) {
            assert_eq!(
                a, b,
                "facing_deg value changed for defense unit '{}'",
                orig.name
            );
        }
    }
}

/// A scenario file with `Some` Option values must keep them through
/// round-trip (exercises the skip_serializing_if attribute from the other
/// side: present fields must NOT be skipped).
#[test]
fn test_present_option_fields_survive() {
    // middle_east.toml is the fixture that uses sensor_config (with a
    // slash-containing value, exercising TOML string quoting too)
    let scenario = ScenarioFile::load_from_file(Path::new("scenarios/middle_east.toml"))
        .expect("middle_east.toml must parse");

    let has_some = scenario
        .radar_stations
        .iter()
        .any(|r| r.sensor_config.is_some());
    assert!(
        has_some,
        "fixture assumption: middle_east.toml must contain a Some(sensor_config) radar"
    );

    let text = toml::to_string_pretty(&scenario).expect("serialize");
    assert!(
        text.contains("sensor_config ="),
        "present sensor_config must appear in output"
    );

    let reparsed: ScenarioFile = toml::from_str(&text).expect("re-parse");
    assert_eq!(reparsed.radar_stations.len(), scenario.radar_stations.len());
    for (orig, new) in scenario
        .radar_stations
        .iter()
        .zip(reparsed.radar_stations.iter())
    {
        assert_eq!(
            orig.sensor_config, new.sensor_config,
            "sensor_config value changed for radar '{}'",
            orig.name
        );
        assert_eq!(orig.facing_deg, new.facing_deg);
    }
}

// ============================================================================
// Builder draft → TOML → engine integration chain
// ============================================================================

use global_thermonuclear_war::scenario::builder::ScenarioDraft;
use global_thermonuclear_war::simulation::SimulationEngine;
use global_thermonuclear_war::types::GeoCoord;

/// Build a small draft in code, serialize it, re-parse it, and load it into
/// a fresh engine — the exact chain the builder UI performs on Test Run and
/// Save. Verifies entity counts and positions survive the whole pipeline.
#[test]
fn test_draft_to_engine_chain() {
    let mut draft = ScenarioDraft::new();

    let idx = draft.add_defense_unit(GeoCoord::new(37.0, 132.0), "Aegis");
    assert_eq!(idx, 0);
    draft.add_radar_station(GeoCoord::new(37.5, 133.0));
    draft.add_satellite(GeoCoord::new(0.0, 130.0));
    draft.add_missile(GeoCoord::new(39.0, 125.5), GeoCoord::new(35.0, 139.0));

    // User edits: rename + adjust metadata + set emplacement facing
    draft.file.defense_units[0].name = "Test Aegis".to_string();
    draft.file.defense_units[0].interceptors = 8;
    // Facing must flow through the whole chain: TOML -> engine sensor azimuth
    draft.file.defense_units[0].facing_deg = Some(95.0);
    draft.file.metadata.name = "Chain Test".to_string();
    draft.filename = "chain_test_builder".to_string();
    draft.sync_metadata();

    // No validation errors
    let (errors, _warnings) = draft.validate();
    assert!(errors.is_empty(), "unexpected errors: {errors:?}");

    // Serialize → re-parse → load into engine
    let toml_text = draft.to_toml().expect("draft must serialize");
    let reparsed: ScenarioFile = toml::from_str(&toml_text).expect("re-parse");

    let mut engine = SimulationEngine::new();
    engine.detection.seed_rng(42);
    reparsed.load_into_engine(&mut engine);

    assert_eq!(engine.defense_units.len(), 1, "defense unit count");
    assert_eq!(engine.radar_stations.len(), 1, "radar count");
    assert_eq!(engine.satellites.len(), 1, "satellite count");
    // Missiles: launch_delay 0 means it launches immediately on first update,
    // but add_missile registers it synchronously
    assert_eq!(engine.missiles.len(), 1, "missile count");

    let unit = &engine.defense_units[0];
    assert_eq!(unit.name, "Test Aegis");
    assert_eq!(unit.interceptors_remaining, 8);
    assert!((unit.position.lat - 37.0).abs() < 1e-9);
    assert!((unit.position.lon - 132.0).abs() < 1e-9);

    // facing_deg must aim the unit's sensors (this draft's Aegis carries a
    // 360-degree SPY-1, which ignores facing - verified separately below).
    // Aegis SPY-1 is full coverage: facing is a no-op, so also verify with a
    // narrow-azimuth platform (THAAD's TPY-2 cone) in a scratch draft.
    let mut thaad_draft = ScenarioDraft::new();
    thaad_draft.add_defense_unit(GeoCoord::new(37.0, 132.0), "THAAD");
    thaad_draft.file.defense_units[0].facing_deg = Some(270.0);
    let thaad_toml = thaad_draft.to_toml().expect("thaad draft serializes");
    let thaad_file: ScenarioFile = toml::from_str(&thaad_toml).expect("thaad re-parse");
    let mut thaad_engine = SimulationEngine::new();
    thaad_file.load_into_engine(&mut thaad_engine);
    let thaad_unit = &thaad_engine.defense_units[0];
    assert!(
        thaad_unit.sensors.iter().all(|s| {
            (s.azimuth_center_deg - 270.0).abs() < 1e-9 && s.azimuth_coverage_deg < 360.0
        }),
        "THAAD sensors must be aimed at facing 270 (got {:?})",
        thaad_unit
            .sensors
            .iter()
            .map(|s| (s.azimuth_center_deg, s.azimuth_coverage_deg))
            .collect::<Vec<_>>()
    );

    let missile = &engine.missiles[0];
    assert!((missile.origin.lat - 39.0).abs() < 1e-9);
    assert!((missile.target.lon - 139.0).abs() < 1e-9);

    // Save to disk also works and produces a loadable file
    let path = draft.save().expect("save");
    assert!(path.exists(), "saved file must exist at {}", path.display());

    let from_disk = ScenarioFile::load_from_file(&path).expect("load from disk");
    assert_eq!(from_disk.metadata.id, "chain_test_builder");
    assert_eq!(from_disk.defense_units.len(), 1);

    // Clean up the saved file (don't pollute scenarios/ with test output)
    std::fs::remove_file(&path).expect("cleanup saved test scenario");
}

/// Imported drafts (edit-existing flow) must stay valid and produce
/// near-identical files when re-saved.
#[test]
fn test_import_round_trip_via_builder() {
    let original = ScenarioFile::load_from_file(Path::new("scenarios/test_thaad.toml"))
        .expect("fixture must parse");

    let mut draft = ScenarioDraft::from_scenario(original.clone(), "test_thaad".to_string());

    // Imported draft must be valid as-is
    let (errors, _warnings) = draft.validate();
    assert!(errors.is_empty(), "imported draft errors: {errors:?}");

    // Serialize and compare against direct serialization of the original
    let via_draft = draft.to_toml().expect("draft serializes");
    let direct = toml::to_string_pretty(&original).expect("direct serializes");
    assert_eq!(
        via_draft, direct,
        "imported draft serialization must match the original's"
    );
}
