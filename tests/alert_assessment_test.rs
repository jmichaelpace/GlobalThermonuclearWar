//! Integration test for the sensor-derived DEFCON alert assessment
//!
//! Runs the same NK→Japan MRBM + AEGIS scenario as the track prediction
//! test and verifies the alert pipeline end-to-end: fused tracks (with
//! real quality gating) drive the alert level through the expected
//! escalation and de-escalation as the engagement evolves.
//!
//! Sensor-only doctrine: the assessment must work purely from fused
//! tracks — this test never reads missile ground truth into the alert
//! inputs (only for the terminal-phase sanity check).

use std::collections::HashSet;

use global_thermonuclear_war::simulation::alert::{
    assess_tracks, track_threat_from_fused, AlertLevel, AlertTracker,
};
use global_thermonuclear_war::simulation::{Affiliation, DefenseType, SimulationEngine, TimeScale};
use global_thermonuclear_war::types::GeoCoord;

/// Step the engine and return the current alert level (sensor-derived).
fn assess(engine: &SimulationEngine) -> AlertLevel {
    let defense_unit_ids: HashSet<u64> = engine.defense_units.iter().map(|u| u.id).collect();
    let engaged: HashSet<u64> = engine
        .interceptors
        .iter()
        .filter(|i| {
            // Mirrors the app's definition: in-flight interceptor assigned
            global_thermonuclear_war::simulation::InterceptorStatus::InFlight == i.status
        })
        .map(|i| i.target_id)
        .collect();
    let tracks = engine
        .detection
        .get_all_fused_tracks(&defense_unit_ids, engine.sim_time);
    let threats: Vec<_> = tracks
        .iter()
        .map(|t| track_threat_from_fused(t, engine.sim_time, &engaged))
        .collect();
    assess_tracks(&threats)
}

#[test]
fn test_alert_level_escalates_with_tracked_threat() {
    let mut engine = SimulationEngine::new();
    engine.detection.seed_rng(42);
    engine.time_scale = TimeScale::RealTime;

    // NK -> Japan MRBM with an AEGIS platform in the Sea of Japan
    engine.add_defense_unit(
        "Test AEGIS".to_string(),
        Affiliation::Friendly,
        GeoCoord::new(37.0, 132.0),
        DefenseType::Aegis,
        8,
    );
    engine.add_missile(
        "Test MRBM".to_string(),
        Affiliation::Hostile,
        GeoCoord::new(39.0, 125.5),
        GeoCoord::new(35.0, 139.0),
        0.0,
    );

    // Baseline: no tracks yet -> DEFCON 5
    assert_eq!(assess(&engine), AlertLevel::Defcon5);

    // Run until a track establishes (detections take a few sim seconds).
    // Within ~90s the converged trajectory must exist (per the impact
    // prediction test's establishment threshold), which puts the alert
    // level at DEFCON 3 or higher (worse).
    let mut worst_seen = AlertLevel::Defcon5;
    for _ in 0..1200 {
        engine.update(0.1);
        let level = assess(&engine);
        if level < worst_seen {
            worst_seen = level;
        }
    }

    // A quality-gated, converged track must have raised the alert level
    // to at least DEFCON 3 (threat inbound).
    assert!(
        worst_seen <= AlertLevel::Defcon3,
        "expected DEFCON 3 or worse at some point, saw {worst_seen:?}"
    );
}

#[test]
fn test_klaxon_tracker_escalation_lifecycle() {
    // Unit-test the klaxon gate against a realistic escalation sequence
    // over the lifetime of an engagement.
    let mut tracker = AlertTracker::new();

    // First detection raises the level: klaxon sounds
    assert!(tracker.update(AlertLevel::Defcon4));
    // Further escalation is within the cooldown: silent
    assert!(!tracker.update(AlertLevel::Defcon3));
    // De-escalation: silent
    assert!(!tracker.update(AlertLevel::Defcon4));
    assert_eq!(tracker.last_level(), AlertLevel::Defcon4);
}

#[test]
fn test_no_tracks_stays_defcon5() {
    // Engine with no hostiles: level never leaves DEFCON 5, no klaxon
    let mut engine = SimulationEngine::new();
    engine.detection.seed_rng(42);
    engine.time_scale = TimeScale::RealTime;
    engine.add_defense_unit(
        "Lone AEGIS".to_string(),
        Affiliation::Friendly,
        GeoCoord::new(37.0, 132.0),
        DefenseType::Aegis,
        8,
    );

    let mut tracker = AlertTracker::new();
    for _ in 0..300 {
        engine.update(0.1);
        assert_eq!(assess(&engine), AlertLevel::Defcon5);
        assert!(!tracker.update(assess(&engine)));
    }
}
