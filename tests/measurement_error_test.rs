//! Integration test for sensor measurement bias and stochastic noise
//!
//! Phase 4 realism: measurements now carry a deterministic per-sensor
//! calibration bias plus zero-mean Gaussian noise. This test proves the
//! error is physically real and behaves as configured:
//!
//! 1. A sensor with a large known azimuth bias produces fused-track
//!    positions systematically displaced in the bias direction — the error
//!    is visible to the tracking layer, not swallowed by the model.
//! 2. A zero-bias sensor with seeded noise keeps the fused track within
//!    the noise envelope (no systematic drift; errors are zero-mean).
//!
//! Sensor-only doctrine: all assertions read fused tracks; ground truth is
//! used only to compute the error against.

use std::collections::HashSet;

use global_thermonuclear_war::simulation::{
    haversine_distance, Affiliation, DefenseType, FusedTrack, SimulationEngine, TimeScale,
};
use global_thermonuclear_war::types::GeoCoord;

/// Get the fused track for the first missile, or None.
fn first_fused_track(engine: &SimulationEngine) -> Option<FusedTrack> {
    let defense_unit_ids: HashSet<u64> = engine.defense_units.iter().map(|u| u.id).collect();
    engine
        .detection
        .get_all_fused_tracks(&defense_unit_ids, engine.sim_time)
        .into_iter()
        .next()
}

/// The AEGIS platform's primary sensor config (per config/platform/aegis.toml;
/// the registry normalizes "AN/SPY-1D" to its key form)
const AEGIS_SENSOR: &str = "AN/SPY-1D";

/// A large azimuth bias (+8 deg) displaces the fused-track position in the
/// direction the biased bearing points. The AEGIS platform sits south of
/// the missile track; the missile flies roughly west-to-east north of the
/// platform, so a +8 deg azimuth error (clockwise) rotates the measured
/// line of bearing southward at the target's range — displacing the
/// estimated position south of the truth by ~sin(8 deg) * range.
#[test]
fn test_azimuth_bias_displaces_fused_track() {
    let mut engine = SimulationEngine::new();
    engine.time_scale = TimeScale::RealTime;
    engine.detection.seed_rng(42);

    engine.add_defense_unit(
        "Biased AEGIS".to_string(),
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

    // Slam a large azimuth bias into the AEGIS sensor config (bypasses
    // TOML: this test proves the pipeline consumes the field)
    engine
        .sensor_configs
        .get_by_name_mut(AEGIS_SENSOR)
        .tracking
        .azimuth_bias_deg = 8.0;

    // Run until a track establishes with a solid measurement history
    let mut track_seen = false;
    for _ in 0..1500 {
        engine.update(0.1);
        if let Some(track) = first_fused_track(&engine) {
            if track.measurement_count >= 10 {
                track_seen = true;
                break;
            }
        }
    }
    assert!(track_seen, "no fused track established");

    let track = first_fused_track(&engine).unwrap();
    let truth = &engine.missiles[0];

    // The bias must visibly displace the track from the truth. sin(8 deg)
    // at the sensor-target range (~600 km) projects to tens of km lateral
    // offset at acquisition, persisting through the flight.
    let error_km = haversine_distance(track.estimated_position, truth.position);
    assert!(
        error_km > 20.0,
        "bias should visibly displace the track: {error_km:.1} km error"
    );

    // Direction check: a positive azimuth bias rotates the measured line of
    // bearing CLOCKWISE (radar convention: azimuth increases east of
    // north). The estimated position therefore lies to the RIGHT of the
    // true sensor->target line. Compute the cross-track sign against that
    // line: positive = right of the line when looking from sensor to target.
    let sensor_pos = engine.defense_units[0].position;
    let true_bearing_deg =
        global_thermonuclear_war::simulation::bearing(sensor_pos, truth.position);
    let est_bearing_deg =
        global_thermonuclear_war::simulation::bearing(sensor_pos, track.estimated_position);
    let bearing_delta = est_bearing_deg - true_bearing_deg;
    let bearing_delta = if bearing_delta > 180.0 {
        bearing_delta - 360.0
    } else if bearing_delta < -180.0 {
        bearing_delta + 360.0
    } else {
        bearing_delta
    };

    // The estimate must sit clockwise (positive bearing offset) of the
    // true line — consistent with the +8 deg azimuth bias
    assert!(
        bearing_delta > 2.0,
        "expected clockwise bearing offset from bias, got {bearing_delta:.2} deg"
    );
}

/// Zero-bias sensor with seeded noise: the fused track stays within the
/// noise envelope — average error small, no systematic drift.
#[test]
fn test_zero_bias_noise_stays_bounded() {
    let mut engine = SimulationEngine::new();
    engine.time_scale = TimeScale::RealTime;
    engine.detection.seed_rng(7);

    engine.add_defense_unit(
        "Clean AEGIS".to_string(),
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

    // Zero the bias (Phase 4 TOMLs carry realistic calibration offsets;
    // this test isolates the stochastic component)
    {
        let config = engine.sensor_configs.get_by_name_mut(AEGIS_SENSOR);
        config.tracking.azimuth_bias_deg = 0.0;
        config.tracking.elevation_bias_deg = 0.0;
        config.tracking.range_bias_km = 0.0;
    }

    // Collect position errors once the track has measurements
    let mut errors: Vec<f64> = Vec::new();
    for _ in 0..1500 {
        engine.update(0.1);
        if let Some(track) = first_fused_track(&engine) {
            if track.measurement_count >= 3 {
                let truth = &engine.missiles[0];
                let err = haversine_distance(track.estimated_position, truth.position);
                errors.push(err);
            }
        }
    }

    assert!(
        errors.len() > 50,
        "expected many track samples, got {}",
        errors.len()
    );

    let avg = errors.iter().sum::<f64>() / errors.len() as f64;
    let max = errors.iter().cloned().fold(0.0, f64::max);

    // Zero-mean noise + a working filter: average error stays small
    // (comparable to noiseless behavior, < 5 km), max within the 5x bound
    assert!(
        avg < 5.0,
        "avg error {avg:.2} km too high for zero-bias noise"
    );
    assert!(max < 25.0, "max error {max:.2} km beyond noise envelope");
}
