//! Impact point prediction: accuracy and stability test.
//!
//! Verifies that the converged impact estimate (the yellow "X" marker rendered
//! from FusedTrack.converged_trajectory) is:
//!   1. Established soon after detection,
//!   2. Converges near the true impact point (within tolerance),
//!   3. Stops wandering: step sizes shrink over time, and the marker never
//!      oscillates along the predicted track.

use std::collections::HashSet;

use global_thermonuclear_war::simulation::{Affiliation, DefenseType, SimulationEngine, TimeScale};
use global_thermonuclear_war::types::GeoCoord;

/// Great-circle distance between two coordinates (km)
fn haversine_distance(p1: GeoCoord, p2: GeoCoord) -> f64 {
    const EARTH_RADIUS_KM: f64 = 6371.0;
    let lat1 = p1.lat.to_radians();
    let lat2 = p2.lat.to_radians();
    let delta_lat = (p2.lat - p1.lat).to_radians();
    let delta_lon = (p2.lon - p1.lon).to_radians();

    let a =
        (delta_lat / 2.0).sin().powi(2) + lat1.cos() * lat2.cos() * (delta_lon / 2.0).sin().powi(2);
    let c = 2.0 * a.sqrt().atan2((1.0 - a).sqrt());

    EARTH_RADIUS_KM * c
}

/// Create a test scenario: one MRBM (North Korea → Japan) and one AEGIS
/// platform in the Sea of Japan with zero interceptors (tracking only, no
/// engagement, so the missile flies its full natural trajectory).
fn setup_test_scenario() -> SimulationEngine {
    let mut engine = SimulationEngine::new();
    engine.detection.seed_rng(42);
    engine.time_scale = TimeScale::RealTime;

    let launch_site = GeoCoord::new(39.0, 125.5);
    let target = GeoCoord::new(35.0, 139.0);
    let aegis_position = GeoCoord::new(37.0, 132.0);

    engine.add_defense_unit(
        "Test AEGIS".to_string(),
        Affiliation::Friendly,
        aegis_position,
        DefenseType::Aegis,
        0, // zero interceptors: never engages, missile flies full trajectory
    );

    engine.add_missile(
        "Test MRBM".to_string(),
        Affiliation::Hostile,
        launch_site,
        target,
        0.0,
    );

    engine
}

/// Query the converged trajectory estimate for the test missile, if any.
fn get_converged_target(engine: &SimulationEngine) -> Option<GeoCoord> {
    let defense_unit_ids: HashSet<_> = engine.defense_units.iter().map(|u| u.id).collect();
    let missile_id = engine.missiles[0].id;
    engine
        .detection
        .get_all_fused_tracks(&defense_unit_ids, engine.sim_time)
        .into_iter()
        .find(|t| t.target_id == missile_id)
        .and_then(|t| t.converged_trajectory.map(|ct| ct.target))
}

#[test]
fn test_impact_prediction_establishes_early() {
    let mut engine = setup_test_scenario();
    let dt = 0.1;

    let mut first_established: Option<f64> = None;
    for _ in 0..((120.0 / dt) as usize) {
        engine.update(dt);
        if get_converged_target(&engine).is_some() {
            first_established = Some(engine.sim_time);
            break;
        }
    }

    let t = first_established.expect("Converged trajectory never established in 120s");
    // Track establishment requires 3+ measurements at typical radar update rates;
    // the estimate should appear well within 90 seconds of launch.
    assert!(
        t <= 90.0,
        "Impact estimate established too late: {:.1}s (> 90s)",
        t
    );
    println!("Impact estimate established at t={:.1}s", t);
}

#[test]
fn test_impact_prediction_accuracy_and_stability() {
    let mut engine = setup_test_scenario();
    let true_target = engine.missiles[0].target;

    let dt = 0.1;
    let total_steps = (400.0 / dt) as usize;

    let mut prev_target: Option<GeoCoord> = None;
    let mut max_step_early: f64 = 0.0; // max marker movement per 10s window, first 60s
    let mut max_step_late: f64 = 0.0; // ...after 60s of tracking
    let mut final_error_km: Option<f64> = None;

    let mut step_10s: f64 = 0.0; // distance moved since last 10s sample
    let mut window_countdown: usize = (10.0 / dt) as usize;
    let mut tracking_started: Option<f64> = None;

    for _ in 0..total_steps {
        engine.update(dt);

        if let Some(current) = get_converged_target(&engine) {
            let track_age = match tracking_started {
                Some(t0) => engine.sim_time - t0,
                None => {
                    tracking_started = Some(engine.sim_time);
                    0.0
                }
            };

            if let Some(prev) = prev_target {
                step_10s = step_10s.max(haversine_distance(prev, current));
            }

            window_countdown -= 1;
            if window_countdown == 0 {
                // Record window max step
                if track_age <= 60.0 {
                    max_step_early = max_step_early.max(step_10s);
                } else {
                    max_step_late = max_step_late.max(step_10s);
                }
                step_10s = 0.0;
                window_countdown = (10.0 / dt) as usize;
            }

            prev_target = Some(current);
            final_error_km = Some(haversine_distance(current, true_target));
        }
    }

    let final_err = final_error_km.expect("No converged estimate during entire flight");
    println!("Final impact prediction error: {:.1} km", final_err);
    println!(
        "Max marker step (early, <=60s tracking): {:.1} km / 10s",
        max_step_early
    );
    println!(
        "Max marker step (late, >60s tracking): {:.1} km / 10s",
        max_step_late
    );

    // Converged: prediction should be within 150 km of truth for an MRBM.
    // (Radar bearing noise over ~1000km range gives tens-of-km cross-range error;
    // 150 km allows margin for early-life fusion noise.)
    assert!(
        final_err < 150.0,
        "Impact prediction error {:.1} km exceeds 150 km tolerance",
        final_err
    );

    // Stable: after the first 60s of tracking, the marker should not move more
    // than 60 km per 10-second window (i.e., no more wandering along the track).
    assert!(
        max_step_late < 60.0,
        "Impact marker still wandering late in track: {:.1} km per 10s (limit 60)",
        max_step_late
    );

    // And overall it should be calmer than the early refinement phase.
    assert!(
        max_step_late <= max_step_early || max_step_late < 20.0,
        "Late-phase marker movement ({:.1} km/10s) should be <= early-phase ({:.1} km/10s)",
        max_step_late,
        max_step_early
    );
}
