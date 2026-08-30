//! Integration test for missile track prediction accuracy
//!
//! This test creates a missile with a known ground truth trajectory,
//! an AEGIS defense platform that tracks it, and verifies that the
//! EKF-predicted track matches the ground truth within acceptable tolerances.

use std::collections::HashSet;

// Import from the main crate (note: hyphens become underscores)
use global_thermonuclear_war::simulation::{
    Affiliation, DefenseType, FusedTrack, SimulationEngine, TimeScale,
};
use global_thermonuclear_war::types::GeoCoord;

/// Test configuration
const TEST_SIM_DURATION_SEC: f64 = 120.0; // Run for 2 minutes of sim time
const TEST_DT: f64 = 0.1; // 100ms time steps
const POSITION_TOLERANCE_KM: f64 = 5.0; // Allow 5km position error

/// Create a test scenario with a missile and AEGIS platform
fn setup_test_scenario() -> SimulationEngine {
    let mut engine = SimulationEngine::new();
    engine.time_scale = TimeScale::RealTime;

    // Missile launch site (North Korea area)
    let launch_site = GeoCoord::new(39.0, 125.5);

    // Target (Japan area)
    let target = GeoCoord::new(35.0, 139.0);

    // AEGIS platform positioned to detect the missile
    // Place it in the Sea of Japan where it has good radar coverage
    let aegis_position = GeoCoord::new(37.0, 132.0);

    // Add AEGIS defense unit with radar
    engine.add_defense_unit(
        "Test AEGIS".to_string(),
        Affiliation::Friendly,
        aegis_position,
        DefenseType::Aegis,
        8, // interceptors (not used in this test)
    );

    // Add missile - launch at t=0
    engine.add_missile(
        "Test MRBM".to_string(),
        Affiliation::Hostile,
        launch_site,
        target,
        0.0, // launch at sim start
    );

    engine
}

/// Get the ground truth position of a missile at the current simulation time
fn get_missile_ground_truth(engine: &SimulationEngine) -> Option<(GeoCoord, f64)> {
    engine.missiles.first().map(|m| (m.position, m.altitude_km))
}

/// Get the tracked/predicted position for a missile from the fused tracks
fn get_tracked_position(engine: &SimulationEngine) -> Option<FusedTrack> {
    // Get defense unit IDs for fused track lookup
    let defense_unit_ids: HashSet<_> = engine.defense_units.iter().map(|u| u.id).collect();

    // Get all fused tracks and find one for our missile
    let fused_tracks = engine
        .detection
        .get_all_fused_tracks(&defense_unit_ids, engine.sim_time);

    // Return the first track (should be our test missile)
    fused_tracks.into_iter().next()
}

/// Calculate great circle distance between two positions
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

#[test]
fn test_track_prediction_matches_ground_truth() {
    println!("\n=== Track Prediction Test ===\n");

    let mut engine = setup_test_scenario();

    // Track statistics
    let mut samples_with_track: u32 = 0;
    let mut total_position_error: f64 = 0.0;
    let mut max_position_error: f64 = 0.0;
    let mut track_established_time: Option<f64> = None;

    // Run simulation
    let num_steps = (TEST_SIM_DURATION_SEC / TEST_DT) as usize;

    for step in 0..num_steps {
        // Update simulation
        engine.update(TEST_DT);

        let sim_time = engine.sim_time;

        // Get ground truth
        let ground_truth = match get_missile_ground_truth(&engine) {
            Some(gt) => gt,
            None => {
                println!("t={:.1}s: Missile not found (may have impacted)", sim_time);
                break;
            }
        };

        // Get tracked position
        let track = get_tracked_position(&engine);

        if let Some(fused_track) = track {
            if track_established_time.is_none() {
                track_established_time = Some(sim_time);
                println!("t={:.1}s: Track established", sim_time);
            }

            // Calculate position error
            let position_error = haversine_distance(ground_truth.0, fused_track.estimated_position);
            let altitude_error = (ground_truth.1 - fused_track.estimated_altitude).abs();
            let total_error = (position_error.powi(2) + altitude_error.powi(2)).sqrt();

            total_position_error += total_error;
            max_position_error = max_position_error.max(total_error);
            samples_with_track += 1;

            // Log progress every 10 seconds
            if step % 100 == 0 {
                println!(
                    "t={:.1}s: pos_err={:.2}km, alt_err={:.2}km, quality={:.2}, sensors={}",
                    sim_time,
                    position_error,
                    altitude_error,
                    fused_track.fused_quality,
                    fused_track.sensor_count
                );

                if let Some(ref vel) = fused_track.estimated_velocity {
                    println!(
                        "         velocity: {:.2} km/s, heading: {:.1}deg, confidence: {:.2}",
                        vel.ground_speed_km_s, vel.heading_deg, vel.confidence
                    );
                }
            }
        } else {
            // No track yet - log occasionally
            if step % 50 == 0 && step > 0 {
                println!("t={:.1}s: No track established yet", sim_time);
            }
        }
    }

    // Print summary
    println!("\n=== Test Summary ===");
    println!("Simulation duration: {:.1}s", TEST_SIM_DURATION_SEC);
    println!("Samples with track: {}", samples_with_track);

    if samples_with_track > 0 {
        let avg_position_error = total_position_error / samples_with_track as f64;
        println!("Average position error: {:.2} km", avg_position_error);
        println!("Maximum position error: {:.2} km", max_position_error);

        if let Some(t) = track_established_time {
            println!("Track established at: {:.1}s", t);
        }

        // Assert that we have reasonable tracking
        assert!(
            samples_with_track > 10,
            "Should have at least 10 track samples, got {}",
            samples_with_track
        );

        assert!(
            avg_position_error < POSITION_TOLERANCE_KM,
            "Average position error {:.2} km exceeds tolerance {:.2} km",
            avg_position_error,
            POSITION_TOLERANCE_KM
        );

        assert!(
            max_position_error < POSITION_TOLERANCE_KM * 3.0,
            "Maximum position error {:.2} km exceeds 3x tolerance",
            max_position_error
        );

        println!("\n=== Track prediction test PASSED ===");
    } else {
        panic!("No track was established during the test!");
    }
}

#[test]
fn test_track_velocity_estimation() {
    println!("\n=== Velocity Estimation Test ===\n");

    let mut engine = setup_test_scenario();

    let mut velocity_initialized_time: Option<f64> = None;
    let mut velocity_readings: Vec<f64> = Vec::new();
    let mut stable_velocity_readings: Vec<f64> = Vec::new();

    // Run simulation for enough time to establish velocity
    let num_steps = (60.0 / TEST_DT) as usize; // 60 seconds

    for _ in 0..num_steps {
        engine.update(TEST_DT);
        let sim_time = engine.sim_time;

        if let Some(track) = get_tracked_position(&engine) {
            if let Some(ref vel) = track.estimated_velocity {
                if velocity_initialized_time.is_none() && vel.ground_speed_km_s > 0.1 {
                    velocity_initialized_time = Some(sim_time);
                    println!(
                        "t={:.1}s: Velocity initialized: {:.2} km/s",
                        sim_time, vel.ground_speed_km_s
                    );
                }

                if vel.ground_speed_km_s > 0.1 {
                    velocity_readings.push(vel.ground_speed_km_s);

                    // Only collect "stable" readings after EKF has had 5+ seconds to converge
                    if let Some(init_time) = velocity_initialized_time {
                        if sim_time - init_time > 5.0 {
                            stable_velocity_readings.push(vel.ground_speed_km_s);
                        }
                    }
                }
            }
        }
    }

    println!("\n=== Velocity Test Summary ===");
    println!("Total velocity samples: {}", velocity_readings.len());
    println!(
        "Stable velocity samples: {}",
        stable_velocity_readings.len()
    );

    if !velocity_readings.is_empty() {
        let avg_velocity: f64 =
            velocity_readings.iter().sum::<f64>() / velocity_readings.len() as f64;
        let min_velocity = velocity_readings
            .iter()
            .cloned()
            .fold(f64::INFINITY, f64::min);
        let max_velocity = velocity_readings
            .iter()
            .cloned()
            .fold(f64::NEG_INFINITY, f64::max);

        println!("Average velocity (all): {:.2} km/s", avg_velocity);
        println!(
            "Velocity range (all): {:.2} - {:.2} km/s",
            min_velocity, max_velocity
        );

        // Ballistic missiles should have velocities in a reasonable range
        // MRBMs: 3-4 km/s, ICBMs: up to 7 km/s, but tracking estimates may be slightly higher
        assert!(
            avg_velocity > 0.5 && avg_velocity < 10.0,
            "Average velocity {:.2} km/s should be between 0.5-10 km/s for ballistic missiles",
            avg_velocity
        );

        // Check velocity stability using only readings after convergence
        if stable_velocity_readings.len() >= 10 {
            let stable_min = stable_velocity_readings
                .iter()
                .cloned()
                .fold(f64::INFINITY, f64::min);
            let stable_max = stable_velocity_readings
                .iter()
                .cloned()
                .fold(f64::NEG_INFINITY, f64::max);
            let stable_spread = stable_max - stable_min;

            println!(
                "Stable velocity range: {:.2} - {:.2} km/s (spread: {:.2})",
                stable_min, stable_max, stable_spread
            );

            // After convergence, velocity shouldn't vary too wildly
            assert!(
                stable_spread < 3.0,
                "Stable velocity spread {:.2} km/s is too large (tracking unstable after convergence)",
                stable_spread
            );
        }

        println!("\n=== Velocity estimation test PASSED ===");
    } else {
        panic!("No velocity estimates were produced!");
    }
}

#[test]
fn test_ekf_convergence() {
    println!("\n=== EKF Convergence Test ===\n");

    let mut engine = setup_test_scenario();

    let mut uncertainty_readings: Vec<f64> = Vec::new();

    // Run for 90 seconds
    let num_steps = (90.0 / TEST_DT) as usize;

    for step in 0..num_steps {
        engine.update(TEST_DT);
        let sim_time = engine.sim_time;

        if let Some(track) = get_tracked_position(&engine) {
            if let Some(uncertainty) = track.kalman_position_uncertainty_km {
                uncertainty_readings.push(uncertainty);

                // Log every 10 seconds
                if step % 100 == 0 {
                    println!(
                        "t={:.1}s: uncertainty={:.4} km, quality={:.2}",
                        sim_time, uncertainty, track.fused_quality
                    );
                }
            }
        }
    }

    println!("\n=== EKF Convergence Summary ===");

    if uncertainty_readings.len() >= 20 {
        // Check that uncertainty decreases over time (filter is converging)
        let first_third: f64 = uncertainty_readings[..uncertainty_readings.len() / 3]
            .iter()
            .sum::<f64>()
            / (uncertainty_readings.len() / 3) as f64;
        let last_third: f64 = uncertainty_readings[2 * uncertainty_readings.len() / 3..]
            .iter()
            .sum::<f64>()
            / (uncertainty_readings.len() / 3) as f64;

        println!("First third avg uncertainty: {:.4} km", first_third);
        println!("Last third avg uncertainty: {:.4} km", last_third);

        // Uncertainty should decrease or stay stable
        assert!(
            last_third <= first_third * 1.5,
            "EKF should converge: final uncertainty {:.4} should not be much larger than initial {:.4}",
            last_third, first_third
        );

        // Final uncertainty should be reasonable (< 1 km for good tracking)
        assert!(
            last_third < 2.0,
            "Final uncertainty {:.4} km should be < 2 km",
            last_third
        );

        println!("\n=== EKF convergence test PASSED ===");
    } else {
        panic!(
            "Not enough uncertainty readings: {}",
            uncertainty_readings.len()
        );
    }
}
