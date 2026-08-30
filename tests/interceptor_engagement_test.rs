//! Interceptor engagement integration tests.
//!
//! Validates the interceptor logic overhaul:
//!   1. Aegis deterministically intercepts an MRBM using only sensor-derived
//!      data (converged trajectory projection + unified arrival-time math).
//!   2. THAAD never engages an ICBM whose apogee exceeds its 150 km ceiling
//!      (engagement envelope strictly enforced).
//!   3. No launches occur without fire-control-quality tracks (sensor-based
//!      fire control, not omniscient).
//!   4. Shoot-Look-Shoot continues engaging after a miss while shots and
//!      time remain (max_shots_per_target, not the old 2-shot total cap).

use std::collections::HashSet;

use global_thermonuclear_war::simulation::{
    Affiliation, DefenseType, InterceptorStatus, SimulationEngine, TimeScale,
};
use global_thermonuclear_war::types::GeoCoord;

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

/// Aegis destroyer in the Sea of Japan vs. a North Korea -> Japan MRBM.
fn setup_aegis_vs_mrbm(interceptor_count: u32) -> SimulationEngine {
    let mut engine = SimulationEngine::new();
    engine.time_scale = TimeScale::RealTime;

    engine.add_defense_unit(
        "Test AEGIS".to_string(),
        Affiliation::Friendly,
        GeoCoord::new(37.0, 132.0),
        DefenseType::Aegis,
        interceptor_count,
    );

    engine.add_missile(
        "Test MRBM".to_string(),
        Affiliation::Hostile,
        GeoCoord::new(39.0, 125.5),
        GeoCoord::new(35.0, 139.0),
        0.0,
    );

    engine
}

/// Run until the missile resolves (intercepted or impacted) or timeout.
/// Returns (final_missile_status_intercepted, sim_time).
fn run_until_resolved(engine: &mut SimulationEngine, max_time: f64) -> (bool, f64) {
    let dt = 0.1;
    let steps = (max_time / dt) as usize;
    let mut intercepted = false;

    for _ in 0..steps {
        engine.update(dt);
        let missile = &engine.missiles[0];
        match missile.status {
            global_thermonuclear_war::simulation::MissileStatus::Intercepted => {
                intercepted = true;
                break;
            }
            global_thermonuclear_war::simulation::MissileStatus::Impacted => break,
            _ => {}
        }
    }

    (intercepted, engine.sim_time)
}

#[test]
fn test_aegis_intercepts_mrbm_deterministic() {
    let mut engine = setup_aegis_vs_mrbm(8);
    let (intercepted, t) = run_until_resolved(&mut engine, 900.0);

    // Report engagement details on failure for diagnosis
    let shots_fired = engine.interceptors.len();
    let hits = engine
        .interceptors
        .iter()
        .filter(|i| i.status == InterceptorStatus::Hit)
        .count();
    let misses = engine
        .interceptors
        .iter()
        .filter(|i| i.status == InterceptorStatus::Miss)
        .count();
    println!(
        "Aegis vs MRBM: shots={}, hits={}, misses={}, resolved at t={:.0}s, intercepted={}",
        shots_fired, hits, misses, t, intercepted
    );

    assert!(
        intercepted,
        "Aegis failed to intercept MRBM: shots={}, hits={}, misses={} by t={:.0}s",
        shots_fired, hits, misses, t
    );
}

#[test]
fn test_thaad_cannot_engage_icbm_apogee() {
    let mut engine = SimulationEngine::new();
    engine.time_scale = TimeScale::RealTime;

    // THAAD battery positioned under the ICBM flight path
    engine.add_defense_unit(
        "Test THAAD".to_string(),
        Affiliation::Friendly,
        GeoCoord::new(45.0, -100.0),
        DefenseType::THAAD,
        8,
    );

    // Continental-range ICBM: apogee ~1200+ km, far above THAAD's 150 km ceiling
    engine.add_missile(
        "Test ICBM".to_string(),
        Affiliation::Hostile,
        GeoCoord::new(40.0, -120.0),
        GeoCoord::new(38.0, -77.0),
        0.0,
    );

    // Verify the missile's actual apogee exceeds THAAD's envelope
    let missile_id = engine.missiles[0].id;
    let missile_apogee = engine
        .get_trajectory(missile_id)
        .map(|t| t.max_altitude_km)
        .expect("trajectory must exist");
    assert!(
        missile_apogee > 150.0,
        "test premise: ICBM apogee {:.0} km should exceed THAAD ceiling",
        missile_apogee
    );

    // Run through midcourse
    let dt = 0.1;
    for _ in 0..((600.0 / dt) as usize) {
        engine.update(dt);
    }

    let launches = engine
        .interceptors
        .iter()
        .filter(|i| i.status != InterceptorStatus::Pending || true)
        .count();
    println!("THAAD vs ICBM: interceptors launched = {}", launches);

    assert!(
        launches == 0,
        "THAAD must not engage an ICBM above its 150 km apogee ceiling, but {} interceptors launched",
        launches
    );
}

#[test]
fn test_no_launch_without_fire_control_track() {
    let mut engine = SimulationEngine::new();
    engine.time_scale = TimeScale::RealTime;

    // Aegis placed FAR from the missile's flight path: the missile is never
    // detected, so no track is ever established. Fire control must refuse to
    // engage — sensors, not omniscience, drive launches.
    engine.add_defense_unit(
        "Distant AEGIS".to_string(),
        Affiliation::Friendly,
        GeoCoord::new(-30.0, 30.0), // South Atlantic
        DefenseType::Aegis,
        8,
    );

    engine.add_missile(
        "Test MRBM".to_string(),
        Affiliation::Hostile,
        GeoCoord::new(39.0, 125.5), // North Korea
        GeoCoord::new(35.0, 139.0), // Japan
        0.0,
    );

    let dt = 0.1;
    for _ in 0..((300.0 / dt) as usize) {
        engine.update(dt);
    }

    let launches = engine.interceptors.len();
    let tracked = {
        let ids: HashSet<_> = engine.defense_units.iter().map(|u| u.id).collect();
        engine
            .detection
            .get_all_fused_tracks(&ids, engine.sim_time)
            .len()
    };
    println!(
        "Distant Aegis: tracked targets = {}, launches = {}",
        tracked, launches
    );

    // No detections -> no tracks -> no launches
    assert!(
        launches == 0,
        "Interceptor launched ({}) against a target the sensor never detected",
        launches
    );
}

#[test]
fn test_salvo_continues_after_miss() {
    // Use an aggressive MRBM target geometry that is genuinely hard but
    // possible: Aegis has 8 interceptors; with max_shots_per_target = 4 the
    // engagement continues after misses instead of capping at 2 total shots.
    let mut engine = setup_aegis_vs_mrbm(8);

    let dt = 0.1;
    let mut any_hit = false;
    for _ in 0..((900.0 / dt) as usize) {
        engine.update(dt);
        if engine
            .interceptors
            .iter()
            .any(|i| i.status == InterceptorStatus::Hit)
        {
            any_hit = true;
            break;
        }
    }

    let total_shots = engine.interceptors.len();
    let misses = engine
        .interceptors
        .iter()
        .filter(|i| i.status == InterceptorStatus::Miss)
        .count();
    println!(
        "Salvo test: total_shots={}, misses={}, any_hit={}",
        total_shots, misses, any_hit
    );

    // Whether or not a hit occurs in this stochastic run, the engagement must
    // be ALLOWED to continue past the old 2-shot cap while misses happen:
    // if 2+ misses occurred, more shots must have been authorized.
    if misses >= 2 {
        assert!(
            total_shots > 2,
            "Engagement stopped at {} shots despite {} misses (old 2-shot total cap)",
            total_shots,
            misses
        );
    }
}
