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
//!   5. Post-miss command-destruct: an interceptor that passes its target
//!      mid-course (outside the terminal phase) resolves as SelfDestruct
//!      promptly instead of flying on to the flight-time timeout, and the
//!      miss feeds shoot-look-shoot follow-up.
//!   6. Synchronized-arrival doctrine: fire control refuses launch solutions
//!      where the interceptor would arrive at the intercept point far
//!      EARLIER than the missile (a chase geometry with no kill window), and
//!      in-flight guidance breaks off (destructs) when no synchronized
//!      solution remains.

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
    // Deterministic measurement noise (realistic calibration bias + seeded
    // Gaussian scatter); the engagement geometry itself is unchanged
    engine.detection.seed_rng(42);

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
    engine.detection.seed_rng(42);

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

    // Every interceptor in the vector counts as a launch attempt (Pending
    // entries only exist post-launch scheduling)
    let launches = engine.interceptors.len();
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
    engine.detection.seed_rng(42);

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

/// Post-miss command-destruct doctrine.
///
/// A mistimed interceptor that passes its target OUTSIDE the terminal phase
/// must resolve as SelfDestruct (a miss outcome) promptly via the CPA
/// divergence tracker — not keep flying toward a stale intercept point until
/// the 1.2x-1.3x flight-time timeout. The miss must also feed the
/// shoot-look-shoot kill-assessment chain.
///
/// This is a white-box resolution test: the interceptor is staged directly
/// with a garbage intercept solution (bypassing the sensor-driven launch
/// chain, which the other tests cover), and the defending unit is placed
/// where it cannot detect the missile, so mid-course guidance is blocked
/// ("no fused track") and cannot re-target the round. The geometry is pure
/// divergence: the interceptor launches ahead of the missile's aimpoint and
/// flies away from it, so 3D range opens monotonically from burnout.
#[test]
fn test_post_miss_command_destruct_mid_course() {
    let mut engine = SimulationEngine::new();
    engine.time_scale = TimeScale::RealTime;
    engine.detection.seed_rng(42);

    let missile_id = engine.add_missile(
        "Staged MRBM".to_string(),
        Affiliation::Hostile,
        GeoCoord::new(39.0, 125.5),
        GeoCoord::new(35.0, 139.0),
        0.0,
    );

    // Unit far from the flight path: it can never detect the missile, so
    // no fused track exists and mid-course guidance cannot re-target the
    // staged interceptor (mirrors test_no_launch_without_fire_control_track).
    let unit_id = engine.add_defense_unit(
        "Staged THAAD".to_string(),
        Affiliation::Friendly,
        GeoCoord::new(-30.0, 30.0), // South Atlantic
        DefenseType::THAAD,
        8,
    );

    // Advance the sim so the missile is mid-flight (t=300s of a ~600s flight:
    // the missile is roughly at the down-range midpoint (37, 132)).
    let dt = 0.1;
    for _ in 0..((300.0 / dt) as usize) {
        engine.update(dt);
    }
    let t0 = engine.sim_time;
    assert_eq!(
        engine.missiles[0].status,
        global_thermonuclear_war::simulation::MissileStatus::Midcourse,
        "test premise: missile should be mid-course at t={:.0}s",
        t0
    );

    // Stage the interceptor BEHIND the missile (an already-passed point on
    // the trajectory), aimed further BACKWARD along the path. The missile
    // moves away down-range while the interceptor flies away backward:
    // range opens monotonically from the first post-boost frame. No closing
    // phase, no kill — pure divergence the CPA tracker must catch in
    // mid-course (Coast phase, progress ~0.4 << terminal threshold 0.7).
    use global_thermonuclear_war::simulation::Interceptor;
    let interceptor = Interceptor::new(
        9999,
        unit_id,
        missile_id,
        Affiliation::Friendly,
        DefenseType::THAAD,         // 12s boost then coast; guidance interval 2.0s
        GeoCoord::new(37.5, 129.0), // behind the missile's current position
        GeoCoord::new(39.0, 126.0), // aim further backward along the path
        80.0,                       // aim altitude (km)
        t0,
        t0 + 40.0, // nominal flight; timeout paths need >= 48s (progress 1.2)
        0.5,
    );
    let interceptor_id = interceptor.id;
    engine.interceptors.push(interceptor);

    // The CPA destruct path should resolve at ~12s boost + ~3s coast
    // hysteresis (1.5x THAAD's 2.0s guidance interval). Run to 40s:
    // comfortably before any timeout path could fire (48s).
    let mut resolved_at: Option<f64> = None;
    for _ in 0..((40.0 / dt) as usize) {
        engine.update(dt);
        let ic = engine
            .interceptors
            .iter()
            .find(|i| i.id == interceptor_id)
            .unwrap();
        if ic.status != InterceptorStatus::InFlight {
            resolved_at = Some(engine.sim_time);
            break;
        }
    }

    let ic = engine
        .interceptors
        .iter()
        .find(|i| i.id == interceptor_id)
        .unwrap();
    println!(
        "Post-miss destruct: status={:?} at t={:?} (launched t={:.0}s), final_miss_dist={:?}, cpa_min={:.1}km",
        ic.status, resolved_at, t0, ic.final_miss_distance_km, ic.cpa_tracking.min_distance_km
    );

    assert!(
        matches!(
            ic.status,
            InterceptorStatus::SelfDestruct | InterceptorStatus::Miss
        ),
        "mistimed interceptor must resolve as a miss/destruct, got {:?}",
        ic.status
    );
    let resolved_at = resolved_at.expect("interceptor must resolve within 40s");
    assert!(
        resolved_at - t0 < 40.0,
        "resolution {:.0}s after launch — CPA destruct should fire well before the 48s timeout path",
        resolved_at - t0
    );
    assert_eq!(
        ic.miss_reason,
        global_thermonuclear_war::simulation::MissReason::OffCourse
    );

    // The destruct must count as a MISS for doctrine: a kill assessment
    // (was_kill=false) is created, enabling shoot-look-shoot follow-up.
    assert!(
        engine
            .kill_assessments
            .iter()
            .any(|ka| ka.target_id == missile_id && !ka.was_kill),
        "no miss kill-assessment queued for shoot-look-shoot follow-up"
    );
}

/// Synchronized-arrival doctrine: no chase shots at receding targets.
///
/// Reproduces the tail-chase bug from the AEGIS geometry scenario: after the
/// first (legitimate) crossing engagement resolved, fire control kept
/// computing "solutions" 50-133 s EARLY — the interceptor beats the missile
/// to the intercept point by minutes — and launched chase shots that could
/// never close (seeker never acquired, 15-20 km CPA misses, missile
/// impacted). Doctrine: launch only inside a time-synchronized window
/// (|interceptor_time - missile_time| within tolerance); if no synchronized
/// point exists in the envelope, HOLD FIRE and accept the leaker.
///
/// This test stages the pure receding geometry: by the time a fire-control
/// track exists, the missile is flying AWAY from the platform and its
/// remaining trajectory stays above SM-3's 600 km ceiling. Every launch
/// solution must be refused. (The real scenario's first CROSSING shot is
/// legitimate — it launches while the missile is still approaching — so
/// this test isolates the receding window with a delayed detection start.)
#[test]
fn test_no_chase_launch_on_receding_target() {
    use global_thermonuclear_war::simulation::MissileStatus;

    let mut engine = SimulationEngine::new();
    engine.time_scale = TimeScale::RealTime;
    engine.detection.seed_rng(42);

    // Aegis positioned near the missile's TARGET (far side of the flight):
    // the missile only becomes detectable as it approaches the platform's
    // hemisphere, by which point it is descending TOWARD impact at (33,125)
    // and flying away from the ship. Its remaining trajectory is terminal
    // descent through altitudes above SM-3's 100 km floor until the final
    // seconds — no exo intercept point exists at all, synchronized or not.
    engine.add_defense_unit(
        "Receding AEGIS".to_string(),
        Affiliation::Friendly,
        GeoCoord::new(33.0, 125.0), // directly at the impact point
        DefenseType::Aegis,
        8,
    );

    // Crossing MRBM passing far north of the ship (from test_aegis_geometry)
    let missile_id = engine.add_missile(
        "Crossing MRBM".to_string(),
        Affiliation::Hostile,
        GeoCoord::new(33.0, 155.0),
        GeoCoord::new(33.0, 125.0),
        0.0,
    );

    let dt = 0.1;
    // Run past the missile's apogee: mid-course, high altitude, receding
    // from the ship at the far end. (Apogee ~700 km for this range class.)
    for _ in 0..((700.0 / dt) as usize) {
        engine.update(dt);
    }

    let missile = &engine.missiles[0];
    let apogee = engine
        .get_trajectory(missile_id)
        .map(|t| t.max_altitude_km)
        .expect("trajectory");
    println!(
        "Receding test: t={:.0}s missile_status={:?} progress={:.2} apogee={:.0}km alt={:.0}km",
        engine.sim_time,
        missile.status,
        missile.flight_progress(),
        apogee,
        missile.altitude_km
    );
    assert!(
        matches!(missile.status, MissileStatus::Midcourse),
        "test premise: missile should be mid-course, got {:?}",
        missile.status
    );

    // Continue running through the rest of the engagement window. Fire
    // control may engage only while a SYNCHRONIZED solution exists (e.g.,
    // during terminal descent if the missile were headed at the ship) —
    // for this crossing geometry none ever exists: every solution the scan
    // finds has the missile arriving at the IP long before the interceptor
    // (chase) or after it (far early), both refused.
    let mut chase_launches = 0;
    let mut any_launch = false;
    for _ in 0..((2400.0 / dt) as usize) {
        engine.update(dt);
        if engine.interceptors.len() > 0 && !any_launch {
            any_launch = true;
            println!(
                "note: launch occurred at t={:.0}s (may be a legitimate terminal solution)",
                engine.sim_time
            );
        }
        // A "chase launch" would be a shot at an interceptor solution the
        // missile reaches BEFORE the interceptor by a wide margin — detect
        // via the aspect angle at launch: approach from behind the missile's
        // heading (< 60 deg) means tail-chase.
        for ic in engine.interceptors.iter() {
            if ic.status == InterceptorStatus::Pending || ic.status == InterceptorStatus::InFlight {
                let m = engine
                    .missiles
                    .iter()
                    .find(|m| m.id == ic.target_id)
                    .unwrap();
                let m_heading = bearing_deg(m.origin, m.target);
                let approach = bearing_deg(ic.launch_position, ic.target_position);
                let diff = (approach - m_heading).rem_euclid(360.0);
                let aspect = if diff > 180.0 { 360.0 - diff } else { diff };
                if aspect < 60.0 {
                    chase_launches += 1;
                }
            }
        }
        if matches!(
            engine.missiles[0].status,
            MissileStatus::Impacted | MissileStatus::Intercepted
        ) {
            break;
        }
    }

    println!(
        "Receding test: any_launch={} chase_launches={} missile_status={:?} shots_total={}",
        any_launch,
        chase_launches,
        engine.missiles[0].status,
        engine.interceptors.len()
    );

    // Doctrine: no chase-geometry launch may EVER occur
    assert_eq!(
        chase_launches, 0,
        "fire control launched a chase shot at a receding target (tail-chase aspect < 60 deg)"
    );
}

/// Helper: bearing in degrees (0 = North, 90 = East) — mirrors physics::bearing.
fn bearing_deg(from: GeoCoord, to: GeoCoord) -> f64 {
    let lat1 = from.lat.to_radians();
    let lat2 = to.lat.to_radians();
    let dlon = (to.lon - from.lon).to_radians();

    let y = dlon.sin() * lat2.cos();
    let x = lat1.cos() * lat2.sin() - lat1.sin() * lat2.cos() * dlon.cos();
    y.atan2(x).to_degrees().rem_euclid(360.0)
}
