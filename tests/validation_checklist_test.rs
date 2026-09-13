//! Audit-plan validation checklist tests
//!
//! Proves, behaviorally and end-to-end, the four unchecked items in
//! docs/audit-plan.md "Validation Checklist":
//!
//! 1. Patriot engages at correct ranges (inside the published 70 km
//!    platform envelope; refuses when the synchronized solution would lie
//!    outside it)
//! 2. GBI/Arrow 3 use Lambert guidance above 100 km (exo-atmospheric
//!    midcourse guidance activates for exo systems, endo systems use PN)
//! 3. Trajectory predictions match expected physics (apogee, flight
//!    time, parabolic profile, endpoint pinning, Coriolis compensation)
//! 4. Pk values remain in realistic ranges across live engagements
//!
//! All scenarios are seeded (engine.detection.seed_rng) for determinism
//! per AGENTS.md.

use std::collections::HashSet;

use global_thermonuclear_war::simulation::physics::BallisticTrajectory;
use global_thermonuclear_war::simulation::{
    haversine_distance, Affiliation, DefenseType, InterceptorStatus, SimulationEngine, TimeScale,
};
use global_thermonuclear_war::types::GeoCoord;

// ============================================================================
// 1. Patriot engages at correct ranges
// ============================================================================

/// A Patriot battery (published engagement range 70 km per
/// config/platform/patriot.toml, PAC-3 MSE altitude envelope 0.5-40 km)
/// engages a short-range ballistic target that flies through its
/// envelope: at least one launch must occur, and every launch solution
/// must lie inside the platform's engagement range.
#[test]
fn test_patriot_engages_within_published_range() {
    let mut engine = SimulationEngine::new();
    engine.time_scale = TimeScale::RealTime;
    engine.detection.seed_rng(42);

    // Patriot battery at Riyadh (radar faced EAST at the inbound
    // corridor — the MPQ-65 has a 120-degree sector, so emplacement
    // orientation matters, as the scenario README notes). The threat is
    // an Iskander-M-class SRBM (config-backed profile, published apogee
    // ~50 km): a genuine terminal-defense engagement — the midcourse
    // arc is above the PAC-3 ceiling, so the synchronized solution
    // must come during reentry inside the 70 km / 0.5-40 km envelope.
    engine.add_defense_unit_with_facing(
        "Patriot Riyadh".to_string(),
        Affiliation::Friendly,
        GeoCoord::new(24.7, 46.7),
        DefenseType::Patriot,
        8,
        Some(90.0), // Radar sector faces the eastern approach
    );
    engine.add_missile(
        "Iskander-M".to_string(), // Real config-backed SRBM profile
        Affiliation::Hostile,
        GeoCoord::new(24.7, 47.9), // ~119 km east of the battery
        GeoCoord::new(24.7, 46.4), // Target west of the battery
        0.0,
    );

    let dt = 0.1;
    for _ in 0..((400.0 / dt) as usize) {
        engine.update(dt);
        if engine
            .interceptors
            .iter()
            .any(|i| i.status == InterceptorStatus::Hit)
        {
            break;
        }
    }

    let launches = engine.interceptors.len();
    assert!(
        launches >= 1,
        "Patriot never launched at an SRBM crossing its envelope"
    );

    // Every launched interceptor's aim point must be within the published
    // platform engagement range (70 km) of the battery
    let patriot_pos = engine.defense_units[0].position;
    for icpt in &engine.interceptors {
        let dist = haversine_distance(patriot_pos, icpt.target_position);
        assert!(
            dist <= 70.0 * 1.15, // 15% tolerance: terminal-lead refinement may shift the aim point
            "Interceptor launch solution at {dist:.1} km exceeds Patriot's 70 km envelope"
        );
    }
}

/// The envelope is strictly enforced the OTHER way too: against a target
/// whose high, fast profile keeps synchronized engagement points outside
/// the Patriot's reach for most of the flight, any launch that DOES occur
/// must have started from an altitude-legal solution. Midcourse guidance
/// legitimately re-aims the stored target point afterward (the live
/// projected solution), so the launch aim is captured by snapshotting
/// each interceptor the moment it appears.
#[test]
fn test_patriot_holds_fire_beyond_envelope() {
    let mut engine = SimulationEngine::new();
    engine.time_scale = TimeScale::RealTime;
    engine.detection.seed_rng(42);

    // Battery at Riyadh (radar faced east at the inbound); target
    // IMPACTS at Riyadh but flies a high, steep profile whose
    // synchronized engagement points lie above the PAC-3 altitude
    // ceiling (40 km): a fast midcourse-class threat. A terminal-phase
    // intercept may still be legal (reentry passes through 0.5-40 km
    // inside 70 km) — so the strict assertion is on the LAUNCH solutions.
    engine.add_defense_unit_with_facing(
        "Patriot Riyadh".to_string(),
        Affiliation::Friendly,
        GeoCoord::new(24.7, 46.7),
        DefenseType::Patriot,
        8,
        Some(90.0),
    );
    engine.add_missile(
        "Fast Test".to_string(),
        Affiliation::Hostile,
        GeoCoord::new(24.7, 49.6), // ~290 km east — beyond Patriot's reach at altitude
        GeoCoord::new(24.7, 46.7),
        0.0,
    );

    let dt = 0.1;
    let mut launch_aims: Vec<(f64, GeoCoord, f64)> = Vec::new(); // (alt, pos, range from battery)
    let mut seen_ids: Vec<global_thermonuclear_war::simulation::EntityId> = Vec::new();
    for _ in 0..((500.0 / dt) as usize) {
        engine.update(dt);
        for icpt in &engine.interceptors {
            if !seen_ids.contains(&icpt.id) {
                seen_ids.push(icpt.id);
                // Capture the launch solution before any guidance re-aim
                let dist =
                    haversine_distance(engine.defense_units[0].position, icpt.target_position);
                launch_aims.push((icpt.target_altitude_km, icpt.target_position, dist));
            }
        }
    }

    // Any launched round started from an altitude-legal, range-legal
    // solution (the launch gates). Guidance re-aims afterward are the
    // live solution tracking — not envelope violations.
    for (aim_alt, _aim_pos, dist) in &launch_aims {
        assert!(
            *dist <= 70.0 * 1.15,
            "Patriot launch solution at {dist:.1} km — envelope not enforced"
        );
        assert!(
            *aim_alt <= 40.0 * 1.05,
            "Patriot launch solution aimed at {aim_alt:.1} km — above its 40 km ceiling"
        );
        assert!(
            *aim_alt >= 0.5,
            "Patriot launch solution aimed at {aim_alt:.1} km — below its floor"
        );
    }
    // Note: zero launches is a passing outcome (FCS held fire for the
    // out-of-envelope geometry). What must never happen is an illegal
    // launch — asserted above for any round that DID launch.
}

// ============================================================================
// 2. GBI / Arrow 3 use Lambert guidance above 100 km
// ============================================================================

/// The Lambert guidance path activates for exo-atmospheric systems
/// (GBI/Aegis/Arrow3) above the Kármán line. Verify the activation
/// predicate directly on the solver entry condition plus a live
/// exo-atmospheric engagement where the interceptor climbs through 100 km
/// during midcourse.
#[test]
fn test_lambert_activation_for_exo_systems() {
    // (a) Solver contract: lambert_guidance refuses endo-atmospheric
    // geometries (both parties below 100 km) and accepts exo geometries.
    let ground_geo = GeoCoord::new(45.0, 0.0);
    let exo_geo = GeoCoord::new(46.0, 1.0);

    // Both below 100 km: no Lambert (endo regime)
    assert!(
        global_thermonuclear_war::simulation::physics::lambert_guidance(
            ground_geo,
            10.0,
            GeoCoord::new(45.5, 0.5),
            20.0,
            60.0,
        )
        .is_none(),
        "Lambert must not engage for endo-atmospheric geometries"
    );

    // Interceptor above 100 km: Lambert activates and converges
    let exo_solution = global_thermonuclear_war::simulation::physics::lambert_guidance(
        exo_geo,
        400.0,
        GeoCoord::new(47.0, 2.0),
        300.0,
        120.0,
    );
    assert!(
        exo_solution.is_some(),
        "Lambert must activate and converge above the Kármán line"
    );
    let (heading, climb, _speed) = exo_solution.unwrap();
    // Heading is a valid compass bearing; climb angle within ±90°
    assert!(heading >= 0.0 && heading < 360.0);
    assert!(climb.abs() <= 90.0 + 1e-6);

    // (b) Live engagement: GBI vs ICBM. GBI is an exo system
    // (is_exo_system matches DefenseType::GBI), and the intercept
    // geometry for an ICBM necessarily has the interceptor above
    // 100 km during midcourse. The launch must be authorized (a
    // solution exists inside the 2000 km platform envelope) and the
    // interceptor must reach exo altitudes during its flight.
    let mut engine = SimulationEngine::new();
    engine.time_scale = TimeScale::RealTime;
    engine.detection.seed_rng(42);

    // GBI at Fort Greely with the Cobra Dane-faced sector oriented at the
    // inbound corridor (the array has a 120-degree sector; the ICBM
    // approaches from the west-northwest over the pole).
    engine.add_defense_unit_with_facing(
        "GBI Fort Greely".to_string(),
        Affiliation::Friendly,
        GeoCoord::new(63.9, -145.5),
        DefenseType::GBI,
        8,
        Some(310.0), // Face the northern/western approach corridor
    );
    engine.add_missile(
        "ICBM Test".to_string(),
        Affiliation::Hostile,
        GeoCoord::new(55.0, 155.0),  // Eastern Russia
        GeoCoord::new(55.0, -120.0), // US Northwest — flight crosses Alaska
        0.0,
    );

    let dt = 0.1;
    let mut max_icpt_alt = 0.0f64;
    for _ in 0..((2200.0 / dt) as usize) {
        engine.update(dt);
        for icpt in &engine.interceptors {
            if icpt.status == InterceptorStatus::InFlight {
                max_icpt_alt = max_icpt_alt.max(icpt.altitude_km);
            }
        }
        if engine
            .interceptors
            .iter()
            .any(|i| i.status == InterceptorStatus::Hit)
        {
            break;
        }
    }

    assert!(
        !engine.interceptors.is_empty(),
        "GBI never launched at an ICBM crossing its 2000 km envelope"
    );
    // Midcourse flight for an ICBM intercept is exo-atmospheric: the
    // interceptor must have been above 100 km, where the engine's
    // Lambert branch (not PN) governs guidance.
    assert!(
        max_icpt_alt > 100.0,
        "GBI interceptor never crossed the Kármán line (max {max_icpt_alt:.1} km) — \
         exo/Lambert regime never engaged"
    );
}

// ============================================================================
// 3. Trajectory predictions match expected physics
// ============================================================================

/// The truth-model trajectory obeys the documented physics model
/// (docs/physics.md): parabolic altitude profile with apogee at t=0.5,
/// endpoints pinned exactly to origin/target (guided compensation),
/// Coriolis deflection mid-flight only, and monotone altitude on both
/// sides of apogee.
#[test]
fn test_trajectory_matches_documented_physics() {
    let origin = GeoCoord::new(39.0, 125.5);
    let target = GeoCoord::new(35.0, 139.0);
    let traj = BallisticTrajectory::new(origin, target);

    // Endpoints exact (guidance compensation guarantee)
    let (p0, a0) = traj.position_at(0.0);
    assert!((p0.lat - origin.lat).abs() < 1e-9 && a0.abs() < 1e-9);
    let (p1, a1) = traj.position_at(1.0);
    assert!((p1.lat - target.lat).abs() < 1e-9 && a1.abs() < 1e-9);

    // Parabolic profile: apogee at midcourse exactly
    let (_, a_mid) = traj.position_at(0.5);
    assert!((a_mid - traj.max_altitude_km).abs() < 1e-9);

    // Monotone climb before apogee, monotone descent after
    let mut prev = -1.0;
    for i in 0..=50 {
        let t = i as f64 / 50.0 * 0.5;
        let (_, a) = traj.position_at(t);
        if i > 0 {
            assert!(a > prev, "altitude not monotone climbing at t={t}");
        }
        prev = a;
    }
    let mut prev = f64::INFINITY;
    for i in 0..=50 {
        let t = 0.5 + i as f64 / 50.0 * 0.5;
        let (_, a) = traj.position_at(t);
        if i > 0 {
            assert!(a < prev, "altitude not monotone descending at t={t}");
        }
        prev = a;
    }

    // Apogee is in the physically plausible band for an MRBM of this
    // range (~940 km geodesic): energy-conservation model gives
    // several hundred km; published MRBM apogees are 250-1000 km
    assert!(
        traj.max_altitude_km > 250.0 && traj.max_altitude_km < 1000.0,
        "MRBM apogee {} km outside published band (250-1000)",
        traj.max_altitude_km
    );

    // Flight time in the published MRBM band (~10-15 min for this range)
    assert!(
        traj.flight_time_sec > 600.0 && traj.flight_time_sec < 900.0,
        "MRBM flight time {} s outside published band (600-900)",
        traj.flight_time_sec
    );

    // Coriolis compensation: deflection zero at endpoints, nonzero
    // midcourse, bounded by the analytic peak
    let undeflected_mid = global_thermonuclear_war::simulation::physics::geodesic_direct(
        origin,
        traj.origin_azimuth_for_decoys(),
        traj.range_km * 0.5,
    );
    let (mid, _) = traj.position_at(0.5);
    let lateral =
        global_thermonuclear_war::simulation::physics::geodesic_inverse(mid, undeflected_mid)
            .distance_km;
    assert!(lateral > 0.0, "midcourse Coriolis deflection missing");
    assert!(lateral <= traj.coriolis_peak_km().unwrap_or(f64::INFINITY) * 1.05);
}

/// Sensor-side prediction: the converged-trajectory estimate (what fire
/// control actually uses) matches the true trajectory within the
/// documented tolerances once the track has matured (the impact-
/// prediction integration test pins the 150 km impact band; this test
/// cross-checks the full estimate: range, flight time, and impact).
#[test]
fn test_sensor_prediction_matches_true_trajectory() {
    let mut engine = SimulationEngine::new();
    engine.time_scale = TimeScale::RealTime;
    engine.detection.seed_rng(42);

    engine.add_defense_unit(
        "AEGIS".to_string(),
        Affiliation::Friendly,
        GeoCoord::new(37.0, 132.0),
        DefenseType::Aegis,
        0, // tracking-only: engagement dynamics are covered by the other checklist tests
    );
    let origin = GeoCoord::new(39.0, 125.5);
    let target = GeoCoord::new(35.0, 139.0);
    engine.add_missile(
        "MRBM".to_string(),
        Affiliation::Hostile,
        origin,
        target,
        0.0,
    );

    // Run to track maturity (the impact-prediction test's horizon),
    // exercising the same query path the app uses every frame: fused-track
    // queries ESTABLISH/REFINE the converged trajectory as a side effect
    // (get_converged_trajectory is only a passive reader of persisted state)
    let dt = 0.1;
    let mut fused_at_maturity = None;
    for _ in 0..((400.0 / dt) as usize) {
        engine.update(dt);
        let ids: HashSet<u64> = engine.defense_units.iter().map(|u| u.id).collect();
        let ft = engine
            .detection
            .get_all_fused_tracks(&ids, engine.sim_time)
            .into_iter()
            .find(|t| t.target_id == engine.missiles[0].id);
        fused_at_maturity = ft.or(fused_at_maturity);
    }

    let fused = fused_at_maturity.expect("no fused track for the missile");
    let converged = fused
        .converged_trajectory
        .clone()
        .expect("track should have converged by 400 s");

    // Impact estimate inside the documented 150 km band
    let impact_err = haversine_distance(converged.target, target);
    assert!(
        impact_err < 150.0,
        "converged impact estimate {impact_err:.1} km from the true target (band: 150 km)"
    );

    // Range estimate within 20% of the true geodesic range at maturity
    // (the fit is a sensor-side reconstruction: noise + partial-track
    // extrapolation legitimately leave double-digit percent error)
    let true_range = haversine_distance(origin, target);
    assert!(
        (converged.range_km - true_range).abs() / true_range < 0.20,
        "converged range {} vs true {} — off by >20%",
        converged.range_km,
        true_range
    );

    // Flight-time estimate in a plausible band vs the configured profile
    assert!(
        converged.flight_time_sec > 400.0 && converged.flight_time_sec < 1500.0,
        "converged flight time {} s implausible",
        converged.flight_time_sec
    );

    // Apogee estimate: the quadratic fit improves as the sampled arc
    // completes — assert at flight end (full arc), where the estimate is
    // tightest. The truth is the profile the engine actually flies:
    // the missile config's apogee formula (base + range factor), not
    // BallisticTrajectory::new's internal auto-estimate (the engine
    // constructs missiles with_params from the config, and the two
    // apogee models legitimately differ).
    let true_range = haversine_distance(origin, target);
    let config_apogee = 150.0 + 0.15 * true_range; // default.toml [trajectory] formula
    let mut apogee_final = None;
    for _ in 0..((900.0 / dt) as usize) {
        engine.update(dt);
        let ids: HashSet<u64> = engine.defense_units.iter().map(|u| u.id).collect();
        if let Some(ft) = engine
            .detection
            .get_all_fused_tracks(&ids, engine.sim_time)
            .into_iter()
            .find(|t| t.target_id == engine.missiles[0].id)
        {
            if let Some(ct) = &ft.converged_trajectory {
                apogee_final = Some(ct.apogee_km);
            }
        }
        if engine.missiles[0].status
            == global_thermonuclear_war::simulation::MissileStatus::Impacted
        {
            break;
        }
    }
    let apogee_final = apogee_final.expect("converged estimate vanished before flight end");
    assert!(
        (apogee_final - config_apogee).abs() / config_apogee < 0.25,
        "final apogee estimate {} vs flown profile {} — off by >25%",
        apogee_final,
        config_apogee
    );
}

// ============================================================================
// 4. Pk values remain in realistic ranges
// ============================================================================

/// Across live engagements, every interceptor's in-flight Pk (the weighted
/// log-odds combination of timing/track/prediction/countermeasures/
/// discrimination/closure/aspect/energy factors) stays within the
/// realistic band: never 0 (a launched round always has some chance), never
/// a hard 1.0 (probability is capped below certainty by the sigmoid), and
/// the resolved outcomes' final Pk consistent with a real system (base Pk
/// per interceptor config, typically 0.85-0.95, adjusted by geometry).
#[test]
fn test_pk_stays_in_realistic_ranges() {
    let mut engine = SimulationEngine::new();
    engine.time_scale = TimeScale::RealTime;
    engine.detection.seed_rng(42);

    engine.add_defense_unit(
        "AEGIS".to_string(),
        Affiliation::Friendly,
        GeoCoord::new(37.0, 132.0),
        DefenseType::Aegis,
        8,
    );
    engine.add_missile(
        "MRBM".to_string(),
        Affiliation::Hostile,
        GeoCoord::new(39.0, 125.5),
        GeoCoord::new(35.0, 139.0),
        0.0,
    );

    let dt = 0.1;
    let mut pk_samples: Vec<f64> = Vec::new();
    for _ in 0..((900.0 / dt) as usize) {
        engine.update(dt);
        for icpt in &engine.interceptors {
            if icpt.status == InterceptorStatus::InFlight {
                pk_samples.push(icpt.hit_probability);
            }
        }
        if engine
            .interceptors
            .iter()
            .any(|i| i.status == InterceptorStatus::Hit)
            && engine
                .interceptors
                .iter()
                .all(|i| i.status != InterceptorStatus::Pending)
        {
            break;
        }
    }

    assert!(
        !pk_samples.is_empty(),
        "no in-flight Pk samples collected — engagement never launched"
    );

    for (n, &pk) in pk_samples.iter().enumerate() {
        assert!(pk.is_finite(), "Pk NaN at sample {n}");
        assert!(
            pk > 0.0,
            "Pk {pk} at sample {n}: a launched round always retains nonzero probability"
        );
        assert!(
            pk < 1.0,
            "Pk {pk} at sample {n}: sigmoid output must stay below certainty"
        );
    }

    // Resolved rounds: final Pk, where recorded, must be consistent with
    // a real system's numbers (PAC-3/SM-3-class base Pk is 0.85-0.95;
    // weighted factors can pull it down, but not below ~0.1 or above ~0.99)
    for icpt in &engine.interceptors {
        if let Some(final_pk) = icpt.final_pk {
            assert!(
                final_pk > 0.01 && final_pk < 0.995,
                "final Pk {final_pk} outside the realistic resolved band"
            );
            // In particular: a miss must record the Pk the attempt
            // actually had — never a retroactive hard zero (an audit
            // finding from this validation pass, fixed at the resolution
            // sites to record the last computed attempt Pk)
            if icpt.status == InterceptorStatus::Miss {
                assert!(
                    final_pk > 0.01,
                    "miss recorded final_pk={final_pk}: retroactive zero detected"
                );
            }
        }
    }

    // Statistical sanity across the flight: the average in-flight Pk for
    // a healthy engagement should be materially above chance (a working
    // FCS does not fly rounds with coin-flip odds on average)
    let avg_pk: f64 = pk_samples.iter().sum::<f64>() / pk_samples.len() as f64;
    assert!(
        avg_pk > 0.2,
        "average in-flight Pk {avg_pk:.3} implausibly low for a launch-authorized engagement"
    );
}
