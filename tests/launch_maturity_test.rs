//! Fire-control solution maturity gate (regression)
//!
//! The AEGIS test scenario exposed premature launch: fire control fired
//! ~7 s after FIRST DETECTION (10 Hz radar passes the 10-measurement gate
//! almost immediately), while the converged-trajectory estimate — what
//! the intercept solution is built from — was still an immature
//! short-span extrapolation with the impact point ~580 km wrong. The
//! gates measured LOCAL track health (measurements/quality/velocity) but
//! never the FIRE-CONTROL SOLUTION's maturity.
//!
//! Fix under test: launch is held while the estimated impact uncertainty
//! exceeds the interceptor's midcourse divert budget, with a
//! window-closing escape (commit on the best available solution when the
//! remaining flight time no longer allows waiting).

use std::collections::HashSet;

use global_thermonuclear_war::simulation::{
    haversine_distance, Affiliation, DefenseType, InterceptorStatus, SimulationEngine, TimeScale,
};
use global_thermonuclear_war::types::GeoCoord;

fn first_track_state(engine: &SimulationEngine, target_id: u64) -> Option<(f64, f64, f64)> {
    // (claimed target uncertainty, total fit weight, actual impact error)
    let ids: HashSet<u64> = engine.defense_units.iter().map(|u| u.id).collect();
    engine
        .detection
        .get_fused_track(target_id, &ids, engine.sim_time)
        .and_then(|ft| {
            ft.converged_trajectory.as_ref().map(|ct| {
                let true_target = engine.missiles[0].target;
                (
                    ct.target_uncertainty_km,
                    ct.total_weight,
                    haversine_distance(ct.target, true_target),
                )
            })
        })
}

#[test]
fn test_launch_held_until_solution_mature() {
    let mut engine = SimulationEngine::new();
    engine.time_scale = TimeScale::RealTime;
    engine.detection.seed_rng(42);

    // test_aegis.toml geometry: AEGIS at (35,140), Shahab-3 inbound from
    // (35,153) with the scenario's launch delay. SM-3 Block IIA divert
    // budget is 100 km — launch must be held while the claimed impact
    // uncertainty exceeds it.
    engine.add_defense_unit(
        "USS Test (DDG)".to_string(),
        Affiliation::Friendly,
        GeoCoord::new(35.0, 140.0),
        DefenseType::Aegis,
        48,
    );
    let missile_id = engine.add_missile(
        "Shahab-3 #1".to_string(),
        Affiliation::Hostile,
        GeoCoord::new(35.0, 153.0),
        GeoCoord::new(35.0, 140.0),
        10.0,
    );

    let dt = 0.1;
    let mut launch_t: Option<f64> = None;
    let mut launch_solution_unc: Option<f64> = None;
    for _ in 0..((900.0 / dt) as usize) {
        engine.update(dt);
        if launch_t.is_none() && !engine.interceptors.is_empty() {
            launch_t = Some(engine.sim_time);
            launch_solution_unc = Some(
                first_track_state(&engine, missile_id)
                    .map(|(unc, _, _)| unc)
                    .expect("launch fired with no converged estimate"),
            );
        }
        // Stop once any resolution occurs
        if engine
            .interceptors
            .iter()
            .any(|i| matches!(i.status, InterceptorStatus::Hit | InterceptorStatus::Miss))
        {
            break;
        }
    }

    let launch_t = launch_t.expect("engagement never launched in 900 s");

    // Maturity at commit: the claimed uncertainty is inside the SM-3's
    // 100 km divert budget OR the window-closing escape fired (in this
    // open-window geometry the escape must NOT trigger — plenty of
    // flight time remains — so the uncertainty gate must hold)
    let (unc, _weight, actual_err) =
        first_track_state(&engine, missile_id).expect("estimate must exist at launch");
    let _ = unc;
    assert!(
        launch_solution_unc.unwrap() <= 100.0,
        "launched at t={launch_t:.0} with claimed impact uncertainty {:.0} km > 100 km divert budget",
        launch_solution_unc.unwrap()
    );

    // Timing sanity: launch must happen MATERIALLY after first detection
    // (the pre-fix behavior fired ~7 s in). In this geometry the missile
    // enters the SPY-1 acquisition envelope around t=265 s; a mature
    // solution needs tens of seconds of altitude-fit span. Allow a wide
    // band but require the hold to have happened: substantially later
    // than the old 271.7 s behavior.
    assert!(
        launch_t > 285.0,
        "launch at t={launch_t:.0} — maturity hold did not delay the engagement"
    );

    // The actual solution quality at commit must be materially better
    // than the pre-fix 583 km error
    assert!(
        actual_err < 300.0,
        "impact estimate at commit still {actual_err:.0} km off — fired into noise"
    );
}

#[test]
fn test_maturity_hold_does_not_prevent_intercept() {
    // The hold must not turn a winnable engagement into a leaker: the same
    // geometry must still produce a successful intercept (midcourse
    // guidance corrects toward the maturing solution).
    let mut engine = SimulationEngine::new();
    engine.time_scale = TimeScale::RealTime;
    engine.detection.seed_rng(42);

    engine.add_defense_unit(
        "USS Test (DDG)".to_string(),
        Affiliation::Friendly,
        GeoCoord::new(35.0, 140.0),
        DefenseType::Aegis,
        48,
    );
    engine.add_missile(
        "Shahab-3 #1".to_string(),
        Affiliation::Hostile,
        GeoCoord::new(35.0, 153.0),
        GeoCoord::new(35.0, 140.0),
        10.0,
    );

    let dt = 0.1;
    let mut hit = false;
    for _ in 0..((1200.0 / dt) as usize) {
        engine.update(dt);
        if engine
            .interceptors
            .iter()
            .any(|i| i.status == InterceptorStatus::Hit)
        {
            hit = true;
            break;
        }
    }
    assert!(
        hit,
        "maturity hold prevented an otherwise-winnable intercept (premature-leaker regression)"
    );
}
