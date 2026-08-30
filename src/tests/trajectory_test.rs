use crate::simulation::{SimulationEngine, MissileStatus};
use crate::types::GeoCoord;
use crate::simulation::physics::{BallisticTrajectory, haversine_distance};

#[test]
fn test_missile_trajectory_impact_accuracy() {
    let mut engine = SimulationEngine::new();
    
    // Add a missile with known origin and target
    let origin = GeoCoord::new(45.0, -120.0);  // Oregon
    let target = GeoCoord::new(35.0, -100.0);  // Texas
    
    let missile_id = engine.add_missile(
        "Test Missile".to_string(),
        crate::simulation::Affiliation::Hostile,
        origin,
        target,
        0.0,  // Launch immediately
    );
    
    // Get trajectory info before simulation starts
    let missile = engine.missiles.first().unwrap();
    let trajectory = &engine.trajectories.get(&missile_id).unwrap();
    
    println!("Original missile target: {},{}", missile.target.lat, missile.target.lon);
    println!("Trajectory target: {},{}", trajectory.target.lat, trajectory.target.lon);
    println!("Missile origin: {},{}", missile.origin.lat, missile.origin.lon);
    println!("Trajectory origin: {},{}", trajectory.origin.lat, trajectory.origin.lon);
    println!("Haversine distance (origin to target): {}", haversine_distance(origin, target));
    println!("Trajectory range_km: {}", trajectory.range_km);
    
    // Run a short simulation step
    let initial_sim_time = engine.sim_time;
    engine.update(1.0);  // 1 second update
    
    // After update check the missile positions and state
    let updated_missile = &engine.missiles.first().unwrap();
    
    println!("After 1 second:");
    println!("Missile position: {},{}", updated_missile.position.lat, updated_missile.position.lon);
    println!("Missile target: {},{}", updated_missile.target.lat, updated_missile.target.lon);
    println!("Missile status: {:?}", updated_missile.status);
    println!("Flight progress: {:.3}", updated_missile.flight_progress());
    
    // Verify trajectory is consistent with missile properties
    assert_eq!(updated_missile.position, trajectory.origin, "Missile position should match origin at start");
    
    // Test for a more significant time step to check flight progress
    engine.update(5.0);  // 5 more seconds
    
    let updated_missile2 = &engine.missiles.first().unwrap();
    println!("After 6 seconds:");
    println!("Missile position: {},{}", updated_missile2.position.lat, updated_missile2.position.lon);
    
    // If missile should have progressed on trajectory, verify that it did
    if updated_missile2.flight_progress() > 0.01 {
        assert_eq!(updated_missile2.target, target, "Missile target should be preserved");
    }
}

#[test]
fn test_ballistic_trajectory_creation_consistency() {
    let origin = GeoCoord::new(45.0, -120.0);
    let target = GeoCoord::new(35.0, -100.0);
    
    // Test creating trajectory from new() function (used in missile initialization)
    let traj1 = BallisticTrajectory::new(origin, target);
    
    // Test creating trajectory with parameters like in engine code 
    let range_km = haversine_distance(origin, target);
    let apogee = 1000.0;  // Some reasonable apogee
    let flight_time = 3600.0;  // 1 hour
    
    let traj2 = BallisticTrajectory::with_params(origin, target, apogee, flight_time);
    
    println!("Trajectory 1 - Range: {:.2}km", traj1.range_km);
    println!("Trajectory 2 - Range: {:.2}km", traj2.range_km);
    println!("Trajectory 1 origin: {},{}", traj1.origin.lat, traj1.origin.lon);
    println!("Trajectory 1 target: {},{}", traj1.target.lat, traj1.target.lon);
    println!("Trajectory 2 origin: {},{}", traj2.origin.lat, traj2.origin.lon);
    println!("Trajectory 2 target: {},{}", traj2.target.lat, traj2.target.lon);
    
    // Make sure that the trajectory creation preserves target properly
    assert_eq!(traj1.target, target, "Trajectory target should match input");
    assert_eq!(traj2.target, target, "Trajectory target should match input");
    
    // Test trajectory position at different progress values
    let (pos_at_0, alt_at_0) = traj1.position_at(0.0);
    let (pos_at_1, alt_at_1) = traj1.position_at(1.0);
    
    assert_eq!(pos_at_0, origin, "Position at progress 0 should be origin");
    assert_eq!(pos_at_1, target, "Position at progress 1 should be target");
}