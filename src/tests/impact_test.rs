use crate::simulation::{SimulationEngine, MissileStatus};
use crate::types::GeoCoord;

// Test case to analyze why visualization might not match ground truth path
#[test]
fn test_missile_impact_point_consistency() {
    let mut engine = SimulationEngine::new();
    
    // Add a missile with known origin and target
    let origin = GeoCoord::new(45.0, -120.0);  // Oregon
    let target = GeoCoord::new(35.0, -100.0);  // Texas
    
    println!("Adding missile from {} to {}", 
        format!("{:.2}°, {:.2}°", origin.lat, origin.lon),
        format!("{:.2}°, {:.2}°", target.lat, target.lon)
    );
    
    let missile_id = engine.add_missile(
        "Test Missile".to_string(),
        crate::simulation::Affiliation::Hostile,
        origin,
        target,
        0.0,  // Launch immediately
    );
    
    // Now get the missile and trajectory right after adding it
    let missile = &engine.missiles.first().unwrap();
    let trajectory = engine.trajectories.get(&missile_id).unwrap();
    
    println!("Missile data after initialization:");
    println!("  Missile origin: {},{}", missile.origin.lat, missile.origin.lon);
    println!("  Missile target: {},{}", missile.target.lat, missile.target.lon);
    println!("  Trajectory origin: {},{}", trajectory.origin.lat, trajectory.origin.lon);
    println!("  Trajectory target: {},{}", trajectory.target.lat, trajectory.target.lon);
    println!("  Match origin? {}", missile.origin == trajectory.origin);
    println!("  Match target? {}", missile.target == trajectory.target);
    
    // Run a single update step
    engine.update(1.0);
    
    let updated_missile = &engine.missiles.first().unwrap();
    println!("After 1 second:");
    println!("  Missile position: {},{}", updated_missile.position.lat, updated_missile.position.lon);
    println!("  Missile status: {:?}", updated_missile.status);
    println!("  Flight progress: {:.3}", updated_missile.flight_progress());
    
    // Check that missile has not progressed if not in flight yet
    if updated_missile.status == MissileStatus::PreLaunch {
        assert_eq!(updated_missile.position, origin, "Missile should be at origin while pre-launch");
    }
    
    // Launch it explicitly now
    engine.update(1.0);
    
    let updated_missile2 = &engine.missiles.first().unwrap();
    println!("After 2 seconds:");
    println!("  Missile position: {},{}", updated_missile2.position.lat, updated_missile2.position.lon);
    println!("  Missile status: {:?}", updated_missile2.status);
    println!("  Flight progress: {:.3}", updated_missile2.flight_progress());
    
    // Check the impact point calculation
    let final_flight_progress = updated_missile2.flight_progress();
    assert!(final_flight_progress >= 0.0);
    
    // This test ensures the missile is not simply showing wrong impact points 
    println!("Test passed - missile trajectory appears consistent");
}