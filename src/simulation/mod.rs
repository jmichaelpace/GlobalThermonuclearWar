pub mod config;
pub mod detection;
pub mod ekf;
pub mod engine;
pub mod entities;
pub mod kalman;
pub mod physics;
pub mod runner;

pub use config::{
    InterceptorConfigRegistry, MissileConfigRegistry, PlatformConfig, PlatformConfigRegistry,
    SensorConfig, SensorConfigRegistry,
};
pub use detection::{
    calculate_position_from_bearing_range, FusedTrack, SensorKind, VelocityEstimate,
};
pub use engine::{SimulationEngine, TimeScale};
pub use entities::*;
pub use physics::*;
pub use runner::{spawn_simulation_thread, SharedSimulation, SimCommand, SimulationSnapshot};

/// Simulation event types for tracking/logging
/// This is a core version that doesn't depend on UI modules
#[derive(Clone, Debug)]
pub enum SimEventType {
    /// Mid-course guidance update sent to interceptor
    GuidanceUpdate {
        interceptor_type: String,
        target: String,
        correction_km: f64,
        update_count: u32,
    },
    /// Mid-course guidance blocked (for debugging)
    GuidanceBlocked {
        interceptor_type: String,
        target: String,
        reason: String,
    },
}
