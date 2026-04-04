pub mod config;
pub mod detection;
pub mod engine;
pub mod entities;
pub mod kalman;
pub mod physics;

pub use config::{
    DefenseConfig, DefenseConfigRegistry,
    SensorConfig, SensorConfigRegistry, RadarBand,
    InterceptorConfig, InterceptorConfigRegistry,
    SatelliteConfig, SatelliteConfigRegistry,
    MissileConfig, MissileConfigRegistry, MissileType, ConfigError
};
pub use detection::{Detection, DetectionSystem, FusedTrack, SensorKind, TrackingState, PositionMeasurement, VelocityEstimate, calculate_position_from_bearing_range};
pub use engine::{SimulationEngine, TimeScale};
pub use entities::*;
pub use physics::*;
