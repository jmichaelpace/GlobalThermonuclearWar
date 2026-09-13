//! Simulation thread runner for decoupled simulation/rendering
//!
//! This module provides thread-safe infrastructure for running the simulation
//! on a dedicated thread while the main thread handles rendering.

use crossbeam::channel::{self, Receiver, Sender};
use parking_lot::RwLock;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

use crate::simulation::config::{
    InterceptorConfigRegistry, MissileConfigRegistry, PlatformConfigRegistry,
    SatelliteConfigRegistry, SensorConfigRegistry,
};
use crate::simulation::detection::{Detection, RadarModeState};
use crate::simulation::engine::{DebrisCloud, SimulationEngine, TimeScale};
use crate::simulation::entities::*;
use crate::simulation::EntityId;

/// Read-only snapshot of simulation state for rendering
/// This is cloned from the simulation thread and read by the render thread
#[derive(Clone)]
pub struct SimulationSnapshot {
    pub sim_time: f64,
    pub time_scale: TimeScale,
    pub missiles: Vec<Missile>,
    pub interceptors: Vec<Interceptor>,
    pub defense_units: Vec<DefenseUnit>,
    pub radar_stations: Vec<RadarStation>,
    pub satellites: Vec<Satellite>,
    pub debris_clouds: Vec<DebrisCloud>,
    /// Decoys released by hostile missiles (renderable, classifiable)
    pub decoys: Vec<crate::simulation::entities::Decoy>,
    pub active_detections: Vec<Detection>,
    pub radar_mode_states: std::collections::HashMap<EntityId, RadarModeState>,
    // Note: FusedTracks are computed on-demand, not stored in snapshot
}

impl Default for SimulationSnapshot {
    fn default() -> Self {
        Self {
            sim_time: 0.0,
            time_scale: TimeScale::Paused,
            missiles: Vec::new(),
            interceptors: Vec::new(),
            defense_units: Vec::new(),
            radar_stations: Vec::new(),
            satellites: Vec::new(),
            debris_clouds: Vec::new(),
            decoys: Vec::new(),
            active_detections: Vec::new(),
            radar_mode_states: std::collections::HashMap::new(),
        }
    }
}

/// Commands sent from the render thread to the simulation thread
#[derive(Clone, Debug)]
pub enum SimCommand {
    /// Set simulation time scale
    SetTimeScale(TimeScale),
    /// Pause the simulation
    Pause,
    /// Resume the simulation
    Resume,
    /// Load a new scenario (scenario data passed as string)
    LoadScenario(String),
    /// Stop the simulation thread
    Shutdown,
}

/// Shared state between simulation and render threads
pub struct SharedSimulation {
    /// Current simulation snapshot (read by render thread)
    pub snapshot: RwLock<SimulationSnapshot>,
    /// Command sender (render thread sends commands)
    pub command_tx: Sender<SimCommand>,
    /// Whether the simulation thread is running
    pub running: AtomicBool,
    /// Config registries (read-only, shared between threads)
    pub sensor_configs: Arc<SensorConfigRegistry>,
    pub platform_configs: Arc<PlatformConfigRegistry>,
    pub missile_configs: Arc<MissileConfigRegistry>,
    pub interceptor_configs: Arc<InterceptorConfigRegistry>,
    pub satellite_configs: Arc<SatelliteConfigRegistry>,
}

impl SharedSimulation {
    /// Create new shared simulation state
    pub fn new(
        sensor_configs: SensorConfigRegistry,
        platform_configs: PlatformConfigRegistry,
        missile_configs: MissileConfigRegistry,
        interceptor_configs: InterceptorConfigRegistry,
        satellite_configs: SatelliteConfigRegistry,
    ) -> (Arc<Self>, Receiver<SimCommand>) {
        let (command_tx, command_rx) = channel::unbounded();

        let shared = Arc::new(Self {
            snapshot: RwLock::new(SimulationSnapshot::default()),
            command_tx,
            running: AtomicBool::new(true),
            sensor_configs: Arc::new(sensor_configs),
            platform_configs: Arc::new(platform_configs),
            missile_configs: Arc::new(missile_configs),
            interceptor_configs: Arc::new(interceptor_configs),
            satellite_configs: Arc::new(satellite_configs),
        });

        (shared, command_rx)
    }

    /// Send a command to the simulation thread
    pub fn send_command(&self, cmd: SimCommand) {
        let _ = self.command_tx.send(cmd);
    }

    /// Check if simulation is running
    pub fn is_running(&self) -> bool {
        self.running.load(Ordering::Relaxed)
    }

    /// Signal the simulation thread to stop
    pub fn shutdown(&self) {
        self.running.store(false, Ordering::Relaxed);
        let _ = self.command_tx.send(SimCommand::Shutdown);
    }
}

/// Spawn the simulation thread
/// Returns a handle that can be used to join the thread
pub fn spawn_simulation_thread(
    shared: Arc<SharedSimulation>,
    mut engine: SimulationEngine,
    command_rx: Receiver<SimCommand>,
) -> JoinHandle<()> {
    thread::spawn(move || {
        let mut last_tick = Instant::now();
        const TARGET_TICK_RATE: Duration = Duration::from_millis(10); // 100Hz

        while shared.running.load(Ordering::Relaxed) {
            // Process all pending commands
            while let Ok(cmd) = command_rx.try_recv() {
                match cmd {
                    SimCommand::SetTimeScale(ts) => {
                        engine.time_scale = ts;
                    }
                    SimCommand::Pause => {
                        engine.time_scale = TimeScale::Paused;
                    }
                    SimCommand::Resume => {
                        if engine.time_scale == TimeScale::Paused {
                            engine.time_scale = TimeScale::RealTime;
                        }
                    }
                    SimCommand::LoadScenario(_scenario_data) => {
                        // TODO: Implement scenario loading
                        // This would require parsing the scenario and resetting the engine
                    }
                    SimCommand::Shutdown => {
                        shared.running.store(false, Ordering::Relaxed);
                        return;
                    }
                }
            }

            // Calculate delta time
            let now = Instant::now();
            let dt = now.duration_since(last_tick).as_secs_f64();
            last_tick = now;

            // Update simulation if not paused
            if engine.time_scale != TimeScale::Paused {
                engine.update(dt);

                // Create snapshot for rendering
                let snapshot = create_snapshot(&engine);
                *shared.snapshot.write() = snapshot;
            }

            // Sleep to maintain target tick rate
            let elapsed = last_tick.elapsed();
            if elapsed < TARGET_TICK_RATE {
                thread::sleep(TARGET_TICK_RATE - elapsed);
            }
        }
    })
}

/// Create a snapshot of the current simulation state
fn create_snapshot(engine: &SimulationEngine) -> SimulationSnapshot {
    SimulationSnapshot {
        sim_time: engine.sim_time,
        time_scale: engine.time_scale,
        missiles: engine.missiles.clone(),
        interceptors: engine.interceptors.clone(),
        defense_units: engine.defense_units.clone(),
        radar_stations: engine.radar_stations.clone(),
        satellites: engine.satellites.clone(),
        debris_clouds: engine.debris_clouds.clone(),
        decoys: engine.decoys.clone(),
        active_detections: engine.detection.active_detections.clone(),
        radar_mode_states: engine.detection.radar_mode_states.clone(),
    }
}
