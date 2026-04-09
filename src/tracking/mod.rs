use crate::effects::EffectType;
use crate::map::GeoCoord;
use crate::simulation::{
    Affiliation, DefenseUnit, EntityId, Interceptor, InterceptorStatus, Missile, MissileStatus,
};
use std::collections::HashMap;

/// Event types for the event log
#[derive(Clone, Debug)]
pub enum EventType {
    MissileLaunch { name: String },
    ThreatDetected { threat_name: String, sensor_name: String },
    InterceptorLaunch { defense_unit: String, target: String },
    InterceptHit { target: String },
    InterceptMiss { target: String },
    MissileImpact { name: String },
    DecoyDeployed { missile_name: String, decoys_active: u32 },
    AllThreatsNeutralized,
}

/// A single event in the log
#[derive(Clone, Debug)]
pub struct SimEvent {
    pub time: f64,
    pub event_type: EventType,
}

/// Event log to track simulation events
#[derive(Clone, Debug, Default)]
pub struct EventLog {
    pub events: Vec<SimEvent>,
    pub max_events: usize,
}

impl EventLog {
    pub fn new() -> Self {
        Self {
            events: Vec::new(),
            max_events: 100,
        }
    }

    pub fn add(&mut self, time: f64, event_type: EventType) {
        self.events.push(SimEvent { time, event_type });
        if self.events.len() > self.max_events {
            self.events.remove(0);
        }
    }

    pub fn clear(&mut self) {
        self.events.clear();
    }
}

/// Request to spawn a visual effect
#[derive(Clone, Debug)]
pub struct EffectRequest {
    pub position: GeoCoord,
    pub effect_type: EffectType,
}

/// Consolidated event tracking state
pub struct EventTracker {
    pub event_log: EventLog,
    missile_states: HashMap<EntityId, MissileStatus>,
    interceptor_count: usize,
    intercept_states: HashMap<EntityId, InterceptorStatus>,
    decoy_counts: HashMap<EntityId, u32>,
}

impl Default for EventTracker {
    fn default() -> Self {
        Self::new()
    }
}

impl EventTracker {
    pub fn new() -> Self {
        Self {
            event_log: EventLog::new(),
            missile_states: HashMap::new(),
            interceptor_count: 0,
            intercept_states: HashMap::new(),
            decoy_counts: HashMap::new(),
        }
    }

    /// Clear all tracking state (called when loading new scenario)
    pub fn clear(&mut self) {
        self.event_log.clear();
        self.missile_states.clear();
        self.interceptor_count = 0;
        self.intercept_states.clear();
        self.decoy_counts.clear();
    }

    /// Check for state changes and generate events
    /// Returns a list of effect requests that should be spawned
    pub fn check_for_events(
        &mut self,
        sim_time: f64,
        missiles: &[Missile],
        interceptors: &[Interceptor],
        defense_units: &[DefenseUnit],
    ) -> Vec<EffectRequest> {
        let mut effects = Vec::new();

        // Check for missile state changes
        for missile in missiles {
            let prev_status = self.missile_states.get(&missile.id).copied();

            match (prev_status, missile.status) {
                // Missile launched
                (Some(MissileStatus::PreLaunch), MissileStatus::Boost)
                | (None, MissileStatus::Boost) => {
                    if missile.affiliation == Affiliation::Hostile {
                        self.event_log.add(
                            sim_time,
                            EventType::MissileLaunch {
                                name: missile.name.clone(),
                            },
                        );
                    }
                }
                // Missile impacted
                (Some(status), MissileStatus::Impacted) if status != MissileStatus::Impacted => {
                    if missile.affiliation == Affiliation::Hostile {
                        self.event_log.add(
                            sim_time,
                            EventType::MissileImpact {
                                name: missile.name.clone(),
                            },
                        );
                        // Request impact visual effect
                        effects.push(EffectRequest {
                            position: missile.target,
                            effect_type: EffectType::Impact,
                        });
                    }
                }
                _ => {}
            }

            self.missile_states.insert(missile.id, missile.status);

            // Check for decoy deployments
            if missile.has_countermeasures {
                let prev_decoys = self.decoy_counts.get(&missile.id).copied().unwrap_or(0);
                if missile.decoys_deployed > prev_decoys {
                    self.event_log.add(
                        sim_time,
                        EventType::DecoyDeployed {
                            missile_name: missile.name.clone(),
                            decoys_active: missile.decoys_deployed,
                        },
                    );
                }
                self.decoy_counts.insert(missile.id, missile.decoys_deployed);
            }
        }

        // Check for new interceptor launches
        if interceptors.len() > self.interceptor_count {
            for interceptor in interceptors.iter().skip(self.interceptor_count) {
                // Find defense unit name
                let unit_name = defense_units
                    .iter()
                    .find(|u| u.id == interceptor.launcher_id)
                    .map(|u| format!("{} ({})", u.name, u.defense_type.name()))
                    .unwrap_or_else(|| "Unknown".to_string());

                // Find target name
                let target_name = missiles
                    .iter()
                    .find(|m| m.id == interceptor.target_id)
                    .map(|m| m.name.clone())
                    .unwrap_or_else(|| "Unknown".to_string());

                self.event_log.add(
                    sim_time,
                    EventType::InterceptorLaunch {
                        defense_unit: unit_name,
                        target: target_name,
                    },
                );
            }
        }
        self.interceptor_count = interceptors.len();

        // Check for intercept results
        for interceptor in interceptors {
            let prev_status = self.intercept_states.get(&interceptor.id).copied();

            let target_name = missiles
                .iter()
                .find(|m| m.id == interceptor.target_id)
                .map(|m| m.name.clone())
                .unwrap_or_else(|| "Unknown".to_string());

            match (prev_status, interceptor.status) {
                (Some(InterceptorStatus::InFlight), InterceptorStatus::Hit)
                | (None, InterceptorStatus::Hit)
                    if prev_status != Some(InterceptorStatus::Hit) =>
                {
                    self.event_log.add(
                        sim_time,
                        EventType::InterceptHit {
                            target: target_name,
                        },
                    );
                    // Request intercept visual effect
                    effects.push(EffectRequest {
                        position: interceptor.target_position,
                        effect_type: EffectType::Intercept,
                    });
                }
                (Some(InterceptorStatus::InFlight), InterceptorStatus::Miss)
                | (None, InterceptorStatus::Miss)
                    if prev_status != Some(InterceptorStatus::Miss) =>
                {
                    self.event_log.add(
                        sim_time,
                        EventType::InterceptMiss {
                            target: target_name,
                        },
                    );
                }
                _ => {}
            }

            self.intercept_states.insert(interceptor.id, interceptor.status);
        }

        effects
    }
}

/// Format simulation time as MM:SS
pub fn format_sim_time(time: f64) -> String {
    let total_secs = time as u64;
    let minutes = total_secs / 60;
    let seconds = total_secs % 60;
    format!("{:02}:{:02}", minutes, seconds)
}
