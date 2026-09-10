use crate::effects::EffectType;
use crate::map::GeoCoord;
use crate::simulation::{
    haversine_distance, Affiliation, DefenseType, DefenseUnit, EntityId, Interceptor,
    InterceptorStatus, MissReason, Missile, MissileStatus, SimEventType,
};
use std::collections::HashMap;

/// Detailed information about an intercept attempt (hit or miss)
#[derive(Clone, Debug)]
pub struct InterceptResult {
    pub id: EntityId,
    pub time: f64,
    pub interceptor_type: String,
    pub defense_unit_name: String,
    pub defense_type: DefenseType,
    pub target_name: String,
    pub intercept_position: GeoCoord,
    pub intercept_altitude_km: f64,
    pub launch_position: GeoCoord,
    pub distance_from_platform_km: f64,
    pub flight_time_sec: f64,
    pub closure_speed_km_s: f64,
    pub target_velocity_km_s: f64,
    pub interceptor_velocity_km_s: f64,
    /// Whether the attempt killed the target
    pub was_hit: bool,
    /// Pk at the moment of the intercept attempt (None if not resolved)
    pub final_pk: Option<f64>,
    /// 3D miss distance at the attempt (None for hits / not resolved)
    pub miss_distance_km: Option<f64>,
    /// Why the attempt missed (None for hits)
    pub miss_reason: Option<MissReason>,
}

/// Event types for the event log
#[derive(Clone, Debug)]
pub enum EventType {
    MissileLaunch {
        name: String,
    },
    ThreatDetected {
        threat_name: String,
        sensor_name: String,
    },
    InterceptorLaunch {
        defense_unit: String,
        target: String,
    },
    InterceptHit {
        target: String,
    },
    InterceptMiss {
        target: String,
    },
    /// Interceptor command-destructed after a confirmed miss (FTS doctrine)
    InterceptorSelfDestruct {
        target: String,
    },
    MissileImpact {
        name: String,
    },
    DecoyDeployed {
        missile_name: String,
        decoys_active: u32,
    },
    AllThreatsNeutralized,
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

impl From<SimEventType> for EventType {
    fn from(event: SimEventType) -> Self {
        match event {
            SimEventType::GuidanceUpdate {
                interceptor_type,
                target,
                correction_km,
                update_count,
            } => EventType::GuidanceUpdate {
                interceptor_type,
                target,
                correction_km,
                update_count,
            },
            SimEventType::GuidanceBlocked {
                interceptor_type,
                target,
                reason,
            } => EventType::GuidanceBlocked {
                interceptor_type,
                target,
                reason,
            },
        }
    }
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
    /// Total events ever added (monotonic; survives the max_events cap so
    /// consumers can reliably diff for "new events since last frame").
    pub events_added: u64,
}

impl EventLog {
    pub fn new() -> Self {
        Self {
            events: Vec::new(),
            max_events: 100,
            events_added: 0,
        }
    }

    pub fn add(&mut self, time: f64, event_type: EventType) {
        self.events.push(SimEvent { time, event_type });
        self.events_added += 1;
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
    pub intercept_results: Vec<InterceptResult>,
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
            intercept_results: Vec::new(),
            missile_states: HashMap::new(),
            interceptor_count: 0,
            intercept_states: HashMap::new(),
            decoy_counts: HashMap::new(),
        }
    }

    /// Clear all tracking state (called when loading new scenario)
    pub fn clear(&mut self) {
        self.event_log.clear();
        self.intercept_results.clear();
        self.missile_states.clear();
        self.interceptor_count = 0;
        self.intercept_states.clear();
        self.decoy_counts.clear();
    }

    /// Check for state changes and generate events
    /// Returns a list of effect requests that should be spawned
    /// interceptor_type_fn: function to get interceptor type name from DefenseType
    pub fn check_for_events(
        &mut self,
        sim_time: f64,
        missiles: &[Missile],
        interceptors: &[Interceptor],
        defense_units: &[DefenseUnit],
        interceptor_type_fn: impl Fn(DefenseType) -> String,
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
                self.decoy_counts
                    .insert(missile.id, missile.decoys_deployed);
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
                            target: target_name.clone(),
                        },
                    );

                    // Find the target missile to get its velocity
                    let target_missile = missiles.iter().find(|m| m.id == interceptor.target_id);
                    let target_velocity = target_missile
                        .map(|m| {
                            // Estimate velocity from trajectory: distance / flight_time
                            let range = haversine_distance(m.origin, m.target);
                            range / m.flight_time.max(1.0)
                        })
                        .unwrap_or(4.0); // Default ~4 km/s for MRBM

                    // Find defense unit name
                    let defense_unit = defense_units
                        .iter()
                        .find(|u| u.id == interceptor.launcher_id);
                    let unit_name = defense_unit
                        .map(|u| u.name.clone())
                        .unwrap_or_else(|| "Unknown".to_string());

                    // Calculate straight-line distance from launch position to intercept
                    let horizontal_dist = haversine_distance(
                        interceptor.launch_position,
                        interceptor.target_position,
                    );
                    let altitude_diff = interceptor.target_altitude_km;
                    let distance_3d = (horizontal_dist.powi(2) + altitude_diff.powi(2)).sqrt();

                    // Flight time
                    let flight_time = interceptor.current_flight_time;

                    // Closure speed = interceptor velocity + target velocity (head-on)
                    let closure_speed = interceptor.current_velocity_km_s + target_velocity;

                    // Store detailed intercept result
                    // Use actual interceptor position (where intercept occurred), not predicted target position
                    self.intercept_results.push(InterceptResult {
                        id: interceptor.id,
                        time: sim_time,
                        interceptor_type: interceptor_type_fn(interceptor.defense_type),
                        defense_unit_name: unit_name,
                        defense_type: interceptor.defense_type,
                        target_name,
                        intercept_position: interceptor.position,
                        intercept_altitude_km: interceptor.altitude_km,
                        launch_position: interceptor.launch_position,
                        distance_from_platform_km: distance_3d,
                        flight_time_sec: flight_time,
                        closure_speed_km_s: closure_speed,
                        target_velocity_km_s: target_velocity,
                        interceptor_velocity_km_s: interceptor.current_velocity_km_s,
                        was_hit: true,
                        final_pk: interceptor.final_pk,
                        miss_distance_km: interceptor.final_miss_distance_km,
                        miss_reason: None,
                    });

                    // Request intercept visual effect at actual intercept location
                    effects.push(EffectRequest {
                        position: interceptor.position,
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
                            target: target_name.clone(),
                        },
                    );

                    // Record the failed attempt as an intercept result for BDA
                    let unit_name = defense_units
                        .iter()
                        .find(|u| u.id == interceptor.launcher_id)
                        .map(|u| u.name.clone())
                        .unwrap_or_else(|| "Unknown".to_string());
                    let horizontal_dist = haversine_distance(
                        interceptor.launch_position,
                        interceptor.target_position,
                    );
                    let altitude_diff = interceptor.target_altitude_km;
                    let distance_3d = (horizontal_dist.powi(2) + altitude_diff.powi(2)).sqrt();
                    self.intercept_results.push(InterceptResult {
                        id: interceptor.id,
                        time: sim_time,
                        interceptor_type: interceptor_type_fn(interceptor.defense_type),
                        defense_unit_name: unit_name,
                        defense_type: interceptor.defense_type,
                        target_name,
                        intercept_position: interceptor.position,
                        intercept_altitude_km: interceptor.altitude_km,
                        launch_position: interceptor.launch_position,
                        distance_from_platform_km: distance_3d,
                        flight_time_sec: interceptor.current_flight_time,
                        closure_speed_km_s: interceptor.current_velocity_km_s,
                        target_velocity_km_s: 0.0,
                        interceptor_velocity_km_s: interceptor.current_velocity_km_s,
                        was_hit: false,
                        final_pk: interceptor.final_pk,
                        miss_distance_km: interceptor.final_miss_distance_km,
                        miss_reason: Some(interceptor.miss_reason),
                    });
                }
                // Post-miss command-destruct (FTS doctrine): the CPA tracker
                // confirmed the interceptor passed the target outside kill
                // radius, so fire control destroyed the round. Distinct from
                // the plain Miss event: same doctrine outcome (follow-up
                // fires via kill assessment), different terminal event.
                (Some(InterceptorStatus::InFlight), InterceptorStatus::SelfDestruct)
                | (None, InterceptorStatus::SelfDestruct)
                    if prev_status != Some(InterceptorStatus::SelfDestruct) =>
                {
                    self.event_log.add(
                        sim_time,
                        EventType::InterceptorSelfDestruct {
                            target: target_name.clone(),
                        },
                    );
                    // Command-destruct visual effect at the interceptor's
                    // last position (the destruct point)
                    effects.push(EffectRequest {
                        position: interceptor.position,
                        effect_type: EffectType::SelfDestruct,
                    });

                    // Record the command-destruct as a failed attempt for BDA.
                    // Self-destruct is fired *after* a confirmed miss (FTS
                    // doctrine), so the miss distance/reason describe the
                    // passed-CPA geometry.
                    let unit_name = defense_units
                        .iter()
                        .find(|u| u.id == interceptor.launcher_id)
                        .map(|u| u.name.clone())
                        .unwrap_or_else(|| "Unknown".to_string());
                    let horizontal_dist = haversine_distance(
                        interceptor.launch_position,
                        interceptor.target_position,
                    );
                    let altitude_diff = interceptor.target_altitude_km;
                    let distance_3d = (horizontal_dist.powi(2) + altitude_diff.powi(2)).sqrt();
                    self.intercept_results.push(InterceptResult {
                        id: interceptor.id,
                        time: sim_time,
                        interceptor_type: interceptor_type_fn(interceptor.defense_type),
                        defense_unit_name: unit_name,
                        defense_type: interceptor.defense_type,
                        target_name,
                        intercept_position: interceptor.position,
                        intercept_altitude_km: interceptor.altitude_km,
                        launch_position: interceptor.launch_position,
                        distance_from_platform_km: distance_3d,
                        flight_time_sec: interceptor.current_flight_time,
                        closure_speed_km_s: interceptor.current_velocity_km_s,
                        target_velocity_km_s: 0.0,
                        interceptor_velocity_km_s: interceptor.current_velocity_km_s,
                        was_hit: false,
                        final_pk: interceptor.final_pk,
                        miss_distance_km: interceptor.final_miss_distance_km,
                        miss_reason: Some(interceptor.miss_reason),
                    });
                }
                _ => {}
            }

            self.intercept_states
                .insert(interceptor.id, interceptor.status);
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::map::GeoCoord;
    use crate::simulation::{DefenseType, InterceptorPhase};

    /// Build a minimal interceptor with overridable status/Pk/miss data.
    fn make_interceptor(id: EntityId, status: InterceptorStatus) -> Interceptor {
        let mut icpt = Interceptor::new(
            id,
            9001,
            100,
            Affiliation::Friendly,
            DefenseType::THAAD,
            GeoCoord {
                lat: 35.0,
                lon: 45.0,
            },
            GeoCoord {
                lat: 36.0,
                lon: 46.0,
            },
            80.0,
            0.0,
            60.0,
            1.0,
        );
        icpt.status = status;
        icpt.phase = InterceptorPhase::Terminal;
        icpt.final_pk = Some(0.72);
        icpt.final_miss_distance_km = Some(0.35);
        icpt.miss_reason = MissReason::PkRoll;
        icpt
    }

    /// A defense unit so unit-name lookups resolve (id matches launcher_id).
    fn make_defense_unit() -> DefenseUnit {
        DefenseUnit::new(
            9001,
            "Test Battery".to_string(),
            Affiliation::Friendly,
            GeoCoord {
                lat: 35.0,
                lon: 45.0,
            },
            DefenseType::THAAD,
            8,
        )
    }

    /// A hostile missile so target-name lookups resolve (id matches target_id).
    fn make_missile() -> Missile {
        Missile::new(
            100,
            "Test Missile".to_string(),
            Affiliation::Hostile,
            GeoCoord {
                lat: 37.0,
                lon: 47.0,
            },
            GeoCoord {
                lat: 38.0,
                lon: 48.0,
            },
            600.0,
        )
    }

    /// InFlight -> Hit transition records a hit result with Pk attached.
    #[test]
    fn test_hit_records_result() {
        let mut tracker = EventTracker::new();
        let missile = make_missile();
        let unit = make_defense_unit();

        // First frame: interceptor observed in flight (records state)
        let icpt = make_interceptor(1, InterceptorStatus::InFlight);
        tracker.check_for_events(10.0, &[missile.clone()], &[icpt], &[unit.clone()], |_| {
            "THAAD".to_string()
        });

        // Second frame: transition to Hit
        let mut icpt = make_interceptor(1, InterceptorStatus::Hit);
        icpt.current_flight_time = 55.0;
        icpt.current_velocity_km_s = 2.8;
        tracker.check_for_events(70.0, &[missile.clone()], &[icpt], &[unit.clone()], |_| {
            "THAAD".to_string()
        });

        assert_eq!(tracker.intercept_results.len(), 1);
        let result = &tracker.intercept_results[0];
        assert!(result.was_hit);
        assert_eq!(result.defense_unit_name, "Test Battery");
        assert_eq!(result.target_name, "Test Missile");
        assert_eq!(result.interceptor_type, "THAAD");
        assert_eq!(result.final_pk, Some(0.72));
        assert_eq!(result.miss_distance_km, Some(0.35));
        assert_eq!(result.miss_reason, None);
        assert!((result.flight_time_sec - 55.0).abs() < 1e-9);
    }

    /// InFlight -> Miss transition records a miss result with reason.
    #[test]
    fn test_miss_records_result() {
        let mut tracker = EventTracker::new();
        let missile = make_missile();
        let unit = make_defense_unit();

        let icpt = make_interceptor(2, InterceptorStatus::InFlight);
        tracker.check_for_events(10.0, &[missile.clone()], &[icpt], &[unit.clone()], |_| {
            "THAAD".to_string()
        });

        let icpt = make_interceptor(2, InterceptorStatus::Miss);
        tracker.check_for_events(80.0, &[missile.clone()], &[icpt], &[unit.clone()], |_| {
            "THAAD".to_string()
        });

        assert_eq!(tracker.intercept_results.len(), 1);
        let result = &tracker.intercept_results[0];
        assert!(!result.was_hit);
        assert_eq!(result.final_pk, Some(0.72));
        assert_eq!(result.miss_reason, Some(MissReason::PkRoll));
        assert_eq!(result.target_name, "Test Missile");
    }

    /// InFlight -> SelfDestruct transition records a miss result (FTS doctrine).
    #[test]
    fn test_self_destruct_records_result() {
        let mut tracker = EventTracker::new();
        let missile = make_missile();
        let unit = make_defense_unit();

        let icpt = make_interceptor(3, InterceptorStatus::InFlight);
        tracker.check_for_events(10.0, &[missile.clone()], &[icpt], &[unit.clone()], |_| {
            "THAAD".to_string()
        });

        let icpt = make_interceptor(3, InterceptorStatus::SelfDestruct);
        tracker.check_for_events(90.0, &[missile.clone()], &[icpt], &[unit.clone()], |_| {
            "THAAD".to_string()
        });

        assert_eq!(tracker.intercept_results.len(), 1);
        let result = &tracker.intercept_results[0];
        assert!(!result.was_hit);
        assert_eq!(result.miss_reason, Some(MissReason::PkRoll));
    }

    /// A frame where the interceptor status doesn't change records nothing new.
    #[test]
    fn test_no_transition_no_result() {
        let mut tracker = EventTracker::new();
        let missile = make_missile();
        let unit = make_defense_unit();

        let icpt = make_interceptor(4, InterceptorStatus::Hit);
        tracker.check_for_events(
            10.0,
            &[missile.clone()],
            &[icpt.clone()],
            &[unit.clone()],
            |_| "THAAD".to_string(),
        );
        // Same status again: no new result (guard against double-recording)
        tracker.check_for_events(
            20.0,
            &[missile.clone()],
            &[icpt.clone()],
            &[unit.clone()],
            |_| "THAAD".to_string(),
        );

        // One result: the (None, Hit) initial-transition path fired once
        assert_eq!(tracker.intercept_results.len(), 1);
    }

    /// Hit, miss, and self-destruct across distinct interceptors accumulate.
    #[test]
    fn test_mixed_results_accumulate() {
        let mut tracker = EventTracker::new();
        let missile = make_missile();
        let unit = make_defense_unit();

        // Frame 1: two interceptors in flight
        let icpts = [
            make_interceptor(5, InterceptorStatus::InFlight),
            make_interceptor(6, InterceptorStatus::InFlight),
        ];
        tracker.check_for_events(10.0, &[missile.clone()], &icpts, &[unit.clone()], |_| {
            "THAAD".to_string()
        });

        // Frame 2: one hits, one misses
        let icpts = [
            make_interceptor(5, InterceptorStatus::Hit),
            make_interceptor(6, InterceptorStatus::Miss),
        ];
        tracker.check_for_events(70.0, &[missile.clone()], &icpts, &[unit.clone()], |_| {
            "THAAD".to_string()
        });

        assert_eq!(tracker.intercept_results.len(), 2);
        assert!(tracker.intercept_results[0].was_hit);
        assert!(!tracker.intercept_results[1].was_hit);
    }
}
