//! Sensor-derived threat alert assessment (DEFCON-style).
//!
//! Computes a defense-condition level from FUSED TRACKS ONLY — an inbound
//! threat the sensor network has not established a track on does not raise
//! the alert level. This mirrors the sim's sensor-only fire-control doctrine
//! (AGENTS.md): no ground-truth fallback anywhere in this module.
//!
//! Tracks are gated by the same quality requirements fire control uses
//! (track establishment: 3+ measurements, quality >= 0.4, staleness < 5s),
//! so weak/flaky contacts contribute less urgency.
//!
//! The level also drives a warning klaxon: `AlertTracker` latches the
//! previous level and emits the contributing track only on ESCALATION,
//! with a cooldown so fluctuating tracks don't blare repeatedly.

use crate::simulation::detection::FusedTrack;
use std::time::Instant;

/// Defense-condition alert levels (1 = most severe, echoing DEFCON).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord)]
pub enum AlertLevel {
    /// Active terminal-phase leakers inbound (no time to spare)
    Defcon1,
    /// Terminal-phase tracks engaged but not yet killed
    Defcon2,
    /// Threats inbound, engaged or engaging
    Defcon3,
    /// Tracks established, no engagements yet
    Defcon4,
    /// No credible tracked threats
    #[default]
    Defcon5,
}

impl AlertLevel {
    pub fn name(self) -> &'static str {
        match self {
            AlertLevel::Defcon1 => "DEFCON 1",
            AlertLevel::Defcon2 => "DEFCON 2",
            AlertLevel::Defcon3 => "DEFCON 3",
            AlertLevel::Defcon4 => "DEFCON 4",
            AlertLevel::Defcon5 => "DEFCON 5",
        }
    }

    /// Display color for the level (used by the UI readout).
    pub fn color(self) -> (u8, u8, u8) {
        match self {
            AlertLevel::Defcon1 => (255, 40, 40),   // Red
            AlertLevel::Defcon2 => (255, 120, 40),  // Orange
            AlertLevel::Defcon3 => (255, 210, 50),  // Yellow
            AlertLevel::Defcon4 => (150, 200, 255), // Blue
            AlertLevel::Defcon5 => (120, 220, 140), // Green
        }
    }

    fn severity(self) -> u8 {
        match self {
            AlertLevel::Defcon1 => 1,
            AlertLevel::Defcon2 => 2,
            AlertLevel::Defcon3 => 3,
            AlertLevel::Defcon4 => 4,
            AlertLevel::Defcon5 => 5,
        }
    }
}

/// Per-track urgency inputs extracted from a fused track.
#[derive(Clone, Debug)]
pub struct TrackThreat {
    /// Whether the track meets the fire-control quality gate
    pub quality_gated: bool,
    /// Time to impact estimate from the converged trajectory (None if the
    /// trajectory hasn't converged yet)
    pub time_to_impact_sec: Option<f64>,
    /// Whether an in-flight interceptor is assigned to this track
    pub engaged: bool,
}

/// Minimum track quality for a track to count toward the alert level
/// (matches fire-control gate: quality >= 0.4).
const MIN_TRACK_QUALITY: f64 = 0.4;
/// Maximum staleness for a track to count (matches fire-control gate: < 5s).
const MAX_STALENESS_SEC: f64 = 5.0;
/// Minimum measurements for a track to count (matches gate: 3+).
const MIN_MEASUREMENTS: usize = 3;
/// Terminal phase begins when time-to-impact drops below this.
const TERMINAL_TTI_SEC: f64 = 120.0;
/// Below this time-to-impact the track is a leaker (kill unlikely in time).
const LATE_TTI_SEC: f64 = 45.0;

/// Convert a fused track into a per-track threat summary.
///
/// `engaged_targets` carries the ids of missiles with an in-flight
/// interceptor assigned (derived from interceptor state by the caller).
pub fn track_threat_from_fused(
    track: &FusedTrack,
    sim_time: f64,
    engaged_targets: &std::collections::HashSet<u64>,
) -> TrackThreat {
    let quality_gated = track.fused_quality >= MIN_TRACK_QUALITY
        && track.staleness_seconds <= MAX_STALENESS_SEC
        && track.measurement_count >= MIN_MEASUREMENTS;

    // Sensor-derived time to impact from the converged trajectory: the
    // estimate's total flight time minus elapsed time since establishment.
    let time_to_impact_sec = track.converged_trajectory.as_ref().map(|traj| {
        let elapsed = sim_time - traj.established_at_sim_time;
        (traj.flight_time_sec - elapsed).max(0.0)
    });

    TrackThreat {
        quality_gated,
        time_to_impact_sec,
        engaged: engaged_targets.contains(&track.target_id),
    }
}

/// Compute the alert level from per-track threat summaries.
///
/// Levels (from most to least severe):
/// - DEFCON 1: any quality-gated terminal track un-engaged with TTI < 45s
///   (leaker — terminal defense is the only option left)
/// - DEFCON 2: any quality-gated terminal track (TTI < 120s)
/// - DEFCON 3: any quality-gated track with a converged trajectory inbound
/// - DEFCON 4: any quality-gated track (tracked but trajectory not converged)
/// - DEFCON 5: no quality-gated tracks
pub fn assess_tracks(tracks: &[TrackThreat]) -> AlertLevel {
    let gated: Vec<&TrackThreat> = tracks.iter().filter(|t| t.quality_gated).collect();
    if gated.is_empty() {
        return AlertLevel::Defcon5;
    }

    let mut level = AlertLevel::Defcon5;

    for t in &gated {
        match t.time_to_impact_sec {
            Some(tti) if tti < LATE_TTI_SEC && !t.engaged => {
                return AlertLevel::Defcon1;
            }
            Some(tti) if tti < TERMINAL_TTI_SEC => {
                level = level.min(AlertLevel::Defcon2);
            }
            Some(_) => {
                level = level.min(AlertLevel::Defcon3);
            }
            None => {
                level = level.min(AlertLevel::Defcon4);
            }
        }
    }
    level
}

/// Latches the previous alert level and gates the warning klaxon.
///
/// The klaxon fires only on ESCALATION (level gets numerically smaller) and
/// at most once per cooldown window, so a track bouncing between levels
/// doesn't blare repeatedly. DE-escalation never sounds.
pub struct AlertTracker {
    last_level: AlertLevel,
    /// Contributing threat for the most recent escalation (None = quiet)
    last_klaxon_at: Option<Instant>,
}

impl Default for AlertTracker {
    fn default() -> Self {
        Self::new()
    }
}

impl AlertTracker {
    /// Minimum wall-clock time between klaxons.
    const KLAXON_COOLDOWN_SEC: f64 = 10.0;

    pub fn new() -> Self {
        Self {
            last_level: AlertLevel::Defcon5,
            last_klaxon_at: None,
        }
    }

    pub fn last_level(&self) -> AlertLevel {
        self.last_level
    }

    /// Feed the current level; returns true if the warning klaxon should
    /// sound NOW (escalation detected and cooldown elapsed).
    pub fn update(&mut self, level: AlertLevel) -> bool {
        let escalated = level.severity() < self.last_level.severity();
        self.last_level = level;
        if !escalated {
            return false;
        }
        // Escalation: check cooldown
        if let Some(last) = self.last_klaxon_at {
            if last.elapsed().as_secs_f64() < Self::KLAXON_COOLDOWN_SEC {
                return false;
            }
        }
        self.last_klaxon_at = Some(Instant::now());
        true
    }

    /// Reset on scenario load/restart (starts fresh at DEFCON 5, klaxon armed).
    pub fn reset(&mut self) {
        self.last_level = AlertLevel::Defcon5;
        self.last_klaxon_at = None;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn threat(gated: bool, tti: Option<f64>, engaged: bool) -> TrackThreat {
        TrackThreat {
            quality_gated: gated,
            time_to_impact_sec: tti,
            engaged,
        }
    }

    #[test]
    fn test_no_tracks_is_defcon5() {
        assert_eq!(assess_tracks(&[]), AlertLevel::Defcon5);
    }

    #[test]
    fn test_ungated_tracks_do_not_count() {
        // Tracks below the quality gate are invisible to the alert system
        let tracks = vec![threat(false, Some(10.0), false), threat(false, None, false)];
        assert_eq!(assess_tracks(&tracks), AlertLevel::Defcon5);
    }

    #[test]
    fn test_unconverged_track_is_defcon4() {
        // Quality-gated but no converged trajectory yet
        assert_eq!(
            assess_tracks(&[threat(true, None, false)]),
            AlertLevel::Defcon4
        );
    }

    #[test]
    fn test_inbound_track_is_defcon3() {
        // Converged trajectory, plenty of time left
        assert_eq!(
            assess_tracks(&[threat(true, Some(600.0), false)]),
            AlertLevel::Defcon3
        );
    }

    #[test]
    fn test_terminal_track_is_defcon2() {
        // Under the terminal threshold
        assert_eq!(
            assess_tracks(&[threat(true, Some(100.0), false)]),
            AlertLevel::Defcon2
        );
        // Engaged terminal tracks still raise to 2 (not yet killed)
        assert_eq!(
            assess_tracks(&[threat(true, Some(100.0), true)]),
            AlertLevel::Defcon2
        );
    }

    #[test]
    fn test_late_unengaged_is_defcon1() {
        // TTI < 45s, nothing on it: leaker
        assert_eq!(
            assess_tracks(&[threat(true, Some(30.0), false)]),
            AlertLevel::Defcon1
        );
    }

    #[test]
    fn test_late_engaged_is_not_defcon1() {
        // TTI < 45s but an interceptor is on it: still DEFCON 2
        assert_eq!(
            assess_tracks(&[threat(true, Some(30.0), true)]),
            AlertLevel::Defcon2
        );
    }

    #[test]
    fn test_severity_ordering() {
        assert!(AlertLevel::Defcon1 < AlertLevel::Defcon2);
        assert!(AlertLevel::Defcon2 < AlertLevel::Defcon3);
        assert!(AlertLevel::Defcon3 < AlertLevel::Defcon4);
        assert!(AlertLevel::Defcon4 < AlertLevel::Defcon5);
    }

    #[test]
    fn test_tracker_klaxon_only_on_escalation() {
        let mut tracker = AlertTracker::new();
        // 5 -> 4: escalation, sounds
        assert!(tracker.update(AlertLevel::Defcon4));
        // 4 -> 4: no change, silent
        assert!(!tracker.update(AlertLevel::Defcon4));
        // 4 -> 5: de-escalation, silent
        assert!(!tracker.update(AlertLevel::Defcon5));
        // 5 -> 2: escalation, but within cooldown of the first klaxon
        assert!(!tracker.update(AlertLevel::Defcon2));
    }

    #[test]
    fn test_tracker_reset_rearms_klaxon() {
        let mut tracker = AlertTracker::new();
        assert!(tracker.update(AlertLevel::Defcon4));
        tracker.reset();
        // After reset, a fresh escalation sounds again
        assert!(tracker.update(AlertLevel::Defcon3));
    }

    #[test]
    fn test_gates_match_fire_control_thresholds() {
        // The alert gate must match the AGENTS.md track establishment
        // thresholds: quality >= 0.4, staleness < 5s, measurements >= 3.
        // (Test the constants so a future edit to one side trips this test.)
        assert_eq!(MIN_TRACK_QUALITY, 0.4);
        assert_eq!(MAX_STALENESS_SEC, 5.0);
        assert_eq!(MIN_MEASUREMENTS, 3);
    }
}
