//! Pre-recorded sound effects for simulation events
//!
//! Sounds are loaded from `assets/sounds/` at startup, decoded fully into
//! memory (f32 sample vectors), and replayed one-shot through a rodio mixer
//! (concurrent sounds mix natively). Each `SoundId` maps to a file:
//!
//! | SoundId             | File                          |
//! |---------------------|-------------------------------|
//! | MissileLaunch       | assets/sounds/missile_launch.wav |
//! | InterceptorLaunch   | assets/sounds/interceptor_launch.wav |
//! | InterceptHit        | assets/sounds/intercept_hit.wav |
//! | InterceptMiss       | assets/sounds/intercept_miss.wav |
//! | SelfDestruct        | assets/sounds/self_destruct.wav |
//! | MissileImpact       | assets/sounds/missile_impact.wav |
//! | DecoyDeployed       | assets/sounds/decoy_deployed.wav |
//!
//! Supported formats: WAV and OGG/Vorbis (whatever rodio auto-detects).
//! Missing files are skipped with a warning — the app runs without them,
//! the same way it runs without a MapTiler key. To override the directory
//! (e.g. for packaged .app bundles), set `GTW_SOUNDS_DIR`.

use std::collections::HashMap;
use std::fs;
use std::io::Cursor;
use std::num::NonZero;
use std::path::{Path, PathBuf};
use std::time::Instant;

use crate::tracking::EventType;
use rodio::Source;

/// Directory containing sound files, relative to the crate root.
const DEFAULT_SOUNDS_DIR: &str = "assets/sounds";

/// Minimum wall-clock time between plays of the same sound, so a salvo of
/// simultaneous interceptors doesn't stack N identical sounds in one frame.
const COOLDOWN_SECS: f64 = 0.12;

/// Identifies a sound effect and the file it loads from.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum SoundId {
    MissileLaunch,
    InterceptorLaunch,
    InterceptHit,
    InterceptMiss,
    SelfDestruct,
    MissileImpact,
    DecoyDeployed,
    /// Klaxon on DEFCON escalation (sensor-derived threat warning)
    ThreatWarning,
}

impl SoundId {
    /// Base file name (without extension) for this sound.
    fn file_stem(self) -> &'static str {
        match self {
            SoundId::MissileLaunch => "missile_launch",
            SoundId::InterceptorLaunch => "interceptor_launch",
            SoundId::InterceptHit => "intercept_hit",
            SoundId::InterceptMiss => "intercept_miss",
            SoundId::SelfDestruct => "self_destruct",
            SoundId::MissileImpact => "missile_impact",
            SoundId::DecoyDeployed => "decoy_deployed",
            SoundId::ThreatWarning => "threat_warning",
        }
    }

    /// All sound ids (used by tests to verify every file exists).
    pub fn all() -> [SoundId; 8] {
        [
            SoundId::MissileLaunch,
            SoundId::InterceptorLaunch,
            SoundId::InterceptHit,
            SoundId::InterceptMiss,
            SoundId::SelfDestruct,
            SoundId::MissileImpact,
            SoundId::DecoyDeployed,
            SoundId::ThreatWarning,
        ]
    }
}

/// A decoded sound effect held fully in memory, ready to replay.
#[derive(Clone)]
struct SoundClip {
    samples: Vec<rodio::Sample>,
    channels: u16,
    sample_rate: u32,
}

impl SoundClip {
    /// Decode a WAV/OGG file into an in-memory clip.
    fn load(path: &Path) -> Result<Self, String> {
        let bytes = fs::read(path).map_err(|e| format!("{e}"))?;
        let decoder =
            rodio::Decoder::new(Cursor::new(bytes)).map_err(|e| format!("decode failed: {e}"))?;
        let channels = decoder.channels().get() as u16;
        let sample_rate = decoder.sample_rate().get();
        let samples: Vec<rodio::Sample> = decoder.collect();
        if samples.is_empty() {
            return Err("file contains no samples".to_string());
        }
        Ok(Self {
            samples,
            channels,
            sample_rate,
        })
    }

    fn as_source(&self) -> rodio::buffer::SamplesBuffer {
        rodio::buffer::SamplesBuffer::new(
            NonZero::new(self.channels).unwrap(),
            NonZero::new(self.sample_rate).unwrap(),
            self.samples.clone(),
        )
    }
}

/// One-shot sound effect player for pre-recorded clips.
///
/// Holds the rodio device sink (must stay alive for playback — dropping it
/// disposes the OS audio stream) and decoded clips.
/// Degrades gracefully to silence if no audio device is available or sound
/// files are missing.
pub struct AudioManager {
    /// None = no audio device (or init failed); all play calls become no-ops.
    sink: Option<rodio::MixerDeviceSink>,
    /// Decoded sound clips, loaded at startup.
    clips: HashMap<SoundId, SoundClip>,
    /// Wall-clock time of the last play per sound, for cooldown throttling.
    last_played: HashMap<SoundId, Instant>,
    /// Mute toggle (user-controlled from the UI).
    pub muted: bool,
}

impl Default for AudioManager {
    fn default() -> Self {
        Self::new()
    }
}

impl AudioManager {
    /// Create a new manager, opening the default audio output and loading all
    /// sound files. Missing files or a missing device degrade to silence.
    pub fn new() -> Self {
        let sink = match rodio::DeviceSinkBuilder::open_default_sink() {
            Ok(mut sink) => {
                sink.log_on_drop(false);
                Some(sink)
            }
            Err(e) => {
                eprintln!("Warning: audio unavailable, continuing without sound ({e})");
                None
            }
        };
        Self {
            sink,
            clips: load_all_clips(),
            last_played: HashMap::new(),
            muted: false,
        }
    }

    /// Reset per-sound cooldown timestamps (called on scenario load/restart so
    /// stale throttling doesn't suppress sounds in a fresh scenario).
    pub fn reset(&mut self) {
        self.last_played.clear();
    }

    /// Play the sound for a simulation event, if it maps to one.
    /// Applies mute and per-sound cooldown throttling.
    pub fn play_event(&mut self, event: &EventType) {
        let Some(sound) = sound_for_event(event) else {
            return;
        };
        self.play(sound);
    }

    /// Play a sound by id (no-op if muted, unavailable, or on cooldown).
    pub fn play(&mut self, sound: SoundId) {
        if self.muted {
            return;
        }
        let Some(sink) = &self.sink else {
            return;
        };
        let Some(clip) = self.clips.get(&sound) else {
            return;
        };
        let now = Instant::now();
        if let Some(last) = self.last_played.get(&sound) {
            if now.duration_since(*last).as_secs_f64() < COOLDOWN_SECS {
                return;
            }
        }
        self.last_played.insert(sound, now);
        sink.mixer().add(clip.as_source());
    }
}

/// Resolve the sounds directory, honoring the `GTW_SOUNDS_DIR` override.
fn sounds_dir() -> PathBuf {
    if let Ok(dir) = std::env::var("GTW_SOUNDS_DIR") {
        return PathBuf::from(dir);
    }
    PathBuf::from(DEFAULT_SOUNDS_DIR)
}

/// Find the sound file for a clip: tries `.wav` then `.ogg` then `.mp3`.
/// Returns the first existing match.
fn find_sound_file(dir: &Path, stem: &str) -> Option<PathBuf> {
    for ext in ["wav", "ogg", "mp3"] {
        let path = dir.join(format!("{stem}.{ext}"));
        if path.exists() {
            return Some(path);
        }
    }
    None
}

/// Load every known sound file, warning (once) about missing ones.
fn load_all_clips() -> HashMap<SoundId, SoundClip> {
    let dir = sounds_dir();
    let mut clips = HashMap::new();
    for sound in SoundId::all() {
        match find_sound_file(&dir, sound.file_stem()) {
            Some(path) => match SoundClip::load(&path) {
                Ok(clip) => {
                    clips.insert(sound, clip);
                }
                Err(e) => eprintln!(
                    "Warning: failed to load sound {} ({e}), it will be silent",
                    path.display()
                ),
            },
            None => eprintln!(
                "Warning: sound file missing for {sound:?} (looked in {}), it will be silent",
                dir.display()
            ),
        }
    }
    clips
}

/// Map a tracked simulation event to its sound.
fn sound_for_event(event: &EventType) -> Option<SoundId> {
    match event {
        EventType::MissileLaunch { .. } => Some(SoundId::MissileLaunch),
        EventType::InterceptorLaunch { .. } => Some(SoundId::InterceptorLaunch),
        EventType::InterceptHit { .. } => Some(SoundId::InterceptHit),
        EventType::InterceptMiss { .. } => Some(SoundId::InterceptMiss),
        EventType::InterceptorSelfDestruct { .. } => Some(SoundId::SelfDestruct),
        EventType::MissileImpact { .. } => Some(SoundId::MissileImpact),
        EventType::DecoyDeployed { .. } => Some(SoundId::DecoyDeployed),
        // DEFCON escalation klaxon
        EventType::ThreatDetected { .. } => Some(SoundId::ThreatWarning),
        // Guidance chatter and neutralization have no sound (out of scope).
        EventType::GuidanceUpdate { .. }
        | EventType::GuidanceBlocked { .. }
        | EventType::AllThreatsNeutralized => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_event_mapping() {
        // Events that map to sounds.
        assert_eq!(
            sound_for_event(&EventType::MissileLaunch { name: "x".into() }),
            Some(SoundId::MissileLaunch)
        );
        assert_eq!(
            sound_for_event(&EventType::InterceptorLaunch {
                defense_unit: "u".into(),
                target: "t".into()
            }),
            Some(SoundId::InterceptorLaunch)
        );
        assert_eq!(
            sound_for_event(&EventType::InterceptHit { target: "t".into() }),
            Some(SoundId::InterceptHit)
        );
        assert_eq!(
            sound_for_event(&EventType::InterceptMiss { target: "t".into() }),
            Some(SoundId::InterceptMiss)
        );
        assert_eq!(
            sound_for_event(&EventType::InterceptorSelfDestruct { target: "t".into() }),
            Some(SoundId::SelfDestruct)
        );
        assert_eq!(
            sound_for_event(&EventType::MissileImpact { name: "n".into() }),
            Some(SoundId::MissileImpact)
        );
        assert_eq!(
            sound_for_event(&EventType::DecoyDeployed {
                missile_name: "m".into(),
                decoys_active: 3
            }),
            Some(SoundId::DecoyDeployed)
        );
        // Events with no sound.
        assert_eq!(
            sound_for_event(&EventType::GuidanceUpdate {
                interceptor_type: "i".into(),
                target: "t".into(),
                correction_km: 1.0,
                update_count: 1
            }),
            None
        );
        assert_eq!(
            sound_for_event(&EventType::GuidanceBlocked {
                interceptor_type: "i".into(),
                target: "t".into(),
                reason: "r".into()
            }),
            None
        );
        // Threat-detected klaxon (DEFCON escalation)
        assert_eq!(
            sound_for_event(&EventType::ThreatDetected {
                threat_name: "t".into(),
                sensor_name: "s".into()
            }),
            Some(SoundId::ThreatWarning)
        );
        assert_eq!(sound_for_event(&EventType::AllThreatsNeutralized), None);
    }

    #[test]
    fn test_all_sound_files_exist() {
        // Every SoundId must have a file on disk so the sim isn't silently
        // missing combat audio. (The app tolerates missing files, but we
        // want the repo to ship all of them.)
        let dir = sounds_dir();
        for sound in SoundId::all() {
            let found = find_sound_file(&dir, sound.file_stem());
            assert!(
                found.is_some(),
                "missing sound file for {sound:?}: {}",
                dir.join(sound.file_stem()).display()
            );
        }
    }

    #[test]
    fn test_clips_decode() {
        // Every shipped sound file must decode to non-empty sample data.
        for sound in SoundId::all() {
            let dir = sounds_dir();
            let Some(path) = find_sound_file(&dir, sound.file_stem()) else {
                panic!("missing sound file for {sound:?}");
            };
            let clip = SoundClip::load(&path)
                .unwrap_or_else(|e| panic!("failed to decode {sound:?}: {e}"));
            assert!(!clip.samples.is_empty(), "{sound:?} decoded empty");
            assert!(clip.sample_rate > 0);
            assert!(clip.channels > 0);
            // Samples must be finite and in range.
            for (n, s) in clip.samples.iter().take(10_000).enumerate() {
                assert!(
                    s.is_finite() && (-1.0..=1.0).contains(s),
                    "{sound:?}: bad sample {s} at index {n}"
                );
            }
        }
    }
}
