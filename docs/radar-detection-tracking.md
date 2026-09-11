## Radar Detection and Tracking
- Radars all have at least three modes, search, track and fire control
- Radar detection follows inverse-square law (R⁴ for radar equation)
- RCS (Radar Cross Section) values in dBsm affect detection probability
- Atmospheric attenuation increases with range and decreases with altitude

### Radar bands ###
- All Radars operate in at least one of the following bands, L,S,C,X or Ku
- Some Radars can operate in more than one band simultaneously
- Each radar band has its strengths and weaknesses, range and resolution specifications

| Band | Frequency | Attenuation | Quality Multiplier | Best For |
|------|-----------|-------------|-------------------|----------|
| **L-band** | 1-2 GHz | Very Low (0.005 dB/km) | 0.85× | Long-range early warning |
| **S-band** | 2-4 GHz | Low (0.010 dB/km) | 0.92× | Balanced search & track |
| **C-band** | 4-8 GHz | Moderate (0.015 dB/km) | 0.96× | Medium-range tracking |
| **X-band** | 8-12 GHz | High (0.020 dB/km) | 1.00× | Fire control, discrimination |
| **Ku-band** | 12-18 GHz | Very High (0.030 dB/km) | 1.05× | Ultra-high resolution |

### Fire Control Modes ###
- Search mode: a radar will detect targets contained in its full altitude and azimuth search range and every target will be updated at a low frequency.
- Track mode: a radar will narrow its radar focus to update a target's position with a higher frequency in order to establish the target's vector and create a full trajectory profile for an interception calculation
- Fire Control mode: a radar will lock onto and follow a target in order to send mid-course updates to an interceptor to increase the chance of a successful merge (interception)
- A radar will have specs on how many targets it can track in all three modes
- A radar's default mode is Search mode.
- A radar will automatically switch to track mode for every detected target in order to establish a track.
- A radar will automatically switch to fire control mode for a target when an interceptor launched at it.

## Tracking ##
- Always use realistic algorithms to establish a target track, such as Extended Kalman Filter
- Always calculate a confidence in the computed target's trajectory
- Always refine a computed target's trajectory with additional radar information over time
- Always compute an interception point on the computed target's trajectory that is within the platform's interceptor's performance envelop

## Measurement Error Model (Phase 4) ##

Radar measurements carry realistic error, injected at measurement creation
(`create_radar_measurement`):

- **Calibration bias** — deterministic per-sensor offsets from the sensor
  TOML's `[tracking]` section: `azimuth_bias_deg`, `elevation_bias_deg`,
  `range_bias_km`. Phased arrays are boresighted tight (0.04-0.16 deg in
  the shipped configs, Skolnik radar-calibration practice); the mechanical
  default is looser (0.35 deg).
- **Stochastic noise** — zero-mean Gaussian per measurement (Box-Muller),
  scaled by detection quality: ~0.01-0.1 km range, ~0.05-0.5 deg angle.
  `noise_multiplier` (default 1.0) scales it for degraded sensors.

Design decisions:
- The **EKF measurement and the Detection position share one bearing
  realization** — the raw-measurement path and the converted-position path
  (linear KF, track initialization) see the same perturbed azimuth.
- The Detection's `range_km` keeps its mode-dependent semantics (ground vs
  slant); bias+noise are applied to both quantities independently.
- `Detection.altitude_km` deliberately remains the true altitude: the
  converged-trajectory altitude fit is calibrated against the true
  parabolic profile. Deriving altitude from noisy elevation would inject a
  flat-earth systematic into the trajectory estimator. (Track *quality*
  still reflects measurement noise through the innovation machinery.)
- The EKF's R-matrix is built from the same noise_std, so filter covariance
  now reflects actual measurement scatter; the innovation-monitoring
  machinery (consistency factors, track health) penalizes biased sensors
  in fusion weights.

Determinism: all detection stochasticity (measurement noise, detection
rolls, false alarms) flows through a `DetectionSystem`-owned `StdRng`
seeded from entropy in production; tests call `engine.detection.seed_rng(n)`
for reproducible scenarios.
