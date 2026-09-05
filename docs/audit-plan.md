# Codebase Audit: Domain Documentation vs Implementation

## Audit Date: 2026-04-11

This document captures the audit of the simulation codebase against the domain requirements documented in:
- `docs/radar-detection-tracking.md`
- `docs/platform-intercept-geometry.md`
- `docs/physics.md`

---

## What's Implemented Well ✅

| Area | Implementation Quality | Key Files |
|------|----------------------|-----------|
| **Radar Modes** (Search/Track/Fire Control) | Full - with mode transitions and time budgeting | `detection.rs:1781-1951` |
| **Radar Equation** (R⁴ law, RCS, attenuation) | Full - all band-specific values per docs | `detection.rs:2049-2095` |
| **Radar Bands** (L/S/C/X/Ku) | Full - correct attenuation coefficients | `config.rs:262-299` |
| **Extended Kalman Filter** | Full - geodetic state, nonlinear dynamics | `ekf.rs` (609 lines) |
| **Track Fusion** | Full - multi-sensor weighted averaging | `detection.rs:1536-1657` |
| **F2T2EA Kill Chain** | Mostly complete - Find through Assess | `engine.rs` |
| **Envelope Validation** | Full - altitude, range, time validation | `engine.rs:2027-2030` |
| **Mid-Course Guidance** | Full - EKF/KF prediction updates | `engine.rs:722-950` |
| **Standard Gravity** | Correct (9.80665 m/s² → 0.00981 km/s²) | `ekf.rs:18`, `kalman.rs:62` |
| **Earth Curvature** | Good - spherical model with geodetic handling | `physics.rs`, `ekf.rs` |
| **Shoot-Look-Shoot / Salvo** | Basic - time-based heuristic | `engine.rs:1796-1797` |

---

## Critical Gaps ❌

### 1. ~~No Proportional Navigation Guidance~~ ✅ FIXED
- **Requirement**: `docs/platform-intercept-geometry.md` specifies PN, Predictive Guidance, or Lambert
- **Status**: IMPLEMENTED (2026-04-11)
- **Solution**: Added PN to terminal guidance with configurable navigation constant (N=4 default)
- **Location**: `config.rs:80-100`, `engine.rs:1329-1355`

### 2. ~~No Lambert Guidance for Exo-Atmospheric~~ ✅ FIXED
- **Requirement**: "Prefer Lambert Guidance model for exo-atmospheric interceptions"
- **Status**: IMPLEMENTED (2026-04-11)
- **Solution**: Added full Lambert solver using universal variable method, integrated for GBI/Aegis/Arrow3 > 100km
- **Location**: `physics.rs:468-670`, `engine.rs:1299-1323`

### 3. ~~No Atmospheric Drag Model~~ ✅ FIXED
- **Requirement**: `docs/physics.md` implies realistic physics
- **Status**: IMPLEMENTED (2026-04-11)
- **Solution**: Added 1976 US Standard Atmosphere density model with physics-based drag calculations
- **Location**: `physics.rs:718-890` (atmosphere model), `entities.rs:229-275` (missile drag), `entities.rs:838-880` (interceptor drag)

### 4. ~~Trajectory Confidence Underutilized~~ ✅ FIXED
- **Requirement**: "Always calculate a confidence in the computed target's trajectory"
- **Status**: IMPLEMENTED (2026-04-12)
- **Solution**: Confidence-driven engagement with quality gates and doctrine selection
- **Location**: `engine.rs:2209-2240` (quality gates), `engine.rs:1847-1905` (doctrine selection)

### 5. ~~Empirical Apogee Calculations~~ ✅ FIXED
- **Requirement**: "Ballistic trajectories use realistic apogee calculations based on range"
- **Status**: IMPLEMENTED (2026-04-11)
- **Solution**: Physics-derived apogee using orbital mechanics (energy conservation, semi-major axis, eccentricity)
- **Location**: `physics.rs:280-370`

### 6. ~~Linear Kalman Uses Constant Gravity~~ ✅ FIXED
- **Requirement**: Realistic physics
- **Status**: IMPLEMENTED (2026-04-12)
- **Solution**: Added altitude-dependent gravity: g(h) = g₀ × (R_earth / (R_earth + h))²
- **Location**: `kalman.rs:55-75`

---

## Minor Gaps (Lower Priority)

| Gap | Status | Notes |
|-----|--------|-------|
| ~~RCS aspect-angle variation~~ | ✅ FIXED | Now varies ±1.5dB based on nose/broadside/tail aspect |
| Radar clutter modeling | Pending | False alarms exist but not from clutter physics |
| Coriolis force | Pending | Not modeled (minor effect for these ranges) |
| WGS84 ellipsoid | Pending | Uses simplified spherical Earth (6371 km) |
| Polarization effects | Pending | Uniform countermeasure attenuation |

---

## Implementation Plan

### Phase 1: Guidance Algorithm Improvements (High Impact) ✅ COMPLETED

#### 1.1 Implement Proportional Navigation (PN) ✅
- **Status**: COMPLETED (2026-04-11)
- **Location**: `engine.rs:1293-1355` (terminal guidance section)
- **Implementation**:
  - Added `navigation_constant` to `MidcourseGuidanceConfig` (default N=4)
  - Uses existing LOS rate calculation
  - Applies PN law: `heading_rate = N × LOS_rate`
  - Limited by interceptor maneuverability (terminal_maneuver_g)
  - Reference: Zarchan, "Tactical and Strategic Missile Guidance"
- **Affects**: All interceptor systems in terminal phase with seeker lock

#### 1.2 Implement Lambert Guidance for Exo-Atmospheric ✅
- **Status**: COMPLETED (2026-04-11)
- **Location**: `physics.rs:468-670` (Lambert solver), `engine.rs:1299-1323` (integration)
- **Implementation**:
  - Full Lambert solver using universal variable method
  - Stumpff functions for elliptical/hyperbolic orbits
  - Geodetic ↔ ECEF coordinate conversions
  - Activates for GBI/Aegis/Arrow3 when altitude > 100 km
  - Computes optimal transfer trajectory heading
  - Reference: Vallado "Fundamentals of Astrodynamics and Applications"
- **Affects**: GBI, Arrow 3, SM-3 Block IIA in exo-atmospheric midcourse

### Phase 2: Physics Fidelity ✅ COMPLETED

#### 2.1 Add Atmospheric Drag Model ✅
- **Status**: COMPLETED (2026-04-11)
- **Location**: `physics.rs:718-890`, `entities.rs:229-275`, `entities.rs:838-880`
- **Implementation**:
  - Full 1976 US Standard Atmosphere with piecewise temperature/pressure model
  - Density varies from 1.225 kg/m³ at sea level to 0 above 100km (Kármán line)
  - Troposphere, stratosphere, mesosphere layers with correct lapse rates
  - `BallisticCoefficient` struct for missiles/interceptors (ICBM β≈20,000 kg/m², interceptor β≈4,000 kg/m²)
  - Drag applied to missiles in midcourse/terminal using cumulative descent integration
  - Drag applied to endo-atmospheric interceptors (Patriot, THAAD, Iron Dome, David's Sling)
  - Reference: NASA-TM-X-74335
- **Affects**: Realistic velocity profiles, especially for terminal phase engagements

#### 2.2 Physics-Derived Apogee ✅
- **Status**: COMPLETED (2026-04-11)
- **Location**: `physics.rs:280-370`
- **Implementation**:
  - `burnout_conditions()`: Estimates altitude, velocity, flight path angle based on range class
  - `apogee_from_burnout()`: Uses orbital mechanics (specific energy, angular momentum, eccentricity)
  - Formula: E = v²/2 - μ/r, a = -μ/(2E), p = h²/μ, e = √(1 - p/a), r_apogee = a(1+e)
  - `estimate_range_from_apogee()`: Bisection search for inverse
  - Reference: Bate, Mueller, White "Fundamentals of Astrodynamics"
- **Affects**: More accurate trajectory prediction for all missile classes

### Phase 3: Confidence-Driven Engagement ✅ COMPLETED

#### 3.1 Trajectory Confidence Quality Gates ✅
- **Status**: COMPLETED (2026-04-12)
- **Location**: `engine.rs:2209-2240`, `engine.rs:1847-1905`
- **Implementation**:
  - **Quality threshold**: Increased from 40% to 60% (`MIN_ENGAGEMENT_QUALITY`)
  - **Velocity confidence gate**: Minimum 50% confidence required (`MIN_VELOCITY_CONFIDENCE`)
  - **Combined confidence**: Track quality + velocity confidence averaged for doctrine decisions
  - **Confidence-based salvo sizing**:
    - High confidence (>80%): Single precision shot
    - Medium confidence (60-80%): Standard 2-shot salvo
    - Lower confidence (50-60%): Increased 3-shot salvo
  - **SLS confidence gate**: Shoot-Look-Shoot only when confidence ≥75% (`SLS_CONFIDENCE_THRESHOLD`)
  - Low confidence tracks trigger immediate salvo fire (SSL) regardless of time available
- **Affects**: More realistic ammunition expenditure, confidence-aware doctrine selection

### Phase 4: Refinements ✅ COMPLETED

#### 4.1 Altitude-Dependent Gravity in Linear KF ✅
- **Status**: COMPLETED (2026-04-12)
- **Location**: `kalman.rs:55-75`
- **Implementation**:
  - Added `g_local = G0 × (R_earth / (R_earth + altitude))²`
  - Consistent with EKF implementation
  - Uses `position[2]` (up component) as altitude
- **Affects**: More accurate tracking at high altitudes (ICBM apogee)

#### 4.2 RCS Aspect-Angle Variation ✅
- **Status**: COMPLETED (2026-04-12)
- **Location**: `entities.rs:175-225`, `detection.rs:1027-1036`, `detection.rs:1129-1138`, `detection.rs:1205-1209`
- **Implementation**:
  - Added `rcs_with_aspect(radar_position)` method to Missile
  - Calculates aspect angle from radar LOS vs missile heading
  - Aspect-dependent RCS modifiers (dB):
    - Nose-on (0-30°): -1.5 dB
    - Forward quarter (30-60°): -0.7 dB
    - Broadside (60-120°): +1.2 dB
    - Rear quarter (120-150°): -0.5 dB
    - Tail-on (150-180°): -1.0 dB
  - Applied to defense unit, radar station, and satellite radar detection
  - Reference: Skolnik, "Introduction to Radar Systems"
- **Affects**: More realistic detection probability based on viewing geometry

---

## Validation Checklist

After implementing changes:
- [x] Run `cargo test` - all tests pass ✅
- [x] Run `cargo clippy` - no new errors ✅
- [x] Test each scenario in `/scenarios/` directory ✅ (scenario audit: all 24 scenarios rewired with explicit `sensor_config`, config-backed missile profiles, and `facing_deg` for narrow-azimuth sensors; `tests/scenario_wiring_test.rs` now guards this permanently)
- [ ] Verify Patriot engages at correct ranges
- [ ] Verify GBI/Arrow 3 use Lambert guidance above 100 km
- [ ] Verify trajectory predictions match expected physics
- [ ] Check Pk values remain in realistic ranges
