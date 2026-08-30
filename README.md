# Global Thermonuclear War

A ballistic missile defense simulation built with Rust and egui. Visualize missile launches, tracking, and intercept scenarios with realistic flight physics and defense system modeling.

## Features

- **Realistic Missile Simulation**: ICBM, IRBM, MRBM, SLBM, and SRBM with configurable trajectories, flight times, and countermeasures
- **Multi-layered Defense Systems**:
  - US: GBI, AEGIS SM-3, THAAD, Patriot PAC-3
  - Israeli: Arrow 3, David's Sling, Iron Dome
  - Russian: S-400
- **Sensor-Based Fire Control**: Realistic two-stage engagement (search/track radars → fire control lock)
- **Track Establishment Requirements**: 3+ measurements, quality thresholds, freshness limits
- **Converged Trajectory Estimation**: Radar measurements are fitted to the parabolic altitude profile ballistic missiles actually fly, yielding stable launch-point/impact-point/apogee estimates that converge as tracking matures
- **Automatic Salvo Doctrine**: Intelligent Shoot-Look-Shoot vs Shoot-Shoot-Look selection
- **Separate Sensor and Interceptor Modeling**: Radars and interceptors configured independently for realistic system composition
- **Early Warning Satellites**: SBIRS, DSP, Tundra, and other space-based detection systems
- **TOML-based Configuration**: All system parameters externalized for easy modification
- **Scenario System**: Pre-built scenarios for different regions and threat environments
- **In-App Scenario Builder**: Create, edit, test, and save scenarios interactively (see below)

### Visualization Features

- **Multiple View Modes**:
  - **2D Map (Mercator)**: Traditional flat map with world wrapping
  - **3D Globe**: Orthographic projection with rotation and zoom
  - **Isometric 3D Intercept View**: Platform-centric view showing engagement envelopes and tracked missiles
- **Track Visualization Modes**:
  - **True Track**: Shows actual missile positions (omniscient view)
  - **Detected Track**: Shows sensor-perceived positions with uncertainty visualization
- **Visual Effects**: Animated impact explosions, intercept effects, and debris clouds
- **Real-time Event Log**: Tracks missile launches, detections, intercepts, and impacts
- **Radar Coverage Visualization**: Shows search, track, and fire control radar ranges by mode

## Realism Features

The simulation models real-world missile defense physics and sensor limitations for accurate scenario analysis.

### Key Realism Highlights

This simulation implements **sensor-based intercepts** instead of "perfect information" intercepts:

✅ **No Omniscient Defense Systems**: Defense units cannot see missiles directly - they only see what sensors detect
✅ **Track Establishment Required**: Must have 3+ sensor measurements before fire control lock authorized
✅ **Fire Control Radar Architecture**: Realistic two-stage process (search/track → fire control lock)
✅ **Undetected = Unengaged**: Missiles outside sensor coverage cannot be intercepted
✅ **No Ground-Truth Fallback**: Launch solutions, mid-course guidance, doctrine timing, Pk factors, and terminal lead ALL project the target through the sensor-derived converged trajectory. If the sensor chain can't produce a solution, the launch is refused — by design
✅ **Engagement Envelope Enforcement**: THAAD cannot engage ICBMs at 400km altitude (max 150km)
✅ **Closure Feasibility Checks**: The launch is refused if the interceptor physically cannot reach the intercept point in time (e.g., a short-range interceptor against a fast-crossing MRBM reentry vehicle)
✅ **Automatic Salvo Doctrine**: System selects Shoot-Look-Shoot vs Shoot-Shoot-Look based on time available
✅ **Realistic Misses**: Crossing-geometry targets legitimately miss due to gimbal-limited terminal homing — low-Pk shots fail honestly instead of being silently corrected

### Sensor & Detection

- **Sensor-Based Intercept Solutions**: Defense systems use only sensor-detected positions, not ground truth
- **Track Quality Thresholds**: Minimum track quality (0.4) required for track establishment; fire-control engagement requires fused quality ≥0.6
- **Track Establishment**: Requires 3+ measurements before a track is trusted
- **Fire Control Quality Gates**: Engagement requires 10+ measurements, velocity confidence ≥0.55, and track staleness <5 seconds
- **Position History Tracking**: Maintains 100 most recent position measurements per sensor
- **Velocity Estimation**: Extended Kalman Filter (preferred) with weighted least-squares fallback
- **Track Staleness**: Tracks degrade without updates; engagements rejected if stale >5 seconds
- **False Alarm Filtering**: Radar clutter and false alarms do not trigger engagements
- **Undetected Missiles**: Cannot engage targets outside sensor coverage
- **Radar Cross Section (RCS)**: Detection probability scales with target RCS signature
- **Radar Equation Physics**: Detection range follows inverse-square law (R⁴ factor)
- **Atmospheric Attenuation**: Signal loss increases with range and decreases with altitude
- **Horizon Angle Limits**: Ground radars cannot detect targets below geometric horizon
- **Scan Rate Limiting**: Sensors update at configured refresh rates (e.g., 1Hz, 20Hz)
- **Track Capacity Limits**: Sensors saturate at max simultaneous track count

### Track Fusion & Uncertainty

- **Multi-Sensor Fusion**: Combines tracks from multiple sensors with quality-weighted averaging
- **Velocity Fusion**: Merges velocity estimates from all tracking sensors by confidence
- **Uncertainty Quantification**: Position error radius 5-50km based on track quality
- **Staleness Growth**: Uncertainty grows ~2km/s for low-quality tracks without updates
- **Multi-Sensor Bonus**: 2+ sensors reduce uncertainty by 10-20%
- **Network-Aided Tracking**: Quality boost when multiple sensors track same target
- **Track Handoff**: Seamless transfer between sensors as missile moves through coverage
- **Confidence Weighting**: Recent, high-quality measurements weighted more heavily

### Converged Trajectory Estimation

From radar measurements alone, the simulation reconstructs the missile's full trajectory — launch point, impact point, apogee, and total flight time — and uses it for the impact-prediction marker, interceptor launch solutions, and mid-course guidance:

- **Quadratic Altitude Fit**: Missiles fly a parabolic altitude profile (h(τ) = 4·A·τ·(1−τ)), which is exactly a quadratic in time. A least-squares fit through the measured altitude history recovers launch time, impact time, and apogee analytically — the parabola's roots and vertex
- **Measurement-Gated Refinement**: Estimates update only when genuinely new radar measurements arrive — no frame-rate jitter
- **Running Weighted Mean**: Each new estimate blends into the accumulated one with weights that grow with fit quality and time span, so the predicted impact point converges instead of wandering
- **Outlier Dampening**: A single wild radar glitch cannot yank the estimate (10× weight reduction for >400 km jumps)
- **Model-Based Fallback**: Before enough history exists for the fit, a consistent (apogee, progress) solver provides a low-weight estimate
- **Stability Result**: The predicted impact point typically settles to within a few km of truth within seconds of track establishment and then freezes

### Radar Frequency Bands & Detection Quality

The simulation models realistic radar frequency band characteristics that affect detection range, atmospheric attenuation, and tracking quality. Different radar bands have fundamental physics trade-offs:

**Radar Band Characteristics:**

| Band | Frequency | Attenuation | Quality Multiplier | Best For |
|------|-----------|-------------|-------------------|----------|
| **L-band** | 1-2 GHz | Very Low (0.005 dB/km) | 0.85× | Long-range early warning |
| **S-band** | 2-4 GHz | Low (0.010 dB/km) | 0.92× | Balanced search & track |
| **C-band** | 4-8 GHz | Moderate (0.015 dB/km) | 0.96× | Medium-range tracking |
| **X-band** | 8-12 GHz | High (0.020 dB/km) | 1.00× | Fire control, discrimination |
| **Ku-band** | 12-18 GHz | Very High (0.030 dB/km) | 1.05× | Ultra-high resolution |

**Physical Trade-offs:**

**Higher Frequency (X-band, Ku-band):**
- ✅ Better angular resolution (sharper beam)
- ✅ Superior track quality (+0-5% bonus)
- ✅ Better discrimination between targets and decoys
- ❌ Higher atmospheric attenuation (shorter effective range)
- ❌ More affected by weather (rain, clouds)

**Lower Frequency (L-band, S-band):**
- ✅ Lower atmospheric attenuation (longer range)
- ✅ Better penetration through weather
- ✅ Less affected by clutter
- ❌ Wider beamwidth (lower angular resolution)
- ❌ Lower track quality (-8-15% penalty)
- ❌ Harder to discriminate decoys

**Real-World Radar Band Usage:**

**X-band Radars (High Precision):**
- **SBX-1** (Sea-Based X-band): 9-10 GHz - Midcourse discrimination, excellent resolution for decoy discrimination at long range despite higher attenuation
- **AN/TPY-2** (THAAD): X-band - Terminal phase tracking with 1-3km accuracy
- **92N6E Grave Stone** (S-400): X-band - Fire control with high precision

**S-band Radars (Balanced Performance):**
- **AN/SPY-1D** (AEGIS): 3 GHz - Volume search with 400km range, good balance of range and resolution
- **91N6E Big Bird** (S-400): S-band - Acquisition and battle management, 600km range
- **EL/M-2084** (Iron Dome): S-band - Multi-mission with high track capacity

**C-band Radars (Medium Range):**
- **AN/MPQ-65** (Patriot): 5.6 GHz - Balanced range and resolution for terminal defense

**L-band Radars (Maximum Range):**
- **Green Pine** (Arrow): 1.2-1.4 GHz - Long-range early warning with minimal atmospheric loss, 800km+ detection

**Detection Quality Degradation:**

Track quality now varies realistically with range and radar band:

**Close Range (0-30% of max range):**
- All bands: 85-100% quality
- Minimal atmospheric loss
- Example: SBX detecting at 1000km → 90-95% quality

**Medium Range (30-70% of max range):**
- X-band: 60-80% quality (higher attenuation)
- S-band: 70-85% quality (balanced)
- L-band: 75-90% quality (low attenuation)
- Example: SBX detecting at 2500km → 65-75% quality

**Maximum Range (70-100% of max range):**
- X-band: 40-60% quality (significant attenuation)
- S-band: 50-70% quality (moderate attenuation)
- L-band: 60-80% quality (minimal attenuation)
- Example: SBX detecting at 4000km → 45-55% quality

**Detection Probability Formula:**

Track quality is calculated from:
1. **Range Factor**: Quality = 1 / (1 + range_ratio²)
   - 100% at close range
   - 50% at nominal max range
2. **RCS Factor**: Target radar cross-section vs sensor minimum
3. **Atmospheric Attenuation**: Frequency-dependent signal loss through atmosphere
4. **Band Quality Multiplier**: Higher frequency = better resolution

**Practical Implications:**

- **X-band radars** provide excellent discrimination at long range but quality degrades faster than S-band
- **S-band radars** maintain good quality across their entire range envelope
- **L-band radars** can detect at extreme ranges with minimal quality loss but lower baseline resolution
- **Multi-sensor fusion** combines X-band precision with S-band/L-band range for optimal performance

### Ballistic Physics

- **Parabolic Trajectories**: Altitude follows realistic h(t) = 4·h_max·t·(1-t) profile
- **Range-Based Apogee**: Short-range (~150km), medium (~600km), ICBM (~1200km+)
- **Flight Time Estimation**: Based on range (SRBM ~5min, ICBM ~30min)
- **Trajectory Reconstruction**: Estimates origin/target from sensor-observed position, velocity, altitude
- **Flight Phase Detection**: Boost (0-10%), Midcourse (10-85%), Terminal (85-100%)
- **Great Circle Routing**: Ground track follows shortest path on Earth's surface
- **Gravity-Based Descent**: Terminal phase acceleration under gravity
- **Atmospheric Drag Model**: Altitude-dependent velocity reduction
  - Exoatmospheric (>100km): No drag
  - High altitude (80-100km): Minimal drag (~2% velocity loss)
  - Terminal phase (<80km): Significant drag (up to 62% velocity loss at sea level)
  - Drag affects intercept timing calculations

### Intercept Calculations

- **Hybrid Fire Control Radar System**: Realistic two-stage engagement process
  - **Search/Track Radars**: Establish initial track (3+ measurements, quality >0.4, freshness <5s)
  - **Fire Control Radars**: High-precision lock (TPY-2, SPY-1, MPQ-53/65) provides 1-3km accuracy
  - **Sensor-Based Gating**: Cannot engage undetected missiles (no track = no intercept)
- **Track Establishment Requirements**:
  - Minimum 3 sensor measurements to establish track
  - Track quality ≥0.4 (0.0-1.0 scale)
  - Track freshness <5 seconds (no stale data)
- **Fire Control Engagement Gates**: 10+ measurements, fused quality ≥0.6, velocity confidence ≥0.55
- **Rendezvous Solutions**: The intercept point is a time-and-place meeting — the solver scans the sensor-derived trajectory for the earliest point where the interceptor arrives before the missile, inside the engagement envelope
- **Unified Arrival-Time Math**: One boost-aware, drag-aware function (`InterceptorKinematics::time_to_cover_distance`) serves launch planning, mid-course guidance, and intercept-time recalculations — three previously-disagreeing formulas caused systematic 1-81s timing errors
- **No-Late-Arrival Constraint**: Solutions are rejected unless the interceptor reaches the aim point no later than 0.5s after the missile
- **Cumulative Drag Integration**: Endo-atmospheric interceptors (Patriot, THAAD, Iron Dome, David's Sling) lose speed continuously below 100km (a = ρv²/(2β), 1976 US Standard Atmosphere) — and the arrival-time math accounts for it
- **Engagement Envelope Validation**: Altitude and range constraints strictly enforced
- **Terminal Closure Feasibility**: Launch refused if required average speed exceeds interceptor capability by >20%
- **Two-Phase Flight Model**: Boost acceleration + coast at max velocity
- **Physics Sub-Stepping**: Up to 100 sub-steps per frame near intercept (~0.7m resolution at 4 km/s closure) — required when kill radii are 30-150m

### Terminal Guidance

- **Proportional Navigation (PN) Guidance**: True PN law implementation for terminal homing
  - Calculates Line-of-Sight (LOS) angle and rate between interceptor and target
  - Commands lateral acceleration proportional to LOS rate (a = N × Vc × dλ/dt)
  - Navigation constant defaults to N=4 (configurable per system)
  - Only activates when seeker has acquired target
- **Seeker Gimbal Limits**: Realistic field-of-view constraints
  - Off-boresight angle tracking (angle between flight direction and target)
  - Gimbal limits: 20-45° depending on interceptor type
  - Seeker can lose lock if target exits gimbal envelope
- **Seeker Acquisition**: Time-based acquisition modeling
  - 0.5-second acquisition delay once target enters seeker FOV
  - Range-gated acquisition: seeker locks only within 1.5× its published acquisition range (e.g., SM-3: 80km seeker → 120km limit)
  - Pk severely reduced (5-30%) without seeker lock
  - Lost lock requires re-acquisition
- **CPA Resolution with Hysteresis**: Hit-to-kill interceptors can't turn around; a confirmed closest-point-of-approach (10 consecutive increasing-distance ticks) that exceeds the kill radius resolves as a miss immediately

### Divert & Energy Management

- **Divert Budget Tracking**: Finite fuel for terminal maneuvers
  - Delta-V budgets vary by system (e.g., GBI EKV: 0.8 km/s, PAC-3: 0.6 km/s, SM-3: 0.4 km/s)
  - PN guidance consumes divert fuel proportionally
  - Interceptor goes ballistic when fuel exhausted
- **Energy State Management**: Dynamic maneuverability tracking
  - Exoatmospheric systems: purely fuel-based (no recovery)
  - Endoatmospheric systems: partial energy recovery from aerodynamic lift
  - Low energy state reduces Pk by up to 30%
- **Early-Arrival Correction**: Instead of artificial speed-bleeding maneuvers, early arrivals are corrected by mid-course guidance re-solving the intercept point from the latest sensor track

### Debris & Fratricide

- **Debris Cloud Modeling**: Realistic post-intercept hazards
  - Successful intercepts create expanding debris clouds
  - Initial radius: 0.5 km, expansion rate: 0.2 km/s
  - Debris remains hazardous for ~10 seconds
- **Debris Cloud Interference**: Fratricide risk for follow-on shots
  - Interceptors passing through debris have probabilistic damage (up to 30%)
  - Deeper penetration = higher damage probability
  - Affected interceptors marked as failed
- **Kill Assessment Delay**: Realistic battle damage assessment
  - 3-second delay before intercept result is known
  - Shoot-Look-Shoot doctrine waits for confirmed miss before follow-up
  - Prevents wasting interceptors on already-destroyed targets

### Mid-Course Guidance

During flight, interceptors receive updated guidance commands from the launching platform's fire control system. This models real-world systems like GBI, SM-3, and THAAD that rely on continuous sensor-fused track updates to refine their intercept solutions.

**How Mid-Course Guidance Works:**

1. **Sensor-Fused Tracking**: The fire control radar maintains a track on the incoming threat, fusing data from multiple sensors (early warning radars, search radars, fire control radars)

2. **Trajectory Prediction**: The battle management system uses the fused track's estimated position and velocity to predict where the target will be at the projected intercept time

3. **Uplink Commands**: The updated predicted intercept point (PIP) is transmitted to the interceptor via datalink

4. **Course Corrections**: The interceptor adjusts its flight path toward the new PIP

**Key Requirement - Fire Control Lock:**

Mid-course guidance requires the launching platform to have an active **fire control lock** on the target. This means:
- A fire control radar (AN/TPY-2, AN/SPY-1, AN/MPQ-65, etc.) must be tracking the target
- The track must be of sufficient quality to compute trajectory predictions
- If fire control lock is lost, mid-course guidance updates stop

**Sensor-Derived Trajectory Projection:**

The guidance system projects the target through the **converged trajectory estimate** (the quadratic-fit trajectory reconstructed from radar measurements — see "Converged Trajectory Estimation" above):
- Projects the fused track position along the estimated origin→target path at the estimated flight profile
- Guidance updates are suppressed until a converged estimate exists — no sensor data, no correction
- The projection reproduces the same parabolic-altitude, constant-ground-speed kinematics threats actually fly, so corrections track the real trajectory closely

**Guidance Update Conditions:**

- Interceptor must be **in flight** (not pending or terminated)
- Interceptor must be past early boost phase (>15% flight progress)
- Interceptor must not be in **Terminal phase** (seeker has taken over)
- **Fire control lock required** on the target
- At least 2 seconds remaining before predicted intercept
- Correction must exceed 0.5 km threshold

**Practical Effects:**

- **No track = No guidance**: If sensors lose the target, mid-course updates stop and the interceptor flies its original solution
- **Poor track = Poor guidance**: Low-quality tracks give inaccurate trajectory predictions, leading to suboptimal corrections
- **Better sensors = Better intercepts**: High-quality fire control tracking with good velocity estimates directly improves intercept success
- **Pk recalculation**: After each guidance update, the interceptor's Pk is recalculated based on the new intercept geometry

### Continuous Probability of Kill (Pk) Evaluation

During flight, each interceptor continuously evaluates its probability of successfully destroying the target. This Pk assessment uses multiple factors that reflect real-world engagement physics:

**Pk Calculation Factors:**

1. **Aspect Angle Factor (3D Geometry)**: Considers both horizontal and vertical crossing angles
   - Horizontal: Head-on (180°) optimal, tail-chase (0°) worst
   - Vertical: Steep crossing angles (60°+) significantly harder
   - Combined factor: 70% horizontal weight, 30% vertical weight

2. **Track Quality Factor**: Higher-quality sensor tracks (more measurements, recent updates, multi-sensor fusion) yield better intercept solutions. Degraded tracks reduce Pk proportionally.

3. **Closure Speed Factor**: Very high closure speeds reduce seeker acquisition time and maneuver capability. Optimal closure speeds (~4 km/s) maximize Pk; >12 km/s significantly degrades it.

4. **Energy State Factor**: Interceptor's remaining maneuver capability
   - Full energy (>50%): No penalty
   - Low energy (20-50%): 10% penalty
   - Depleted (<20%): Up to 30% penalty

5. **Countermeasures Factor**: Each decoy deployed by the target reduces Pk by degrading the interceptor's ability to discriminate the real warhead.

6. **Prediction Error Factor**: Compares the planned intercept point against the target's predicted future trajectory. Large deviations between where the interceptor is heading and where the missile will actually be reduce Pk significantly.

7. **Timing Synchronization Factor**: The interceptor and target must arrive at the intercept point within a small time window. If timing diverges beyond a margin (based on seeker range and closure speed), Pk drops to zero.

8. **Seeker Acquisition Factor**: Whether the seeker has locked onto target
   - Acquired: No penalty
   - In FOV but not acquired: 70% penalty
   - Outside gimbal limits: 95% penalty

**Pk Formula:**
```
Pk = base_pk × aspect_factor × track_quality_factor × closure_factor
     × energy_factor × countermeasures_factor × prediction_error_factor
     × timing_factor × seeker_factor
```

**Practical Implications:**
- Interceptors with degraded Pk may still attempt engagement but are less likely to succeed
- Early detection and high-quality tracks are critical for successful intercepts
- Countermeasures (decoys) significantly reduce intercept probability
- Timing mismatches from stale tracks or maneuvering targets can cause complete misses
- Seeker acquisition is critical—blind shots rarely succeed
- Energy-depleted interceptors cannot make final corrections
- The simulation logs Pk at intercept for post-engagement analysis

### Defense System Capabilities

**Engagement Ranges** (from real-world published data):
- Patriot PAC-3: ~70km, terminal phase, 0.5-40km altitude
- THAAD: ~200km, endo/exo-atmospheric, 40-150km altitude
- AEGIS SM-3: ~500km, exo-atmospheric midcourse, 70-500km altitude
- GBI: ~2000km, midcourse intercept, 200-2000km altitude
- Iron Dome: ~70km, short-range rockets, 0.5-10km altitude
- Arrow 3: ~400km, exo-atmospheric, 20-400km altitude
- S-400: ~400km, multi-layer, 0.01-400km altitude

**Realistic Engagement Envelopes**:
- **SRBMs** (Short-Range, <1000km): Apogee ~80-100km → THAAD, Patriot can engage
- **MRBMs** (Medium-Range, 1000-3500km): Apogee ~100-300km → THAAD, AEGIS can engage
- **IRBMs** (Intermediate-Range, 3500-5500km): Apogee ~300-600km → AEGIS, Arrow 3 can engage
- **ICBMs** (Intercontinental, >5500km): Apogee ~400-1200km → GBI required (midcourse phase)
- **Note**: THAAD cannot engage ICBMs at apogee (400+km altitude exceeds 150km max altitude)

**Detection Ranges**:
- AN/TPY-2 (THAAD): 1000km forward-based mode
- AN/SPY-1D (AEGIS): 400km volume search
- Sea-Based X-band: 1000km discrimination
- SBIRS-GEO: Global missile launch detection

**System Constraints**:
- Interceptor inventory limits per unit
- Salvo size doctrine (2-4 shots per target)
- Shoot-Look-Shoot vs Shoot-Shoot-Look timing
- Engagement zone altitude/range windows

### Engagement Doctrine

- **Salvo Fire**: Multiple interceptors (default salvo of 2) per target for redundancy
- **Automatic Doctrine Selection**: System chooses optimal engagement strategy based on sensor-derived time-to-impact and track confidence
  - **Shoot-Look-Shoot (SLS)**: When time permits and track confidence ≥0.75
    - Launch first interceptor
    - Assess hit/miss after a 3-second kill-assessment delay
    - Launch follow-up only if the first missed
    - **Advantage**: More efficient (conserves interceptors)
  - **Shoot-Shoot-Look (SSL)**: When time is limited or confidence is lower
    - Fire the salvo immediately (spread by a 5s launch delay)
    - Assess results after all shots
    - **Advantage**: Higher kill probability
- **Confidence-Scaled Salvos**: High-confidence tracks get 1 shot, medium 2, low 3
- **Per-Target Shot Cap**: `max_shots_per_target` (default 4) bounds total shots across the whole engagement; SLS keeps firing after misses while shots and time remain
- **Shot Accounting**: Tracks shots fired per target to enforce doctrine limits
- **Priority Targeting**: Detections sorted by quality and network track status

### Countermeasures & Deception

- **Decoy Deployment**: Missiles carry 0-6 decoys (configurable)
- **Detection Quality Degradation**: Each decoy reduces detection probability by 30%
- **RCS Simulation**: Decoys affect sensor tracking quality

## Requirements

- Rust 1.70+ (Edition 2021)
- macOS (currently targeting Apple Silicon)

## Installation

### Install Rust

If you don't have Rust installed, install it using rustup:

```bash
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
```

Follow the on-screen instructions, then restart your terminal or run:

```bash
source $HOME/.cargo/env
```

Verify the installation:

```bash
rustc --version
cargo --version
```

### Clone the Repository

```bash
git clone https://github.com/jmichaelpace/GlobalThermonuclearWar.git
cd GlobalThermonuclearWar
```

### Install Project Dependencies

Cargo will automatically download and compile dependencies on first build:

```bash
cargo build
```

### Configure MapTiler API Key

The application uses MapTiler for map tiles. You'll need a free API key:

1. **Create a MapTiler account:**
   - Go to [https://www.maptiler.com/](https://www.maptiler.com/)
   - Click "Sign Up" and create a free account

2. **Get your API key:**
   - After signing in, go to [https://cloud.maptiler.com/account/keys/](https://cloud.maptiler.com/account/keys/)
   - Copy your API key (the free tier includes 100,000 requests/month)

3. **Create a `.env` file** in the project root directory:

   ```bash
   echo 'MAPTILER_API_KEY=your_api_key_here' > .env
   ```

   Or manually create a file named `.env` with the following content:

   ```
   MAPTILER_API_KEY=your_api_key_here
   ```

   Replace `your_api_key_here` with your actual MapTiler API key.

> **Note:** The `.env` file is excluded from git via `.gitignore` to keep your API key private.

## Building

### Build for Development

```bash
cargo build
```

### Build for Release

```bash
cargo build --release
```

## Running

### Run Development Build

```bash
cargo run
```

### Run Release Build

```bash
cargo run --release
```

## Configuration

### Scenario Files

Scenarios are defined in TOML files under `scenarios/`:

```
scenarios/
├── pacific_theater.toml    # North Korea vs US West Coast
├── middle_east.toml        # Iran vs Israel/Saudi Arabia
├── european_theater.toml   # Russia vs NATO
├── demo.toml              # Multi-region demonstration
└── README.md              # Scenario file format documentation
```

**Creating Custom Scenarios:**

Two ways:

1. **In-App Scenario Builder** (recommended): Click **Builder** in the top bar
   - Place defense units, radars, satellites, and missiles by picking a tool and clicking the map (missiles: click launch point, then target)
   - Edit any entity's properties in the panel; import existing scenarios for editing
   - Live validation (errors block saving; warnings explain likely problems)
   - **Test Run** loads the draft into the live engine without saving; **Save** writes `scenarios/<filename>.toml` and it appears immediately in the scenario selector
2. **By hand**: Create a new `.toml` file in the `scenarios/` directory, define metadata/defense units/radars/satellites/missiles, and it automatically appears in the in-game scenario selector

See `scenarios/README.md` for complete format documentation.

**Example Scenario File:**

```toml
[metadata]
id = "my_scenario"
name = "My Custom Scenario"
description = "A custom scenario"
region = "Pacific"
center_lat = 40.0
center_lon = 180.0
zoom = 2.5

[[defense_units]]
name = "THAAD Battery"
affiliation = "Friendly"
lat = 37.5
lon = -122.0
type = "THAAD"
interceptors = 48

[[missiles]]
name = "Test Missile"
affiliation = "Hostile"
origin_lat = 39.0
origin_lon = 125.5
target_lat = 37.5
target_lon = -122.0
launch_delay_sec = 30.0
```

### System Configuration

All system parameters are defined in TOML files under `config/`:

```
config/
├── platform/          # Combined defense system configs (legacy)
│
├── sensors/           # Radar and detection systems
│   ├── spy_1.toml         # AN/SPY-1D (AEGIS)
│   ├── tpy_2.toml         # AN/TPY-2 (THAAD)
│   ├── sbx.toml           # Sea-Based X-band (GBI)
│   └── ...
│
├── interceptors/      # Interceptor missiles
│   ├── sm3_block_iia.toml # AEGIS SM-3
│   ├── thaad_interceptor.toml
│   ├── gbi_ekv.toml
│   └── ...
│
├── satellites/        # Early warning satellites
│   ├── sbirs_geo.toml     # US GEO early warning
│   ├── tundra.toml        # Russian EKS
│   └── ...
│
└── missiles/          # Offensive missiles (by type)
    ├── icbm/
    │   ├── hwasong_15.toml
    │   └── hwasong_14.toml
    ├── irbm/
    ├── mrbm/
    ├── slbm/
    └── srbm/
```

### Example: Missile Configuration

```toml
[system]
name = "Hwasong-15"
description = "North Korean ICBM"
country = "DPRK"

[classification]
type = "ICBM"

[range]
min_range_km = 8500.0
max_range_km = 13000.0

[trajectory]
apogee_base_km = 800.0
apogee_range_factor = 0.08
flight_time_base_sec = 1200.0
flight_time_range_factor = 0.1

[countermeasures]
has_countermeasures = true
default_decoys = 2
max_decoys = 6
```

### Example: Sensor Configuration

```toml
[system]
name = "AN/TPY-2"
description = "THAAD X-band radar"
country = "USA"

[detection]
detection_range_km = 1000.0
azimuth_coverage_deg = 120.0
elevation_min_deg = 0.0
elevation_max_deg = 90.0
radar_band = "X"  # X-band for high-resolution tracking

[tracking]
max_simultaneous_tracks = 50
track_update_rate_hz = 20.0
minimum_rcs_dbsm = -30.0
```

## Architecture

The codebase is organized into modular components for maintainability and performance:

```
src/
├── main.rs              # Application entry point (binary module tree)
├── lib.rs               # Library target: exposes types, simulation, scenario for tests
├── app.rs               # Main application state and rendering
├── effects/             # Visual effects system
│   └── mod.rs           # EffectsManager, explosion/intercept animations
├── tracking/            # Event tracking and logging
│   └── mod.rs           # EventTracker, EventLog, simulation events
├── view/                # Map projection abstractions
│   └── mod.rs           # MapProjection trait, Mercator/Globe projections
├── map/                 # Geographic utilities
│   ├── tiles.rs         # Map tile caching (MapTiler, NASA GIBS)
│   └── viewport.rs      # 2D viewport management
├── rendering/           # Rendering utilities
│   ├── colors.rs        # Centralized color definitions by affiliation/mode
│   ├── overlays.rs      # Detection range overlays
│   └── symbols.rs       # Military symbology
├── ui/                  # Feature UI modules (binary only)
│   └── scenario_builder.rs  # Scenario builder panel, tools, draft overlay
├── scenario/            # Scenario loading and building
│   ├── loader.rs        # TOML scenario parser (Serialize + Deserialize)
│   └── builder.rs       # Draft model, validation, save (lib, unit-tested)
└── simulation/          # Core simulation engine
    ├── engine.rs        # SimulationEngine, engagement logic, fire control
    ├── entities.rs      # Missile, Interceptor, DefenseUnit, InterceptorKinematics
    ├── detection.rs     # Sensor modeling, track fusion, converged trajectories
    ├── physics.rs       # Ballistic trajectories, Lambert guidance
    ├── ekf.rs           # Extended Kalman filter (geodetic state, radar measurements)
    ├── kalman.rs        # Linear Kalman filter for track estimation
    ├── runner.rs        # Sim/render thread split via crossbeam channels
    └── config.rs        # Configuration registries

tests/                   # Integration tests (run against the library target)
├── track_prediction_test.rs        # Track velocity estimation, EKF convergence
├── impact_prediction_test.rs       # Converged trajectory accuracy/stability
├── interceptor_engagement_test.rs  # Deterministic intercept, envelopes, doctrine
└── scenario_builder_test.rs        # TOML round-trip, draft→engine chain
```

### Key Design Patterns

- **Sensor-Derived Fire Control**: ALL engagement decisions (launch solutions, guidance, doctrine timing, Pk factors, terminal lead) project the target through the converged trajectory estimate from radar measurements — there is deliberately no ground-truth fallback
- **Single Source of Truth for Arrival Timing**: `InterceptorKinematics::time_to_cover_distance` (boost- and drag-aware) serves every arrival-time calculation
- **HashMap for O(1) Lookups**: Trajectory and track lookups use HashMaps instead of linear searches
- **Modular Effects System**: Visual effects (explosions, intercepts) are managed by `EffectsManager` with spawnable effect requests
- **Consolidated Event Tracking**: All simulation event state is encapsulated in `EventTracker`
- **Projection Abstraction**: `MapProjection` trait enables unified rendering across 2D/3D views
- **Centralized Colors**: Affiliation-based colors defined once in `rendering/colors.rs`
- **Dual Module Trees**: The binary and library targets declare identical module trees; integration tests run against the library. Keep `pub mod`/`pub use` in both trees in sync

## Development

### Code Style

```bash
cargo fmt      # Format code
cargo clippy   # Run linter
cargo test     # Run tests
cargo check    # Quick compile check
```

### Testing

Integration tests live in `tests/` and run against the library target:

```bash
cargo test                                              # Full suite
cargo test --test interceptor_engagement_test           # One suite
cargo test test_aegis_intercepts_mrbm_deterministic     # One test by name
```

The suite covers impact-prediction accuracy/stability (the yellow X marker must converge near truth and stop moving), deterministic Aegis-vs-MRBM interception, engagement envelope enforcement (THAAD refuses ICBM apogees), no-launch-without-track-quality, shoot-look-shoot doctrine, and TOML scenario round-trips on every scenario file.

Unit tests also live inline (`#[cfg(test)]`): scenario builder draft/validation logic in `src/scenario/builder.rs` and headless egui render tests for the builder panel in `src/ui/scenario_builder.rs`.

## Documentation

Domain documentation lives in `docs/`:

- `radar-detection-tracking.md` — sensor modeling, track establishment, fusion
- `platform-intercept-geometry.md` — intercept kinematics and firing logic
- `physics.md` — trajectory and flight physics
- `audit-plan.md` — implementation progress vs. domain requirements
- `ballistic-track-explainer.md` — how missile tracks are computed, explained at a high-school level with code references
- `interceptor-logic-explainer.md` — how interceptors are launched, guided, and resolved, same style

Configuration reference: `config/` (equipment specs), `scenarios/README.md` (scenario format), `AGENTS.md` (development conventions for coding agents).

## License

MIT

## Acknowledgments

"A strange game. The only winning move is not to play."
