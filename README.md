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
- **Automatic Salvo Doctrine**: Intelligent Shoot-Look-Shoot vs Shoot-Shoot-Look selection
- **Separate Sensor and Interceptor Modeling**: Radars and interceptors configured independently for realistic system composition
- **Early Warning Satellites**: SBIRS, DSP, Tundra, and other space-based detection systems
- **TOML-based Configuration**: All system parameters externalized for easy modification
- **Scenario System**: Pre-built scenarios for different regions and threat environments

## Realism Features

The simulation models real-world missile defense physics and sensor limitations for accurate scenario analysis.

### Key Realism Highlights

This simulation implements **sensor-based intercepts** instead of "perfect information" intercepts:

✅ **No Omniscient Defense Systems**: Defense units cannot see missiles directly - they only see what sensors detect
✅ **Track Establishment Required**: Must have 3+ sensor measurements before fire control lock authorized
✅ **Fire Control Radar Architecture**: Realistic two-stage process (search/track → fire control lock)
✅ **Undetected = Unengaged**: Missiles outside sensor coverage cannot be intercepted
✅ **Engagement Envelope Enforcement**: THAAD cannot engage ICBMs at 400km altitude (max 150km)
✅ **Automatic Salvo Doctrine**: System selects Shoot-Look-Shoot vs Shoot-Shoot-Look based on time available

### Sensor & Detection

- **Sensor-Based Intercept Solutions**: Defense systems use only sensor-detected positions, not ground truth
- **Track Quality Thresholds**: Minimum track quality (0.4) required for engagement authorization
- **Track Establishment**: Requires 3+ measurements before fire control lock authorized
- **Position History Tracking**: Maintains 5 most recent position measurements per sensor
- **Velocity Estimation**: Calculated from position deltas using weighted least-squares
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

### Intercept Calculations

- **Hybrid Fire Control Radar System**: Realistic two-stage engagement process
  - **Search/Track Radars**: Establish initial track (3+ measurements, quality >0.4, freshness <5s)
  - **Fire Control Radars**: High-precision lock (TPY-2, SPY-1, MPQ-53/65) provides 1-3km accuracy
  - **Sensor-Based Gating**: Cannot engage undetected missiles (no track = no intercept)
- **Track Establishment Requirements**:
  - Minimum 3 sensor measurements to establish track
  - Track quality ≥0.4 (0.0-1.0 scale)
  - Track freshness <5 seconds (no stale data)
- **Fire Control Lock**: Once track established, precision radar provides intercept-quality data
- **Iterative Refinement**: 20-iteration convergence to find valid intercept geometry
- **Engagement Envelope Validation**: Altitude and range constraints strictly enforced
- **Time-to-Impact Margin**: Requires ≥3s margin before impact
- **Interceptor Kinematics**: Realistic boost/coast/terminal phase flight modeling
- **Two-Phase Flight Model**: Boost acceleration + coast at max velocity

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

- **Salvo Fire**: Multiple interceptors (default 2) per target for redundancy
- **Automatic Doctrine Selection**: System chooses optimal engagement strategy based on time available
  - **Shoot-Look-Shoot (SLS)**: When time permits (~400s+ to impact)
    - Launch first interceptor
    - Assess hit/miss after 10s
    - Launch second only if first missed
    - **Advantage**: More efficient (conserves interceptors)
  - **Shoot-Shoot-Look (SSL)**: When time is limited (<400s to impact)
    - Launch both interceptors immediately
    - Assess results after both shots
    - **Advantage**: Higher kill probability
- **Follow-up Shots**: Additional attempts after initial miss (up to salvo_size limit)
- **Shot Accounting**: Tracks shots fired per target to enforce salvo limits
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

1. Create a new `.toml` file in the `scenarios/` directory
2. Define metadata, defense units, radars, satellites, and missiles
3. The scenario automatically appears in the in-game scenario selector
4. See `scenarios/README.md` for complete format documentation

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

## Development

### Code Style

```bash
cargo fmt      # Format code
cargo clippy   # Run linter
cargo test     # Run tests
cargo check    # Quick compile check
```

## License

MIT

## Acknowledgments

"A strange game. The only winning move is not to play."
