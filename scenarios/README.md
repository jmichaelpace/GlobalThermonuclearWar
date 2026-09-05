# Scenario Files

This directory contains scenario definition files in TOML format.

## File Format

Each scenario file should have the following structure:

### Metadata Section

```toml
[metadata]
id = "unique_scenario_id"              # Unique identifier (snake_case)
name = "Display Name"                   # Human-readable name
description = "Scenario description"    # Brief description
region = "Region Name"                  # Geographic region
center_lat = 40.0                       # Center latitude for map view
center_lon = 180.0                      # Center longitude for map view
zoom = 2.5                              # Initial zoom level
```

### Defense Units

```toml
[[defense_units]]
name = "Unit Name"                      # Display name
affiliation = "Friendly"                # Friendly, Hostile, or Neutral
lat = 37.5                              # Latitude
lon = -122.0                            # Longitude
type = "THAAD"                          # Defense system type
interceptors = 48                       # Number of interceptors
facing_deg = 285.0                      # Optional: sensor emplacement azimuth (0=North)

# Available defense types:
# - THAAD: Terminal High Altitude Area Defense
# - Aegis: Aegis Ballistic Missile Defense
# - Patriot: Patriot PAC-3
# - GBI: Ground-Based Interceptor
# - IronDome: Iron Dome
# - Arrow3: Arrow 3
# - DavidsSling: David's Sling
# - S400: S-400 Triumf
```

`facing_deg` aims the unit's fire control radar cone at the threat axis
(real batteries are emplaced facing the expected threat direction). It has
no effect on 360-degree sensors (Aegis SPY-1, S-400 Big Bird); it is
essential for narrow-azimuth sensors (TPY-2, Green Pine, EL/M-2084, MPQ-65,
Cobra Dane, Grave Stone) — an unaimed cone will not see its own test targets.

### Radar Stations

```toml
[[radar_stations]]
name = "Radar Name"                     # Display name
affiliation = "Friendly"                # Friendly, Hostile, or Neutral
lat = 52.7                              # Latitude
lon = 174.1                             # Longitude
range_km = 2500.0                       # Display/false-alarm range (see note)
sensor_config = "Cobra Dane"            # REQUIRED: references config/sensors/
facing_deg = 240.0                      # Optional: azimuth the radar faces (0=North)
```

**`sensor_config` is required.** The engine resolves radar behavior
(detection range, azimuth, band, track rates) from the named sensor config —
the `range_km` field only affects false-alarm placement. Without an explicit
`sensor_config`, the station's display name is used for lookup, which
usually misses and silently falls back to the 200 km Generic Radar.
The value must match a `config/sensors/*.toml` file stem or its `system.name`
(e.g. `"Cobra Dane"`, `"AN/TPY-2"`, `"SBX-1"`, `"tpy_2"`).

### Satellites

```toml
[[satellites]]
name = "Satellite Name"                 # Display name
affiliation = "Friendly"                # Friendly, Hostile, or Neutral
lat = 0.0                               # Latitude (GEO satellites at equator)
lon = 120.0                             # Longitude
altitude_km = 35786.0                   # Altitude in kilometers
sensor_type = "Infrared"                # Infrared or Radar
```

### Missiles

```toml
[[missiles]]
name = "Missile Name"                   # MUST match a config/missiles/ profile
affiliation = "Hostile"                 # Usually Hostile
origin_lat = 39.0                       # Launch latitude
origin_lon = 125.5                      # Launch longitude
target_lat = 37.5                       # Target latitude
target_lon = -122.0                     # Target longitude
launch_delay_sec = 30.0                 # Delay before launch (seconds)
```

**Missile names must reference a real profile.** The engine looks the name
up in `config/missiles/` (stripping `" #N"` suffixes) to get the trajectory,
RCS, and countermeasures; unknown names silently fall back to the Generic
Ballistic Missile (typed MRBM, apogee 150 + 0.15×range). Use real designations
("Hwasong-15", "Shahab-3", "Iskander-M", "Bulava", "Grad", ...) with optional
`" #1"`-style suffixes for multiples. `tests/scenario_wiring_test.rs` fails
on any missile that resolves to the generic default.

## Missile Type Guidelines

Missile class is auto-derived from range at load:

- **SRBM** (Short-Range, <1000km): apogee per profile (Iskander ~60-90km, Grad ~6-9km) → THAAD, Patriot, Iron Dome (Grad only) can engage
- **MRBM** (Medium-Range, 1000-3000km): apogee ~250-500km per profile → AEGIS and Arrow 3 (exo, above 100km), THAAD (descent below 150km)
- **IRBM** (Intermediate-Range, 3000-5500km): apogee ~400-700km → AEGIS, GBI
- **ICBM** (Intercontinental, >5500km): apogee ~450-1600km → GBI required

## Available Scenarios

### Complex Scenarios
Full-scale theater scenarios with multiple defense layers and realistic threat environments:

- **`pacific_theater.toml`** - North Korea vs US/Japan (Hwasong-15 ICBMs, THAAD/GBI/Aegis layers, Cobra Dane + SBX)
- **`middle_east.toml`** - Iran vs Israel/Saudi Arabia (Shahab-3/Emad/Sejjil, all Israeli layers aimed at the Iran axis)
- **`european_theater.toml`** - Russian Iskander/RS-26 strikes vs NATO defenses (Fylingdales + Thule early warning)
- **`north_atlantic.toml`** - Russian SLBMs (Bulava/Sineva) vs US East Coast (Thule + PAVE PAWS + Aegis)
- **`moscow_defense.toml`** - Red-side: Hostile S-400/Don-2N defending Moscow vs Friendly Trident II SLBMs (IFF and envelope-refusal test)
- **`demo.toml`** - Multi-region demonstration (default scenario at startup)

### Platform Test Scenarios
Simplified scenarios for isolated platform testing. Each contains a single defense unit (aimed with `facing_deg`) with config-backed test missiles:

- **`test_thaad.toml`** - THAAD Battery vs Burkan-2/Shahab-3 (descent through 40-150km envelope)
- **`test_aegis.toml`** - AEGIS Destroyer vs Shahab-3/Sejjil/RS-26 Rubezh (exo, 100-600km envelope)
- **`test_patriot.toml`** - Patriot Battery vs Iskander-M terminal phase (0.5-40km altitude)
- **`test_gbi.toml`** - GBI Site vs Hwasong-15 midcourse (200-2000km envelope)
- **`test_iron_dome.toml`** - Iron Dome Battery vs Grad rockets (0-10km, south Lebanon geometry)
- **`test_arrow3.toml`** - Arrow 3 Battery vs Shahab-3/RS-26 Rubezh (exo-atmospheric, 100-1000km envelope)
- **`test_davids_sling.toml`** - David's Sling Battery vs Iskander-M/Shahab-3 (terminal window)
- **`test_s400.toml`** - S-400 Battery vs multi-layer threats (40N6 is 0.01-30km - high-altitude refusals expected)
- **`test_aegis_geometry.toml`** - AEGIS vs head-on/crossing/oblique geometries

**Use platform test scenarios to:**
- Validate individual platform engagement logic
- Verify salvo fire doctrine (Shoot-Look-Shoot vs Shoot-Shoot-Look)
- Measure intercept success rates within specified envelopes
- Debug fire control radar track establishment requirements

### Sensor Test Scenarios
Radar-only scenarios for testing detection, tracking, and track quality without interceptors. Each contains a single radar (explicitly wired via `sensor_config`, aimed via `facing_deg` where its cone is < 360°) with missiles at various ranges to test the detection envelope:

- **`test_sensor_sbx.toml`** - Sea-Based X-band (4000km, X-band, 25° "soda straw" arc, cued midcourse discrimination)
- **`test_sensor_cobra_dane.toml`** - Cobra Dane, Shemya Island (3000km, L-band, 120° cone aimed at NK)
- **`test_sensor_tpy2.toml`** - AN/TPY-2 THAAD radar (1000km, X-band, 120° cone, forward-based mode)
- **`test_sensor_spy1.toml`** - AN/SPY-1 AEGIS radar (500km, S-band, 360°, volume search)
- **`test_sensor_fylingdales.toml`** - Fylingdales BMEWS, UK (3000km, L-band/UHF stand-in, 360°)
- **`test_sensor_pave_paws.toml`** - PAVE PAWS Beale AFB (3000km, L-band/UHF stand-in, 240° cone)
- **`test_sensor_green_pine.toml`** - Green Pine, Israel (800km, L-band, 120° cone aimed at Iran)
- **`test_sensor_don2n.toml`** - Don-2N Moscow ABM radar (1000km, X-band, 360°, SLBM approach ladder)
- **`test_sensor_thule.toml`** - Thule BMEWS, Greenland (3000km, L-band/UHF stand-in, 240° cone)

**Use sensor test scenarios to:**
- Validate detection range and coverage
- Test track establishment (3+ measurements required)
- Measure track quality and staleness
- Verify multi-sensor track fusion (when combined with other scenarios)
- Debug sensor-based gating (undetected missiles cannot be engaged)
- Measure detection probability vs range, altitude, RCS

## Example Scenario

See `pacific_theater.toml`, `middle_east.toml`, or `demo.toml` for complete examples.

## Creating New Scenarios

Two ways:

### In-App Scenario Builder (recommended)

Click **Builder** in the top bar. You can:

- **Place entities on the map**: pick a tool (Defense / Radar / Satellite / Missile), click to place. Missiles use a two-click flow: launch point, then target (with a rubber-band preview; Esc cancels).
- **Edit properties**: click a placed entity (Select tool) or pick it from the entity lists — edit name, affiliation, type (dropdowns prevent case-sensitivity mistakes), interceptors, ranges, launch delays, etc.
- **Import existing scenarios**: the Import section clones any loaded scenario into the draft for editing.
- **Validation is live**: errors (bad coordinates, duplicate names, origin == target, ...) block Save/Test Run; warnings (no missiles, impact point far from all defenders, unknown sensor config) don't block but explain likely problems.
- **Test Run** loads the draft into the live engine without saving — place entities, watch the engagement, tweak, re-test.
- **Save** writes `scenarios/<filename>.toml` and the new scenario appears immediately in the Scenarios panel (loading it selects it).

### By hand

1. Create a new `.toml` file in this directory
2. Follow the format above
3. Use realistic coordinates and equipment specifications
4. Test in the simulation
5. The scenario will automatically appear in the scenario selection menu

## Notes

- All coordinates use decimal degrees (negative for West/South)
- Distances and ranges are in kilometers
- Launch delays are in seconds from scenario start
- Affiliation determines IFF (Identification Friend or Foe); units engage
  anything that is not own-side and not Neutral (red-side defense works)
- Defense system types must match the available types exactly (case-sensitive)
- Radar stations require `sensor_config` (see above); narrow-cone radars also need `facing_deg`
- Missile classes (SRBM/MRBM/IRBM/ICBM) are auto-derived from range at load:
  SRBM < 1000 km ≤ MRBM < 3000 km ≤ IRBM < 5500 km ≤ ICBM — but trajectory,
  RCS, and countermeasures come from the named profile in `config/missiles/`
- `tests/scenario_wiring_test.rs` guards all of the above: every scenario's
  radars must resolve real sensor configs, every missile must resolve a real
  profile, and every scenario must pass builder validation
