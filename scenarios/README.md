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

### Radar Stations

```toml
[[radar_stations]]
name = "Radar Name"                     # Display name
affiliation = "Friendly"                # Friendly, Hostile, or Neutral
lat = 52.7                              # Latitude
lon = 174.1                             # Longitude
range_km = 2500.0                       # Detection range in kilometers
```

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
name = "Missile Name"                   # Display name
affiliation = "Hostile"                 # Usually Hostile
origin_lat = 39.0                       # Launch latitude
origin_lon = 125.5                      # Launch longitude
target_lat = 37.5                       # Target latitude
target_lon = -122.0                     # Target longitude
launch_delay_sec = 30.0                 # Delay before launch (seconds)
```

## Missile Type Guidelines

The simulation automatically determines missile type based on range:

- **SRBM** (Short-Range, <1000km): Apogee ~80-100km → THAAD, Patriot can engage
- **MRBM** (Medium-Range, 1000-3500km): Apogee ~100-300km → THAAD, AEGIS can engage
- **IRBM** (Intermediate-Range, 3500-5500km): Apogee ~300-600km → AEGIS, Arrow 3 can engage
- **ICBM** (Intercontinental, >5500km): Apogee ~400-1200km → GBI required

## Available Scenarios

### Complex Scenarios
Full-scale theater scenarios with multiple defense layers and realistic threat environments:

- **`pacific_theater.toml`** - North Korea vs US/Japan (ICBM, IRBM, MRBM threats)
- **`middle_east.toml`** - Iran vs Israel/Saudi Arabia (MRBM/IRBM threats)
- **`european_theater.toml`** - Russian SRBM strikes vs NATO defenses
- **`demo.toml`** - Multi-region demonstration scenario with SRBM tests

### Platform Test Scenarios
Simplified scenarios for isolated platform testing. Each contains a single defense unit with appropriate test missiles within its engagement envelope:

- **`test_thaad.toml`** - THAAD Battery vs SRBM/MRBM (40-150km altitude envelope)
- **`test_aegis.toml`** - AEGIS Destroyer vs MRBM/IRBM (70-500km altitude envelope)
- **`test_patriot.toml`** - Patriot Battery vs SRBM terminal phase (0.5-40km altitude)
- **`test_gbi.toml`** - GBI Site vs ICBM midcourse (200-2000km altitude envelope)
- **`test_iron_dome.toml`** - Iron Dome Battery vs short-range rockets (0.5-10km altitude)
- **`test_arrow3.toml`** - Arrow 3 Battery vs MRBM/IRBM (20-400km altitude envelope)
- **`test_davids_sling.toml`** - David's Sling Battery vs SRBM/MRBM (10-70km altitude)
- **`test_s400.toml`** - S-400 Battery vs multi-layer threats (0.01-400km altitude)

**Use platform test scenarios to:**
- Validate individual platform engagement logic
- Verify salvo fire doctrine (Shoot-Look-Shoot vs Shoot-Shoot-Look)
- Measure intercept success rates within specified envelopes
- Debug fire control radar track establishment requirements

### Sensor Test Scenarios
Radar-only scenarios for testing detection, tracking, and track quality without interceptors. Each contains a single radar with missiles at various ranges to test detection envelope:

- **`test_sensor_sbx.toml`** - Sea-Based X-band (2000km range, ICBM discrimination)
- **`test_sensor_cobra_dane.toml`** - Cobra Dane, Shemya Island (3000km range, L-band early warning)
- **`test_sensor_tpy2.toml`** - AN/TPY-2 THAAD radar (1000km range, X-band forward-based mode)
- **`test_sensor_spy1.toml`** - AN/SPY-1 AEGIS radar (400km range, S-band volume search)
- **`test_sensor_fylingdales.toml`** - Fylingdales BMEWS, UK (3000km range, UHF phased-array)
- **`test_sensor_pave_paws.toml`** - PAVE PAWS Beale AFB (3000km range, UHF early warning)
- **`test_sensor_green_pine.toml`** - Green Pine/Super Green Pine, Israel (800km range, Arrow fire control)
- **`test_sensor_don2n.toml`** - Don-2N Moscow ABM radar (1000km range, Russian battle management)
- **`test_sensor_thule.toml`** - Thule BMEWS, Greenland (3000km range, Arctic coverage)

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
- Affiliation determines IFF (Identification Friend or Foe)
- Defense system types must match the available types exactly (case-sensitive)
- Missile classes (SRBM/MRBM/IRBM/ICBM) are auto-derived from range at load:
  SRBM < 1000 km ≤ MRBM < 3000 km ≤ IRBM < 5500 km ≤ ICBM
