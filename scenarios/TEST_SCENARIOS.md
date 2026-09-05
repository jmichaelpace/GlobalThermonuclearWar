# Test Scenario Quick Reference

All test scenarios use **config-backed missile profiles** (missile names resolve
to `config/missiles/` — the `" #N"` suffix is stripped by the lookup) and
**explicit sensor wiring** (every radar station declares `sensor_config`, every
narrow-azimuth sensor/platform has `facing_deg`).

Apogee values below are computed from the missile config:
`apogee = apogee_base_km + apogee_range_factor * range_km`.

## Platform Tests (Defense System + Interceptor)

| Scenario | System | Interceptor Envelope | Threats (config) | Purpose |
|----------|--------|----------------------|------------------|---------|
| `test_thaad.toml` | THAAD + TPY-2 | 40-150 km alt, 200 km range, 2.8 km/s | Burkan-2 (115/105 km apogee), Shahab-3 (356 km apogee) | Descent intercepts through the envelope; salvo fire |
| `test_aegis.toml` | AEGIS + SPY-1D | 100-600 km alt (exo), 2,500 km flyout, 4.5 km/s | Shahab-3 (363 km), Sejjil (570 km), RS-26 Rubezh (680 km apogee) | Exo midcourse/descent intercepts |
| `test_patriot.toml` | Patriot + MPQ-65 | 0.5-40 km alt, 70 km range, 1.7 km/s | Iskander-M x3 (59-73 km apogee) | Terminal-phase-only engagement (apogees above ceiling) |
| `test_gbi.toml` | GBI + Cobra Dane + SBX cueing | 200-2000 km alt, 2000 km range, 8.0 km/s | Hwasong-15 x2 (1,525/1,600 km apogee) | Long-range midcourse |
| `test_iron_dome.toml` | Iron Dome + EL/M-2084 | 0-10 km alt, 70 km range, 0.7 km/s | Grad x3 (7.6-9.3 km apogee, south Lebanon geometry) | Rocket-class intercepts, rapid engagement |
| `test_arrow3.toml` | Arrow 3 + Green Pine | 100-1000 km alt (exo), 2,400 km flyout, ~3.0 km/s | Shahab-3 x2, RS-26 Rubezh (439-695 km apogee) | Exo-atmospheric intercepts at apogee/midcourse |
| `test_davids_sling.toml` | David's Sling + EL/M-2084 | 2-15 km alt, 300 km range, 2.55 km/s | Iskander-M x2, Shahab-3 (72-265 km apogee) | Terminal-window engagement (tiny window vs MRBM — misses expected) |
| `test_s400.toml` | S-400 + Big Bird/Grave Stone | 40N6: 0.01-30 km alt, 400 km range, 2.1 km/s | Iskander-M, Shahab-3, Sejjil, RS-26 Rubezh | Terminal-only vs SRBM; high-altitude refusals expected (40N6 is a SAM, not a midcourse BMD interceptor) |
| `test_aegis_geometry.toml` | AEGIS + SPY-1D | 100-600 km alt (exo) | Shahab-3 x3 (#HEAD-ON / #CROSSING / #OBLIQUE) | Guidance geometry: head-on engages, crossing legitimately misses (gimbal-limited terminal homing) |

## Sensor Tests (Radar Only - No Interceptors)

| Scenario | Sensor Config | Nominal Range | Max (1.5×) | Azimuth | Band | Threats | Purpose |
|----------|---------------|---------------|------------|---------|------|---------|---------|
| `test_sensor_sbx.toml` | SBX-1 | 4000 km | 6000 km | 25° ("soda straw") | X | RS-26 Rubezh, Hwasong-15 x2 | Narrow-arc discrimination, cued by PAVE PAWS; off-beam arc stays undetected by SBX |
| `test_sensor_cobra_dane.toml` | Cobra Dane | 3000 km | 4500 km | 120° (facing 240°) | L | Hwasong-15 x5 | Early warning, NK launch-region coverage |
| `test_sensor_tpy2.toml` | AN/TPY-2 | 1000 km | 1500 km | 120° (facing 180°) | X | Iskander-M, Shahab-3, RS-26 Rubezh, Hwasong-15 x2 | Forward-based mode, range ladder incl. azimuth/range-gated non-detections |
| `test_sensor_spy1.toml` | AN/SPY-1D | 500 km | 750 km | 360° | S | Iskander-M, Shahab-3 x2, RS-26 Rubezh x2 | Volume search, range ladder past nominal |
| `test_sensor_fylingdales.toml` | Fylingdales | 3000 km | 4500 km | 360° | L (UHF stand-in) | RS-26 Rubezh x2, Hwasong-15 x3 | European early warning, east-axis ladder |
| `test_sensor_pave_paws.toml` | PAVE PAWS | 3000 km | 4500 km | 240° (facing 280°) | L (UHF stand-in) | Bulava, RS-26 Rubezh, Hwasong-15 x3 | CONUS early warning, SLBM + ICBM arcs |
| `test_sensor_green_pine.toml` | Green Pine | 800 km | 1200 km | 120° (facing 67°) | L | Iskander-M, Shahab-3 x2, RS-26 Rubezh x2 | Arrow fire control, Iran-axis ladder incl. azimuth-gated non-detection |
| `test_sensor_don2n.toml` | Don-2N | 1000 km | 1500 km | 360° | X | Iskander-M, Sineva, Bulava, Trident II x2 | ABM battle management, western SLBM approach ladder |
| `test_sensor_thule.toml` | Thule | 3000 km | 4500 km | 240° (facing 55°) | L (UHF stand-in) | Sineva, Hwasong-15 x4 | Arctic early warning, Russian corridor |

**Note**: Detection range rings show the maximum possible detection range (1.5× nominal). Detection probability degrades significantly with range, especially beyond nominal range. See `DETECTION_RANGES.md` for details.

## Testing Objectives

### Platform Tests
- ✅ Validate engagement envelope enforcement (altitude/range limits)
- ✅ Verify salvo fire doctrine (Shoot-Look-Shoot vs Shoot-Shoot-Look)
- ✅ Test fire control radar track establishment (3+ measurements, quality >0.4, staleness <5s)
- ✅ Measure intercept success rates
- ✅ Verify undetected missiles cannot be engaged

### Sensor Tests
- ✅ Validate detection range vs published specifications
- ✅ Test track quality degradation with range/altitude
- ✅ Verify track establishment requirements
- ✅ Measure detection probability vs RCS, range, altitude
- ✅ Test track staleness and freshness limits
- ✅ Debug sensor-based gating (no track = no engagement authorization)
- ✅ Verify azimuth gating (narrow-cone radars must be aimed with `facing_deg`)

## Test Methodology

### For Platform Tests:
1. Load scenario
2. Run simulation
3. Observe engagement decisions (should engage threats within envelope, refuse threats outside)
4. Check salvo fire behavior (SLS vs SSL based on time available)
5. Verify track establishment before engagement authorization

### For Sensor Tests:
1. Load scenario
2. Run simulation
3. Monitor which missiles are detected at which ranges
4. Check track quality values (should degrade with range)
5. Verify track establishment (3+ measurements before authorization)
6. Note which missiles fall outside detection envelope (should remain undetected)

## Expected Behaviors

### Platform Tests:
- **THAAD**: Engages Burkan-2s (descent through 40-150 km); Shahab-3 only on descent below 150 km
- **AEGIS**: Engages Shahab-3/Sejjil inside the 100-600 km exo band; RS-26 Rubezh on descent
- **Patriot**: Terminal-only — all Iskander apogees are above the 40 km ceiling
- **GBI**: Hwasong-15 midcourse at 1,500+ km altitude
- **Iron Dome**: Grad rockets at 7-9 km apogee — the only rocket-class test that actually works
- **David's Sling**: Tiny terminal windows vs 72-265 km apogees — misses/refusals expected vs the MRBM
- **S-400**: Iskander terminal engagement; Shahab-3/Sejjil/RS-26 refused above 30 km (realistic SAM limits)
- **AEGIS geometry**: HEAD-ON engages; CROSSING legitimately misses; OBLIQUE partial-lead

### Sensor Tests:
- **Long-range radars (3000-4000 km nominal)**: Detect ICBMs across their arcs; quality degrades past nominal
- **Mid-range radars (800-1000 km nominal)**: Detect within range, degrade at edge; azimuth gating for Green Pine/TPY-2/Cobra Dane
- **Short-range radars (500 km nominal)**: Strong quality nearby, lose distant tracks
- **All Systems**: Require 3+ measurements before track establishment
- **All Systems**: Track quality >0.4 required for engagement authorization
- **All radars with <360° azimuth**: Must have `facing_deg` set — an unaimed cone misses its own test targets

## Scenario Combinations

For advanced testing, combine scenarios:
- Load `test_sensor_sbx.toml` + add AEGIS units from `test_aegis.toml` for multi-sensor fusion
- Load `test_sensor_tpy2.toml` + add THAAD from `test_thaad.toml` for integrated sensor/shooter
- Verify multi-sensor fusion improves track quality (2+ sensors reduce uncertainty by 10-20%)