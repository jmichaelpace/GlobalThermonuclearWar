# Test Scenario Quick Reference

## Platform Tests (Defense System + Interceptor)

| Scenario | System | Envelope | Threats | Purpose |
|----------|--------|----------|---------|---------|
| `test_thaad.toml` | THAAD | 40-150km alt, ~200km range | 2 SRBM, 1 MRBM | Terminal high-altitude intercept, salvo fire |
| `test_aegis.toml` | AEGIS | 70-500km alt, ~500km range | 2 MRBM, 1 IRBM | Exo-atmospheric midcourse |
| `test_patriot.toml` | Patriot PAC-3 | 0.5-40km alt, ~70km range | 3 SRBM | Terminal phase, low altitude |
| `test_gbi.toml` | GBI | 200-2000km alt, ~2000km range | 2 ICBM | Long-range midcourse |
| `test_iron_dome.toml` | Iron Dome | 0.5-10km alt, ~70km range | 3 rockets | Very low altitude, rapid engagement |
| `test_arrow3.toml` | Arrow 3 | 20-400km alt, ~400km range | 2 MRBM, 1 IRBM | Exo-atmospheric |
| `test_davids_sling.toml` | David's Sling | 10-70km alt, ~300km range | 2 SRBM, 1 MRBM | Mid-altitude layer |
| `test_s400.toml` | S-400 | 0.01-400km alt, ~400km range | 4 missiles (low to very high) | Multi-layer capability |

## Sensor Tests (Radar Only - No Interceptors)

| Scenario | Radar System | Nominal Range | Max Range (1.5×) | Band | Threats | Purpose |
|----------|--------------|---------------|------------------|------|---------|---------|
| `test_sensor_sbx.toml` | Sea-Based X-band | 2000km | **3000km** | X-band | 4 ICBM at varying ranges | Midcourse discrimination |
| `test_sensor_cobra_dane.toml` | Cobra Dane (Shemya) | 3000km | **4500km** | L-band | 5 ICBM (800-3500km from radar) | Early warning, Pacific coverage |
| `test_sensor_tpy2.toml` | AN/TPY-2 (THAAD) | 1000km | **1500km** | X-band | SRBM/MRBM/IRBM/ICBM | Forward-based mode, high resolution |
| `test_sensor_spy1.toml` | AN/SPY-1 (AEGIS) | 400km | **600km** | S-band | SRBM/MRBM/IRBM at varying ranges | Volume search, AEGIS BMD |
| `test_sensor_fylingdales.toml` | Fylingdales BMEWS (UK) | 3000km | **4500km** | UHF | MRBM/IRBM/ICBM (1000-3500km) | European early warning |
| `test_sensor_pave_paws.toml` | PAVE PAWS (Beale AFB) | 3000km | **4500km** | UHF | SLBM/IRBM/ICBM (800-3500km) | Continental US early warning |
| `test_sensor_green_pine.toml` | Green Pine (Israel) | 800km | **1200km** | L-band | SRBM/MRBM/IRBM (250-1000km) | Arrow fire control |
| `test_sensor_don2n.toml` | Don-2N (Moscow) | 1000km | **1500km** | Phased-array | SRBM/MRBM/IRBM (300-1200km) | ABM battle management |
| `test_sensor_thule.toml` | Thule BMEWS (Greenland) | 3000km | **4500km** | UHF | SLBM/ICBM (1000-3500km) | Arctic early warning |

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
- **THAAD**: Should engage SRBMs/MRBMs with apogee 40-150km, refuse ICBMs at 400km
- **AEGIS**: Should engage MRBMs/IRBMs with apogee 70-500km
- **Patriot**: Should engage only terminal phase SRBMs below 40km altitude
- **GBI**: Should engage ICBMs at midcourse (200-2000km altitude)
- **Iron Dome**: Should engage very low altitude rockets (0.5-10km)
- **All Systems**: Should fire salvo_size=2 interceptors (SLS or SSL based on time)

### Sensor Tests:
- **Long-range radars (3000km)**: Should detect ICBMs at 1000-3000km, lose track beyond
- **Mid-range radars (1000-2000km)**: Should detect threats within range, degrade quality at edge
- **Short-range radars (400-800km)**: Should lose track of distant threats, strong quality nearby
- **All Systems**: Should require 3+ measurements before track establishment
- **All Systems**: Track quality should be >0.4 for engagement authorization

## Scenario Combinations

For advanced testing, combine scenarios:
- Load `test_sensor_sbx.toml` + add AEGIS units from `test_aegis.toml` for multi-sensor fusion
- Load `test_sensor_tpy2.toml` + add THAAD from `test_thaad.toml` for integrated sensor/shooter
- Verify multi-sensor fusion improves track quality (2+ sensors reduce uncertainty by 10-20%)
