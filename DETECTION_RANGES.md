# Detection Range System

## Overview

The simulation uses a **probabilistic detection model** where radar detection probability degrades with range rather than having a hard cutoff. This models real-world radar physics.

## Detection Range Rings

The detection range rings shown on the map represent the **maximum possible detection range** where probabilistic detection can occur.

### Range Calculation

**Displayed Range** = Nominal Range × 1.5

Examples:
- **SBX**: Nominal 2000km → Ring shows 3000km
- **Cobra Dane**: Nominal 3000km → Ring shows 4500km
- **AN/TPY-2**: Nominal 1000km → Ring shows 1500km
- **AN/SPY-1**: Nominal 400km → Ring shows 600km

## Why 1.5× Nominal Range?

Real-world radars don't have a hard detection cutoff. Instead, detection probability follows the **radar equation** which degrades with range (approximately R⁴ for two-way radar propagation).

The simulation models this by:
1. **Hard cutoff at 1.5× nominal**: No detection beyond this range
2. **Probabilistic detection within range**: Detection probability calculated using:
   - Range (inverse-square law)
   - Radar Cross Section (RCS) of target
   - Atmospheric attenuation
   - Altitude effects
   - Countermeasures (decoys, jamming)

## Detection Probability Curve

```
Detection Probability
     ^
100% |████████▄
     |         ▀▄
     |           ▀▄
     |             ▀▄
     |               ▀▄
     |                 ▀▄
  0% |___________________▀▄____
     0      Nominal    1.5× Nominal
```

- **0-80% of nominal**: Very high detection probability
- **80-100% of nominal**: High probability, begins to degrade
- **100-130% of nominal**: Moderate probability, significant degradation
- **130-150% of nominal**: Low probability, only strongest returns
- **Beyond 150%**: No detection possible

## Sensor-Specific Examples

### Long-Range Early Warning (3000km nominal)

**PAVE PAWS, Cobra Dane, Fylingdales, Thule**
- Ring shows: 4500km
- High confidence detection: 0-2500km
- Degraded detection: 2500-3500km
- Low probability detection: 3500-4500km
- No detection: >4500km

### Midcourse Discrimination (1000-2000km nominal)

**SBX (2000km nominal)**
- Ring shows: 3000km
- High confidence: 0-1600km
- Degraded: 1600-2400km
- Low probability: 2400-3000km
- No detection: >3000km

**AN/TPY-2 (1000km nominal)**
- Ring shows: 1500km
- High confidence: 0-800km
- Degraded: 800-1200km
- Low probability: 1200-1500km
- No detection: >1500km

### Fire Control Radars (400-800km nominal)

**AN/SPY-1 AEGIS (400km nominal)**
- Ring shows: 600km
- High confidence: 0-320km
- Degraded: 320-480km
- Low probability: 480-600km
- No detection: >600km

**Green Pine (800km nominal)**
- Ring shows: 1200km
- High confidence: 0-640km
- Degraded: 640-960km
- Low probability: 960-1200km
- No detection: >1200km

## Factors Affecting Detection

### 1. Range (Primary Factor)
Detection probability degrades with the 4th power of range (R⁴ in radar equation).

### 2. Radar Cross Section (RCS)
- Large RCS (10+ dBsm): Easier to detect at max range
- Medium RCS (0-10 dBsm): Nominal detection
- Small RCS (<0 dBsm): May not detect at max range

### 3. Altitude
- Higher altitude: Less atmospheric attenuation, easier to detect
- Lower altitude: More attenuation, harder to detect
- Horizon limiting: Ground radars cannot see below geometric horizon

### 4. Countermeasures
- **Decoys**: Each decoy reduces detection probability by 30%
- **6 decoys deployed**: Detection probability reduced by ~88%
- **Jamming**: Simulated through decoy effects

### 5. Azimuth/Elevation Coverage
- Target must be within radar's azimuth coverage (e.g., 120° for TPY-2)
- Target must be within elevation limits (e.g., 0-90° for most radars)
- Below horizon = no detection regardless of range

## Code Implementation

### Detection Range Check

```rust
// Early exit if way out of range
if range_km > detection_range * 1.5 {
    return None;
}
```

Located in `src/simulation/detection.rs:551`

### Detection Probability Calculation

Uses the `calculate_detection_probability()` function which models:
- Inverse-square law with range
- RCS scaling
- Atmospheric attenuation
- Altitude effects

### Rendering

Detection range rings are rendered at **1.5× nominal range**:

```rust
// Show actual max detection range (1.5× nominal for probabilistic detection)
let max_range_km = station.detection_range_km * 1.5;
```

## Testing Detection Ranges

Use the sensor test scenarios to verify detection behavior:

### Test Methodology

1. **Load sensor test scenario** (e.g., `test_sensor_sbx.toml`)
2. **Enable detection ranges** (View → Show Detection Ranges)
3. **Run simulation**
4. **Observe**:
   - Missiles within inner 2/3 of ring: High confidence detection (green tracking lines)
   - Missiles in middle zone: Degraded quality (yellow tracking lines)
   - Missiles near ring edge: Low quality or intermittent detection (red tracking lines)
   - Missiles beyond ring: No detection at all

### Expected Behavior

**SBX Test Scenario**:
- ICBM Near (1000km): High confidence, quality ~0.8-0.9
- ICBM Mid (1500km): Good confidence, quality ~0.6-0.7
- ICBM Far (2000km): Degraded, quality ~0.4-0.5
- ICBM Limit (2500km): Low confidence, quality ~0.2-0.3
- ICBM Beyond (>3000km): No detection

## Visual Indicators

- **Detection range ring**: Maximum possible detection range (1.5× nominal)
- **Tracking lines**: Sensor to target, color indicates quality:
  - Bright/thick: High quality (>0.7)
  - Medium: Good quality (0.4-0.7)
  - Dim/thin: Poor quality (<0.4)
- **Missile transparency**: Reflects track quality in Detected Track mode

## Summary

**Key Takeaway**: The detection range rings now accurately show the maximum range at which probabilistic detection can occur (1.5× nominal range). Detection probability degrades significantly with range, especially beyond the nominal range.
