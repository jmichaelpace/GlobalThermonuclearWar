# Visual Changes: Track Quality and Fire Control Indicators

## Overview

Missile icons now provide visual feedback about tracking confidence and fire control radar lock status through dynamic transparency and color changes.

## Changes Implemented

### 1. Track Quality-Based Transparency

**What**: Missile transparency varies based on how precise the sensor track is.

**How it works**:
- **High quality track (quality > 0.7)**: Nearly opaque (alpha ~230-255)
- **Medium quality track (quality 0.4-0.7)**: Semi-transparent (alpha ~130-190)
- **Low quality track (quality < 0.4)**: Very transparent (alpha ~50-130)

**Formula**: `alpha = quality * 205 + 50` (maps 0.0-1.0 to 50-255)

**When visible**: Only in "Detected Track" view mode (not "True Track" mode)

### 2. Fire Control Radar Lock Indicator

**What**: Missile color changes from light red to dark red when under active fire control radar lock.

**Trigger conditions** (all must be met):
- ✅ Track has 3+ sensor measurements
- ✅ Track quality ≥ 0.4
- ✅ Track staleness ≤ 5 seconds
- ✅ Missile affiliation is Hostile

**Visual changes when locked**:
- **Missile body**: Changes from bright red `RGB(255, 50, 50)` to dark maroon `RGB(120, 0, 0)`
- **Missile outline**: Changes to dark maroon `RGB(100, 0, 0)`
- **Targeting ring**: Bright orange double ring appears around the missile
  - Inner ring: `RGB(255, 100, 0)` at 2.0× size, 2.5px stroke
  - Outer ring: `RGB(255, 100, 0, 120)` at 2.3× size, 1.5px stroke (glowing effect)

**When visible**: Only in "Detected Track" view mode

## Technical Details

### Modified Files

1. **`src/rendering/symbols.rs`**
   - Updated `draw_missile()` function signature
   - Added parameters: `track_quality: Option<f64>`, `fire_control_locked: bool`
   - Applied transparency scaling based on track quality
   - Applied color change for fire control lock

2. **`src/app.rs`**
   - Updated all 4 calls to `draw_missile()`
   - Added fire control lock detection logic:
     ```rust
     let fire_control_locked = fused_track.measurement_count >= 3
         && fused_track.fused_quality >= 0.4
         && fused_track.staleness_seconds <= 5.0;
     ```
   - Passed `None` for track_quality in TrueTrack mode (no transparency changes)
   - Passed `Some(fused_track.fused_quality)` in DetectedTrack mode

### Rendering Modes

#### True Track Mode
- Shows actual missile positions (ground truth)
- **No transparency changes** (track_quality = None)
- **No fire control color changes** (fire_control_locked = false)
- Always fully opaque with standard colors

#### Detected Track Mode
- Shows sensor-perceived missile positions
- **Transparency varies** based on `fused_track.fused_quality`
- **Color changes** when fire control requirements met
- Represents what defense systems actually see

## Visual Interpretation Guide

### Missile Appearance Examples

| Track Quality | Measurements | Appearance | Meaning |
|--------------|--------------|------------|---------|
| 0.2 | 1 | Bright red, very transparent | Poor track, no fire control |
| 0.5 | 2 | Bright red, semi-transparent | Decent track, not locked yet |
| 0.6 | 3 | **Dark maroon + orange ring, semi-transparent** | **Fire control locked** |
| 0.9 | 5 | **Dark maroon + orange ring, nearly opaque** | **Strong fire control lock** |

### What This Tells You

**Transparent missiles**:
- Sensor track is uncertain
- Position could be off by tens of kilometers
- Cannot reliably engage

**Opaque missiles**:
- High confidence sensor track
- Position accurate to single-digit kilometers
- Good engagement potential

**Dark maroon missiles with bright orange ring**:
- **Fire control radar has locked on**
- Track establishment complete (3+ measurements)
- Ready for intercept authorization
- Defense systems can engage
- Orange targeting ring makes them immediately identifiable

**Bright red missiles (no ring)**:
- Detected but not locked
- Insufficient measurements or quality
- Cannot engage yet

## Realism Benefits

1. **Situational Awareness**: Instantly see which threats are well-tracked vs uncertain
2. **Fire Control Status**: Know which missiles are under precision tracking lock
3. **Engagement Readiness**: Dark red = ready to engage, light red = still building track
4. **Track Quality**: Transparency shows confidence level without cluttering the display

## Testing

To observe these visual changes:

1. **Load a sensor test scenario**: e.g., `test_sensor_tpy2.toml`
2. **Switch to "Detected Track" mode**: View → Track View Mode → Detected Track
3. **Run simulation**: Observe missiles as they are detected:
   - Initially very transparent (low quality, few measurements)
   - Gradually become more opaque (quality improves)
   - **Turn dark red** when fire control lock achieved (3+ measurements, quality ≥ 0.4)
4. **Compare to True Track mode**: Switch back to see the difference

## Code Example

```rust
// Fire control lock detection
let fire_control_locked = fused_track.measurement_count >= 3
    && fused_track.fused_quality >= 0.4
    && fused_track.staleness_seconds <= 5.0;

// Draw missile with transparency and color based on track state
MilitarySymbols::draw_missile(
    painter,
    pos,
    Affiliation::Hostile,
    MissileStatus::Midcourse,
    heading,
    10.0,
    Some(fused_track.fused_quality),  // Transparency based on quality
    fire_control_locked,               // Dark red if locked
);
```

## Future Enhancements

Potential additions:
- Pulsing effect for fire control locked targets
- Different colors for different lock qualities
- Visual indicator showing number of measurements contributing to track
- Track quality trend indicator (improving/degrading)
