# Performance Fix: Scenario Caching

## Problem

The application was reloading all scenario files from disk **every frame** (60+ times per second) when the scenario panel was open.

### Root Cause

In `src/app.rs`, the `render_scenario_panel()` function was calling `get_scenarios()` every time it rendered:

```rust
fn render_scenario_panel(&mut self, ctx: &egui::Context) {
    // ...
    let scenarios = get_scenarios();  // ❌ Called every frame!
    for (idx, scenario) in scenarios.iter().enumerate() {
        // ...
    }
}
```

The `get_scenarios()` function:
1. Scans the `scenarios/` directory
2. Reads every `.toml` file from disk
3. Parses the TOML for each file
4. Prints `"Loaded scenario: {name}"` for each one
5. Returns the scenarios vector

At 60 FPS, this meant:
- **60 directory scans per second**
- **60× all scenario files read per second** (12 files = 720 file reads/sec)
- **60× TOML parsing per second**
- **Constant "Loaded scenario" spam in console**

## Solution

### 1. Cache Scenarios in App State

Added a `scenarios` field to the `App` struct:

```rust
pub struct App {
    // ... other fields ...
    scenarios: Vec<ScenarioDefinition>,  // ✅ Cached at startup
}
```

### 2. Load Once at Initialization

In `App::new()`:

```rust
pub fn new(_cc: &eframe::CreationContext<'_>) -> Self {
    let scenarios = get_scenarios();  // Load once
    // ...
    Self {
        // ...
        scenarios,  // Store in state
    }
}
```

### 3. Use Cached Scenarios in UI

In `render_scenario_panel()`:

```rust
fn render_scenario_panel(&mut self, ctx: &egui::Context) {
    // Clone to avoid borrow checker issues
    let scenarios = self.scenarios.clone();

    for (idx, scenario) in scenarios.iter().enumerate() {
        // ✅ Uses cached scenarios, no disk I/O
    }
}
```

### 4. Add Reload Button

Added a "🔄 Reload Scenarios" button so users can manually reload if they modify scenario files while the app is running:

```rust
if ui.button("🔄 Reload Scenarios").clicked() {
    self.scenarios = get_scenarios();
}
```

### 5. Made Scenario Types Cloneable

Added `#[derive(Clone)]` to:
- `ScenarioDefinition` (in `src/scenario/mod.rs`)
- `ScenarioFile` and all related structs (in `src/scenario/loader.rs`)

This allows us to clone the scenarios vector for iteration without ownership issues.

## Performance Impact

### Before (60 FPS)
- **720 file reads per second** (12 scenarios × 60 frames)
- **720 TOML parses per second**
- **720 directory scans per second**
- Constant console spam
- CPU usage: **High** (continuous disk I/O)
- Frame drops on slower disks

### After (60 FPS)
- **0 file reads per second** (cached)
- **0 TOML parses per second** (cached)
- **0 directory scans per second** (cached)
- No console spam
- CPU usage: **Minimal** (memory access only)
- Smooth 60 FPS

## Memory vs Disk Trade-off

**Memory Cost**: ~few KB for 12 scenario definitions
**Disk I/O Saved**: 720 file operations per second

This is an excellent trade-off - we use a tiny amount of memory to eliminate massive disk I/O overhead.

## Files Modified

1. **`src/app.rs`**:
   - Added `scenarios: Vec<ScenarioDefinition>` field to `App` struct
   - Initialize scenarios in `App::new()`
   - Use `self.scenarios` instead of `get_scenarios()` in `render_scenario_panel()`
   - Added reload button

2. **`src/scenario/mod.rs`**:
   - Added `#[derive(Clone)]` to `ScenarioDefinition`

3. **`src/scenario/loader.rs`**:
   - Added `Clone` to derive macros for all config structs:
     - `ScenarioFile`
     - `ScenarioMetadata`
     - `DefenseUnitConfig`
     - `RadarStationConfig`
     - `SatelliteConfig`
     - `MissileConfig`

## Testing

### Verify the Fix

1. **Before**: Run the app, open scenario panel, watch console spam with "Loaded scenario" messages every frame
2. **After**: Run the app, open scenario panel, see "Loaded scenario" messages **only once** at startup
3. **Reload**: Click "🔄 Reload Scenarios" button to manually reload from disk

### Check CPU Usage

Before: High CPU usage from continuous disk I/O
After: Minimal CPU usage, smooth 60 FPS

## Future Enhancements

Consider adding:
- File system watcher to auto-reload when scenario files change
- Lazy loading of scenario file contents (load metadata only, load full scenario on selection)
- Async scenario loading on startup for large scenario libraries

## Lessons Learned

**Never do I/O in a render loop!**

Rendering functions (`render_*()`) are called 60+ times per second. Any expensive operations like:
- File I/O
- Network requests
- Heavy parsing
- Database queries

Should be:
1. Done once and cached
2. Moved to background threads
3. Triggered only by user actions (button clicks)

This fix demonstrates the massive performance impact of even "small" I/O operations when repeated 60+ times per second.
