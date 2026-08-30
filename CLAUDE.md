# Global Thermonuclear War

## Project Overview
A standalone desktop application built with Rust and egui, targeting Apple Silicon.

## Technology Stack
- **Language**: Rust (Edition 2021)
- **UI Framework**: egui via eframe
- **Target Platform**: macOS (aarch64-apple-darwin)

## Build and Development

### Install Dependencies
```bash
cargo build
```

### Run Development Build
```bash
cargo run
```

### Build for Production
```bash
cargo build --release
```

### Run Tests
```bash
cargo test
```

### Check for Errors Without Building
```bash
cargo check
```

### Format Code
```bash
cargo fmt
```

### Run Linter
```bash
cargo clippy
```

## Code Style

### Naming Conventions
- Use snake_case for functions, variables, and modules
- Use PascalCase for types, traits, and enum variants
- Use SCREAMING_SNAKE_CASE for constants

### Formatting
- Run `cargo fmt` before committing
- Follow Rust standard style guidelines
- Maximum line length: 100 characters

### Error Handling
- Use `Result<T, E>` for recoverable errors
- Use `.expect()` with descriptive messages for unrecoverable errors
- Avoid `.unwrap()` in production code

## Project Architecture

### Directory Structure
```
src/
├── main.rs          # Application entry point and App struct
└── [future modules]
```

### egui Patterns
- Implement `eframe::App` trait for main application
- Use `egui::CentralPanel` for main content area
- Use `ui.horizontal()` and `ui.vertical()` for layouts

## Git Workflow

### Commit Messages
- Use imperative mood: "Add feature" not "Added feature"
- Prefix with type: feat:, fix:, refactor:, docs:, test:

### Before Committing
- Run `cargo fmt` to format code
- Run `cargo clippy` to check for warnings
- Run `cargo test` to ensure tests pass
- Run `cargo build` to verify compilation

## Dependencies
- See Cargo.toml for complete dependency list
- Prefer crates from the egui ecosystem when possible
- Check crates.io for compatibility before adding dependencies

## Realism Requirements
- Use real-world physics constants (e.g., standard gravity 9.80665 m/s²)
- Prefer established defense/aerospace models (e.g., Kepler, Lambert, Kalman filter)
- Never simplify physics for performance unless explicitly asked
- Cite the real-world basis for any model or formula used in comments

### Real-World Fidelity
This simulation models real-world missile defense systems and must maintain accuracy
- **Preserve equipment specifications**: Detection ranges, engagement ranges, interceptor counts, and sensor capabilities must match real-world published data.  Never modify configuration ranges or specifications in the /config files without asking first
- **Config files are authoritative**: Equipment specs in `config/` TOML files represent researched real-world values.
- **Defense system capabilities**: See configuration files in `config/platform/` for authoritative engagement ranges, altitude envelopes, and interceptor specifications for each defense system (Patriot, THAAD, Aegis, GBI, Iron Dome, Arrow 3, David's Sling, S-400)

### When Modifying Simulation Code
1. Do not modify configuration items (Equipment specs) that have a TOML comment on the same line after the config setting
2. Verify changes don't break detection/engagement range accuracy
3. Test that defense systems engage appropriate threat types
4. Ensure interceptors launch when threats are in engagement envelope
5. Run the simulation with each scenario to verify expected behaviors
6. Do not modify existing equipment files without asking first

## Domain References
When working on specific subsystems, read these files first:
- Radar, detection and/or tracking: @docs/radar-detection-tracking.md
- Intercept kinematics and/or platform firing logic: @docs/platform-intercept-geometry.md
- physics modeling: @docs/physics.md

## Context Compaction Behavior
The audit plan at `docs/audit-plan.md` tracks implementation progress against domain requirements.

**Before context compaction**: Update the plan file to mark completed items and note current progress.

**After context compaction**: Read `docs/audit-plan.md` to restore context on what has been done and what remains.