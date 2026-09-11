# AGENTS.md

## Project Overview
Ballistic missile defense simulation in Rust + egui, targeting macOS Apple Silicon. Models realistic trajectories, sensor limitations, and multi-layer defense with sensor-based (not omniscient) fire control.

## Key Commands
- `cargo build` / `cargo run` - dev build/run
- `cargo build --release` - production build
- `cargo test` - run tests
- `cargo test test_track_velocity_estimation` - run a single test by name
- `cargo fmt` - format (run before commit)
- `cargo clippy` - lint
- `cargo check` - quick compile check

## Testing Gotchas
- Running tests: `tests/track_prediction_test.rs` (3 tests), `tests/impact_prediction_test.rs` (2 tests), `tests/interceptor_engagement_test.rs` (4 tests), `tests/scenario_builder_test.rs` (5 tests: TOML round-trip on all scenarios, Option-field handling, draft→engine chain, import flow), `tests/alert_assessment_test.rs` (3 tests: DEFCON escalation from real fused tracks, klaxon gate, quiet engine), `tests/measurement_error_test.rs` (2 tests: azimuth bias displaces fused tracks, zero-bias noise stays bounded) — plus `#[cfg(test)]` unit tests in `src/scenario/builder.rs` (7 tests, run in both lib and bin targets), `src/simulation/physics.rs` (8 WGS-84 geodesy tests), `src/simulation/alert.rs` (11), `src/tracking/mod.rs` (5), `src/audio/mod.rs` (3).
- **Engine tests must seed the detection RNG** (`engine.detection.seed_rng(42);` right after `SimulationEngine::new()`) or they will be nondeterministic: detection rolls, false alarms, and measurement noise all draw from it. Track-prediction max-error tolerance is 5x avg (not 3x) because measurements carry realistic Gaussian noise (Phase 4).
- `src/tests/` (impact_test.rs, trajectory_test.rs) is **orphaned** — declared in no module, never compiled. Edits there have no effect; put new tests in `tests/` or `#[cfg(test)]` modules.
- `test_ekf_convergence` **fails at HEAD** (uncertainty diverges instead of converging). Don't assume your change broke it — verify with `git stash` before chasing it.
- `test_fire_control.sh` / `test_salvo_fire.sh` are manual smoke scripts (launch GUI, grep logs; salvo script requires selecting the Middle East scenario by hand). Not CI tests.
- To verify behavior changes end-to-end: run the app and select a `scenarios/test_*.toml` platform/sensor test scenario (see `scenarios/README.md`).

## Fire Control Architecture (post-interceptor-overhaul)
- ALL engagement decisions are **sensor-derived only**: launch solutions, mid-course guidance, SLS/SSL doctrine timing, Pk factors, and terminal lead all project the target through `DetectionSystem::get_converged_trajectory()` (quadratic-fit trajectory estimate from radar measurements). There is deliberately NO ground-truth fallback — if the sensor chain can't produce a solution, the launch is refused. Don't "fix" failed engagements by reading `self.trajectories` or missile truth fields in engine decision paths.
- Interceptor arrival-time math has a single source of truth: `InterceptorKinematics::time_to_cover_distance()` (boost-aware, drag-aware). Launch, guidance, and intercept-time recalcs all must call it — do not reintroduce ad-hoc `dist/max_velocity` estimates.
- Interceptor drag is stateful/cumulative in `Interceptor::update_kinematics(dt)`; `distance_traveled_km` is the actual integrated path length.
- CPA is tracked ONLY by the guidance-side hysteresis tracker (`passed_cpa`); `resolve_intercepts` consumes the flag, never re-derives it.
- `SimulationEngine.max_shots_per_target` (default 4) caps total shots per target; `salvo_size` is per-salvo only. Crossing-geometry targets legitimately miss (gimbal-limited terminal homing) — that's realistic, not a bug.

## Architecture
- Entry: `src/main.rs` → `src/app.rs` (implements `eframe::App`) → `src/simulation/runner.rs` (sim/render thread split via crossbeam channels; rayon parallelism inside engine.rs).
- Simulation engine: `src/simulation/` (engine.rs, entities.rs, detection.rs, physics.rs, ekf.rs, kalman.rs, config.rs).
- `src/lib.rs` exposes `types`, `simulation`, and `scenario` for the integration tests.
- UI: `src/ui/` (scenario builder UI); rendering: `src/rendering/`; map tiles: `src/map/`; projections: `src/view/`; scenario TOML loading: `src/scenario/`.
- Config registry: `src/simulation/config.rs` loads TOML from `config/`.
- **Scenario builder**: pure logic (draft model, validation, save) in `src/scenario/builder.rs` (lib target, unit-tested); egui UI in `src/ui/scenario_builder.rs` (bin target). `App` owns it as an `Option<ScenarioBuilder>` — `Some` = builder mode. Drafts serialize through the same `ScenarioFile` structs the loader uses (`tests/scenario_builder_test.rs` proves round-trip on every existing scenario). Validation errors block Save/Test Run by design — don't bypass `to_toml()`'s gate.
- Note: the bin target declares its own module tree (`main.rs`); `crate::` paths in `src/ui/` and `src/scenario/` resolve against the bin's modules, while integration tests use the lib's identical tree. Keep both trees' `pub mod`/`pub use` in sync when adding modules.

## Domain Docs (read before touching subsystems)
- Radar/detection/tracking: `docs/radar-detection-tracking.md`
- Intercept kinematics/firing logic: `docs/platform-intercept-geometry.md`
- Physics modeling: `docs/physics.md`
- `docs/audit-plan.md` tracks implementation progress vs. domain requirements; consult it for what's done vs. pending before large refactors.

## Configuration
- Simulation-wide params: `config/simulation.toml` (physics sub-stepping, Pk weights)
- System specs: `config/sensors/`, `config/interceptors/`, `config/satellites/`, `config/missiles/`
- `config/platform/` is **legacy** — use sensor/interceptor dirs for new work
- **Config files are authoritative**: they encode real-world published specs (ranges, altitudes, interceptor counts). Do not modify equipment specs without asking first.
- Scenarios: TOML in `scenarios/`; defense unit `type` strings are case-sensitive (`THAAD`, `Aegis`, `Patriot`, `GBI`, `IronDome`, `Arrow3`, `DavidsSling`, `S400`).

## Environment
- Rust 1.70+, Edition 2021.
- `.env` in repo root with `MAPTILER_API_KEY=...` (gitignored). Loaded via dotenvy at startup (`src/main.rs`); without a key the app still runs, just without map tiles.
- `.cargo/config.toml` pins target `aarch64-apple-darwin` with `target-cpu=native` — builds only work as-is on Apple Silicon.

## Realism Requirements
- Real-world physics constants (9.80665 m/s² gravity); never simplify physics for performance unless asked.
- Defense uses only sensor-detected data, not ground truth: track establishment requires 3+ measurements, quality ≥ 0.4, freshness < 5s. Undetected = unengaged.
- Engagement envelopes strictly enforced (e.g., THAAD cannot engage ICBM apogees above its 150km ceiling).
- Defense capabilities must match real-world published data; cite the real-world basis for models/formulas in comments.

## Git Conventions
- Branches: `develop` is the dev mainline; work on `feature/*` branches; PRs merge to `master`.
- Commit subjects: plain imperative sentences (e.g., "Fix EKF velocity extraction and add ground truth fallback") — no `feat:`/`fix:` prefixes despite what CLAUDE.md says.
- Before committing: `cargo fmt`, then `cargo clippy`, then `cargo test` (see Testing Gotchas for expected failures).