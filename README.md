# Global Thermonuclear War

A ballistic missile defense simulation built with Rust and egui. Visualize missile launches, tracking, and intercept scenarios with realistic flight physics and defense system modeling.

## Features

- **Realistic Missile Simulation**: ICBM, IRBM, MRBM, SLBM, and SRBM with configurable trajectories, flight times, and countermeasures
- **Multi-layered Defense Systems**:
  - US: GBI, AEGIS SM-3, THAAD, Patriot PAC-3
  - Israeli: Arrow 3, David's Sling, Iron Dome
  - Russian: S-400
- **Separate Sensor and Interceptor Modeling**: Radars and interceptors configured independently for realistic system composition
- **Early Warning Satellites**: SBIRS, DSP, Tundra, and other space-based detection systems
- **TOML-based Configuration**: All system parameters externalized for easy modification
- **Scenario System**: Pre-built scenarios for different regions and threat environments

## Requirements

- Rust 1.70+ (Edition 2021)
- macOS (currently targeting Apple Silicon)

## Building

### Install Dependencies

```bash
cargo build
```

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

All simulation parameters are defined in TOML files under `config/`:

```
config/
├── defense/           # Combined defense system configs (legacy)
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

[tracking]
max_simultaneous_tracks = 50
track_update_rate_hz = 20.0
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
