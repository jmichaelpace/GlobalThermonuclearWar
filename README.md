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

## Installation

### Install Rust

If you don't have Rust installed, install it using rustup:

```bash
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
```

Follow the on-screen instructions, then restart your terminal or run:

```bash
source $HOME/.cargo/env
```

Verify the installation:

```bash
rustc --version
cargo --version
```

### Clone the Repository

```bash
git clone https://github.com/jmichaelpace/GlobalThermonuclearWar.git
cd GlobalThermonuclearWar
```

### Install Project Dependencies

Cargo will automatically download and compile dependencies on first build:

```bash
cargo build
```

### Configure MapTiler API Key

The application uses MapTiler for map tiles. You'll need a free API key:

1. **Create a MapTiler account:**
   - Go to [https://www.maptiler.com/](https://www.maptiler.com/)
   - Click "Sign Up" and create a free account

2. **Get your API key:**
   - After signing in, go to [https://cloud.maptiler.com/account/keys/](https://cloud.maptiler.com/account/keys/)
   - Copy your API key (the free tier includes 100,000 requests/month)

3. **Create a `.env` file** in the project root directory:

   ```bash
   echo 'MAPTILER_API_KEY=your_api_key_here' > .env
   ```

   Or manually create a file named `.env` with the following content:

   ```
   MAPTILER_API_KEY=your_api_key_here
   ```

   Replace `your_api_key_here` with your actual MapTiler API key.

> **Note:** The `.env` file is excluded from git via `.gitignore` to keep your API key private.

## Building

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
