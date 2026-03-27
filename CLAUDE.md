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
