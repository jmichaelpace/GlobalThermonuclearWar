use crate::effects::{EffectsManager, EffectType};
use crate::map::{GeoCoord, GibsTileCache, GibsTileCoord, MapStyle, TileCache, Viewport};
use crate::rendering::{colors, DetectionOverlays, MilitarySymbols};
use crate::tracking::{EventTracker, EventType, format_sim_time};
use crate::scenario::{get_scenarios, ScenarioDefinition};
use crate::simulation::{
    bearing, calculate_position_from_bearing_range, Affiliation,
    BallisticTrajectory, DefenseUnit, EntityId, FusedTrack, Interceptor, Missile, MissileStatus,
    RadarStation, Satellite, SensorType, SimulationEngine, TimeScale,
};
use crate::simulation::config::{RadarMode, RadarType};
use eframe::egui;
use std::time::Instant;

/// Represents a selected entity
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Selection {
    Missile(EntityId),
    DefenseUnit(EntityId),
    Satellite(EntityId),
    RadarStation(EntityId),
}

/// View mode for rendering
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ViewMode {
    Map2D,       // Traditional flat map (Mercator)
    Globe,       // 3D globe (orthographic projection)
    Intercept3D, // Isometric 3D intercept view
}

/// Track visualization mode
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TrackViewMode {
    /// Show actual missile positions (omniscient view)
    TrueTrack,
    /// Show positions as sensors perceive them (with uncertainty)
    DetectedTrack,
}

/// Globe view state
#[allow(dead_code)]
pub struct GlobeState {
    /// Center latitude of view (degrees)
    pub center_lat: f64,
    /// Center longitude of view (degrees)
    pub center_lon: f64,
    /// Globe radius in screen pixels
    pub radius: f32,
    /// Rotation velocity for smooth rotation
    pub rotation_velocity: (f64, f64),
    /// Is user currently dragging
    pub dragging: bool,
    /// Last drag position
    pub last_drag_pos: Option<egui::Pos2>,
}

impl Default for GlobeState {
    fn default() -> Self {
        Self {
            center_lat: 30.0,
            center_lon: 45.0,  // Default to Middle East view
            radius: 300.0,
            rotation_velocity: (0.0, 0.0),
            dragging: false,
            last_drag_pos: None,
        }
    }
}

impl GlobeState {
    /// Project a geographic coordinate to screen position (orthographic projection)
    /// Returns None if the point is on the back side of the globe
    pub fn geo_to_screen(&self, coord: GeoCoord, screen_center: egui::Pos2) -> Option<egui::Pos2> {
        let lat = coord.lat.to_radians();
        let lon = coord.lon.to_radians();
        let center_lat = self.center_lat.to_radians();
        let center_lon = self.center_lon.to_radians();

        // Orthographic projection formulas
        let cos_c = center_lat.sin() * lat.sin() + center_lat.cos() * lat.cos() * (lon - center_lon).cos();

        // Point is on the back of the globe if cos_c < 0
        if cos_c < 0.0 {
            return None;
        }

        let x = lat.cos() * (lon - center_lon).sin();
        let y = center_lat.cos() * lat.sin() - center_lat.sin() * lat.cos() * (lon - center_lon).cos();

        Some(egui::Pos2::new(
            screen_center.x + (x * self.radius as f64) as f32,
            screen_center.y - (y * self.radius as f64) as f32,  // Flip Y for screen coords
        ))
    }

    /// Check if a geographic point is visible (on the front of the globe)
    pub fn is_visible(&self, coord: GeoCoord) -> bool {
        let lat = coord.lat.to_radians();
        let lon = coord.lon.to_radians();
        let center_lat = self.center_lat.to_radians();
        let center_lon = self.center_lon.to_radians();

        // cos_c determines if point is on front (positive) or back (negative) of globe
        let cos_c = center_lat.sin() * lat.sin() + center_lat.cos() * lat.cos() * (lon - center_lon).cos();
        cos_c >= 0.0
    }

    /// Get the center of the globe view as a GeoCoord
    pub fn center(&self) -> GeoCoord {
        GeoCoord::new(self.center_lat, self.center_lon)
    }

    /// Convert screen position to geographic coordinates
    /// Returns None if outside the globe
    pub fn screen_to_geo(&self, screen_pos: egui::Pos2, screen_center: egui::Pos2) -> Option<GeoCoord> {
        let dx = (screen_pos.x - screen_center.x) as f64 / self.radius as f64;
        let dy = -(screen_pos.y - screen_center.y) as f64 / self.radius as f64;  // Flip Y

        let rho = (dx * dx + dy * dy).sqrt();

        // Outside the globe
        if rho > 1.0 {
            return None;
        }

        let c = rho.asin();
        let center_lat = self.center_lat.to_radians();
        let center_lon = self.center_lon.to_radians();

        let lat = if rho == 0.0 {
            center_lat
        } else {
            (c.cos() * center_lat.sin() + dy * c.sin() * center_lat.cos() / rho).asin()
        };

        let lon = if center_lat.cos().abs() < 1e-10 {
            center_lon + (dx / -dy * center_lat.signum()).atan()
        } else {
            center_lon + (dx * c.sin() / (rho * center_lat.cos() * c.cos() - dy * center_lat.sin() * c.sin())).atan()
        };

        Some(GeoCoord::new(lat.to_degrees(), lon.to_degrees()))
    }

    /// Rotate the globe based on drag delta
    pub fn rotate(&mut self, delta: egui::Vec2) {
        let sensitivity = 0.3;
        self.center_lon -= (delta.x as f64) * sensitivity;
        self.center_lat += (delta.y as f64) * sensitivity;

        // Clamp latitude
        self.center_lat = self.center_lat.clamp(-89.0, 89.0);

        // Wrap longitude
        while self.center_lon > 180.0 {
            self.center_lon -= 360.0;
        }
        while self.center_lon < -180.0 {
            self.center_lon += 360.0;
        }
    }

    /// Apply inertial rotation (for smooth rotation after drag release)
    pub fn apply_inertia(&mut self, dt: f64) {
        if !self.dragging && (self.rotation_velocity.0.abs() > 0.1 || self.rotation_velocity.1.abs() > 0.1) {
            self.center_lon += self.rotation_velocity.0 * dt;
            self.center_lat += self.rotation_velocity.1 * dt;

            // Clamp latitude
            self.center_lat = self.center_lat.clamp(-89.0, 89.0);

            // Wrap longitude
            while self.center_lon > 180.0 {
                self.center_lon -= 360.0;
            }
            while self.center_lon < -180.0 {
                self.center_lon += 360.0;
            }

            // Dampen velocity
            self.rotation_velocity.0 *= 0.95;
            self.rotation_velocity.1 *= 0.95;
        }
    }
}

/// Isometric 3D view state for intercept visualization
pub struct IsometricViewState {
    /// Horizontal orbit angle (degrees, 0 = North)
    pub azimuth_deg: f64,
    /// Vertical tilt angle (degrees, 0 = top-down, 90 = horizon)
    pub elevation_deg: f64,
    /// Zoom level (distance multiplier, 1.0 = default)
    pub zoom: f64,
    /// Is user currently dragging
    pub dragging: bool,
    /// Defense unit ID to focus on (if any)
    pub focus_unit_id: Option<EntityId>,
    /// Auto-show on engagement toggle
    pub auto_show_on_engagement: bool,
}

impl Default for IsometricViewState {
    fn default() -> Self {
        Self {
            azimuth_deg: 45.0,      // NE viewing angle
            elevation_deg: 30.0,    // Slight tilt
            zoom: 1.0,
            dragging: false,
            focus_unit_id: None,
            auto_show_on_engagement: false,
        }
    }
}

impl IsometricViewState {
    /// Project 3D local coordinates (km) to 2D screen position
    /// x = East, y = North, z = Up (altitude)
    pub fn project(&self, x: f64, y: f64, z: f64, center: egui::Pos2, scale: f64) -> egui::Pos2 {
        let az_rad = self.azimuth_deg.to_radians();
        let el_rad = self.elevation_deg.to_radians();

        // Rotate around vertical axis (azimuth)
        let rx = x * az_rad.cos() - y * az_rad.sin();
        let ry = x * az_rad.sin() + y * az_rad.cos();

        // Apply elevation tilt and zoom
        let screen_x = rx * scale * self.zoom;
        let screen_y = (-ry * el_rad.cos() - z * el_rad.sin()) * scale * self.zoom;

        egui::pos2(center.x + screen_x as f32, center.y + screen_y as f32)
    }

    /// Handle drag for orbit rotation
    pub fn handle_drag(&mut self, delta: egui::Vec2) {
        self.azimuth_deg = (self.azimuth_deg - delta.x as f64 * 0.5) % 360.0;
        if self.azimuth_deg < 0.0 {
            self.azimuth_deg += 360.0;
        }
        self.elevation_deg = (self.elevation_deg + delta.y as f64 * 0.3).clamp(10.0, 80.0);
    }

    /// Handle scroll for zoom
    pub fn handle_zoom(&mut self, delta: f32) {
        let factor = 1.0 + delta as f64 * 0.1;
        self.zoom = (self.zoom * factor).clamp(0.2, 5.0);
    }
}

/// View preset for quick navigation
pub struct ViewPreset {
    pub name: &'static str,
    pub center: GeoCoord,
    pub zoom: f64,
}

fn get_view_presets() -> Vec<ViewPreset> {
    vec![
        ViewPreset {
            name: "World",
            center: GeoCoord::new(20.0, 0.0),
            zoom: 2.0,
        },
        ViewPreset {
            name: "Pacific",
            center: GeoCoord::new(35.0, 180.0),
            zoom: 2.5,
        },
        ViewPreset {
            name: "North America",
            center: GeoCoord::new(40.0, -100.0),
            zoom: 3.5,
        },
        ViewPreset {
            name: "Europe",
            center: GeoCoord::new(50.0, 10.0),
            zoom: 4.0,
        },
        ViewPreset {
            name: "Middle East",
            center: GeoCoord::new(30.0, 45.0),
            zoom: 4.0,
        },
        ViewPreset {
            name: "East Asia",
            center: GeoCoord::new(35.0, 125.0),
            zoom: 4.0,
        },
        ViewPreset {
            name: "North Atlantic",
            center: GeoCoord::new(55.0, -30.0),
            zoom: 3.0,
        },
    ]
}

// Event types and EventLog moved to src/tracking/mod.rs

// Visual effects are now managed by EffectsManager in src/effects/mod.rs

pub struct App {
    viewport: Viewport,
    tile_cache: TileCache,
    simulation: SimulationEngine,
    last_update: Instant,
    show_debug_info: bool,
    show_detection_ranges: bool,
    show_trajectories: bool,
    show_tracking_lines: bool,
    show_scenario_panel: bool,
    show_radar_stats: bool,
    selection: Option<Selection>,
    current_scenario: usize,
    // Consolidated event tracking
    event_tracker: EventTracker,
    // Visual effects manager
    effects_manager: EffectsManager,
    // View mode (Map2D, Globe, or Intercept3D)
    view_mode: ViewMode,
    globe_state: GlobeState,
    // Isometric 3D intercept view state
    isometric_state: IsometricViewState,
    // NASA GIBS tile cache for globe view
    gibs_tile_cache: GibsTileCache,
    // Track view mode (True or Detected)
    track_view_mode: TrackViewMode,
    // Whether to show false alarms in detected mode
    show_false_alarms: bool,
    // Cached scenarios (loaded once at startup)
    scenarios: Vec<ScenarioDefinition>,
}

impl App {
    pub fn new(_cc: &eframe::CreationContext<'_>) -> Self {
        let api_key = std::env::var("MAPTILER_API_KEY")
            .unwrap_or_else(|_| "YOUR_API_KEY_HERE".to_string());

        let mut simulation = SimulationEngine::new();

        // Load the default demo scenario
        let scenarios = get_scenarios();
        if let Some(scenario) = scenarios.get(4) {
            // Demo scenario
            scenario.load(&mut simulation);
        }

        Self {
            viewport: Viewport::default(),
            tile_cache: TileCache::new(api_key),
            simulation,
            last_update: Instant::now(),
            show_debug_info: false,
            show_detection_ranges: true,
            show_trajectories: false,
            show_tracking_lines: true,
            show_scenario_panel: false,
            show_radar_stats: false,
            selection: None,
            current_scenario: 4, // Demo scenario
            event_tracker: EventTracker::new(),
            effects_manager: EffectsManager::new(),
            view_mode: ViewMode::Map2D,
            globe_state: GlobeState::default(),
            isometric_state: IsometricViewState::default(),
            gibs_tile_cache: GibsTileCache::new(),
            track_view_mode: TrackViewMode::DetectedTrack,
            show_false_alarms: true,
            scenarios, // Cache scenarios loaded at startup
        }
    }

    /// Check for state changes and generate events
    fn generate_events(&mut self) {
        let sim_time = self.simulation.sim_time;

        // Use EventTracker to check for events and get effect requests
        let effect_requests = self.event_tracker.check_for_events(
            sim_time,
            &self.simulation.missiles,
            &self.simulation.interceptors,
            &self.simulation.defense_units,
        );

        // Spawn requested visual effects
        for request in effect_requests {
            self.effects_manager.spawn(request.position, request.effect_type, sim_time);
        }
    }

    /// Clear event tracking state (called when loading new scenario)
    fn clear_event_tracking(&mut self) {
        self.event_tracker.clear();
        self.effects_manager.clear();
    }

    /// Update visual effects (remove finished ones)
    fn update_visual_effects(&mut self) {
        let sim_time = self.simulation.sim_time;
        self.effects_manager.update(sim_time);
    }

    /// Render visual effects (explosions, etc.)
    fn render_visual_effects(&self, painter: &egui::Painter, screen_rect: egui::Rect) {
        let sim_time = self.simulation.sim_time;

        for effect in self.effects_manager.effects() {
            let progress = effect.progress(sim_time);
            let positions = self.viewport.geo_to_screen_wrapped(effect.position, screen_rect);

            for pos in positions {
                match effect.effect_type {
                    EffectType::Impact => {
                        EffectsManager::render_impact_effect(painter, pos, progress);
                    }
                    EffectType::Intercept => {
                        EffectsManager::render_intercept_effect(painter, pos, progress);
                    }
                    EffectType::Debris => {
                        EffectsManager::render_debris_effect(painter, pos, progress);
                    }
                }
            }
        }
    }

    // Note: Effect rendering methods moved to EffectsManager in src/effects/mod.rs

    /// Render the event log
    fn render_event_log(&self, ui: &mut egui::Ui) {
        ui.heading("Event Log");
        ui.add_space(4.0);

        if self.event_tracker.event_log.events.is_empty() {
            ui.label(egui::RichText::new("No events yet").weak().italics());
            return;
        }

        egui::ScrollArea::vertical()
            .max_height(200.0)
            .stick_to_bottom(true)
            .show(ui, |ui| {
                for event in self.event_tracker.event_log.events.iter().rev().take(20) {
                    let time_str = format_sim_time(event.time);
                    let (icon, text, color) = match &event.event_type {
                        EventType::MissileLaunch { name } => (
                            "🚀",
                            format!("{} launched", name),
                            egui::Color32::from_rgb(255, 150, 150),
                        ),
                        EventType::ThreatDetected { threat_name, sensor_name } => (
                            "📡",
                            format!("{} detected by {}", threat_name, sensor_name),
                            egui::Color32::from_rgb(255, 200, 100),
                        ),
                        EventType::InterceptorLaunch { defense_unit, target } => (
                            "🎯",
                            format!("{} engaging {}", defense_unit, target),
                            egui::Color32::from_rgb(100, 200, 255),
                        ),
                        EventType::InterceptHit { target } => (
                            "✓",
                            format!("{} INTERCEPTED", target),
                            egui::Color32::from_rgb(100, 255, 100),
                        ),
                        EventType::InterceptMiss { target } => (
                            "✗",
                            format!("Missed {}", target),
                            egui::Color32::from_rgb(255, 150, 50),
                        ),
                        EventType::MissileImpact { name } => (
                            "💥",
                            format!("{} IMPACT!", name),
                            egui::Color32::from_rgb(255, 50, 50),
                        ),
                        EventType::DecoyDeployed { missile_name, decoys_active } => (
                            "🎭",
                            format!("{} deployed decoy ({})", missile_name, decoys_active),
                            egui::Color32::from_rgb(200, 150, 255),
                        ),
                        EventType::AllThreatsNeutralized => (
                            "🛡",
                            "All threats neutralized".to_string(),
                            egui::Color32::from_rgb(100, 255, 100),
                        ),
                    };

                    ui.horizontal(|ui| {
                        ui.label(egui::RichText::new(time_str).small().weak());
                        ui.label(icon);
                        ui.label(egui::RichText::new(text).small().color(color));
                    });
                }
            });
    }

    fn render_map(&mut self, ui: &mut egui::Ui) {
        match self.view_mode {
            ViewMode::Map2D => self.render_map_2d(ui),
            ViewMode::Globe => self.render_globe(ui),
            ViewMode::Intercept3D => self.render_intercept_3d_fullscreen(ui),
        }
    }

    fn render_map_2d(&mut self, ui: &mut egui::Ui) {
        let available_rect = ui.available_rect_before_wrap();

        // Process any pending tile loads
        let tiles_loaded = self.tile_cache.process_pending(ui.ctx());
        if tiles_loaded {
            ui.ctx().request_repaint();
        }

        // Handle input
        let response = ui.allocate_rect(available_rect, egui::Sense::click_and_drag());

        // Handle click for entity selection
        if response.clicked() {
            if let Some(pos) = response.interact_pointer_pos() {
                self.selection = self.find_entity_at(pos, available_rect);
            }
        }

        // Pan with drag
        if response.dragged() {
            self.viewport.pan(response.drag_delta(), available_rect);
            ui.ctx().request_repaint();
        }

        // Zoom with scroll
        let scroll_delta = ui.input(|i| i.raw_scroll_delta.y);
        if scroll_delta != 0.0 {
            if let Some(pointer_pos) = ui.input(|i| i.pointer.hover_pos()) {
                if available_rect.contains(pointer_pos) {
                    let zoom_delta = scroll_delta as f64 * 0.01;
                    self.viewport.zoom_at(zoom_delta, pointer_pos, available_rect);
                    ui.ctx().request_repaint();
                }
            }
        }

        // Get visible tiles
        let visible_tiles = self.viewport.get_visible_tiles(available_rect);

        // Render tiles
        let painter = ui.painter_at(available_rect);

        // Draw background (ocean color)
        painter.rect_filled(available_rect, 0.0, egui::Color32::from_rgb(20, 40, 60));

        // Draw tiles
        for visible_tile in &visible_tiles {
            let tile_rect = self.viewport.visible_tile_screen_rect(visible_tile, available_rect);

            if tile_rect.intersects(available_rect) {
                if let Some(texture) = self.tile_cache.get_tile(visible_tile.coord) {
                    painter.image(
                        texture.id(),
                        tile_rect,
                        egui::Rect::from_min_max(egui::pos2(0.0, 0.0), egui::pos2(1.0, 1.0)),
                        egui::Color32::WHITE,
                    );
                } else {
                    painter.rect_filled(tile_rect, 0.0, egui::Color32::from_rgb(30, 50, 70));
                }
            }
        }

        // Draw simulation entities
        self.render_entities(&painter, available_rect);

        // Show tooltip on hover
        if let Some(hover_pos) = response.hover_pos() {
            self.render_hover_tooltip(ui, hover_pos, available_rect);
        }

        // Request repaint if simulation is running or any tiles are loading
        if self.simulation.time_scale != TimeScale::Paused
            || self.tile_cache.has_loading_tiles()
        {
            ui.ctx().request_repaint();
        }
    }

    fn render_globe(&mut self, ui: &mut egui::Ui) {
        let available_rect = ui.available_rect_before_wrap();
        let screen_center = available_rect.center();

        // Initialize globe radius if not set (only on first render)
        if self.globe_state.radius < 10.0 {
            self.globe_state.radius = (available_rect.width().min(available_rect.height()) * 0.45) as f32;
        }

        // Process pending tile loads (both regular and GIBS)
        let tiles_loaded = self.tile_cache.process_pending(ui.ctx());
        let gibs_loaded = self.gibs_tile_cache.process_pending(ui.ctx());
        if tiles_loaded || gibs_loaded {
            ui.ctx().request_repaint();
        }

        // Handle input
        let response = ui.allocate_rect(available_rect, egui::Sense::click_and_drag());

        // Handle click for entity selection
        if response.clicked() {
            if let Some(pos) = response.interact_pointer_pos() {
                self.selection = self.find_entity_at_globe(pos, screen_center);
            }
        }

        // Rotate globe with drag
        if response.dragged() {
            self.globe_state.rotate(response.drag_delta());
            // Store velocity for inertia
            self.globe_state.rotation_velocity = (
                -response.drag_delta().x as f64 * 0.3,
                response.drag_delta().y as f64 * 0.3,
            );
            self.globe_state.dragging = true;
            ui.ctx().request_repaint();
        } else {
            self.globe_state.dragging = false;
        }

        // Apply inertia
        self.globe_state.apply_inertia(1.0 / 60.0);

        // Zoom with scroll (changes globe radius)
        // Check both raw and smooth scroll delta
        let scroll_delta = ui.input(|i| {
            let smooth = i.smooth_scroll_delta.y;
            let raw = i.raw_scroll_delta.y;
            if smooth.abs() > raw.abs() { smooth } else { raw }
        });
        if scroll_delta.abs() > 0.01 {
            let zoom_factor = 1.0 + scroll_delta * 0.005;
            self.globe_state.radius = (self.globe_state.radius * zoom_factor)
                .clamp(80.0, available_rect.width().min(available_rect.height()) * 1.5);
            ui.ctx().request_repaint();
        }

        // Also handle +/- keys for zoom
        if ui.input(|i| i.key_pressed(egui::Key::Equals) || i.key_pressed(egui::Key::Plus)) {
            self.globe_state.radius = (self.globe_state.radius * 1.1)
                .clamp(80.0, available_rect.width().min(available_rect.height()) * 1.5);
            ui.ctx().request_repaint();
        }
        if ui.input(|i| i.key_pressed(egui::Key::Minus)) {
            self.globe_state.radius = (self.globe_state.radius * 0.9)
                .clamp(80.0, available_rect.width().min(available_rect.height()) * 1.5);
            ui.ctx().request_repaint();
        }

        let painter = ui.painter_at(available_rect);

        // Draw background (space)
        painter.rect_filled(available_rect, 0.0, egui::Color32::from_rgb(5, 10, 20));

        // Draw globe
        self.render_globe_surface(&painter, screen_center);

        // Draw entities on globe
        self.render_entities_globe(&painter, screen_center);

        // Draw globe outline (atmosphere glow)
        let glow_color = egui::Color32::from_rgba_unmultiplied(100, 150, 255, 30);
        for i in 1..=4 {
            painter.circle_stroke(
                screen_center,
                self.globe_state.radius + i as f32 * 3.0,
                egui::Stroke::new(3.0, glow_color),
            );
        }

        // Globe edge
        painter.circle_stroke(
            screen_center,
            self.globe_state.radius,
            egui::Stroke::new(2.0, egui::Color32::from_rgb(80, 120, 180)),
        );

        // Draw zoom buttons as an overlay Area (so they receive input properly)
        // Cap max radius to avoid tile rendering issues at extreme zoom
        let max_radius = available_rect.width().min(available_rect.height()) * 0.9;
        let globe_radius = self.globe_state.radius;

        egui::Area::new(egui::Id::new("globe_zoom_buttons"))
            .fixed_pos(egui::pos2(available_rect.right() - 55.0, available_rect.top() + 10.0))
            .order(egui::Order::Foreground)
            .show(ui.ctx(), |ui| {
                ui.vertical(|ui| {
                    if ui.button(egui::RichText::new("+").size(20.0)).clicked() {
                        self.globe_state.radius = (globe_radius * 1.25).clamp(80.0, max_radius);
                    }
                    if ui.button(egui::RichText::new("−").size(20.0)).clicked() {
                        self.globe_state.radius = (globe_radius * 0.8).clamp(80.0, max_radius);
                    }
                });
            });

        // Request repaint if simulation is running or rotating
        if self.simulation.time_scale != TimeScale::Paused
            || self.tile_cache.has_loading_tiles()
            || self.gibs_tile_cache.has_loading_tiles()
            || self.globe_state.rotation_velocity.0.abs() > 0.1
            || self.globe_state.rotation_velocity.1.abs() > 0.1
        {
            ui.ctx().request_repaint();
        }
    }

    fn render_globe_surface(&mut self, painter: &egui::Painter, screen_center: egui::Pos2) {
        // 1. First fill with ocean color as base (slightly smaller to avoid z-fighting with tiles)
        painter.circle_filled(
            screen_center,
            self.globe_state.radius - 2.0,
            egui::Color32::from_rgb(15, 35, 70), // Deep ocean blue
        );

        // 2. Render NASA GIBS Blue Marble tiles
        self.render_gibs_tiles(painter, screen_center);

        // 3. Draw graticules (lat/lon grid) on top
        let graticule_color = egui::Color32::from_rgba_unmultiplied(255, 255, 255, 40);

        // Latitude lines
        for lat in (-80..=80).step_by(20) {
            let lat = lat as f64;
            let mut points: Vec<egui::Pos2> = Vec::new();

            for lon in (-180..=180).step_by(5) {
                let lon = lon as f64;
                if let Some(screen_pos) = self.globe_state.geo_to_screen(
                    GeoCoord::new(lat, lon),
                    screen_center,
                ) {
                    points.push(screen_pos);
                } else if !points.is_empty() {
                    if points.len() >= 2 {
                        painter.add(egui::Shape::line(
                            points.clone(),
                            egui::Stroke::new(0.5, graticule_color),
                        ));
                    }
                    points.clear();
                }
            }
            if points.len() >= 2 {
                painter.add(egui::Shape::line(points, egui::Stroke::new(0.5, graticule_color)));
            }
        }

        // Longitude lines
        for lon in (-180..180).step_by(30) {
            let lon = lon as f64;
            let mut points: Vec<egui::Pos2> = Vec::new();

            for lat in (-90..=90).step_by(5) {
                let lat = lat as f64;
                if let Some(screen_pos) = self.globe_state.geo_to_screen(
                    GeoCoord::new(lat, lon),
                    screen_center,
                ) {
                    points.push(screen_pos);
                } else if !points.is_empty() {
                    if points.len() >= 2 {
                        painter.add(egui::Shape::line(
                            points.clone(),
                            egui::Stroke::new(0.5, graticule_color),
                        ));
                    }
                    points.clear();
                }
            }
            if points.len() >= 2 {
                painter.add(egui::Shape::line(points, egui::Stroke::new(0.5, graticule_color)));
            }
        }
    }

    /// Render NASA GIBS Blue Marble tiles on the globe
    fn render_gibs_tiles(&mut self, painter: &egui::Painter, screen_center: egui::Pos2) {
        // Choose zoom level based on globe size
        // Keep zoom low (1-3) for performance - Blue Marble looks good at low zoom
        let zoom = match self.globe_state.radius as u32 {
            0..=120 => 1,
            121..=250 => 2,
            _ => 3, // Cap at 3 for performance
        };

        // Determine which tiles are potentially visible
        let mut needed_tiles: std::collections::HashSet<GibsTileCoord> = std::collections::HashSet::new();

        // Sample visible points on the globe to find needed tiles
        // Use 10-degree sampling to ensure we catch all tiles at all zoom levels
        let step = 10;
        for lat in (-90..=90).step_by(step) {
            for lon in (-180..180).step_by(step) {
                let lat = lat as f64;
                let lon = lon as f64;
                // Check if this point is on the visible hemisphere
                if self.globe_state.geo_to_screen(GeoCoord::new(lat, lon), screen_center).is_some() {
                    let tile_coord = GibsTileCache::geo_to_tile(lat, lon, zoom);
                    needed_tiles.insert(tile_coord);
                }
            }
        }

        // Limit max tiles to prevent performance issues (30 is enough for visible hemisphere)
        let max_tiles = 30;
        let num_needed = needed_tiles.len().min(max_tiles);
        let mut num_rendered = 0;

        // Sort tiles for deterministic render order (prevents flicker)
        let mut tiles_vec: Vec<_> = needed_tiles.into_iter().collect();
        tiles_vec.sort_by(|a, b| {
            a.z.cmp(&b.z).then(a.row.cmp(&b.row)).then(a.col.cmp(&b.col))
        });

        // Render each needed tile (limited)
        for tile_coord in tiles_vec.into_iter().take(max_tiles) {
            if self.render_single_gibs_tile(painter, screen_center, tile_coord) {
                num_rendered += 1;
            }
        }

        // Debug: show loading status
        let status_text = format!("Tiles: {}/{} (z{})", num_rendered, num_needed, zoom);
        painter.text(
            egui::pos2(screen_center.x - self.globe_state.radius + 10.0,
                       screen_center.y - self.globe_state.radius + 10.0),
            egui::Align2::LEFT_TOP,
            status_text,
            egui::FontId::proportional(12.0),
            egui::Color32::from_rgb(200, 200, 200),
        );
    }

    /// Render a single GIBS tile projected onto the globe
    /// Returns true if the tile was rendered
    fn render_single_gibs_tile(&mut self, painter: &egui::Painter, screen_center: egui::Pos2, tile_coord: GibsTileCoord) -> bool {
        // Get the tile texture (request if not loaded)
        let texture = match self.gibs_tile_cache.get_tile(tile_coord) {
            Some(tex) => tex.clone(),
            None => return false, // Tile not loaded yet
        };

        // Get tile geographic bounds
        let (lat_min, lat_max, lon_min, lon_max) = GibsTileCache::tile_bounds(&tile_coord);

        // Sample colors from the tile and draw as colored points on the globe
        // This is simpler than mesh rendering and works better with egui
        let texture_size = texture.size();
        let _tex_width = texture_size[0] as f32;
        let _tex_height = texture_size[1] as f32;

        // Create a mesh for the tile
        // Use fewer subdivisions for better performance (6x6 = 72 triangles per tile)
        let subdivisions = 6;
        let lat_step = (lat_max - lat_min) / subdivisions as f64;
        let lon_step = (lon_max - lon_min) / subdivisions as f64;

        let mut mesh = egui::Mesh::with_texture(texture.id());

        for lat_i in 0..subdivisions {
            for lon_i in 0..subdivisions {
                let lat0 = lat_max - (lat_i as f64) * lat_step;
                let lat1 = lat_max - ((lat_i + 1) as f64) * lat_step;
                let lon0 = lon_min + (lon_i as f64) * lon_step;
                let lon1 = lon_min + ((lon_i + 1) as f64) * lon_step;

                // Get screen positions for quad corners
                let pos_tl = self.globe_state.geo_to_screen(GeoCoord::new(lat0, lon0), screen_center);
                let pos_tr = self.globe_state.geo_to_screen(GeoCoord::new(lat0, lon1), screen_center);
                let pos_bl = self.globe_state.geo_to_screen(GeoCoord::new(lat1, lon0), screen_center);
                let pos_br = self.globe_state.geo_to_screen(GeoCoord::new(lat1, lon1), screen_center);

                // Only render if all corners are visible
                if let (Some(tl), Some(tr), Some(bl), Some(br)) = (pos_tl, pos_tr, pos_bl, pos_br) {
                    // Check if quad is too distorted (crossing back of globe)
                    let max_dist = self.globe_state.radius * 0.5;
                    if tl.distance(tr) > max_dist || tl.distance(bl) > max_dist ||
                       tr.distance(br) > max_dist || bl.distance(br) > max_dist {
                        continue;
                    }

                    // UV coordinates - map to texture coordinates
                    // For GIBS EPSG:4326 tiles: u maps to longitude, v maps to latitude
                    let u0 = lon_i as f32 / subdivisions as f32;
                    let u1 = (lon_i + 1) as f32 / subdivisions as f32;
                    let v0 = lat_i as f32 / subdivisions as f32;
                    let v1 = (lat_i + 1) as f32 / subdivisions as f32;

                    // Add two triangles for this quad
                    let idx = mesh.vertices.len() as u32;

                    mesh.vertices.push(egui::epaint::Vertex {
                        pos: tl,
                        uv: egui::pos2(u0, v0),
                        color: egui::Color32::WHITE,
                    });
                    mesh.vertices.push(egui::epaint::Vertex {
                        pos: tr,
                        uv: egui::pos2(u1, v0),
                        color: egui::Color32::WHITE,
                    });
                    mesh.vertices.push(egui::epaint::Vertex {
                        pos: bl,
                        uv: egui::pos2(u0, v1),
                        color: egui::Color32::WHITE,
                    });
                    mesh.vertices.push(egui::epaint::Vertex {
                        pos: br,
                        uv: egui::pos2(u1, v1),
                        color: egui::Color32::WHITE,
                    });

                    // Triangle 1: TL, TR, BL
                    mesh.indices.push(idx);
                    mesh.indices.push(idx + 1);
                    mesh.indices.push(idx + 2);

                    // Triangle 2: TR, BR, BL
                    mesh.indices.push(idx + 1);
                    mesh.indices.push(idx + 3);
                    mesh.indices.push(idx + 2);
                }
            }
        }

        if !mesh.vertices.is_empty() {
            painter.add(egui::Shape::mesh(mesh));
            true
        } else {
            false
        }
    }

    /// Render terrain elevation shading for major mountain ranges
    fn render_terrain_shading(&self, painter: &egui::Painter, screen_center: egui::Pos2) {
        // Define major mountain ranges and highland areas
        // Each entry: (center_lat, center_lon, radius_deg, height_factor)
        let mountain_ranges: Vec<(f64, f64, f64, f32)> = vec![
            // Himalayas/Tibet
            (32.0, 85.0, 12.0, 0.9),
            (28.0, 85.0, 8.0, 1.0),
            // Andes
            (-15.0, -70.0, 5.0, 0.85),
            (-30.0, -70.0, 4.0, 0.8),
            (-45.0, -72.0, 3.0, 0.7),
            // Rockies
            (40.0, -106.0, 6.0, 0.7),
            (45.0, -110.0, 5.0, 0.65),
            (52.0, -118.0, 4.0, 0.6),
            // Alps
            (46.5, 10.0, 3.0, 0.7),
            // Caucasus
            (42.5, 44.0, 3.0, 0.65),
            // Urals
            (58.0, 60.0, 4.0, 0.5),
            // Atlas Mountains
            (32.0, -5.0, 3.0, 0.55),
            // Ethiopian Highlands
            (10.0, 38.0, 4.0, 0.6),
            // East African Rift
            (-3.0, 37.0, 2.5, 0.65),
            // Scandinavian Mountains
            (65.0, 14.0, 3.0, 0.5),
            // Appalachians
            (38.0, -79.0, 3.0, 0.4),
            // Japanese Alps
            (36.0, 138.0, 2.0, 0.6),
            // New Zealand Alps
            (-43.5, 170.0, 2.0, 0.65),
            // Carpathians
            (47.0, 25.0, 3.0, 0.5),
            // Hindu Kush
            (36.0, 71.0, 4.0, 0.8),
            // Tian Shan
            (42.0, 80.0, 5.0, 0.75),
            // Kunlun
            (36.0, 85.0, 6.0, 0.7),
            // Alaska Range
            (63.0, -151.0, 3.0, 0.7),
            // Sierra Nevada
            (37.0, -119.0, 2.5, 0.6),
            // Great Dividing Range (Australia)
            (-32.0, 149.0, 4.0, 0.4),
        ];

        let highlight_color = egui::Color32::from_rgba_unmultiplied(140, 120, 90, 100);
        let shadow_color = egui::Color32::from_rgba_unmultiplied(40, 35, 25, 80);

        for (center_lat, center_lon, radius_deg, height_factor) in &mountain_ranges {
            // Check if mountain range center is visible
            if let Some(center_pos) = self.globe_state.geo_to_screen(
                GeoCoord::new(*center_lat, *center_lon),
                screen_center,
            ) {
                // Scale radius based on globe size
                let screen_radius = (*radius_deg as f32 / 180.0) * self.globe_state.radius * 3.0;

                // Draw highlight (illuminated side - assume sun from upper-left)
                let highlight_offset = 2.0 * height_factor;
                painter.circle_filled(
                    egui::pos2(center_pos.x - highlight_offset, center_pos.y - highlight_offset),
                    screen_radius * 0.9,
                    highlight_color,
                );

                // Draw shadow (dark side)
                let shadow_offset = 3.0 * height_factor;
                painter.circle_filled(
                    egui::pos2(center_pos.x + shadow_offset, center_pos.y + shadow_offset),
                    screen_radius * 0.7,
                    shadow_color,
                );
            }
        }

        // Add ice caps at poles
        self.render_ice_caps(painter, screen_center);
    }

    /// Render polar ice caps
    fn render_ice_caps(&self, painter: &egui::Painter, screen_center: egui::Pos2) {
        let ice_color = egui::Color32::from_rgba_unmultiplied(220, 230, 240, 180);

        // Arctic ice cap
        if let Some(pos) = self.globe_state.geo_to_screen(GeoCoord::new(90.0, 0.0), screen_center) {
            let ice_radius = (15.0_f32 / 180.0) * self.globe_state.radius * 3.0;
            painter.circle_filled(pos, ice_radius, ice_color);
        }

        // Antarctic ice cap
        if let Some(pos) = self.globe_state.geo_to_screen(GeoCoord::new(-90.0, 0.0), screen_center) {
            let ice_radius = (20.0_f32 / 180.0) * self.globe_state.radius * 3.0;
            painter.circle_filled(pos, ice_radius, ice_color);
        }
    }

    fn render_globe_coastlines(&self, painter: &egui::Painter, screen_center: egui::Pos2) {
        let land_color = egui::Color32::from_rgb(60, 90, 55);
        let coast_color = egui::Color32::from_rgb(90, 130, 85);

        // Draw filled continent shapes using polygon approximations
        let continents: Vec<(&str, Vec<(f64, f64)>)> = vec![
            // North America (simplified outline)
            ("North America", vec![
                (49.0, -125.0), (60.0, -140.0), (70.0, -140.0), (71.0, -155.0), // Alaska
                (65.0, -168.0), (55.0, -165.0), (55.0, -130.0), // West Coast
                (49.0, -125.0), (48.0, -123.0), (45.0, -124.0), (42.0, -124.0),
                (35.0, -121.0), (32.0, -117.0), (23.0, -110.0), (22.0, -106.0), // Baja
                (18.0, -105.0), (15.0, -92.0), (18.0, -88.0), (21.0, -87.0), // Yucatan
                (25.0, -80.0), (30.0, -81.0), (32.0, -80.0), (35.0, -76.0),
                (37.0, -76.0), (39.0, -74.0), (41.0, -72.0), (42.0, -70.0),
                (44.0, -68.0), (45.0, -67.0), (47.0, -68.0), (50.0, -65.0),
                (52.0, -56.0), (47.0, -53.0), (46.0, -60.0), (45.0, -64.0),
                (44.0, -66.0), (60.0, -65.0), (65.0, -62.0), (70.0, -80.0),
                (75.0, -95.0), (70.0, -130.0), (49.0, -125.0),
            ]),
            // South America
            ("South America", vec![
                (12.0, -72.0), (11.0, -75.0), (4.0, -77.0), (-1.0, -80.0),
                (-5.0, -81.0), (-14.0, -77.0), (-18.0, -70.0), (-24.0, -70.0),
                (-27.0, -71.0), (-33.0, -72.0), (-42.0, -74.0), (-46.0, -76.0),
                (-52.0, -75.0), (-55.0, -68.0), (-55.0, -66.0), (-52.0, -68.0),
                (-48.0, -66.0), (-42.0, -64.0), (-38.0, -57.0), (-34.0, -54.0),
                (-32.0, -52.0), (-25.0, -48.0), (-23.0, -44.0), (-16.0, -39.0),
                (-8.0, -35.0), (-5.0, -35.0), (0.0, -50.0), (5.0, -54.0),
                (7.0, -58.0), (10.0, -62.0), (10.5, -68.0), (12.0, -72.0),
            ]),
            // Europe
            ("Europe", vec![
                (36.0, -6.0), (37.0, -9.0), (43.0, -9.0), (44.0, -2.0),
                (48.0, -5.0), (49.0, -1.0), (51.0, 2.0), (54.0, 5.0),
                (54.0, 9.0), (57.0, 10.0), (58.0, 6.0), (62.0, 5.0),
                (64.0, 11.0), (68.0, 15.0), (70.0, 26.0), (70.0, 30.0),
                (65.0, 30.0), (60.0, 30.0), (60.0, 25.0), (55.0, 22.0),
                (54.0, 18.0), (55.0, 14.0), (52.0, 14.0), (48.0, 17.0),
                (47.0, 16.0), (46.0, 14.0), (45.0, 14.0), (44.0, 12.0),
                (41.0, 17.0), (40.0, 19.0), (38.0, 22.0), (36.0, 28.0),
                (41.0, 29.0), (42.0, 28.0), (45.0, 30.0), (46.0, 38.0),
                (44.0, 40.0), (42.0, 44.0), (44.0, 40.0), (46.0, 38.0),
                (42.0, 28.0), (41.0, 29.0), (36.0, 28.0), (36.0, -6.0),
            ]),
            // Africa
            ("Africa", vec![
                (37.0, -6.0), (36.0, 0.0), (37.0, 10.0), (32.0, 32.0),
                (30.0, 33.0), (23.0, 37.0), (12.0, 44.0), (11.0, 51.0),
                (2.0, 51.0), (-1.0, 42.0), (-12.0, 40.0), (-16.0, 38.0),
                (-26.0, 33.0), (-34.0, 26.0), (-35.0, 20.0), (-33.0, 18.0),
                (-30.0, 17.0), (-22.0, 14.0), (-17.0, 12.0), (-6.0, 12.0),
                (4.0, 10.0), (6.0, 1.0), (4.0, -3.0), (5.0, -8.0),
                (10.0, -15.0), (15.0, -17.0), (21.0, -17.0), (28.0, -13.0),
                (33.0, -9.0), (36.0, -6.0), (37.0, -6.0),
            ]),
            // Asia (mainland)
            ("Asia", vec![
                (42.0, 28.0), (41.0, 29.0), (42.0, 35.0), (36.0, 36.0),
                (33.0, 35.0), (30.0, 48.0), (27.0, 57.0), (25.0, 60.0),
                (23.0, 68.0), (8.0, 77.0), (6.0, 80.0), (15.0, 80.0),
                (22.0, 88.0), (21.0, 92.0), (8.0, 98.0), (2.0, 103.0),
                (1.0, 104.0), (7.0, 117.0), (22.0, 114.0), (23.0, 117.0),
                (31.0, 122.0), (35.0, 120.0), (38.0, 122.0), (40.0, 124.0),
                (45.0, 131.0), (50.0, 140.0), (53.0, 141.0), (55.0, 137.0),
                (60.0, 150.0), (62.0, 163.0), (65.0, 170.0), (68.0, 180.0),
                (70.0, 180.0), (75.0, 140.0), (78.0, 100.0), (77.0, 70.0),
                (70.0, 60.0), (70.0, 30.0), (65.0, 30.0), (60.0, 30.0),
                (60.0, 30.0), (55.0, 38.0), (50.0, 40.0), (45.0, 38.0),
                (42.0, 44.0), (42.0, 28.0),
            ]),
            // Australia
            ("Australia", vec![
                (-12.0, 130.0), (-12.0, 136.0), (-14.0, 141.0), (-10.0, 142.0),
                (-17.0, 146.0), (-19.0, 147.0), (-24.0, 153.0), (-28.0, 154.0),
                (-32.0, 152.0), (-34.0, 151.0), (-38.0, 146.0), (-39.0, 146.0),
                (-38.0, 141.0), (-35.0, 137.0), (-35.0, 135.0), (-32.0, 133.0),
                (-32.0, 127.0), (-34.0, 123.0), (-34.0, 116.0), (-31.0, 115.0),
                (-25.0, 113.0), (-22.0, 114.0), (-20.0, 119.0), (-17.0, 122.0),
                (-14.0, 126.0), (-13.0, 129.0), (-12.0, 130.0),
            ]),
        ];

        // Render filled continents
        for (_name, coords) in &continents {
            let mut visible_points: Vec<egui::Pos2> = Vec::new();
            let mut all_visible = true;

            for &(lat, lon) in coords {
                if let Some(pos) = self.globe_state.geo_to_screen(
                    GeoCoord::new(lat, lon),
                    screen_center,
                ) {
                    visible_points.push(pos);
                } else {
                    all_visible = false;
                }
            }

            // Only draw filled polygon if most points are visible
            if visible_points.len() >= 3 {
                if all_visible || visible_points.len() > coords.len() / 2 {
                    // Draw filled shape
                    painter.add(egui::Shape::convex_polygon(
                        visible_points.clone(),
                        land_color,
                        egui::Stroke::new(1.0, coast_color),
                    ));
                } else {
                    // Draw just the outline for partial visibility
                    painter.add(egui::Shape::line(
                        visible_points,
                        egui::Stroke::new(1.5, coast_color),
                    ));
                }
            }
        }

        // Draw major islands as smaller filled shapes
        self.render_major_islands(painter, screen_center);
    }

    fn render_major_islands(&self, painter: &egui::Painter, screen_center: egui::Pos2) {
        let land_color = egui::Color32::from_rgb(60, 90, 55);
        let coast_color = egui::Color32::from_rgb(90, 130, 85);

        // Major islands
        let islands: Vec<Vec<(f64, f64)>> = vec![
            // Greenland
            vec![
                (60.0, -43.0), (67.0, -53.0), (76.0, -68.0), (83.0, -42.0),
                (82.0, -22.0), (77.0, -18.0), (70.0, -22.0), (60.0, -43.0),
            ],
            // UK
            vec![
                (50.0, -5.0), (51.0, 1.0), (53.0, 0.0), (54.0, -3.0),
                (56.0, -6.0), (58.0, -5.0), (58.0, -3.0), (57.0, -2.0),
                (55.0, -1.5), (53.0, -1.0), (52.0, 1.5), (51.5, 1.0),
                (50.0, -5.0),
            ],
            // Japan main islands
            vec![
                (31.0, 131.0), (33.0, 130.0), (34.0, 132.0), (35.0, 136.0),
                (36.0, 140.0), (38.0, 140.0), (41.0, 140.0), (42.0, 141.0),
                (43.0, 145.0), (42.0, 145.0), (40.0, 140.0), (35.0, 140.0),
                (33.0, 136.0), (31.0, 131.0),
            ],
            // New Zealand North
            vec![
                (-34.5, 173.0), (-37.0, 175.0), (-38.0, 178.0), (-41.0, 175.0),
                (-41.0, 173.0), (-39.0, 174.0), (-36.0, 174.0), (-34.5, 173.0),
            ],
            // New Zealand South
            vec![
                (-41.0, 174.0), (-42.0, 172.0), (-44.0, 169.0), (-46.0, 167.0),
                (-46.0, 170.0), (-45.0, 171.0), (-43.0, 173.0), (-41.0, 174.0),
            ],
            // Iceland
            vec![
                (63.5, -20.0), (64.0, -22.0), (65.5, -24.0), (66.5, -23.0),
                (66.0, -18.0), (65.0, -14.0), (64.0, -15.0), (63.5, -20.0),
            ],
            // Madagascar
            vec![
                (-12.0, 49.0), (-14.0, 50.0), (-19.0, 47.0), (-24.0, 47.0),
                (-25.0, 45.0), (-22.0, 43.0), (-17.0, 44.0), (-12.0, 49.0),
            ],
            // Indonesia - Sumatra
            vec![
                (5.5, 95.0), (2.0, 99.0), (-1.0, 104.0), (-5.0, 105.0),
                (-6.0, 103.0), (-2.0, 101.0), (2.0, 97.0), (5.5, 95.0),
            ],
            // Indonesia - Java
            vec![
                (-6.0, 105.5), (-7.0, 107.0), (-8.0, 112.0), (-8.5, 114.0),
                (-7.5, 114.0), (-6.5, 110.0), (-6.0, 106.0), (-6.0, 105.5),
            ],
        ];

        for island in &islands {
            let mut visible_points: Vec<egui::Pos2> = Vec::new();

            for &(lat, lon) in island {
                if let Some(pos) = self.globe_state.geo_to_screen(
                    GeoCoord::new(lat, lon),
                    screen_center,
                ) {
                    visible_points.push(pos);
                }
            }

            if visible_points.len() >= 3 {
                painter.add(egui::Shape::convex_polygon(
                    visible_points,
                    land_color,
                    egui::Stroke::new(1.0, coast_color),
                ));
            }
        }
    }

    fn render_entities_globe(&self, painter: &egui::Painter, screen_center: egui::Pos2) {
        // Draw detection ranges first (below entities)
        if self.show_detection_ranges {
            self.render_detection_ranges_globe(painter, screen_center);
        }

        // Draw tracking lines
        if self.show_tracking_lines {
            self.render_tracking_lines_globe(painter, screen_center);
        }

        // Draw trajectories
        if self.show_trajectories {
            self.render_trajectories_globe(painter, screen_center);
        }

        // Draw defense units
        for unit in &self.simulation.defense_units {
            if let Some(pos) = self.globe_state.geo_to_screen(unit.position, screen_center) {
                MilitarySymbols::draw_defense_unit(
                    painter,
                    pos,
                    unit.affiliation,
                    unit.defense_type,
                    unit.status,
                    10.0,
                );
            }
        }

        // Draw missiles - check track view mode
        match self.track_view_mode {
            TrackViewMode::TrueTrack => {
                // Show actual missile positions
                for missile in &self.simulation.missiles {
                    if let Some(pos) = self.globe_state.geo_to_screen(missile.position, screen_center) {
                        let heading = bearing(missile.position, missile.target);
                        MilitarySymbols::draw_missile(
                            painter,
                            pos,
                            missile.affiliation,
                            missile.status,
                            ((heading - 90.0) as f32).to_radians(),
                            8.0,
                            None, // No track quality in true track mode
                            false, // Not fire control locked in true track mode
                        );
                    }
                }
            }
            TrackViewMode::DetectedTrack => {
                // Show missiles at sensor-perceived positions with uncertainty
                let defense_unit_ids: std::collections::HashSet<u64> = self.simulation.defense_units.iter().map(|u| u.id).collect();
                let fused_tracks = self.simulation.detection.get_all_fused_tracks(&defense_unit_ids);
                for track in &fused_tracks {
                    self.render_detected_missile_globe(painter, screen_center, &track);
                }

                // Render false alarms if enabled
                if self.show_false_alarms {
                    self.render_false_alarms_globe(painter, screen_center);
                }
            }
        }

        // Draw interceptors
        for interceptor in &self.simulation.interceptors {
            if interceptor.status != crate::simulation::InterceptorStatus::InFlight {
                continue;
            }
            if let Some(pos) = self.globe_state.geo_to_screen(interceptor.position, screen_center) {
                let color = egui::Color32::from_rgb(255, 255, 0);
                painter.circle_filled(pos, 4.0, color);
            }
        }
    }

    fn render_trajectories_globe(&self, painter: &egui::Painter, screen_center: egui::Pos2) {
        for missile in &self.simulation.missiles {
            if !matches!(
                missile.status,
                MissileStatus::Boost | MissileStatus::Midcourse | MissileStatus::Terminal
            ) {
                continue;
            }

            let trajectory = BallisticTrajectory::new(missile.origin, missile.target);
            let progress = missile.flight_progress();

            // Draw trajectory arc
            let mut points: Vec<egui::Pos2> = Vec::new();
            let segments = 50;

            for i in 0..=segments {
                let t = i as f64 / segments as f64;
                let (pos, _alt) = trajectory.position_at(t);

                if let Some(screen_pos) = self.globe_state.geo_to_screen(pos, screen_center) {
                    points.push(screen_pos);
                } else if !points.is_empty() {
                    // Crossed to back of globe
                    if points.len() >= 2 {
                        let color = if t < progress {
                            egui::Color32::from_rgba_unmultiplied(180, 80, 40, 150)
                        } else {
                            egui::Color32::from_rgba_unmultiplied(255, 150, 100, 180)
                        };
                        painter.add(egui::Shape::line(
                            points.clone(),
                            egui::Stroke::new(2.0, color),
                        ));
                    }
                    points.clear();
                }
            }

            // Draw remaining points
            if points.len() >= 2 {
                painter.add(egui::Shape::line(
                    points,
                    egui::Stroke::new(2.0, egui::Color32::from_rgb(255, 150, 100)),
                ));
            }

            // Draw impact marker
            if let Some(impact_pos) = self.globe_state.geo_to_screen(missile.target, screen_center) {
                MilitarySymbols::draw_target_designator(painter, impact_pos, 8.0);
            }
        }
    }

    /// Render detection and engagement ranges on the globe
    fn render_detection_ranges_globe(&self, painter: &egui::Painter, screen_center: egui::Pos2) {
        // Defense unit ranges
        for unit in &self.simulation.defense_units {
            let config = self.simulation.sensor_configs.get_by_name(&unit.sensor_config_name);
            let base_range = config.detection.detection_range_km;
            let platform_config = self.simulation.platform_configs.get_by_defense_type(unit.defense_type);
            let engagement_range_km = platform_config.launcher.engagement_range_km;

            // Check if this is a phased array radar
            if config.detection.radar_type == RadarType::PhasedArray {
                // Draw three mode-specific ranges for phased arrays

                // FireControl range (innermost, red/orange)
                let fc_range = base_range * config.detection.get_range_multiplier(RadarMode::FireControl) * 1.5;
                self.draw_range_circle_globe(
                    painter,
                    screen_center,
                    unit.position,
                    fc_range,
                    egui::Stroke::new(2.0, colors::fire_control_color(unit.affiliation)),
                );

                // Track range (middle, yellow)
                let track_range = base_range * config.detection.get_range_multiplier(RadarMode::Track) * 1.5;
                self.draw_range_circle_globe(
                    painter,
                    screen_center,
                    unit.position,
                    track_range,
                    egui::Stroke::new(1.5, colors::track_mode_color(unit.affiliation)),
                );

                // Search range (outermost, cyan/green)
                let search_range = base_range * config.detection.get_range_multiplier(RadarMode::Search) * 1.5;
                self.draw_range_circle_globe(
                    painter,
                    screen_center,
                    unit.position,
                    search_range,
                    egui::Stroke::new(1.0, colors::search_mode_color(unit.affiliation)),
                );

                // Draw mode capacity indicator
                if let Some(screen_pos) = self.globe_state.geo_to_screen(unit.position, screen_center) {
                    let mode_state = self.simulation.detection.radar_mode_states.get(&unit.id);
                    if let Some(state) = mode_state {
                        let fc_count = state.target_assignments.values().filter(|&&(m, _)| m == RadarMode::FireControl).count();
                        let track_count = state.target_assignments.values().filter(|&&(m, _)| m == RadarMode::Track).count();
                        let total_count = state.target_assignments.len();

                        let label = format!(
                            "FC:{}/{} T:{} All:{}/{}",
                            fc_count,
                            config.tracking.max_fire_control_tracks,
                            track_count,
                            total_count,
                            config.tracking.max_simultaneous_tracks
                        );

                        painter.text(
                            screen_pos + egui::vec2(0.0, 20.0),
                            egui::Align2::CENTER_TOP,
                            label,
                            egui::FontId::monospace(10.0),
                            egui::Color32::from_rgba_unmultiplied(255, 255, 255, 200),
                        );
                    }
                }
            } else {
                // Mechanical radar - single range circle
                let max_detection_range_km = base_range * 1.5;
                self.draw_range_circle_globe(
                    painter,
                    screen_center,
                    unit.position,
                    max_detection_range_km,
                    egui::Stroke::new(1.5, colors::detection_range_stroke(unit.affiliation)),
                );
            }

            // Draw engagement range circle (for all radars)
            self.draw_range_circle_globe(
                painter,
                screen_center,
                unit.position,
                engagement_range_km,
                egui::Stroke::new(2.0, colors::engagement_envelope_color(unit.affiliation)),
            );
        }

        // Radar station detection ranges
        for station in &self.simulation.radar_stations {
            let config = self.simulation.sensor_configs.get_by_name(&station.sensor_config_name);
            let base_range = station.detection_range_km;

            // Check if this is a phased array radar
            if config.detection.radar_type == RadarType::PhasedArray {
                // Draw three mode-specific ranges for phased arrays with mode-specific azimuth coverage

                // Search range (outermost, cyan/green) - drawn first so it's behind
                let search_range = base_range * config.detection.get_range_multiplier(RadarMode::Search) * 1.5;
                let search_coverage = config.detection.get_effective_azimuth_coverage(RadarMode::Search);

                if search_coverage >= 360.0 {
                    // Full circle for omnidirectional
                    let search_color = match station.affiliation {
                        Affiliation::Friendly => egui::Color32::from_rgba_unmultiplied(100, 220, 255, 70),
                        Affiliation::Hostile => egui::Color32::from_rgba_unmultiplied(255, 150, 100, 70),
                        Affiliation::Neutral => egui::Color32::from_rgba_unmultiplied(150, 255, 150, 70),
                    };
                    self.draw_range_circle_globe(
                        painter,
                        screen_center,
                        station.position,
                        search_range,
                        egui::Stroke::new(1.0, search_color),
                    );
                } else {
                    // Sector for directional radars
                    let search_fill = match station.affiliation {
                        Affiliation::Friendly => egui::Color32::from_rgba_unmultiplied(100, 220, 255, 25),
                        Affiliation::Hostile => egui::Color32::from_rgba_unmultiplied(255, 150, 100, 25),
                        Affiliation::Neutral => egui::Color32::from_rgba_unmultiplied(150, 255, 150, 25),
                    };
                    let search_stroke = match station.affiliation {
                        Affiliation::Friendly => egui::Color32::from_rgba_unmultiplied(100, 220, 255, 70),
                        Affiliation::Hostile => egui::Color32::from_rgba_unmultiplied(255, 150, 100, 70),
                        Affiliation::Neutral => egui::Color32::from_rgba_unmultiplied(150, 255, 150, 70),
                    };
                    self.draw_range_sector_globe(
                        painter,
                        screen_center,
                        station.position,
                        station.facing_deg,
                        search_coverage,
                        search_range,
                        search_fill,
                        egui::Stroke::new(1.0, search_stroke),
                    );
                }

                // Track range (middle, yellow)
                let track_range = base_range * config.detection.get_range_multiplier(RadarMode::Track) * 1.5;
                let track_coverage = config.detection.get_effective_azimuth_coverage(RadarMode::Track);

                if track_coverage >= 360.0 {
                    let track_color = match station.affiliation {
                        Affiliation::Friendly => egui::Color32::from_rgba_unmultiplied(255, 255, 100, 90),
                        Affiliation::Hostile => egui::Color32::from_rgba_unmultiplied(255, 200, 80, 90),
                        Affiliation::Neutral => egui::Color32::from_rgba_unmultiplied(200, 255, 100, 90),
                    };
                    self.draw_range_circle_globe(
                        painter,
                        screen_center,
                        station.position,
                        track_range,
                        egui::Stroke::new(1.5, track_color),
                    );
                } else {
                    let track_fill = match station.affiliation {
                        Affiliation::Friendly => egui::Color32::from_rgba_unmultiplied(255, 255, 100, 35),
                        Affiliation::Hostile => egui::Color32::from_rgba_unmultiplied(255, 200, 80, 35),
                        Affiliation::Neutral => egui::Color32::from_rgba_unmultiplied(200, 255, 100, 35),
                    };
                    let track_stroke = match station.affiliation {
                        Affiliation::Friendly => egui::Color32::from_rgba_unmultiplied(255, 255, 100, 90),
                        Affiliation::Hostile => egui::Color32::from_rgba_unmultiplied(255, 200, 80, 90),
                        Affiliation::Neutral => egui::Color32::from_rgba_unmultiplied(200, 255, 100, 90),
                    };
                    self.draw_range_sector_globe(
                        painter,
                        screen_center,
                        station.position,
                        station.facing_deg,
                        track_coverage,
                        track_range,
                        track_fill,
                        egui::Stroke::new(1.5, track_stroke),
                    );
                }

                // FireControl range (innermost, red/orange) - drawn last so it's on top
                let fc_range = base_range * config.detection.get_range_multiplier(RadarMode::FireControl) * 1.5;
                let fc_coverage = config.detection.get_effective_azimuth_coverage(RadarMode::FireControl);

                if fc_coverage >= 360.0 {
                    let fc_color = match station.affiliation {
                        Affiliation::Friendly => egui::Color32::from_rgba_unmultiplied(255, 150, 50, 120),
                        Affiliation::Hostile => egui::Color32::from_rgba_unmultiplied(255, 50, 50, 120),
                        Affiliation::Neutral => egui::Color32::from_rgba_unmultiplied(255, 200, 50, 120),
                    };
                    self.draw_range_circle_globe(
                        painter,
                        screen_center,
                        station.position,
                        fc_range,
                        egui::Stroke::new(2.0, fc_color),
                    );
                } else {
                    let fc_fill = match station.affiliation {
                        Affiliation::Friendly => egui::Color32::from_rgba_unmultiplied(255, 150, 50, 45),
                        Affiliation::Hostile => egui::Color32::from_rgba_unmultiplied(255, 50, 50, 45),
                        Affiliation::Neutral => egui::Color32::from_rgba_unmultiplied(255, 200, 50, 45),
                    };
                    let fc_stroke = match station.affiliation {
                        Affiliation::Friendly => egui::Color32::from_rgba_unmultiplied(255, 150, 50, 120),
                        Affiliation::Hostile => egui::Color32::from_rgba_unmultiplied(255, 50, 50, 120),
                        Affiliation::Neutral => egui::Color32::from_rgba_unmultiplied(255, 200, 50, 120),
                    };
                    self.draw_range_sector_globe(
                        painter,
                        screen_center,
                        station.position,
                        station.facing_deg,
                        fc_coverage,
                        fc_range,
                        fc_fill,
                        egui::Stroke::new(2.0, fc_stroke),
                    );
                }

                // Draw mode capacity indicator
                if let Some(screen_pos) = self.globe_state.geo_to_screen(station.position, screen_center) {
                    let mode_state = self.simulation.detection.radar_mode_states.get(&station.id);
                    if let Some(state) = mode_state {
                        let fc_count = state.target_assignments.values().filter(|&&(m, _)| m == RadarMode::FireControl).count();
                        let track_count = state.target_assignments.values().filter(|&&(m, _)| m == RadarMode::Track).count();
                        let total_count = state.target_assignments.len();

                        let label = format!(
                            "FC:{}/{} T:{} All:{}/{}",
                            fc_count,
                            config.tracking.max_fire_control_tracks,
                            track_count,
                            total_count,
                            config.tracking.max_simultaneous_tracks
                        );

                        painter.text(
                            screen_pos + egui::vec2(0.0, 20.0),
                            egui::Align2::CENTER_TOP,
                            label,
                            egui::FontId::monospace(10.0),
                            egui::Color32::from_rgba_unmultiplied(255, 255, 255, 200),
                        );
                    }
                }
            } else {
                // Mechanical radar - single range circle
                let max_detection_range_km = base_range * 1.5;
                let stroke_color = match station.affiliation {
                    Affiliation::Friendly => egui::Color32::from_rgba_unmultiplied(100, 220, 150, 100),
                    Affiliation::Hostile => egui::Color32::from_rgba_unmultiplied(255, 150, 50, 100),
                    Affiliation::Neutral => egui::Color32::from_rgba_unmultiplied(220, 220, 100, 100),
                };

                self.draw_range_circle_globe(
                    painter,
                    screen_center,
                    station.position,
                    max_detection_range_km,
                    egui::Stroke::new(1.5, stroke_color),
                );
            }
        }
    }

    /// Draw a range circle on the globe surface
    fn draw_range_circle_globe(
        &self,
        painter: &egui::Painter,
        screen_center: egui::Pos2,
        center: GeoCoord,
        range_km: f64,
        stroke: egui::Stroke,
    ) {
        // Convert range from km to degrees (approximate at equator)
        // Earth radius = 6371 km, so 1 degree ≈ 111.32 km
        let range_deg = range_km / 111.32;

        // Generate points around the circle at the given range
        let segments = 36;
        let mut points: Vec<egui::Pos2> = Vec::new();

        for i in 0..=segments {
            let angle = (i as f64 / segments as f64) * std::f64::consts::TAU;

            // Calculate point at range from center
            // Using simple approximation: lat offset = range * cos(angle), lon offset = range * sin(angle) / cos(lat)
            let lat_offset = range_deg * angle.cos();
            let lon_offset = range_deg * angle.sin() / center.lat.to_radians().cos().max(0.1);

            let point = GeoCoord::new(
                (center.lat + lat_offset).clamp(-90.0, 90.0),
                center.lon + lon_offset,
            );

            if let Some(screen_pos) = self.globe_state.geo_to_screen(point, screen_center) {
                points.push(screen_pos);
            } else if !points.is_empty() {
                // Point crossed to back of globe
                if points.len() >= 2 {
                    painter.add(egui::Shape::line(points.clone(), stroke));
                }
                points.clear();
            }
        }

        // Draw remaining points
        if points.len() >= 2 {
            painter.add(egui::Shape::line(points, stroke));
        }
    }

    /// Draw a radar sector arc on the globe (for directional radars)
    fn draw_range_sector_globe(
        &self,
        painter: &egui::Painter,
        screen_center: egui::Pos2,
        center: GeoCoord,
        facing_deg: f64,      // Direction radar faces (0 = North, 90 = East)
        coverage_deg: f64,    // Width of coverage arc
        range_km: f64,
        fill_color: egui::Color32,
        stroke: egui::Stroke,
    ) {
        // Convert range from km to degrees
        let range_deg = range_km / 111.32;

        // Calculate start and end angles for the sector
        let half_coverage = coverage_deg / 2.0;
        let start_bearing = facing_deg - half_coverage;
        let end_bearing = facing_deg + half_coverage;

        // Generate arc points
        let segments = 36;
        let mut arc_points: Vec<egui::Pos2> = Vec::new();

        for i in 0..=segments {
            let t = i as f64 / segments as f64;
            let bearing = start_bearing + (end_bearing - start_bearing) * t;
            let angle = (90.0 - bearing).to_radians(); // Convert from geographic to math angle

            // Calculate point at range from center
            let lat_offset = range_deg * angle.sin();
            let lon_offset = range_deg * angle.cos() / center.lat.to_radians().cos().max(0.1);

            let point = GeoCoord::new(
                (center.lat + lat_offset).clamp(-90.0, 90.0),
                center.lon + lon_offset,
            );

            if let Some(screen_pos) = self.globe_state.geo_to_screen(point, screen_center) {
                arc_points.push(screen_pos);
            } else if !arc_points.is_empty() {
                break; // Stop if we hit the back of the globe
            }
        }

        // Draw filled sector if we have enough points
        if arc_points.len() >= 2 {
            if let Some(center_screen) = self.globe_state.geo_to_screen(center, screen_center) {
                // Create polygon for filled sector
                let mut polygon_points = vec![center_screen];
                polygon_points.extend(arc_points.iter());

                if polygon_points.len() >= 3 {
                    painter.add(egui::Shape::convex_polygon(
                        polygon_points.clone(),
                        fill_color,
                        egui::Stroke::NONE,
                    ));
                }

                // Draw arc outline
                painter.add(egui::Shape::line(arc_points.clone(), stroke));

                // Draw radial lines
                if !arc_points.is_empty() {
                    painter.line_segment([center_screen, arc_points[0]], stroke);
                    painter.line_segment([center_screen, *arc_points.last().unwrap()], stroke);
                }
            }
        }
    }

    /// Draw a radar sector arc on the 2D map (for directional radars)
    fn draw_range_sector_2d(
        &self,
        painter: &egui::Painter,
        screen_rect: egui::Rect,
        center: GeoCoord,
        facing_deg: f64,      // Direction radar faces (0 = North, 90 = East)
        coverage_deg: f64,    // Width of coverage arc
        range_km: f64,
        fill_color: egui::Color32,
        stroke: egui::Stroke,
    ) {
        // Get all wrapped positions for this location
        let center_positions = self.viewport.geo_to_screen_wrapped(center, screen_rect);

        // Convert range from km to degrees
        let range_deg = range_km / 111.32;

        // Calculate start and end angles for the sector
        let half_coverage = coverage_deg / 2.0;
        let start_bearing = facing_deg - half_coverage;
        let end_bearing = facing_deg + half_coverage;

        // Generate arc points
        let segments = 36;

        for center_screen in center_positions {
            let mut arc_points: Vec<egui::Pos2> = Vec::new();

            for i in 0..=segments {
                let t = i as f64 / segments as f64;
                let bearing = start_bearing + (end_bearing - start_bearing) * t;
                let angle = (90.0 - bearing).to_radians(); // Convert from geographic to math angle

                // Calculate point at range from center
                let lat_offset = range_deg * angle.sin();
                let lon_offset = range_deg * angle.cos() / center.lat.to_radians().cos().max(0.1);

                let point = GeoCoord::new(
                    (center.lat + lat_offset).clamp(-90.0, 90.0),
                    center.lon + lon_offset,
                );

                let screen_pos = self.viewport.geo_to_screen(point, screen_rect);
                arc_points.push(screen_pos);
            }

            // Draw sector outline only (no fill)
            if arc_points.len() >= 2 {
                // Draw arc outline
                painter.add(egui::Shape::line(arc_points.clone(), stroke));

                // Draw radial lines from center to arc endpoints
                if !arc_points.is_empty() {
                    painter.line_segment([center_screen, arc_points[0]], stroke);
                    painter.line_segment([center_screen, *arc_points.last().unwrap()], stroke);
                }
            }
        }
    }

    /// Render tracking lines from sensors to detected targets on the globe
    fn render_tracking_lines_globe(&self, painter: &egui::Painter, screen_center: egui::Pos2) {
        for detection in &self.simulation.detection.active_detections {
            // Find the sensor position - check both unit.id and unit.sensors[].sensor_id
            let sensor_data = self
                .simulation
                .defense_units
                .iter()
                .find(|u| u.id == detection.sensor_id || u.sensors.iter().any(|s| s.sensor_id == detection.sensor_id))
                .map(|u| (u.position, u.affiliation))
                .or_else(|| {
                    self.simulation
                        .radar_stations
                        .iter()
                        .find(|s| s.id == detection.sensor_id)
                        .map(|s| (s.position, s.affiliation))
                })
                .or_else(|| {
                    self.simulation
                        .satellites
                        .iter()
                        .find(|s| s.id == detection.sensor_id)
                        .map(|s| (s.position, s.affiliation))
                });

            // Find the target position
            let target_pos = self
                .simulation
                .missiles
                .iter()
                .find(|m| m.id == detection.target_id)
                .map(|m| m.position);

            if let (Some((sensor_geo, affiliation)), Some(target_geo)) = (sensor_data, target_pos) {
                // Get screen positions
                let sensor_screen = self.globe_state.geo_to_screen(sensor_geo, screen_center);
                let target_screen = self.globe_state.geo_to_screen(target_geo, screen_center);

                if let (Some(from), Some(to)) = (sensor_screen, target_screen) {
                    let color = match affiliation {
                        Affiliation::Friendly => {
                            let alpha = (detection.detection_quality * 200.0) as u8;
                            egui::Color32::from_rgba_unmultiplied(100, 200, 255, alpha)
                        }
                        Affiliation::Hostile => {
                            let alpha = (detection.detection_quality * 200.0) as u8;
                            egui::Color32::from_rgba_unmultiplied(255, 100, 100, alpha)
                        }
                        Affiliation::Neutral => {
                            let alpha = (detection.detection_quality * 200.0) as u8;
                            egui::Color32::from_rgba_unmultiplied(200, 200, 100, alpha)
                        }
                    };

                    // Draw dashed tracking line
                    let stroke = egui::Stroke::new(1.0, color);
                    painter.line_segment([from, to], stroke);
                }
            }
        }
    }

    /// Render a detected missile on the globe view
    fn render_detected_missile_globe(
        &self,
        painter: &egui::Painter,
        screen_center: egui::Pos2,
        fused_track: &FusedTrack,
    ) {
        // Find the actual missile to get its position
        let missile = self
            .simulation
            .missiles
            .iter()
            .find(|m| m.id == fused_track.target_id);

        let Some(missile) = missile else {
            return; // Can't render without the missile
        };

        if let Some(pos) = self
            .globe_state
            .geo_to_screen(missile.position, screen_center)
        {
            // Convert uncertainty to screen pixels (approximate)
            // On globe, 1 degree ≈ globe_radius * (π/180) pixels
            let deg_per_pixel = 180.0 / (std::f64::consts::PI * self.globe_state.radius as f64);
            let uncertainty_deg = fused_track.uncertainty_radius_km / 111.32;
            let uncertainty_pixels = (uncertainty_deg / deg_per_pixel).max(8.0) as f32;

            // Draw uncertainty ellipse
            DetectionOverlays::draw_uncertainty_ellipse(
                painter,
                pos,
                uncertainty_pixels,
                fused_track.fused_quality,
            );

            let heading = bearing(missile.position, missile.target);

            // Fire control lock only when defense unit radar is tracking
            // AND track quality requirements are met
            let fire_control_locked = fused_track.has_fire_control_lock
                && fused_track.measurement_count >= 3
                && fused_track.fused_quality >= 0.4
                && fused_track.staleness_seconds <= 5.0;

            // Draw missile symbol
            MilitarySymbols::draw_missile(
                painter,
                pos,
                Affiliation::Hostile,
                MissileStatus::Midcourse,
                ((heading - 90.0) as f32).to_radians(),
                8.0,
                Some(fused_track.fused_quality), // Pass track quality for transparency
                fire_control_locked,
            );

            // Draw sensor count badge
            self.draw_track_info_badge(painter, pos, fused_track);
        }
    }

    /// Render false alarms on the globe view
    fn render_false_alarms_globe(&self, painter: &egui::Painter, screen_center: egui::Pos2) {
        for detection in &self.simulation.detection.active_detections {
            if !detection.is_false_alarm {
                continue;
            }

            // Get sensor position to calculate false alarm location
            if let Some(sensor_pos) = self.get_sensor_position(detection.sensor_id) {
                let false_alarm_pos = calculate_position_from_bearing_range(
                    sensor_pos,
                    detection.bearing_deg,
                    detection.range_km,
                );

                if let Some(pos) = self.globe_state.geo_to_screen(false_alarm_pos, screen_center) {
                    DetectionOverlays::draw_false_alarm_marker(
                        painter,
                        pos,
                        detection.detection_quality,
                    );
                }
            }
        }
    }

    fn find_entity_at_globe(&self, pos: egui::Pos2, screen_center: egui::Pos2) -> Option<Selection> {
        let click_radius = 15.0;

        // Check missiles
        for missile in &self.simulation.missiles {
            if let Some(entity_pos) = self.globe_state.geo_to_screen(missile.position, screen_center) {
                if pos.distance(entity_pos) < click_radius {
                    return Some(Selection::Missile(missile.id));
                }
            }
        }

        // Check defense units
        for unit in &self.simulation.defense_units {
            if let Some(entity_pos) = self.globe_state.geo_to_screen(unit.position, screen_center) {
                if pos.distance(entity_pos) < click_radius {
                    return Some(Selection::DefenseUnit(unit.id));
                }
            }
        }

        // Check satellites
        for satellite in &self.simulation.satellites {
            if let Some(entity_pos) = self.globe_state.geo_to_screen(satellite.position, screen_center) {
                if pos.distance(entity_pos) < click_radius {
                    return Some(Selection::Satellite(satellite.id));
                }
            }
        }

        None
    }

    /// Render tooltip when hovering over an entity
    fn render_hover_tooltip(&self, ui: &mut egui::Ui, hover_pos: egui::Pos2, screen_rect: egui::Rect) {
        use crate::simulation::MissileStatus;
        let hover_radius = 20.0;

        // Check missiles
        for missile in &self.simulation.missiles {
            let entity_pos = self.viewport.geo_to_screen(missile.position, screen_rect);
            if hover_pos.distance(entity_pos) < hover_radius {
                let tti = if matches!(missile.status, MissileStatus::Boost | MissileStatus::Midcourse | MissileStatus::Terminal) {
                    let remaining = missile.flight_time - missile.current_flight_time;
                    format!("TTI: {:.0}s", remaining.max(0.0))
                } else {
                    format!("{:?}", missile.status)
                };

                egui::Area::new(egui::Id::new("entity_tooltip"))
                    .fixed_pos(hover_pos + egui::vec2(15.0, 15.0))
                    .order(egui::Order::Tooltip)
                    .show(ui.ctx(), |ui| {
                        egui::Frame::popup(ui.style()).show(ui, |ui| {
                            ui.label(egui::RichText::new(&missile.name).strong());
                            ui.label(format!("Alt: {:.0} km", missile.altitude_km));
                            ui.label(tti);
                            if missile.has_countermeasures {
                                let decoy_eff = (1.0 - missile.decoy_effectiveness()) * 100.0;
                                ui.label(format!(
                                    "Decoys: {}/{} (-{:.0}% P(hit))",
                                    missile.decoys_deployed,
                                    missile.max_decoys,
                                    decoy_eff
                                ));
                            }
                        });
                    });
                return;
            }
        }

        // Check defense units
        for unit in &self.simulation.defense_units {
            let entity_pos = self.viewport.geo_to_screen(unit.position, screen_rect);
            if hover_pos.distance(entity_pos) < hover_radius {
                egui::Area::new(egui::Id::new("entity_tooltip"))
                    .fixed_pos(hover_pos + egui::vec2(15.0, 15.0))
                    .order(egui::Order::Tooltip)
                    .show(ui.ctx(), |ui| {
                        egui::Frame::popup(ui.style()).show(ui, |ui| {
                            ui.label(egui::RichText::new(&unit.name).strong());
                            ui.label(format!("Type: {}", unit.defense_type.name()));
                            ui.label(format!("Status: {:?}", unit.status));
                            ui.label(format!("Interceptors: {}", unit.interceptors_remaining));
                        });
                    });
                return;
            }
        }

        // Check interceptors
        for interceptor in &self.simulation.interceptors {
            let entity_pos = self.viewport.geo_to_screen(interceptor.position, screen_rect);
            if hover_pos.distance(entity_pos) < hover_radius {
                let target_name = self.simulation.missiles
                    .iter()
                    .find(|m| m.id == interceptor.target_id)
                    .map(|m| m.name.clone())
                    .unwrap_or_else(|| "Unknown".to_string());

                egui::Area::new(egui::Id::new("entity_tooltip"))
                    .fixed_pos(hover_pos + egui::vec2(15.0, 15.0))
                    .order(egui::Order::Tooltip)
                    .show(ui.ctx(), |ui| {
                        egui::Frame::popup(ui.style()).show(ui, |ui| {
                            ui.label(egui::RichText::new("Interceptor").strong());
                            ui.label(format!("Target: {}", target_name));
                            ui.label(format!("Status: {:?}", interceptor.status));
                            ui.label(format!("P(hit): {:.0}%", interceptor.hit_probability * 100.0));
                        });
                    });
                return;
            }
        }

        // Check radar stations
        for station in &self.simulation.radar_stations {
            let entity_pos = self.viewport.geo_to_screen(station.position, screen_rect);
            if hover_pos.distance(entity_pos) < hover_radius {
                egui::Area::new(egui::Id::new("entity_tooltip"))
                    .fixed_pos(hover_pos + egui::vec2(15.0, 15.0))
                    .order(egui::Order::Tooltip)
                    .show(ui.ctx(), |ui| {
                        egui::Frame::popup(ui.style()).show(ui, |ui| {
                            ui.label(egui::RichText::new(&station.name).strong());
                            ui.label(format!("Type: {:?}", station.sensor_type));
                            ui.label(format!("Range: {:.0} km", station.detection_range_km));
                        });
                    });
                return;
            }
        }

        // Check satellites
        for satellite in &self.simulation.satellites {
            let entity_pos = self.viewport.geo_to_screen(satellite.position, screen_rect);
            if hover_pos.distance(entity_pos) < hover_radius {
                egui::Area::new(egui::Id::new("entity_tooltip"))
                    .fixed_pos(hover_pos + egui::vec2(15.0, 15.0))
                    .order(egui::Order::Tooltip)
                    .show(ui.ctx(), |ui| {
                        egui::Frame::popup(ui.style()).show(ui, |ui| {
                            ui.label(egui::RichText::new(&satellite.name).strong());
                            ui.label(format!("Type: {:?}", satellite.sensor_type));
                            ui.label(format!("Coverage: {:.0} km", satellite.coverage_radius_km()));
                        });
                    });
                return;
            }
        }
    }

    fn render_entities(&self, painter: &egui::Painter, screen_rect: egui::Rect) {
        // Draw detection ranges first (below entities)
        if self.show_detection_ranges {
            self.render_detection_ranges(painter, screen_rect);
        }

        // Draw tracking lines
        if self.show_tracking_lines {
            self.render_tracking_lines(painter, screen_rect);
        }

        // Draw trajectories
        if self.show_trajectories {
            self.render_trajectories(painter, screen_rect);
        }

        // Draw defense units
        for unit in &self.simulation.defense_units {
            self.render_defense_unit(painter, screen_rect, unit);
        }

        // Draw radar stations
        for station in &self.simulation.radar_stations {
            self.render_radar_station(painter, screen_rect, station);
        }

        // Draw satellites
        for satellite in &self.simulation.satellites {
            self.render_satellite(painter, screen_rect, satellite);
        }

        // Draw missiles - check track view mode
        match self.track_view_mode {
            TrackViewMode::TrueTrack => {
                // Show actual missile positions
                for missile in &self.simulation.missiles {
                    self.render_missile(painter, screen_rect, missile);
                }
            }
            TrackViewMode::DetectedTrack => {
                // Show missiles at sensor-perceived positions with uncertainty
                let defense_unit_ids: std::collections::HashSet<u64> = self.simulation.defense_units.iter().map(|u| u.id).collect();
                let fused_tracks = self.simulation.detection.get_all_fused_tracks(&defense_unit_ids);
                for track in &fused_tracks {
                    self.render_detected_missile(painter, screen_rect, &track);
                }

                // Render false alarms if enabled
                if self.show_false_alarms {
                    self.render_false_alarms(painter, screen_rect);
                }
            }
        }

        // Draw interceptors
        for interceptor in &self.simulation.interceptors {
            self.render_interceptor(painter, screen_rect, interceptor);
        }

        // Draw visual effects (explosions, etc.)
        self.render_visual_effects(painter, screen_rect);
    }

    fn render_detection_ranges(&self, painter: &egui::Painter, screen_rect: egui::Rect) {
        // Defense unit detection ranges
        for unit in &self.simulation.defense_units {
            let positions = self.viewport.geo_to_screen_wrapped(unit.position, screen_rect);

            // Show actual max detection range (1.5× nominal for probabilistic detection)
            let sensor_config = self.simulation.sensor_configs.get_by_name(&unit.sensor_config_name);
            let max_range_km = sensor_config.detection.detection_range_km * 1.5;

            // Convert km to screen pixels (approximate)
            let range_deg = max_range_km / 111.32;
            let base_center = self.viewport.geo_to_screen(unit.position, screen_rect);
            let edge_pos = GeoCoord::new(
                unit.position.lat + range_deg,
                unit.position.lon,
            );
            let edge_screen = self.viewport.geo_to_screen(edge_pos, screen_rect);
            let radius = (base_center.y - edge_screen.y).abs();

            let stroke_color = match unit.affiliation {
                Affiliation::Friendly => egui::Color32::from_rgb(80, 180, 255),
                Affiliation::Hostile => egui::Color32::from_rgb(255, 80, 80),
                Affiliation::Neutral => egui::Color32::from_rgb(100, 255, 100),
            };

            for center in positions {
                painter.circle_stroke(
                    center,
                    radius,
                    egui::Stroke::new(1.5, stroke_color),
                );
            }
        }

        // Radar station detection ranges
        for station in &self.simulation.radar_stations {
            let config = self.simulation.sensor_configs.get_by_name(&station.sensor_config_name);
            let base_range = station.detection_range_km;

            // Check if this is a phased array radar
            if config.detection.radar_type == RadarType::PhasedArray {
                // Draw three mode-specific ranges for phased arrays with mode-specific azimuth coverage

                // Search range (outermost, cyan/green) - drawn first so it's behind
                let search_range = base_range * config.detection.get_range_multiplier(RadarMode::Search) * 1.5;
                let search_coverage = config.detection.get_effective_azimuth_coverage(RadarMode::Search);

                if search_coverage >= 360.0 {
                    // Full circle for omnidirectional
                    let search_color = match station.affiliation {
                        Affiliation::Friendly => egui::Color32::from_rgba_unmultiplied(100, 220, 255, 70),
                        Affiliation::Hostile => egui::Color32::from_rgba_unmultiplied(255, 150, 100, 70),
                        Affiliation::Neutral => egui::Color32::from_rgba_unmultiplied(150, 255, 150, 70),
                    };
                    let range_deg = search_range / 111.32;
                    let base_center = self.viewport.geo_to_screen(station.position, screen_rect);
                    let edge_pos = GeoCoord::new(station.position.lat + range_deg, station.position.lon);
                    let edge_screen = self.viewport.geo_to_screen(edge_pos, screen_rect);
                    let radius = (base_center.y - edge_screen.y).abs();
                    for center in self.viewport.geo_to_screen_wrapped(station.position, screen_rect) {
                        painter.circle_stroke(center, radius, egui::Stroke::new(1.0, search_color));
                    }
                } else {
                    // Sector for directional radars
                    let search_fill = match station.affiliation {
                        Affiliation::Friendly => egui::Color32::from_rgba_unmultiplied(100, 220, 255, 25),
                        Affiliation::Hostile => egui::Color32::from_rgba_unmultiplied(255, 150, 100, 25),
                        Affiliation::Neutral => egui::Color32::from_rgba_unmultiplied(150, 255, 150, 25),
                    };
                    let search_stroke = match station.affiliation {
                        Affiliation::Friendly => egui::Color32::from_rgba_unmultiplied(100, 220, 255, 70),
                        Affiliation::Hostile => egui::Color32::from_rgba_unmultiplied(255, 150, 100, 70),
                        Affiliation::Neutral => egui::Color32::from_rgba_unmultiplied(150, 255, 150, 70),
                    };
                    self.draw_range_sector_2d(
                        painter,
                        screen_rect,
                        station.position,
                        station.facing_deg,
                        search_coverage,
                        search_range,
                        search_fill,
                        egui::Stroke::new(1.0, search_stroke),
                    );
                }

                // Track range (middle, yellow)
                let track_range = base_range * config.detection.get_range_multiplier(RadarMode::Track) * 1.5;
                let track_coverage = config.detection.get_effective_azimuth_coverage(RadarMode::Track);

                if track_coverage >= 360.0 {
                    let track_color = match station.affiliation {
                        Affiliation::Friendly => egui::Color32::from_rgba_unmultiplied(255, 255, 100, 90),
                        Affiliation::Hostile => egui::Color32::from_rgba_unmultiplied(255, 200, 80, 90),
                        Affiliation::Neutral => egui::Color32::from_rgba_unmultiplied(200, 255, 100, 90),
                    };
                    let range_deg = track_range / 111.32;
                    let base_center = self.viewport.geo_to_screen(station.position, screen_rect);
                    let edge_pos = GeoCoord::new(station.position.lat + range_deg, station.position.lon);
                    let edge_screen = self.viewport.geo_to_screen(edge_pos, screen_rect);
                    let radius = (base_center.y - edge_screen.y).abs();
                    for center in self.viewport.geo_to_screen_wrapped(station.position, screen_rect) {
                        painter.circle_stroke(center, radius, egui::Stroke::new(1.5, track_color));
                    }
                } else {
                    let track_fill = match station.affiliation {
                        Affiliation::Friendly => egui::Color32::from_rgba_unmultiplied(255, 255, 100, 35),
                        Affiliation::Hostile => egui::Color32::from_rgba_unmultiplied(255, 200, 80, 35),
                        Affiliation::Neutral => egui::Color32::from_rgba_unmultiplied(200, 255, 100, 35),
                    };
                    let track_stroke = match station.affiliation {
                        Affiliation::Friendly => egui::Color32::from_rgba_unmultiplied(255, 255, 100, 90),
                        Affiliation::Hostile => egui::Color32::from_rgba_unmultiplied(255, 200, 80, 90),
                        Affiliation::Neutral => egui::Color32::from_rgba_unmultiplied(200, 255, 100, 90),
                    };
                    self.draw_range_sector_2d(
                        painter,
                        screen_rect,
                        station.position,
                        station.facing_deg,
                        track_coverage,
                        track_range,
                        track_fill,
                        egui::Stroke::new(1.5, track_stroke),
                    );
                }

                // FireControl range (innermost, red/orange) - drawn last so it's on top
                let fc_range = base_range * config.detection.get_range_multiplier(RadarMode::FireControl) * 1.5;
                let fc_coverage = config.detection.get_effective_azimuth_coverage(RadarMode::FireControl);

                if fc_coverage >= 360.0 {
                    let fc_color = match station.affiliation {
                        Affiliation::Friendly => egui::Color32::from_rgba_unmultiplied(255, 150, 50, 120),
                        Affiliation::Hostile => egui::Color32::from_rgba_unmultiplied(255, 50, 50, 120),
                        Affiliation::Neutral => egui::Color32::from_rgba_unmultiplied(255, 200, 50, 120),
                    };
                    let range_deg = fc_range / 111.32;
                    let base_center = self.viewport.geo_to_screen(station.position, screen_rect);
                    let edge_pos = GeoCoord::new(station.position.lat + range_deg, station.position.lon);
                    let edge_screen = self.viewport.geo_to_screen(edge_pos, screen_rect);
                    let radius = (base_center.y - edge_screen.y).abs();
                    for center in self.viewport.geo_to_screen_wrapped(station.position, screen_rect) {
                        painter.circle_stroke(center, radius, egui::Stroke::new(2.0, fc_color));
                    }
                } else {
                    let fc_fill = match station.affiliation {
                        Affiliation::Friendly => egui::Color32::from_rgba_unmultiplied(255, 150, 50, 45),
                        Affiliation::Hostile => egui::Color32::from_rgba_unmultiplied(255, 50, 50, 45),
                        Affiliation::Neutral => egui::Color32::from_rgba_unmultiplied(255, 200, 50, 45),
                    };
                    let fc_stroke = match station.affiliation {
                        Affiliation::Friendly => egui::Color32::from_rgba_unmultiplied(255, 150, 50, 120),
                        Affiliation::Hostile => egui::Color32::from_rgba_unmultiplied(255, 50, 50, 120),
                        Affiliation::Neutral => egui::Color32::from_rgba_unmultiplied(255, 200, 50, 120),
                    };
                    self.draw_range_sector_2d(
                        painter,
                        screen_rect,
                        station.position,
                        station.facing_deg,
                        fc_coverage,
                        fc_range,
                        fc_fill,
                        egui::Stroke::new(2.0, fc_stroke),
                    );
                }
            } else {
                // Mechanical radar - single range circle
                let max_range_km = base_range * 1.5;
                let range_deg = max_range_km / 111.32;

                let base_center = self.viewport.geo_to_screen(station.position, screen_rect);
                let edge_pos = GeoCoord::new(station.position.lat + range_deg, station.position.lon);
                let edge_screen = self.viewport.geo_to_screen(edge_pos, screen_rect);
                let radius = (base_center.y - edge_screen.y).abs();

                let stroke_color = match station.affiliation {
                    Affiliation::Friendly => egui::Color32::from_rgb(100, 220, 150),
                    Affiliation::Hostile => egui::Color32::from_rgb(255, 150, 50),
                    Affiliation::Neutral => egui::Color32::from_rgb(220, 220, 100),
                };

                for center in self.viewport.geo_to_screen_wrapped(station.position, screen_rect) {
                    painter.circle_stroke(center, radius, egui::Stroke::new(1.5, stroke_color));
                }
            }
        }
    }

    fn render_tracking_lines(&self, painter: &egui::Painter, screen_rect: egui::Rect) {
        // Draw lines from sensors to detected targets
        for detection in &self.simulation.detection.active_detections {
            // Find the sensor position - check both unit.id and unit.sensors[].sensor_id
            let sensor_pos = self
                .simulation
                .defense_units
                .iter()
                .find(|u| u.id == detection.sensor_id || u.sensors.iter().any(|s| s.sensor_id == detection.sensor_id))
                .map(|u| (u.position, u.affiliation))
                .or_else(|| {
                    self.simulation
                        .radar_stations
                        .iter()
                        .find(|s| s.id == detection.sensor_id)
                        .map(|s| (s.position, s.affiliation))
                })
                .or_else(|| {
                    self.simulation
                        .satellites
                        .iter()
                        .find(|s| s.id == detection.sensor_id)
                        .map(|s| (s.position, s.affiliation))
                });

            // Find the target position
            let target_pos = self
                .simulation
                .missiles
                .iter()
                .find(|m| m.id == detection.target_id)
                .map(|m| m.position);

            if let (Some((sensor_geo, affiliation)), Some(target_geo)) = (sensor_pos, target_pos) {
                DetectionOverlays::draw_tracking_line(
                    painter,
                    &self.viewport,
                    screen_rect,
                    sensor_geo,
                    target_geo,
                    affiliation,
                    detection.detection_quality,
                );
            }
        }
    }

    fn render_trajectories(&self, painter: &egui::Painter, screen_rect: egui::Rect) {
        for missile in &self.simulation.missiles {
            if matches!(
                missile.status,
                MissileStatus::Boost | MissileStatus::Midcourse | MissileStatus::Terminal
            ) {
                let trajectory = BallisticTrajectory::new(missile.origin, missile.target);
                let progress = missile.flight_progress();
                let num_points = 100;

                // Collect trajectory data with altitude
                let mut trajectory_data: Vec<(GeoCoord, f64, f64)> = Vec::new(); // (pos, alt, t)
                for i in 0..=num_points {
                    let t = i as f64 / num_points as f64;
                    let (pos, alt) = trajectory.position_at(t);
                    trajectory_data.push((pos, alt, t));
                }

                // Draw past path (traveled portion) - dimmer
                let past_points: Vec<GeoCoord> = trajectory_data
                    .iter()
                    .filter(|(_, _, t)| *t <= progress)
                    .map(|(pos, _, _)| *pos)
                    .collect();

                if past_points.len() >= 2 {
                    let past_segments = self.geo_path_to_screen_segments(&past_points, screen_rect);
                    for segment in past_segments {
                        if segment.len() >= 2 {
                            painter.add(egui::Shape::line(
                                segment,
                                egui::Stroke::new(3.0, egui::Color32::from_rgb(180, 80, 40)),
                            ));
                        }
                    }
                }

                // Draw future path (projected portion) - brighter, dashed appearance
                let future_points: Vec<GeoCoord> = trajectory_data
                    .iter()
                    .filter(|(_, _, t)| *t >= progress)
                    .map(|(pos, _, _)| *pos)
                    .collect();

                if future_points.len() >= 2 {
                    let future_segments = self.geo_path_to_screen_segments(&future_points, screen_rect);
                    for segment in future_segments {
                        if segment.len() >= 2 {
                            painter.add(egui::Shape::line(
                                segment,
                                egui::Stroke::new(2.0, egui::Color32::from_rgb(255, 150, 100)),
                            ));
                        }
                    }
                }

                // Draw time markers along future path (every 5 minutes of flight time)
                let time_interval_sec = 300.0; // 5 minutes
                let total_flight_time = missile.flight_time;
                let current_time = missile.current_flight_time;

                let mut marker_time = ((current_time / time_interval_sec).ceil() * time_interval_sec) as f64;
                while marker_time < total_flight_time {
                    let marker_progress = marker_time / total_flight_time;
                    let (marker_pos, marker_alt) = trajectory.position_at(marker_progress);
                    let marker_screen = self.viewport.geo_to_screen(marker_pos, screen_rect);

                    // Circle marker
                    painter.circle_filled(
                        marker_screen,
                        5.0,
                        egui::Color32::from_rgb(255, 200, 100),
                    );
                    painter.circle_stroke(
                        marker_screen,
                        5.0,
                        egui::Stroke::new(1.5, egui::Color32::BLACK),
                    );

                    // Combined time and altitude label
                    let minutes_remaining = ((total_flight_time - marker_time) / 60.0) as i32;
                    if minutes_remaining > 0 {
                        let label = format!("T-{}m  {:.0}km", minutes_remaining, marker_alt);
                        let label_pos = egui::pos2(marker_screen.x + 8.0, marker_screen.y);

                        // Draw background box for legibility
                        let font = egui::FontId::proportional(11.0);
                        let galley = painter.layout_no_wrap(
                            label.clone(),
                            font.clone(),
                            egui::Color32::WHITE,
                        );
                        let text_rect = egui::Rect::from_min_size(
                            egui::pos2(label_pos.x - 2.0, label_pos.y - galley.size().y / 2.0 - 2.0),
                            egui::vec2(galley.size().x + 4.0, galley.size().y + 4.0),
                        );
                        painter.rect_filled(
                            text_rect,
                            3.0,
                            egui::Color32::from_rgba_unmultiplied(0, 0, 0, 180),
                        );

                        // Draw text
                        painter.text(
                            label_pos,
                            egui::Align2::LEFT_CENTER,
                            label,
                            font,
                            egui::Color32::from_rgb(255, 220, 150),
                        );
                    }

                    marker_time += time_interval_sec;
                }

                // Draw apogee marker (highest point)
                let (apogee_pos, apogee_alt) = trajectory.position_at(0.5);
                if progress < 0.5 {
                    let apogee_screen = self.viewport.geo_to_screen(apogee_pos, screen_rect);

                    // Apogee circle
                    painter.circle_filled(
                        apogee_screen,
                        7.0,
                        egui::Color32::from_rgba_unmultiplied(150, 150, 255, 100),
                    );
                    painter.circle_stroke(
                        apogee_screen,
                        7.0,
                        egui::Stroke::new(2.0, egui::Color32::from_rgb(180, 180, 255)),
                    );

                    // Apogee label with background
                    let apogee_label = format!("APOGEE {:.0}km", apogee_alt);
                    let font = egui::FontId::proportional(11.0);
                    let galley = painter.layout_no_wrap(
                        apogee_label.clone(),
                        font.clone(),
                        egui::Color32::WHITE,
                    );
                    let label_pos = egui::pos2(apogee_screen.x, apogee_screen.y - 12.0);
                    let text_rect = egui::Rect::from_min_size(
                        egui::pos2(
                            label_pos.x - galley.size().x / 2.0 - 3.0,
                            label_pos.y - galley.size().y - 3.0,
                        ),
                        egui::vec2(galley.size().x + 6.0, galley.size().y + 4.0),
                    );
                    painter.rect_filled(
                        text_rect,
                        3.0,
                        egui::Color32::from_rgba_unmultiplied(40, 40, 80, 200),
                    );
                    painter.text(
                        label_pos,
                        egui::Align2::CENTER_BOTTOM,
                        apogee_label,
                        font,
                        egui::Color32::from_rgb(200, 200, 255),
                    );
                }

                // Draw intercept windows - where defense units could engage
                self.render_intercept_windows(painter, screen_rect, missile, &trajectory);

                // Draw impact point marker at all wrapped positions
                for target_screen in self.viewport.geo_to_screen_wrapped(missile.target, screen_rect) {
                    MilitarySymbols::draw_target_designator(painter, target_screen, 10.0);

                    // Time to impact with background
                    let tti = (missile.flight_time - missile.current_flight_time).max(0.0);
                    let minutes = (tti / 60.0) as i32;
                    let seconds = (tti % 60.0) as i32;
                    let tti_label = format!("TTI {:02}:{:02}", minutes, seconds);
                    let font = egui::FontId::proportional(12.0);
                    let galley = painter.layout_no_wrap(
                        tti_label.clone(),
                        font.clone(),
                        egui::Color32::WHITE,
                    );
                    let label_pos = egui::pos2(target_screen.x, target_screen.y + 16.0);
                    let text_rect = egui::Rect::from_min_size(
                        egui::pos2(
                            label_pos.x - galley.size().x / 2.0 - 4.0,
                            label_pos.y - 2.0,
                        ),
                        egui::vec2(galley.size().x + 8.0, galley.size().y + 4.0),
                    );
                    painter.rect_filled(
                        text_rect,
                        3.0,
                        egui::Color32::from_rgba_unmultiplied(80, 0, 0, 220),
                    );
                    painter.rect_stroke(
                        text_rect,
                        3.0,
                        egui::Stroke::new(1.0, egui::Color32::from_rgb(255, 100, 100)),
                        egui::StrokeKind::Outside,
                    );
                    painter.text(
                        label_pos,
                        egui::Align2::CENTER_TOP,
                        tti_label,
                        font,
                        egui::Color32::from_rgb(255, 150, 150),
                    );
                }
            }
        }
    }

    /// Render intercept windows along a missile trajectory
    fn render_intercept_windows(
        &self,
        painter: &egui::Painter,
        screen_rect: egui::Rect,
        missile: &Missile,
        trajectory: &BallisticTrajectory,
    ) {
        use crate::simulation::haversine_distance;

        // Check each defense unit for potential intercept windows
        for unit in &self.simulation.defense_units {
            if unit.affiliation == missile.affiliation {
                continue; // Can't intercept friendly missiles
            }

            let platform_config = self.simulation.platform_configs.get_by_defense_type(unit.defense_type);
            let engagement_range = platform_config.launcher.engagement_range_km;

            // Sample along the future trajectory to find intercept windows
            let progress = missile.flight_progress();
            let mut in_range = false;
            let mut window_start_t = 0.0;

            for i in 0..=50 {
                let t = progress + (1.0 - progress) * (i as f64 / 50.0);
                let (pos, alt) = trajectory.position_at(t);
                let dist = haversine_distance(unit.position, pos);

                // Check if within engagement envelope (simplified)
                let in_envelope = dist < engagement_range && alt > 20.0; // Min 20km altitude for intercept

                if in_envelope && !in_range {
                    // Entering intercept window
                    in_range = true;
                    window_start_t = t;
                } else if !in_envelope && in_range {
                    // Exiting intercept window - draw it
                    self.draw_intercept_window(
                        painter,
                        screen_rect,
                        trajectory,
                        window_start_t,
                        t,
                        unit.defense_type.name(),
                    );
                    in_range = false;
                }
            }

            // If still in range at end, draw the window
            if in_range {
                self.draw_intercept_window(
                    painter,
                    screen_rect,
                    trajectory,
                    window_start_t,
                    1.0,
                    unit.defense_type.name(),
                );
            }
        }
    }

    /// Draw an intercept window segment on the trajectory
    fn draw_intercept_window(
        &self,
        painter: &egui::Painter,
        screen_rect: egui::Rect,
        trajectory: &BallisticTrajectory,
        start_t: f64,
        end_t: f64,
        unit_name: &str,
    ) {
        // Collect points in the intercept window
        let num_points = 20;
        let mut points = Vec::new();

        for i in 0..=num_points {
            let t = start_t + (end_t - start_t) * (i as f64 / num_points as f64);
            let (pos, _) = trajectory.position_at(t);
            points.push(pos);
        }

        // Convert to screen coordinates
        let segments = self.geo_path_to_screen_segments(&points, screen_rect);

        // Draw highlighted intercept window
        for segment in segments {
            if segment.len() >= 2 {
                // Thick green line for intercept window
                painter.add(egui::Shape::line(
                    segment.clone(),
                    egui::Stroke::new(6.0, egui::Color32::from_rgba_unmultiplied(100, 255, 100, 100)),
                ));
            }
        }

        // Label at midpoint
        let mid_t = (start_t + end_t) / 2.0;
        let (mid_pos, _) = trajectory.position_at(mid_t);
        let mid_screen = self.viewport.geo_to_screen(mid_pos, screen_rect);

        painter.text(
            egui::pos2(mid_screen.x, mid_screen.y - 8.0),
            egui::Align2::CENTER_BOTTOM,
            unit_name,
            egui::FontId::proportional(8.0),
            egui::Color32::from_rgb(100, 255, 100),
        );
    }

    /// Convert a path of geo coordinates to screen segments, splitting at large jumps
    fn geo_path_to_screen_segments(&self, geo_points: &[GeoCoord], screen_rect: egui::Rect) -> Vec<Vec<egui::Pos2>> {
        if geo_points.is_empty() {
            return vec![];
        }

        let mut segments = Vec::new();
        let mut current_segment = Vec::new();

        let max_jump = screen_rect.width() * 0.5; // If points are more than half screen apart, split

        for geo_point in geo_points {
            let screen_point = self.viewport.geo_to_screen(*geo_point, screen_rect);

            if let Some(last_point) = current_segment.last() {
                let distance = screen_point.distance(*last_point);
                if distance > max_jump {
                    // Large jump detected - probably date line crossing
                    if current_segment.len() >= 2 {
                        segments.push(current_segment);
                    }
                    current_segment = vec![screen_point];
                } else {
                    current_segment.push(screen_point);
                }
            } else {
                current_segment.push(screen_point);
            }
        }

        if current_segment.len() >= 2 {
            segments.push(current_segment);
        }

        segments
    }

    fn render_missile(&self, painter: &egui::Painter, screen_rect: egui::Rect, missile: &Missile) {
        let positions = self.viewport.geo_to_screen_wrapped(missile.position, screen_rect);

        // Calculate heading based on great circle bearing toward target
        let heading = if matches!(
            missile.status,
            MissileStatus::Boost | MissileStatus::Midcourse | MissileStatus::Terminal
        ) {
            // Get the great circle bearing from current position to target
            // bearing() returns degrees: 0=North, 90=East, 180=South, 270=West
            let bearing_deg = bearing(missile.position, missile.target);
            // Convert to screen angle: bearing 0 (North) -> -π/2 (up on screen)
            // Screen angle uses standard math convention where 0 = right, π/2 = down
            ((bearing_deg - 90.0) as f32).to_radians()
        } else {
            -std::f32::consts::FRAC_PI_2 // Point up by default
        };

        for pos in positions {
            // Draw reentry glow effect for terminal phase missiles
            if missile.status == MissileStatus::Terminal {
                self.render_reentry_glow(painter, pos, heading, missile);
            }

            // Draw smoke/contrail for in-flight missiles
            if matches!(missile.status, MissileStatus::Boost | MissileStatus::Midcourse | MissileStatus::Terminal) {
                self.render_missile_trail(painter, pos, heading, missile);
            }

            MilitarySymbols::draw_missile(
                painter,
                pos,
                missile.affiliation,
                missile.status,
                heading,
                10.0,
                None, // No track quality in true track mode
                false, // Not fire control locked in true track mode
            );
        }
    }

    /// Render a missile based on sensor detection (not true position)
    /// Shows uncertainty ellipse based on track quality
    fn render_detected_missile(
        &self,
        painter: &egui::Painter,
        screen_rect: egui::Rect,
        fused_track: &FusedTrack,
    ) {
        // Find the actual missile to get its position (using actual position since
        // predicted_position tracking isn't fully implemented yet)
        let missile = self
            .simulation
            .missiles
            .iter()
            .find(|m| m.id == fused_track.target_id);

        let Some(missile) = missile else {
            return; // Can't render without the missile
        };

        let positions = self
            .viewport
            .geo_to_screen_wrapped(missile.position, screen_rect);

        // Convert uncertainty from km to screen pixels
        let km_per_degree = 111.32;
        let uncertainty_deg = fused_track.uncertainty_radius_km / km_per_degree;
        let center_screen = self.viewport.geo_to_screen(missile.position, screen_rect);
        let edge_pos = GeoCoord::new(
            missile.position.lat + uncertainty_deg,
            missile.position.lon,
        );
        let edge_screen = self.viewport.geo_to_screen(edge_pos, screen_rect);
        let uncertainty_pixels = (center_screen.y - edge_screen.y).abs().max(8.0);

        let bearing_deg = bearing(missile.position, missile.target);
        let heading = ((bearing_deg - 90.0) as f32).to_radians();

        for pos in positions {
            // Draw uncertainty ellipse first (behind the symbol)
            DetectionOverlays::draw_uncertainty_ellipse(
                painter,
                pos,
                uncertainty_pixels,
                fused_track.fused_quality,
            );

            // Fire control lock only when defense unit radar is tracking
            // AND track quality requirements are met
            let fire_control_locked = fused_track.has_fire_control_lock
                && fused_track.measurement_count >= 3
                && fused_track.fused_quality >= 0.4
                && fused_track.staleness_seconds <= 5.0;

            // Draw missile symbol
            MilitarySymbols::draw_missile(
                painter,
                pos,
                Affiliation::Hostile, // Detected tracks are typically hostile
                MissileStatus::Midcourse, // Use midcourse style for detected
                heading,
                10.0,
                Some(fused_track.fused_quality), // Pass track quality for transparency
                fire_control_locked,
            );

            // Draw sensor count badge
            self.draw_track_info_badge(painter, pos, fused_track);
        }
    }

    /// Draw a small badge showing track info (sensor count, quality)
    fn draw_track_info_badge(
        &self,
        painter: &egui::Painter,
        pos: egui::Pos2,
        track: &FusedTrack,
    ) {
        let badge_pos = egui::Pos2::new(pos.x + 14.0, pos.y - 14.0);

        // Background pill (wider to fit sensor count + quality)
        let badge_size = egui::vec2(46.0, 14.0);
        painter.rect_filled(
            egui::Rect::from_center_size(badge_pos, badge_size),
            4.0,
            egui::Color32::from_rgba_unmultiplied(0, 0, 0, 200),
        );

        // Quality indicator color
        let quality_color = if track.fused_quality > 0.7 {
            egui::Color32::from_rgb(100, 255, 100) // Green
        } else if track.fused_quality > 0.4 {
            egui::Color32::from_rgb(255, 255, 100) // Yellow
        } else {
            egui::Color32::from_rgb(255, 150, 100) // Orange/red
        };

        // Text showing sensor count and quality percentage
        let quality_pct = (track.fused_quality * 100.0) as u8;
        let text = format!("{}S {}%", track.sensor_count, quality_pct);
        painter.text(
            badge_pos,
            egui::Align2::CENTER_CENTER,
            text,
            egui::FontId::proportional(9.0),
            quality_color,
        );
    }

    /// Render false alarms (clutter/noise detections)
    fn render_false_alarms(&self, painter: &egui::Painter, screen_rect: egui::Rect) {
        for detection in &self.simulation.detection.active_detections {
            if !detection.is_false_alarm {
                continue;
            }

            // Get sensor position to calculate false alarm location
            if let Some(sensor_pos) = self.get_sensor_position(detection.sensor_id) {
                let false_alarm_pos = calculate_position_from_bearing_range(
                    sensor_pos,
                    detection.bearing_deg,
                    detection.range_km,
                );

                let positions = self
                    .viewport
                    .geo_to_screen_wrapped(false_alarm_pos, screen_rect);

                for pos in positions {
                    DetectionOverlays::draw_false_alarm_marker(
                        painter,
                        pos,
                        detection.detection_quality,
                    );
                }
            }
        }
    }

    /// Get the position of a sensor by its ID
    fn get_sensor_position(&self, sensor_id: EntityId) -> Option<GeoCoord> {
        // Check defense units
        if let Some(unit) = self
            .simulation
            .defense_units
            .iter()
            .find(|u| u.id == sensor_id)
        {
            return Some(unit.position);
        }
        // Check radar stations
        if let Some(station) = self
            .simulation
            .radar_stations
            .iter()
            .find(|r| r.id == sensor_id)
        {
            return Some(station.position);
        }
        // Check satellites
        if let Some(sat) = self
            .simulation
            .satellites
            .iter()
            .find(|s| s.id == sensor_id)
        {
            return Some(sat.position);
        }
        None
    }

    /// Render reentry glow effect - plasma heating during atmospheric entry
    fn render_reentry_glow(&self, painter: &egui::Painter, pos: egui::Pos2, heading: f32, missile: &Missile) {
        // Calculate intensity based on altitude - more glow at lower altitudes
        // Reentry heating is strongest between 80km and 30km altitude
        let alt = missile.altitude_km;
        let glow_intensity = if alt > 80.0 {
            0.0
        } else if alt > 30.0 {
            (80.0 - alt) / 50.0 // 0.0 at 80km, 1.0 at 30km
        } else {
            1.0 - (30.0 - alt) / 30.0 // Decreases below 30km as speed reduces
        };

        if glow_intensity <= 0.0 {
            return;
        }

        // Create plasma glow layers
        let glow_intensity = (glow_intensity as f32).clamp(0.0, 1.0);

        // Outer orange/red glow (plasma sheath)
        let outer_glow_size = 20.0 + glow_intensity * 15.0;
        let outer_alpha = (glow_intensity * 120.0) as u8;
        painter.circle_filled(
            pos,
            outer_glow_size,
            egui::Color32::from_rgba_unmultiplied(255, 100, 30, outer_alpha),
        );

        // Middle yellow glow (hot gas)
        let mid_glow_size = 12.0 + glow_intensity * 8.0;
        let mid_alpha = (glow_intensity * 180.0) as u8;
        painter.circle_filled(
            pos,
            mid_glow_size,
            egui::Color32::from_rgba_unmultiplied(255, 200, 50, mid_alpha),
        );

        // Inner white-hot core
        let inner_glow_size = 6.0 + glow_intensity * 4.0;
        let inner_alpha = (glow_intensity * 220.0) as u8;
        painter.circle_filled(
            pos,
            inner_glow_size,
            egui::Color32::from_rgba_unmultiplied(255, 255, 200, inner_alpha),
        );

        // Plasma trail behind the RV
        let trail_length = 25.0 + glow_intensity * 20.0;
        let trail_dir_x = -heading.cos(); // Opposite to heading
        let trail_dir_y = -heading.sin();

        // Draw tapered plasma trail
        let num_trail_segments = 8;
        for i in 0..num_trail_segments {
            let t = i as f32 / num_trail_segments as f32;
            let segment_dist = trail_length * t;
            let segment_size = (1.0 - t) * 8.0 * glow_intensity;
            let segment_alpha = ((1.0 - t * t) * glow_intensity * 150.0) as u8;

            let segment_pos = egui::pos2(
                pos.x + trail_dir_x * segment_dist,
                pos.y + trail_dir_y * segment_dist,
            );

            // Gradient from white-yellow to orange-red
            let r = 255;
            let g = (255.0 - t * 155.0) as u8;
            let b = (200.0 - t * 170.0) as u8;

            painter.circle_filled(
                segment_pos,
                segment_size.max(1.0),
                egui::Color32::from_rgba_unmultiplied(r, g, b, segment_alpha),
            );
        }
    }

    /// Render smoke/contrail behind missile
    fn render_missile_trail(&self, painter: &egui::Painter, pos: egui::Pos2, heading: f32, missile: &Missile) {
        // Trail characteristics depend on phase
        let (trail_length, trail_width, trail_alpha) = match missile.status {
            MissileStatus::Boost => (40.0, 6.0, 180u8), // Thick rocket exhaust
            MissileStatus::Midcourse => (20.0, 2.0, 80u8), // Light trail in space (less visible)
            MissileStatus::Terminal => (30.0, 4.0, 120u8), // Ionization trail
            _ => return,
        };

        let trail_dir_x = -heading.cos();
        let trail_dir_y = -heading.sin();

        // Draw tapered trail
        let num_segments = 10;
        for i in 0..num_segments {
            let t = i as f32 / num_segments as f32;
            let segment_dist = trail_length * t + 8.0; // Start behind the missile icon
            let segment_width = trail_width * (1.0 - t * 0.7);
            let segment_alpha = (trail_alpha as f32 * (1.0 - t * t)) as u8;

            let segment_pos = egui::pos2(
                pos.x + trail_dir_x * segment_dist,
                pos.y + trail_dir_y * segment_dist,
            );

            let color = match missile.status {
                MissileStatus::Boost => {
                    // Rocket exhaust: orange-white gradient
                    let r = 255;
                    let g = (255.0 - t * 105.0) as u8;
                    let b = (200.0 - t * 150.0) as u8;
                    egui::Color32::from_rgba_unmultiplied(r, g, b, segment_alpha)
                }
                MissileStatus::Midcourse => {
                    // Faint white trail
                    egui::Color32::from_rgba_unmultiplied(200, 200, 220, segment_alpha)
                }
                MissileStatus::Terminal => {
                    // Ionized air: blue-white
                    egui::Color32::from_rgba_unmultiplied(180, 200, 255, segment_alpha)
                }
                _ => egui::Color32::TRANSPARENT,
            };

            painter.circle_filled(segment_pos, segment_width.max(0.5), color);
        }
    }

    fn render_interceptor(
        &self,
        painter: &egui::Painter,
        screen_rect: egui::Rect,
        interceptor: &Interceptor,
    ) {
        use crate::simulation::InterceptorStatus;

        // Don't render pending interceptors (not launched yet)
        if interceptor.status == InterceptorStatus::Pending {
            return;
        }

        // Draw predicted path to intercept point for in-flight interceptors
        if interceptor.status == InterceptorStatus::InFlight {
            self.render_interceptor_trajectory(painter, screen_rect, interceptor);
        }

        // Only render in-flight interceptors visually
        let positions = self.viewport.geo_to_screen_wrapped(interceptor.position, screen_rect);

        // Calculate heading toward target
        let heading = {
            let bearing_deg = bearing(interceptor.position, interceptor.target_position);
            ((bearing_deg - 90.0) as f32).to_radians()
        };

        let color = match interceptor.status {
            InterceptorStatus::Pending => return, // Already handled above, but needed for exhaustive match
            InterceptorStatus::InFlight => egui::Color32::from_rgb(60, 120, 220),   // Dark blue for in-flight
            InterceptorStatus::Hit => egui::Color32::from_rgb(50, 255, 50),        // Bright green for hit
            InterceptorStatus::Miss => egui::Color32::from_rgb(255, 150, 50),      // Orange for miss
            InterceptorStatus::SelfDestruct => egui::Color32::from_rgb(150, 150, 150),
        };

        for pos in positions {
            // Draw interceptor as a larger, more visible triangle
            let size = 12.0;
            let cos_h = heading.cos();
            let sin_h = heading.sin();

            let tip = egui::pos2(
                pos.x + cos_h * size,
                pos.y + sin_h * size,
            );
            let left = egui::pos2(
                pos.x + (heading + 2.5).cos() * size * 0.5,
                pos.y + (heading + 2.5).sin() * size * 0.5,
            );
            let right = egui::pos2(
                pos.x + (heading - 2.5).cos() * size * 0.5,
                pos.y + (heading - 2.5).sin() * size * 0.5,
            );

            // Draw glow/halo effect behind interceptor
            if interceptor.status == InterceptorStatus::InFlight {
                painter.circle_filled(
                    pos,
                    8.0,
                    egui::Color32::from_rgba_unmultiplied(255, 255, 0, 80),
                );
            }

            // Draw the interceptor triangle
            painter.add(egui::Shape::convex_polygon(
                vec![tip, left, right],
                color,
                egui::Stroke::new(2.0, egui::Color32::WHITE),
            ));

            // Draw bright trail behind interceptor
            if interceptor.status == InterceptorStatus::InFlight {
                let trail_length = 8;
                let progress = interceptor.flight_progress();
                let mut trail_points = Vec::new();

                for i in 0..=trail_length {
                    let t = progress - (i as f64 * 0.015);
                    if t < 0.0 {
                        break;
                    }
                    let trail_pos = crate::simulation::interpolate_great_circle(
                        interceptor.launch_position,
                        interceptor.target_position,
                        t,
                    );
                    let screen_pos = self.viewport.geo_to_screen(trail_pos, screen_rect);
                    trail_points.push(screen_pos);
                }

                if trail_points.len() >= 2 {
                    for i in 0..trail_points.len() - 1 {
                        let alpha = 220 - (i as u8 * 25);
                        let width = 3.0 - (i as f32 * 0.3);
                        painter.line_segment(
                            [trail_points[i], trail_points[i + 1]],
                            egui::Stroke::new(width, egui::Color32::from_rgba_unmultiplied(255, 255, 0, alpha)),
                        );
                    }
                }
            }

            // Draw intercept effect for completed intercepts
            match interceptor.status {
                InterceptorStatus::Hit => {
                    // Bright green burst for successful intercept
                    painter.circle_filled(
                        pos,
                        16.0,
                        egui::Color32::from_rgba_unmultiplied(50, 255, 50, 80),
                    );
                    painter.circle_stroke(
                        pos,
                        16.0,
                        egui::Stroke::new(3.0, egui::Color32::from_rgb(50, 255, 50)),
                    );
                    painter.circle_stroke(
                        pos,
                        10.0,
                        egui::Stroke::new(2.0, egui::Color32::from_rgb(150, 255, 150)),
                    );
                }
                InterceptorStatus::Miss => {
                    // Bright orange X for missed intercept
                    let x_size = 10.0;
                    painter.line_segment(
                        [
                            egui::pos2(pos.x - x_size, pos.y - x_size),
                            egui::pos2(pos.x + x_size, pos.y + x_size),
                        ],
                        egui::Stroke::new(3.0, egui::Color32::from_rgb(255, 100, 0)),
                    );
                    painter.line_segment(
                        [
                            egui::pos2(pos.x + x_size, pos.y - x_size),
                            egui::pos2(pos.x - x_size, pos.y + x_size),
                        ],
                        egui::Stroke::new(3.0, egui::Color32::from_rgb(255, 100, 0)),
                    );
                }
                _ => {}
            }
        }

        // Draw dashed line from interceptor to target missile (if in flight)
        if interceptor.status == InterceptorStatus::InFlight {
            if let Some(target_missile) = self.simulation.missiles.iter().find(|m| m.id == interceptor.target_id) {
                let interceptor_screen = self.viewport.geo_to_screen(interceptor.position, screen_rect);
                let target_screen = self.viewport.geo_to_screen(target_missile.position, screen_rect);

                // Targeting line to target (dark blue)
                painter.line_segment(
                    [interceptor_screen, target_screen],
                    egui::Stroke::new(1.5, egui::Color32::from_rgba_unmultiplied(60, 120, 220, 200)),
                );
            }
        }
    }

    /// Render interceptor trajectory with phase-based coloring and velocity info
    fn render_interceptor_trajectory(
        &self,
        painter: &egui::Painter,
        screen_rect: egui::Rect,
        interceptor: &Interceptor,
    ) {
        use crate::simulation::{InterceptorKinematics, InterceptorPhase, interpolate_great_circle};

        let kin = InterceptorKinematics::for_defense_type(interceptor.defense_type);
        let flight_duration = interceptor.intercept_time - interceptor.launch_time;
        let progress = interceptor.flight_progress();

        // Draw remaining path from current position to intercept point
        let segments = 20;
        let mut current_segment_start: Option<(egui::Pos2, InterceptorPhase)> = None;
        let mut path_segments: Vec<(Vec<egui::Pos2>, InterceptorPhase)> = Vec::new();

        for i in 0..=segments {
            let t = progress + (1.0 - progress) * (i as f64 / segments as f64);
            let pos = interpolate_great_circle(
                interceptor.launch_position,
                interceptor.target_position,
                t.min(1.0),
            );
            let screen_pos = self.viewport.geo_to_screen(pos, screen_rect);
            let phase = kin.phase_at_progress(t, flight_duration);

            match &mut current_segment_start {
                None => {
                    current_segment_start = Some((screen_pos, phase));
                    path_segments.push((vec![screen_pos], phase));
                }
                Some((_, current_phase)) => {
                    if *current_phase == phase {
                        // Same phase, add to current segment
                        if let Some(last_seg) = path_segments.last_mut() {
                            last_seg.0.push(screen_pos);
                        }
                    } else {
                        // New phase, start new segment
                        if let Some(last_seg) = path_segments.last_mut() {
                            last_seg.0.push(screen_pos); // Connect to new segment
                        }
                        path_segments.push((vec![screen_pos], phase));
                        *current_phase = phase;
                    }
                }
            }
        }

        // Draw each segment with phase-appropriate color
        for (points, phase) in path_segments {
            if points.len() < 2 {
                continue;
            }

            // Check for date line crossing
            let valid = points.windows(2).all(|w| w[0].distance(w[1]) < screen_rect.width() * 0.3);
            if !valid {
                continue;
            }

            let (color, width) = match phase {
                InterceptorPhase::Boost => (
                    egui::Color32::from_rgba_unmultiplied(60, 120, 220, 180), // Dark blue - thrusting
                    2.5,
                ),
                InterceptorPhase::Coast => (
                    egui::Color32::from_rgba_unmultiplied(100, 200, 255, 150), // Light blue - coasting
                    2.0,
                ),
                InterceptorPhase::Terminal => (
                    egui::Color32::from_rgba_unmultiplied(50, 255, 150, 200), // Green - homing
                    2.5,
                ),
            };

            painter.add(egui::Shape::line(points, egui::Stroke::new(width, color)));
        }

        // Draw intercept point marker
        let intercept_screen = self.viewport.geo_to_screen(interceptor.target_position, screen_rect);
        let diamond_size = 6.0;
        let diamond_points = vec![
            egui::pos2(intercept_screen.x, intercept_screen.y - diamond_size),
            egui::pos2(intercept_screen.x + diamond_size, intercept_screen.y),
            egui::pos2(intercept_screen.x, intercept_screen.y + diamond_size),
            egui::pos2(intercept_screen.x - diamond_size, intercept_screen.y),
        ];
        painter.add(egui::Shape::convex_polygon(
            diamond_points,
            egui::Color32::TRANSPARENT,
            egui::Stroke::new(2.0, egui::Color32::from_rgb(255, 255, 100)),
        ));

        // Draw velocity indicator (small text showing current speed)
        let current_pos = self.viewport.geo_to_screen(interceptor.position, screen_rect);
        let velocity_text = format!("{:.1} km/s", interceptor.current_velocity_km_s);
        let phase_text = match interceptor.phase {
            InterceptorPhase::Boost => "BOOST",
            InterceptorPhase::Coast => "COAST",
            InterceptorPhase::Terminal => "TERMINAL",
        };
        let label = format!("{} {}", phase_text, velocity_text);

        painter.text(
            egui::pos2(current_pos.x + 15.0, current_pos.y - 5.0),
            egui::Align2::LEFT_CENTER,
            label,
            egui::FontId::proportional(9.0),
            egui::Color32::from_rgb(255, 255, 150),
        );
    }

    fn render_defense_unit(
        &self,
        painter: &egui::Painter,
        screen_rect: egui::Rect,
        unit: &DefenseUnit,
    ) {
        let positions = self.viewport.geo_to_screen_wrapped(unit.position, screen_rect);
        let size = 12.0;

        for pos in positions {
            MilitarySymbols::draw_defense_unit(
                painter,
                pos,
                unit.affiliation,
                unit.defense_type,
                unit.status,
                size,
            );

            // Label below the symbol
            painter.text(
                egui::pos2(pos.x, pos.y + size * 1.5 + 4.0),
                egui::Align2::CENTER_TOP,
                unit.defense_type.name(),
                egui::FontId::proportional(10.0),
                egui::Color32::WHITE,
            );
        }
    }

    fn render_radar_station(
        &self,
        painter: &egui::Painter,
        screen_rect: egui::Rect,
        station: &RadarStation,
    ) {
        let positions = self.viewport.geo_to_screen_wrapped(station.position, screen_rect);
        let size = 14.0;

        for pos in positions {
            MilitarySymbols::draw_radar_station(painter, pos, station.affiliation, size);

            // Label below the symbol
            painter.text(
                egui::pos2(pos.x, pos.y + size * 0.6 + 4.0),
                egui::Align2::CENTER_TOP,
                &station.name,
                egui::FontId::proportional(9.0),
                egui::Color32::WHITE,
            );
        }
    }

    fn render_satellite(
        &self,
        painter: &egui::Painter,
        screen_rect: egui::Rect,
        satellite: &Satellite,
    ) {
        let positions = self.viewport.geo_to_screen_wrapped(satellite.position, screen_rect);
        let size = 12.0;

        // Calculate coverage radius once
        let coverage_radius = if self.show_detection_ranges {
            let range_deg = satellite.coverage_radius_km() / 111.32;
            let edge_pos = GeoCoord::new(
                satellite.position.lat + range_deg,
                satellite.position.lon,
            );
            let base_pos = self.viewport.geo_to_screen(satellite.position, screen_rect);
            let edge_screen = self.viewport.geo_to_screen(edge_pos, screen_rect);
            Some((base_pos.y - edge_screen.y).abs())
        } else {
            None
        };

        for pos in positions {
            MilitarySymbols::draw_satellite(
                painter,
                pos,
                satellite.affiliation,
                satellite.sensor_type,
                size,
            );

            // Draw coverage circle
            if let Some(radius) = coverage_radius {
                let coverage_color = match satellite.affiliation {
                    Affiliation::Friendly => egui::Color32::from_rgba_unmultiplied(100, 200, 255, 40),
                    Affiliation::Hostile => egui::Color32::from_rgba_unmultiplied(255, 100, 100, 40),
                    Affiliation::Neutral => egui::Color32::from_rgba_unmultiplied(200, 200, 200, 40),
                };
                painter.circle_stroke(pos, radius, egui::Stroke::new(1.0, coverage_color));
            }
        }
    }

    fn render_time_controls(&mut self, ui: &mut egui::Ui) {
        ui.horizontal(|ui| {
            ui.label("Time:");
            ui.monospace(self.simulation.format_time());
            ui.separator();

            if ui.button("⏮").on_hover_text("Reset").clicked() {
                self.simulation.load_sample_scenario();
            }

            let play_pause = if self.simulation.time_scale == TimeScale::Paused {
                "▶"
            } else {
                "⏸"
            };
            if ui.button(play_pause).clicked() {
                self.simulation.toggle_pause();
            }

            ui.separator();
            ui.label("Speed:");

            for scale in TimeScale::all() {
                let selected = self.simulation.time_scale == *scale;
                if ui.selectable_label(selected, scale.name()).clicked() {
                    self.simulation.set_time_scale(*scale);
                }
            }
        });
    }

    fn render_engagement_panel(&mut self, ctx: &egui::Context) {
        egui::SidePanel::left("engagement_panel")
            .resizable(true)
            .default_width(220.0)
            .show(ctx, |ui| {
                egui::ScrollArea::vertical().show(ui, |ui| {
                    self.render_engagement_status(ui);
                    ui.add_space(8.0);
                    ui.separator();
                    ui.add_space(4.0);
                    self.render_event_log(ui);
                });
            });
    }

    fn render_scenario_panel(&mut self, ctx: &egui::Context) {
        egui::SidePanel::left("scenario_panel")
            .resizable(true)
            .default_width(280.0)
            .show(ctx, |ui| {
                egui::ScrollArea::vertical().show(ui, |ui| {
                ui.heading("Scenarios");
                ui.separator();

                // Clone scenario list to avoid borrow checker issues
                // (We need to mutate self inside the iteration)
                let scenarios = self.scenarios.clone();

                for (idx, scenario) in scenarios.iter().enumerate() {
                    let is_current = idx == self.current_scenario;

                    ui.push_id(idx, |ui| {
                        egui::Frame::NONE
                            .fill(if is_current {
                                egui::Color32::from_rgba_unmultiplied(60, 80, 120, 255)
                            } else {
                                egui::Color32::TRANSPARENT
                            })
                            .corner_radius(4.0)
                            .inner_margin(8.0)
                            .show(ui, |ui| {
                                ui.horizontal(|ui| {
                                    ui.vertical(|ui| {
                                        ui.strong(&scenario.name);
                                        ui.label(
                                            egui::RichText::new(&scenario.description)
                                                .small()
                                                .weak(),
                                        );
                                        ui.horizontal(|ui| {
                                            ui.label(
                                                egui::RichText::new(format!(
                                                    "Region: {}",
                                                    scenario.region
                                                ))
                                                .small(),
                                            );
                                        });
                                    });
                                });

                                if !is_current {
                                    ui.horizontal(|ui| {
                                        if ui.button("Load").clicked() {
                                            self.current_scenario = idx;
                                            scenario.load(&mut self.simulation);
                                            self.viewport.center = scenario.center;
                                            self.viewport.zoom = scenario.zoom;
                                            self.selection = None;
                                            self.clear_event_tracking();
                                        }
                                    });
                                } else {
                                    ui.horizontal(|ui| {
                                        ui.label(
                                            egui::RichText::new("Currently Loaded")
                                                .small()
                                                .italics()
                                                .color(egui::Color32::from_rgb(150, 200, 150)),
                                        );
                                        if ui.small_button("Restart").clicked() {
                                            scenario.load(&mut self.simulation);
                                            self.selection = None;
                                            self.clear_event_tracking();
                                        }
                                    });
                                }
                            });
                    });

                    ui.add_space(4.0);
                }

                ui.separator();
                ui.add_space(8.0);

                // Reload scenarios button
                if ui.button("🔄 Reload Scenarios").clicked() {
                    self.scenarios = get_scenarios();
                }

                ui.add_space(8.0);
                ui.separator();

                // Scenario info
                if let Some(_scenario) = self.scenarios.get(self.current_scenario) {
                    ui.heading("Current Scenario");
                    ui.add_space(4.0);

                    egui::Grid::new("scenario_stats")
                        .num_columns(2)
                        .spacing([10.0, 4.0])
                        .show(ui, |ui| {
                            ui.label("Missiles:");
                            ui.label(format!("{}", self.simulation.missiles.len()));
                            ui.end_row();

                            ui.label("Defense Units:");
                            ui.label(format!("{}", self.simulation.defense_units.len()));
                            ui.end_row();

                            ui.label("Radar Stations:");
                            ui.label(format!("{}", self.simulation.radar_stations.len()));
                            ui.end_row();

                            ui.label("Satellites:");
                            ui.label(format!("{}", self.simulation.satellites.len()));
                            ui.end_row();
                        });
                }

                }); // end ScrollArea
            });
    }

    fn render_engagement_status(&self, ui: &mut egui::Ui) {
        use crate::simulation::{InterceptorStatus, MissileStatus};

        ui.heading("Engagement Status");
        ui.add_space(4.0);

        // Calculate statistics
        let total_threats = self.simulation.missiles.iter()
            .filter(|m| m.affiliation == Affiliation::Hostile)
            .count();

        let threats_in_flight = self.simulation.missiles.iter()
            .filter(|m| m.affiliation == Affiliation::Hostile)
            .filter(|m| matches!(m.status, MissileStatus::Boost | MissileStatus::Midcourse | MissileStatus::Terminal))
            .count();

        let threats_prelaunch = self.simulation.missiles.iter()
            .filter(|m| m.affiliation == Affiliation::Hostile)
            .filter(|m| m.status == MissileStatus::PreLaunch)
            .count();

        let threats_intercepted = self.simulation.missiles.iter()
            .filter(|m| m.affiliation == Affiliation::Hostile)
            .filter(|m| m.status == MissileStatus::Intercepted)
            .count();

        let threats_impacted = self.simulation.missiles.iter()
            .filter(|m| m.affiliation == Affiliation::Hostile)
            .filter(|m| m.status == MissileStatus::Impacted)
            .count();

        let interceptors_launched = self.simulation.interceptors.len();

        let interceptors_in_flight = self.simulation.interceptors.iter()
            .filter(|i| i.status == InterceptorStatus::InFlight)
            .count();

        let intercept_hits = self.simulation.interceptors.iter()
            .filter(|i| i.status == InterceptorStatus::Hit)
            .count();

        let intercept_misses = self.simulation.interceptors.iter()
            .filter(|i| i.status == InterceptorStatus::Miss)
            .count();

        let total_interceptors_available: u32 = self.simulation.defense_units.iter()
            .map(|u| u.interceptors_remaining)
            .sum();

        // Threat Status Section
        ui.label(egui::RichText::new("Threats").strong());
        egui::Grid::new("threat_stats")
            .num_columns(2)
            .spacing([10.0, 2.0])
            .show(ui, |ui| {
                ui.label("Total:");
                ui.label(format!("{}", total_threats));
                ui.end_row();

                ui.label("Pre-launch:");
                ui.colored_label(egui::Color32::GRAY, format!("{}", threats_prelaunch));
                ui.end_row();

                ui.label("In Flight:");
                ui.colored_label(egui::Color32::from_rgb(255, 200, 100), format!("{}", threats_in_flight));
                ui.end_row();

                ui.label("Intercepted:");
                ui.colored_label(egui::Color32::from_rgb(100, 255, 100), format!("{}", threats_intercepted));
                ui.end_row();

                ui.label("Impacted:");
                ui.colored_label(egui::Color32::from_rgb(255, 80, 80), format!("{}", threats_impacted));
                ui.end_row();
            });

        ui.add_space(8.0);

        // Defense Status Section
        ui.label(egui::RichText::new("Defense").strong());
        egui::Grid::new("defense_stats")
            .num_columns(2)
            .spacing([10.0, 2.0])
            .show(ui, |ui| {
                ui.label("Interceptors Left:");
                ui.label(format!("{}", total_interceptors_available));
                ui.end_row();

                ui.label("Launched:");
                ui.label(format!("{}", interceptors_launched));
                ui.end_row();

                ui.label("In Flight:");
                ui.colored_label(egui::Color32::from_rgb(60, 120, 220), format!("{}", interceptors_in_flight));
                ui.end_row();

                ui.label("Hits:");
                ui.colored_label(egui::Color32::from_rgb(100, 255, 100), format!("{}", intercept_hits));
                ui.end_row();

                ui.label("Misses:");
                ui.colored_label(egui::Color32::from_rgb(255, 150, 50), format!("{}", intercept_misses));
                ui.end_row();
            });

        // Success rate
        if intercept_hits + intercept_misses > 0 {
            ui.add_space(8.0);
            let success_rate = (intercept_hits as f64 / (intercept_hits + intercept_misses) as f64) * 100.0;
            let rate_color = if success_rate >= 70.0 {
                egui::Color32::from_rgb(100, 255, 100)
            } else if success_rate >= 40.0 {
                egui::Color32::from_rgb(255, 255, 100)
            } else {
                egui::Color32::from_rgb(255, 100, 100)
            };
            ui.horizontal(|ui| {
                ui.label("Intercept Rate:");
                ui.colored_label(rate_color, format!("{:.0}%", success_rate));
            });
        }

        // Defense outcome summary
        if threats_impacted > 0 || threats_intercepted > 0 {
            ui.add_space(8.0);
            ui.separator();
            ui.add_space(4.0);

            if threats_impacted == 0 && threats_in_flight == 0 && threats_prelaunch == 0 {
                ui.colored_label(
                    egui::Color32::from_rgb(100, 255, 100),
                    "✓ ALL THREATS NEUTRALIZED"
                );
            } else if threats_impacted > 0 {
                ui.colored_label(
                    egui::Color32::from_rgb(255, 80, 80),
                    format!("⚠ {} IMPACT(S) DETECTED", threats_impacted)
                );
            }
        }
    }

    fn render_debug_overlay(&self, ui: &mut egui::Ui) {
        egui::Frame::popup(ui.style())
            .fill(egui::Color32::from_rgba_unmultiplied(0, 0, 0, 200))
            .show(ui, |ui| {
                ui.label(format!(
                    "Center: {:.4}, {:.4}",
                    self.viewport.center.lat, self.viewport.center.lon
                ));
                ui.label(format!("Zoom: {:.2}", self.viewport.zoom));
                ui.separator();
                ui.label(format!("Missiles: {}", self.simulation.missiles.len()));
                ui.label(format!("Defense Units: {}", self.simulation.defense_units.len()));
                ui.label(format!("Satellites: {}", self.simulation.satellites.len()));
                ui.separator();
                ui.label(format!(
                    "Active Detections: {}",
                    self.simulation.detection.active_detections.len()
                ));
                ui.label(format!(
                    "Active Tracks: {}",
                    self.simulation.detection.active_tracks.len()
                ));
            });
    }

    /// Find an entity at the given screen position
    fn find_entity_at(&self, pos: egui::Pos2, screen_rect: egui::Rect) -> Option<Selection> {
        let click_radius = 15.0; // Pixels

        // Check missiles first (highest priority)
        for missile in &self.simulation.missiles {
            let entity_pos = self.viewport.geo_to_screen(missile.position, screen_rect);
            if pos.distance(entity_pos) < click_radius {
                return Some(Selection::Missile(missile.id));
            }
        }

        // Check defense units
        for unit in &self.simulation.defense_units {
            let entity_pos = self.viewport.geo_to_screen(unit.position, screen_rect);
            if pos.distance(entity_pos) < click_radius {
                return Some(Selection::DefenseUnit(unit.id));
            }
        }

        // Check satellites
        for satellite in &self.simulation.satellites {
            let entity_pos = self.viewport.geo_to_screen(satellite.position, screen_rect);
            if pos.distance(entity_pos) < click_radius {
                return Some(Selection::Satellite(satellite.id));
            }
        }

        // Check radar stations
        for station in &self.simulation.radar_stations {
            let entity_pos = self.viewport.geo_to_screen(station.position, screen_rect);
            if pos.distance(entity_pos) < click_radius {
                return Some(Selection::RadarStation(station.id));
            }
        }

        None
    }

    /// Render the info panel for the selected entity
    fn render_info_panel(&mut self, ctx: &egui::Context) {
        let Some(selection) = self.selection else {
            return;
        };

        egui::SidePanel::right("info_panel")
            .resizable(true)
            .default_width(250.0)
            .show(ctx, |ui| {
                ui.horizontal(|ui| {
                    ui.heading("Entity Info");
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        if ui.button("✕").clicked() {
                            self.selection = None;
                        }
                    });
                });
                ui.separator();

                match selection {
                    Selection::Missile(id) => {
                        if let Some(missile) = self.simulation.missiles.iter().find(|m| m.id == id) {
                            self.render_missile_info(ui, missile);
                        } else {
                            self.selection = None;
                        }
                    }
                    Selection::DefenseUnit(id) => {
                        if let Some(unit) = self.simulation.defense_units.iter().find(|u| u.id == id) {
                            self.render_defense_unit_info(ui, unit);
                        } else {
                            self.selection = None;
                        }
                    }
                    Selection::Satellite(id) => {
                        if let Some(sat) = self.simulation.satellites.iter().find(|s| s.id == id) {
                            self.render_satellite_info(ui, sat);
                        } else {
                            self.selection = None;
                        }
                    }
                    Selection::RadarStation(id) => {
                        if let Some(station) = self.simulation.radar_stations.iter().find(|s| s.id == id) {
                            self.render_radar_station_info(ui, station);
                        } else {
                            self.selection = None;
                        }
                    }
                }
            });
    }

    fn render_missile_info(&self, ui: &mut egui::Ui, missile: &Missile) {
        ui.colored_label(
            match missile.affiliation {
                Affiliation::Friendly => egui::Color32::from_rgb(100, 180, 255),
                Affiliation::Hostile => egui::Color32::from_rgb(255, 100, 100),
                Affiliation::Neutral => egui::Color32::GRAY,
            },
            format!("🚀 {}", missile.name),
        );
        ui.add_space(8.0);

        egui::Grid::new("missile_info_grid")
            .num_columns(2)
            .spacing([10.0, 4.0])
            .show(ui, |ui| {
                ui.label("Status:");
                ui.label(format!("{:?}", missile.status));
                ui.end_row();

                ui.label("Affiliation:");
                ui.label(format!("{:?}", missile.affiliation));
                ui.end_row();

                ui.label("Position:");
                ui.label(format!("{:.2}°, {:.2}°", missile.position.lat, missile.position.lon));
                ui.end_row();

                ui.label("Altitude:");
                ui.label(format!("{:.0} km", missile.altitude_km));
                ui.end_row();

                ui.label("Origin:");
                ui.label(format!("{:.2}°, {:.2}°", missile.origin.lat, missile.origin.lon));
                ui.end_row();

                ui.label("Target:");
                ui.label(format!("{:.2}°, {:.2}°", missile.target.lat, missile.target.lon));
                ui.end_row();

                ui.label("Flight Progress:");
                ui.label(format!("{:.1}%", missile.flight_progress() * 100.0));
                ui.end_row();

                ui.label("Flight Time:");
                let elapsed = missile.current_flight_time as u64;
                let total = missile.flight_time as u64;
                ui.label(format!("{}:{:02} / {}:{:02}",
                    elapsed / 60, elapsed % 60,
                    total / 60, total % 60
                ));
                ui.end_row();

                if missile.has_countermeasures {
                    ui.label("Countermeasures:");
                    ui.colored_label(
                        egui::Color32::from_rgb(200, 150, 255),
                        "Yes"
                    );
                    ui.end_row();

                    ui.label("Decoys:");
                    ui.label(format!("{} / {}", missile.decoys_deployed, missile.max_decoys));
                    ui.end_row();

                    ui.label("Hit Prob Reduction:");
                    let reduction = (1.0 - missile.decoy_effectiveness()) * 100.0;
                    ui.colored_label(
                        if reduction > 0.0 { egui::Color32::from_rgb(200, 150, 255) } else { egui::Color32::GRAY },
                        format!("-{:.0}%", reduction)
                    );
                    ui.end_row();
                }
            });

        // Progress bar
        ui.add_space(8.0);
        let progress = missile.flight_progress() as f32;
        ui.add(egui::ProgressBar::new(progress).text(format!("{:.0}%", progress * 100.0)));

        // Decoy bar if applicable
        if missile.has_countermeasures && missile.max_decoys > 0 {
            ui.add_space(4.0);
            let decoy_pct = (missile.max_decoys - missile.decoys_deployed) as f32 / missile.max_decoys as f32;
            ui.add(egui::ProgressBar::new(decoy_pct)
                .text(format!("Decoys: {}/{}", missile.max_decoys - missile.decoys_deployed, missile.max_decoys))
                .fill(egui::Color32::from_rgb(200, 150, 255)));
        }
    }

    fn render_defense_unit_info(&self, ui: &mut egui::Ui, unit: &DefenseUnit) {
        ui.colored_label(
            match unit.affiliation {
                Affiliation::Friendly => egui::Color32::from_rgb(100, 180, 255),
                Affiliation::Hostile => egui::Color32::from_rgb(255, 100, 100),
                Affiliation::Neutral => egui::Color32::GRAY,
            },
            format!("🛡 {}", unit.name),
        );
        ui.add_space(8.0);

        egui::Grid::new("defense_unit_info_grid")
            .num_columns(2)
            .spacing([10.0, 4.0])
            .show(ui, |ui| {
                ui.label("Type:");
                ui.label(unit.defense_type.name());
                ui.end_row();

                ui.label("Status:");
                ui.label(format!("{:?}", unit.status));
                ui.end_row();

                ui.label("Affiliation:");
                ui.label(format!("{:?}", unit.affiliation));
                ui.end_row();

                ui.label("Position:");
                ui.label(format!("{:.2}°, {:.2}°", unit.position.lat, unit.position.lon));
                ui.end_row();

                ui.label("Detection Range:");
                let sensor_config = self.simulation.sensor_configs.get_by_name(&unit.sensor_config_name);
                ui.label(format!("{:.0} km", sensor_config.detection.detection_range_km));
                ui.end_row();

                ui.label("Engagement Range:");
                let platform_config = self.simulation.platform_configs.get_by_defense_type(unit.defense_type);
                ui.label(format!("{:.0} km", platform_config.launcher.engagement_range_km));
                ui.end_row();

                ui.label("Interceptors:");
                ui.label(format!("{} / {}", unit.interceptors_remaining, unit.max_interceptors));
                ui.end_row();

                ui.label("Sensor Type:");
                ui.label(format!("{:?}", unit.sensor_type));
                ui.end_row();
            });

        // Interceptor bar
        ui.add_space(8.0);
        let ammo_pct = unit.interceptors_remaining as f32 / unit.max_interceptors as f32;
        ui.add(egui::ProgressBar::new(ammo_pct)
            .text(format!("{}/{}", unit.interceptors_remaining, unit.max_interceptors))
            .fill(egui::Color32::from_rgb(100, 200, 100)));

        // Sensor Status Section
        ui.add_space(12.0);
        ui.separator();
        ui.heading("Sensor Status");
        ui.add_space(4.0);

        if !unit.sensors.is_empty() {
            for sensor in &unit.sensors {
                let config = self.simulation.sensor_configs.get_by_name(&sensor.config_name);

                ui.group(|ui| {
                    ui.label(egui::RichText::new(&sensor.config_name).strong());

                    // Radar type and role
                    ui.horizontal(|ui| {
                        ui.label("Role:");
                        ui.label(format!("{:?}", sensor.role));
                    });

                    ui.horizontal(|ui| {
                        ui.label("Type:");
                        ui.label(format!("{:?}", config.detection.radar_type));
                    });

                    // Azimuth coverage
                    ui.horizontal(|ui| {
                        ui.label("Coverage:");
                        ui.label(format!("{:.0}° @ {:.0}°", sensor.azimuth_coverage_deg, sensor.azimuth_center_deg));
                    });

                    // Mode state for phased arrays
                    if let Some(mode_state) = self.simulation.detection.radar_mode_states.get(&sensor.sensor_id) {
                        ui.horizontal(|ui| {
                            ui.label("Modes:");
                            let fc = mode_state.stats.last_fc_count;
                            let track = mode_state.stats.last_track_count;
                            let search = mode_state.stats.last_search_count;
                            ui.label(format!("FC:{} T:{} S:{}", fc, track, search));
                        });

                        ui.horizontal(|ui| {
                            ui.label("Tracks:");
                            ui.label(format!("{}/{}",
                                mode_state.target_assignments.len(),
                                config.tracking.max_simultaneous_tracks));
                        });

                        // Time budget utilization
                        let util = mode_state.stats.last_time_budget_utilization;
                        let color = if util > 0.95 {
                            egui::Color32::from_rgb(255, 100, 100)
                        } else if util > 0.75 {
                            egui::Color32::from_rgb(255, 200, 100)
                        } else {
                            egui::Color32::from_rgb(100, 200, 100)
                        };
                        ui.horizontal(|ui| {
                            ui.label("Time Budget:");
                            ui.colored_label(color, format!("{:.0}%", util * 100.0));
                        });
                    }
                });
                ui.add_space(4.0);
            }
        } else {
            // Legacy single sensor
            let config = self.simulation.sensor_configs.get_by_name(&unit.sensor_config_name);
            ui.label(format!("Sensor: {}", unit.sensor_config_name));
            ui.horizontal(|ui| {
                ui.label("Type:");
                ui.label(format!("{:?}", config.detection.radar_type));
            });

            if let Some(mode_state) = self.simulation.detection.radar_mode_states.get(&unit.id) {
                ui.horizontal(|ui| {
                    ui.label("Tracks:");
                    ui.label(format!("{}/{}",
                        mode_state.target_assignments.len(),
                        config.tracking.max_simultaneous_tracks));
                });
            }
        }

        // Active Engagements Section
        ui.add_space(12.0);
        ui.separator();
        ui.heading("Engagements");
        ui.add_space(4.0);

        // Count interceptors launched by this unit
        let active_interceptors: Vec<_> = self.simulation.interceptors.iter()
            .filter(|i| i.launcher_id == unit.id && i.status == crate::simulation::InterceptorStatus::InFlight)
            .collect();

        let pending_interceptors: Vec<_> = self.simulation.interceptors.iter()
            .filter(|i| i.launcher_id == unit.id && i.status == crate::simulation::InterceptorStatus::Pending)
            .collect();

        if active_interceptors.is_empty() && pending_interceptors.is_empty() {
            ui.colored_label(egui::Color32::GRAY, "No active engagements");
        } else {
            ui.label(format!("In Flight: {}", active_interceptors.len()));
            ui.label(format!("Pending: {}", pending_interceptors.len()));

            for interceptor in active_interceptors.iter().take(5) {
                // Find target name
                let target_name = self.simulation.missiles.iter()
                    .find(|m| m.id == interceptor.target_id)
                    .map(|m| m.name.as_str())
                    .unwrap_or("Unknown");

                let progress = interceptor.flight_progress();
                ui.horizontal(|ui| {
                    ui.label(format!("→ {} ({:.0}%)", target_name, progress * 100.0));
                });
            }
            if active_interceptors.len() > 5 {
                ui.label(format!("  ...and {} more", active_interceptors.len() - 5));
            }

            // 3D Intercept Visualization
            ui.add_space(8.0);
            ui.label(egui::RichText::new("Intercept Geometry").strong());

            // Draw 3D view for first active interceptor
            if let Some(interceptor) = active_interceptors.first() {
                if let Some(missile) = self.simulation.missiles.iter().find(|m| m.id == interceptor.target_id) {
                    self.render_intercept_3d_view(ui, unit, interceptor, missile);
                }
            }
        }

        // Detections Section
        ui.add_space(12.0);
        ui.separator();
        ui.heading("Detections");
        ui.add_space(4.0);

        // Count detections for this unit's sensors
        let detection_count: usize = if !unit.sensors.is_empty() {
            unit.sensors.iter()
                .map(|s| self.simulation.detection.detections_for_sensor(s.sensor_id).len())
                .sum()
        } else {
            self.simulation.detection.detections_for_sensor(unit.id).len()
        };

        if detection_count == 0 {
            ui.colored_label(egui::Color32::GRAY, "No current detections");
        } else {
            ui.label(format!("Active detections: {}", detection_count));

            // Show tracked targets
            let tracked_count = self.simulation.detection.active_tracks.iter()
                .filter(|t| {
                    if !unit.sensors.is_empty() {
                        unit.sensors.iter().any(|s| s.sensor_id == t.tracker_id)
                    } else {
                        t.tracker_id == unit.id
                    }
                })
                .count();

            ui.label(format!("Stable tracks: {}", tracked_count));
        }
    }

    /// Render a 3D visualization of the intercept geometry
    fn render_intercept_3d_view(
        &self,
        ui: &mut egui::Ui,
        unit: &DefenseUnit,
        interceptor: &Interceptor,
        missile: &Missile,
    ) {
        let view_size = egui::vec2(220.0, 180.0);
        let (response, painter) = ui.allocate_painter(view_size, egui::Sense::hover());
        let rect = response.rect;

        // Background
        painter.rect_filled(rect, 4.0, egui::Color32::from_rgb(20, 25, 35));
        painter.rect_stroke(rect, 4.0, egui::Stroke::new(1.0, egui::Color32::from_rgb(60, 70, 90)), egui::StrokeKind::Inside);

        // 3D projection parameters (isometric-like)
        let center = rect.center();
        let scale = 0.6; // Scale factor for the view

        // Calculate relative positions (km from unit)
        let unit_pos = unit.position;

        // Convert geo coords to local XY (km), Z = altitude
        let to_local = |geo: GeoCoord, alt: f64| -> (f64, f64, f64) {
            let dx = (geo.lon - unit_pos.lon) * 111.32 * unit_pos.lat.to_radians().cos();
            let dy = (geo.lat - unit_pos.lat) * 111.32;
            (dx, dy, alt)
        };

        // Project 3D to 2D screen coordinates (isometric projection)
        let project = |x: f64, y: f64, z: f64| -> egui::Pos2 {
            // Isometric angles
            let iso_x = (x - y) * 0.7 * scale;
            let iso_y = -(x + y) * 0.4 * scale - z * 0.8 * scale;

            egui::pos2(
                center.x + iso_x as f32,
                center.y + iso_y as f32 + 30.0, // Offset down to leave room for labels
            )
        };

        // Find max distance for scaling
        let missile_local = to_local(missile.position, missile.altitude_km);
        let intercept_local = to_local(interceptor.target_position, interceptor.target_altitude_km);
        let interceptor_local = to_local(interceptor.position, interceptor.altitude_km);

        let max_dist = [
            (missile_local.0.abs() + missile_local.1.abs()).max(missile_local.2),
            (intercept_local.0.abs() + intercept_local.1.abs()).max(intercept_local.2),
            (interceptor_local.0.abs() + interceptor_local.1.abs()).max(interceptor_local.2),
            500.0, // Minimum scale
        ].iter().cloned().fold(0.0_f64, f64::max);

        let norm = 80.0 / max_dist; // Normalize to fit in view

        // Helper to project normalized coordinates
        let proj = |x: f64, y: f64, z: f64| -> egui::Pos2 {
            project(x * norm, y * norm, z * norm)
        };

        // Draw ground plane grid
        let grid_color = egui::Color32::from_rgba_unmultiplied(100, 100, 100, 40);
        let grid_extent = max_dist * 0.8;
        for i in -2..=2 {
            let offset = (i as f64) * grid_extent / 2.0;
            // X lines
            let p1 = proj(-grid_extent, offset, 0.0);
            let p2 = proj(grid_extent, offset, 0.0);
            painter.line_segment([p1, p2], egui::Stroke::new(0.5, grid_color));
            // Y lines
            let p1 = proj(offset, -grid_extent, 0.0);
            let p2 = proj(offset, grid_extent, 0.0);
            painter.line_segment([p1, p2], egui::Stroke::new(0.5, grid_color));
        }

        // Draw axes
        let axis_len = max_dist * 0.6;
        let origin = proj(0.0, 0.0, 0.0);

        // X axis (East) - Red
        let x_end = proj(axis_len, 0.0, 0.0);
        painter.line_segment([origin, x_end], egui::Stroke::new(1.5, egui::Color32::from_rgb(200, 80, 80)));
        painter.text(x_end + egui::vec2(5.0, 0.0), egui::Align2::LEFT_CENTER, "E",
            egui::FontId::proportional(10.0), egui::Color32::from_rgb(200, 80, 80));

        // Y axis (North) - Green
        let y_end = proj(0.0, axis_len, 0.0);
        painter.line_segment([origin, y_end], egui::Stroke::new(1.5, egui::Color32::from_rgb(80, 200, 80)));
        painter.text(y_end + egui::vec2(0.0, -8.0), egui::Align2::CENTER_BOTTOM, "N",
            egui::FontId::proportional(10.0), egui::Color32::from_rgb(80, 200, 80));

        // Z axis (Altitude) - Blue
        let z_end = proj(0.0, 0.0, axis_len);
        painter.line_segment([origin, z_end], egui::Stroke::new(1.5, egui::Color32::from_rgb(80, 150, 255)));
        painter.text(z_end + egui::vec2(5.0, 0.0), egui::Align2::LEFT_CENTER, "Alt",
            egui::FontId::proportional(10.0), egui::Color32::from_rgb(80, 150, 255));

        // Draw defense unit (launcher) at origin
        painter.circle_filled(origin, 6.0, egui::Color32::from_rgb(100, 200, 100));
        painter.circle_stroke(origin, 6.0, egui::Stroke::new(1.5, egui::Color32::WHITE));

        // Draw missile trajectory (from origin toward target, showing current position)
        let missile_origin = to_local(missile.origin, 0.0);
        let missile_target = to_local(missile.target, 0.0);
        let missile_current = to_local(missile.position, missile.altitude_km);

        // Draw missile path as arc (simplified - just origin to current to projected impact)
        let missile_path_color = egui::Color32::from_rgb(255, 100, 100);

        // Missile trajectory points (simplified arc)
        let num_points = 20;
        let mut missile_points: Vec<egui::Pos2> = Vec::new();
        for i in 0..=num_points {
            let t = i as f64 / num_points as f64;
            let progress = t * missile.flight_progress();

            // Interpolate position along path
            let x = missile_origin.0 + (missile_target.0 - missile_origin.0) * progress;
            let y = missile_origin.1 + (missile_target.1 - missile_origin.1) * progress;

            // Parabolic altitude profile
            let alt = missile.altitude_km * 4.0 * progress * (1.0 - progress) / (missile.flight_progress().max(0.01));
            let alt = alt.min(missile.altitude_km * 1.5);

            missile_points.push(proj(x, y, alt.max(0.0)));
        }

        if missile_points.len() >= 2 {
            painter.add(egui::Shape::line(missile_points, egui::Stroke::new(2.0, missile_path_color)));
        }

        // Draw missile current position
        let missile_pos = proj(missile_current.0, missile_current.1, missile_current.2);
        painter.circle_filled(missile_pos, 5.0, egui::Color32::from_rgb(255, 80, 80));

        // Draw vertical line from missile to ground (altitude indicator)
        let missile_ground = proj(missile_current.0, missile_current.1, 0.0);
        painter.line_segment([missile_ground, missile_pos],
            egui::Stroke::new(1.0, egui::Color32::from_rgba_unmultiplied(255, 100, 100, 100)));

        // Draw interceptor trajectory
        let interceptor_color = egui::Color32::from_rgb(100, 200, 255);
        let interceptor_current = to_local(interceptor.position, interceptor.altitude_km);
        let intercept_point = to_local(interceptor.target_position, interceptor.target_altitude_km);

        // Interceptor path from origin to current position
        let mut interceptor_points: Vec<egui::Pos2> = Vec::new();
        let num_int_points = 15;
        for i in 0..=num_int_points {
            let t = (i as f64 / num_int_points as f64) * interceptor.flight_progress();

            let x = intercept_point.0 * t;
            let y = intercept_point.1 * t;
            let z = intercept_point.2 * t;

            interceptor_points.push(proj(x, y, z));
        }

        if interceptor_points.len() >= 2 {
            painter.add(egui::Shape::line(interceptor_points, egui::Stroke::new(2.0, interceptor_color)));
        }

        // Draw interceptor current position
        let int_pos = proj(interceptor_current.0, interceptor_current.1, interceptor_current.2);
        painter.circle_filled(int_pos, 4.0, egui::Color32::from_rgb(100, 200, 255));

        // Draw predicted intercept point
        let intercept_screen = proj(intercept_point.0, intercept_point.1, intercept_point.2);

        // Dashed line from interceptor to intercept point (predicted path)
        let dash_color = egui::Color32::from_rgba_unmultiplied(100, 200, 255, 150);
        painter.line_segment([int_pos, intercept_screen], egui::Stroke::new(1.0, dash_color));

        // Draw intercept point marker (X)
        let x_size = 6.0;
        painter.line_segment(
            [intercept_screen + egui::vec2(-x_size, -x_size), intercept_screen + egui::vec2(x_size, x_size)],
            egui::Stroke::new(2.0, egui::Color32::from_rgb(255, 200, 50))
        );
        painter.line_segment(
            [intercept_screen + egui::vec2(x_size, -x_size), intercept_screen + egui::vec2(-x_size, x_size)],
            egui::Stroke::new(2.0, egui::Color32::from_rgb(255, 200, 50))
        );

        // Draw vertical line from intercept point to ground
        let intercept_ground = proj(intercept_point.0, intercept_point.1, 0.0);
        painter.line_segment([intercept_ground, intercept_screen],
            egui::Stroke::new(1.0, egui::Color32::from_rgba_unmultiplied(255, 200, 50, 80)));

        // Legend
        let legend_y = rect.max.y - 15.0;
        let legend_x = rect.min.x + 8.0;

        painter.circle_filled(egui::pos2(legend_x, legend_y), 3.0, egui::Color32::from_rgb(255, 80, 80));
        painter.text(egui::pos2(legend_x + 8.0, legend_y), egui::Align2::LEFT_CENTER, "Missile",
            egui::FontId::proportional(9.0), egui::Color32::from_rgb(200, 200, 200));

        painter.circle_filled(egui::pos2(legend_x + 55.0, legend_y), 3.0, egui::Color32::from_rgb(100, 200, 255));
        painter.text(egui::pos2(legend_x + 63.0, legend_y), egui::Align2::LEFT_CENTER, "Interceptor",
            egui::FontId::proportional(9.0), egui::Color32::from_rgb(200, 200, 200));

        // Stats below visualization
        ui.add_space(4.0);
        ui.horizontal(|ui| {
            ui.label(format!("Alt: {:.0}km", interceptor.target_altitude_km));
            ui.separator();
            ui.label(format!("Dist: {:.0}km",
                crate::simulation::haversine_distance(unit.position, interceptor.target_position)));
        });
        ui.horizontal(|ui| {
            ui.label(format!("ToI: {:.1}s",
                (interceptor.intercept_time - self.simulation.sim_time).max(0.0)));
            ui.separator();
            ui.label(format!("Pk: {:.0}%", interceptor.hit_probability * 100.0));
        });
    }

    fn render_intercept_3d_fullscreen(&mut self, ui: &mut egui::Ui) {
        let available_rect = ui.available_rect_before_wrap();

        // Allocate the entire space for the 3D view
        let (response, painter) = ui.allocate_painter(available_rect.size(), egui::Sense::click_and_drag());
        let rect = response.rect;

        // Handle interactions
        if response.dragged() {
            self.isometric_state.handle_drag(response.drag_delta());
            self.isometric_state.dragging = true;
        } else {
            self.isometric_state.dragging = false;
        }

        // Handle scroll for zoom
        let scroll = ui.input(|i| i.raw_scroll_delta.y);
        if scroll.abs() > 0.1 {
            self.isometric_state.handle_zoom(scroll * 0.01);
        }

        // Background - dark space color
        painter.rect_filled(rect, 0.0, egui::Color32::from_rgb(10, 12, 20));

        // Find the focused defense unit
        let focus_unit_id = self.isometric_state.focus_unit_id.or_else(|| {
            match self.selection {
                Some(Selection::DefenseUnit(id)) => Some(id),
                _ => self.simulation.defense_units.first().map(|u| u.id),
            }
        });

        let Some(unit_id) = focus_unit_id else {
            // No unit to focus on - show message
            painter.text(
                rect.center(),
                egui::Align2::CENTER_CENTER,
                "No defense unit selected\nSelect a defense unit to view intercepts",
                egui::FontId::proportional(18.0),
                egui::Color32::from_rgb(150, 150, 150),
            );
            return;
        };

        let Some(unit) = self.simulation.defense_units.iter().find(|u| u.id == unit_id) else {
            return;
        };

        let unit_pos = unit.position;
        let unit_name = unit.name.clone();
        let unit_defense_type = unit.defense_type;

        // Get platform and interceptor configs for engagement envelope
        let platform_config = self.simulation.platform_configs.get_by_defense_type(unit_defense_type);
        let interceptor_config = self.simulation.interceptor_configs.get_by_name(&platform_config.launcher.interceptor_type);
        let engagement_range = platform_config.launcher.engagement_range_km;
        let min_engage_alt = interceptor_config.altitude_envelope.min_engagement_altitude_km;
        let max_engage_alt = interceptor_config.altitude_envelope.max_engagement_altitude_km;

        // Find all interceptors launched by this unit
        let unit_interceptors: Vec<_> = self.simulation.interceptors
            .iter()
            .filter(|i| i.launcher_id == unit_id)
            .collect();

        // Get sensor IDs for this unit
        let sensor_ids: Vec<EntityId> = if !unit.sensors.is_empty() {
            unit.sensors.iter().map(|s| s.sensor_id).collect()
        } else {
            vec![unit_id]
        };

        // Find all missiles being TRACKED by this unit's sensors using persistent tracks
        // This uses active_tracks which persist between radar pings with degrading quality
        let mut tracked_with_quality: std::collections::HashMap<EntityId, f64> = std::collections::HashMap::new();
        for track in &self.simulation.detection.active_tracks {
            if sensor_ids.contains(&track.tracker_id) {
                // Use the best quality if multiple sensors track the same target
                let entry = tracked_with_quality.entry(track.target_id).or_insert(0.0);
                *entry = entry.max(track.track_quality);
            }
        }

        // Also include missiles being targeted by active interceptors (always show at full quality)
        for interceptor in &unit_interceptors {
            tracked_with_quality.entry(interceptor.target_id).or_insert(1.0);
            // Boost quality for actively intercepted targets
            if let Some(q) = tracked_with_quality.get_mut(&interceptor.target_id) {
                *q = q.max(0.8);
            }
        }

        // Get the actual missile objects with their track quality
        let tracked_missiles: Vec<(&Missile, f64)> = self.simulation.missiles
            .iter()
            .filter_map(|m| {
                tracked_with_quality.get(&m.id).map(|&quality| (m, quality))
            })
            .collect();

        // Base scale on engagement envelope (fixed) - this prevents flickering when tracks come/go
        let envelope_extent = engagement_range.max(max_engage_alt).max(100.0);
        let mut max_dist = envelope_extent * 1.3; // Base margin for envelope

        // Only expand view if missiles/interceptors are significantly beyond the envelope
        for interceptor in &unit_interceptors {
            let dist = crate::simulation::haversine_distance(unit_pos, interceptor.target_position);
            let max_extent = dist.max(interceptor.target_altitude_km);
            if max_extent > envelope_extent * 1.2 {
                max_dist = max_dist.max(max_extent * 1.1);
            }
        }

        for (missile, _quality) in &tracked_missiles {
            let dist = crate::simulation::haversine_distance(unit_pos, missile.position);
            let max_extent = dist.max(missile.altitude_km);
            if max_extent > envelope_extent * 1.2 {
                max_dist = max_dist.max(max_extent * 1.1);
            }
        }

        // Calculate scale based on view size
        let view_min = rect.width().min(rect.height()) as f64;
        let scale = view_min * 0.35 / max_dist;
        let center = rect.center();

        // Helper: convert geo coords to local XY (km), Z = altitude
        let to_local = |geo: GeoCoord, alt: f64| -> (f64, f64, f64) {
            let dx = (geo.lon - unit_pos.lon) * 111.32 * unit_pos.lat.to_radians().cos();
            let dy = (geo.lat - unit_pos.lat) * 111.32;
            (dx, dy, alt)
        };

        // Helper: project 3D to 2D using isometric state
        let proj = |x: f64, y: f64, z: f64| -> egui::Pos2 {
            self.isometric_state.project(x, y, z, center, scale)
        };

        // Draw ground plane grid
        let grid_color = egui::Color32::from_rgba_unmultiplied(60, 70, 90, 60);
        let grid_extent = max_dist * 0.8;
        let grid_lines = 5;
        for i in -grid_lines..=grid_lines {
            let offset = (i as f64) * grid_extent / grid_lines as f64;
            // X lines (East-West)
            let p1 = proj(-grid_extent, offset, 0.0);
            let p2 = proj(grid_extent, offset, 0.0);
            painter.line_segment([p1, p2], egui::Stroke::new(0.5, grid_color));
            // Y lines (North-South)
            let p1 = proj(offset, -grid_extent, 0.0);
            let p2 = proj(offset, grid_extent, 0.0);
            painter.line_segment([p1, p2], egui::Stroke::new(0.5, grid_color));
        }

        // Draw axes
        let axis_len = max_dist * 0.5;
        let origin = proj(0.0, 0.0, 0.0);

        // X axis (East) - Red
        let x_end = proj(axis_len, 0.0, 0.0);
        painter.line_segment([origin, x_end], egui::Stroke::new(2.0, egui::Color32::from_rgb(200, 80, 80)));
        painter.text(x_end + egui::vec2(8.0, 0.0), egui::Align2::LEFT_CENTER, "E",
            egui::FontId::proportional(14.0), egui::Color32::from_rgb(200, 80, 80));

        // Y axis (North) - Green
        let y_end = proj(0.0, axis_len, 0.0);
        painter.line_segment([origin, y_end], egui::Stroke::new(2.0, egui::Color32::from_rgb(80, 200, 80)));
        painter.text(y_end + egui::vec2(0.0, -12.0), egui::Align2::CENTER_BOTTOM, "N",
            egui::FontId::proportional(14.0), egui::Color32::from_rgb(80, 200, 80));

        // Z axis (Altitude) - Blue
        let z_end = proj(0.0, 0.0, axis_len);
        painter.line_segment([origin, z_end], egui::Stroke::new(2.0, egui::Color32::from_rgb(80, 150, 255)));
        painter.text(z_end + egui::vec2(8.0, 0.0), egui::Align2::LEFT_CENTER, "Alt",
            egui::FontId::proportional(14.0), egui::Color32::from_rgb(80, 150, 255));

        // Draw defense unit marker at origin
        painter.circle_filled(origin, 12.0, egui::Color32::from_rgb(80, 180, 80));
        painter.circle_stroke(origin, 12.0, egui::Stroke::new(2.0, egui::Color32::WHITE));
        painter.text(origin + egui::vec2(0.0, 20.0), egui::Align2::CENTER_TOP, &unit_name,
            egui::FontId::proportional(12.0), egui::Color32::from_rgb(150, 220, 150));

        // Draw 3D engagement envelope (lightly shaded yellow)
        let envelope_fill = egui::Color32::from_rgba_unmultiplied(255, 220, 100, 25);
        let envelope_stroke = egui::Color32::from_rgba_unmultiplied(255, 200, 50, 80);
        let envelope_stroke_strong = egui::Color32::from_rgba_unmultiplied(255, 200, 50, 150);

        // Draw vertical lines at cardinal points showing the envelope
        let num_radial_lines = 16;
        for i in 0..num_radial_lines {
            let angle = (i as f64) * 2.0 * std::f64::consts::PI / (num_radial_lines as f64);
            let x = engagement_range * angle.cos();
            let y = engagement_range * angle.sin();

            // Vertical line from min to max altitude at outer range
            let bottom = proj(x, y, min_engage_alt);
            let top = proj(x, y, max_engage_alt);
            painter.line_segment([bottom, top], egui::Stroke::new(1.0, envelope_stroke));
        }

        // Draw horizontal circles at min and max engagement altitudes
        let num_circle_points = 32;
        for alt in [min_engage_alt, max_engage_alt] {
            let mut circle_points: Vec<egui::Pos2> = Vec::new();
            for i in 0..=num_circle_points {
                let angle = (i as f64) * 2.0 * std::f64::consts::PI / (num_circle_points as f64);
                let x = engagement_range * angle.cos();
                let y = engagement_range * angle.sin();
                circle_points.push(proj(x, y, alt));
            }
            // Draw as line segments for the outline
            for j in 0..circle_points.len() - 1 {
                painter.line_segment(
                    [circle_points[j], circle_points[j + 1]],
                    egui::Stroke::new(1.5, envelope_stroke_strong)
                );
            }
        }

        // Draw filled envelope wall segments (semi-transparent) - match the 16 radial lines
        for i in 0..num_radial_lines {
            let angle1 = (i as f64) * 2.0 * std::f64::consts::PI / (num_radial_lines as f64);
            let angle2 = ((i + 1) as f64) * 2.0 * std::f64::consts::PI / (num_radial_lines as f64);

            let x1 = engagement_range * angle1.cos();
            let y1 = engagement_range * angle1.sin();
            let x2 = engagement_range * angle2.cos();
            let y2 = engagement_range * angle2.sin();

            // Create a quad for the "wall" segment of the envelope
            let points = vec![
                proj(x1, y1, min_engage_alt),
                proj(x2, y2, min_engage_alt),
                proj(x2, y2, max_engage_alt),
                proj(x1, y1, max_engage_alt),
            ];
            painter.add(egui::Shape::convex_polygon(points, envelope_fill, egui::Stroke::NONE));
        }

        // Draw the top "dome" surface as filled segments - match the 16 radial lines
        for i in 0..num_radial_lines {
            let angle1 = (i as f64) * 2.0 * std::f64::consts::PI / (num_radial_lines as f64);
            let angle2 = ((i + 1) as f64) * 2.0 * std::f64::consts::PI / (num_radial_lines as f64);

            let x1 = engagement_range * angle1.cos();
            let y1 = engagement_range * angle1.sin();
            let x2 = engagement_range * angle2.cos();
            let y2 = engagement_range * angle2.sin();

            // Triangle from center to outer edge at max altitude
            let points = vec![
                proj(0.0, 0.0, max_engage_alt),
                proj(x1, y1, max_engage_alt),
                proj(x2, y2, max_engage_alt),
            ];
            painter.add(egui::Shape::convex_polygon(points, envelope_fill, egui::Stroke::NONE));
        }

        // Draw missiles with track quality degradation
        for (missile, track_quality) in &tracked_missiles {
            let missile_origin = to_local(missile.origin, 0.0);
            let missile_target = to_local(missile.target, 0.0);
            let missile_current = to_local(missile.position, missile.altitude_km);

            // Adjust colors based on track quality (fades as quality degrades)
            let quality_alpha = (track_quality * 255.0) as u8;
            let quality_alpha_half = (track_quality * 128.0) as u8;
            let missile_path_color = egui::Color32::from_rgba_unmultiplied(255, 100, 100, quality_alpha.max(80));
            let missile_future_color = egui::Color32::from_rgba_unmultiplied(255, 100, 100, quality_alpha_half.max(40));

            let flight_prog = missile.flight_progress().clamp(0.01, 0.99);

            // Back-calculate apogee from current altitude and flight progress
            // For parabola: alt = 4 * apogee * t * (1-t)
            // At t=flight_prog: current_alt = 4 * apogee * flight_prog * (1 - flight_prog)
            // So: apogee = current_alt / (4 * flight_prog * (1 - flight_prog))
            let parabola_factor = 4.0 * flight_prog * (1.0 - flight_prog);
            let estimated_apogee = if parabola_factor > 0.01 && missile.altitude_km > 1.0 {
                missile.altitude_km / parabola_factor
            } else {
                missile.altitude_km.max(50.0)
            };

            // Draw PAST trajectory (origin to current position) - solid line with arc
            let num_past_points = 25;
            let mut past_points: Vec<egui::Pos2> = Vec::new();
            for i in 0..=num_past_points {
                let t = i as f64 / num_past_points as f64; // 0 to 1 along past path

                // Interpolate x,y from origin to current position
                let x = missile_origin.0 + (missile_current.0 - missile_origin.0) * t;
                let y = missile_origin.1 + (missile_current.1 - missile_origin.1) * t;

                // Map t (0-1 of past path) to total flight progress (0 to flight_prog)
                let total_t = t * flight_prog;
                // Parabolic altitude - this will equal missile.altitude_km when t=1
                let alt = estimated_apogee * 4.0 * total_t * (1.0 - total_t);

                past_points.push(proj(x, y, alt.max(0.0)));
            }

            if past_points.len() >= 2 {
                painter.add(egui::Shape::line(past_points, egui::Stroke::new(2.5, missile_path_color)));
            }

            // Draw FUTURE trajectory (current to target) - faded line
            let num_future_points = 20;
            let mut future_points: Vec<egui::Pos2> = Vec::new();
            for i in 0..=num_future_points {
                let t = i as f64 / num_future_points as f64; // 0 to 1 of remaining path

                // Interpolate x,y from current position to target
                let x = missile_current.0 + (missile_target.0 - missile_current.0) * t;
                let y = missile_current.1 + (missile_target.1 - missile_current.1) * t;

                // Map t to remaining flight (flight_prog to 1.0)
                let total_t = flight_prog + (1.0 - flight_prog) * t;
                let alt = estimated_apogee * 4.0 * total_t * (1.0 - total_t);

                future_points.push(proj(x, y, alt.max(0.0)));
            }

            if future_points.len() >= 2 {
                painter.add(egui::Shape::line(future_points, egui::Stroke::new(1.5, missile_future_color)));
            }

            // Draw missile current position with quality-based appearance
            let missile_pos = proj(missile_current.0, missile_current.1, missile_current.2);
            let marker_color = egui::Color32::from_rgba_unmultiplied(255, 80, 80, quality_alpha.max(100));
            let marker_size = if *track_quality > 0.5 { 8.0 } else { 6.0 };
            painter.circle_filled(missile_pos, marker_size, marker_color);

            // Solid stroke for good tracks, dashed/faded for stale tracks
            if *track_quality > 0.5 {
                painter.circle_stroke(missile_pos, marker_size, egui::Stroke::new(1.5, egui::Color32::WHITE));
            } else {
                // Show uncertainty circle for degraded tracks
                let uncertainty_radius = ((1.0 - track_quality) * 20.0) as f32; // Up to 20 pixels
                painter.circle_stroke(
                    missile_pos,
                    marker_size + uncertainty_radius,
                    egui::Stroke::new(1.0, egui::Color32::from_rgba_unmultiplied(255, 200, 100, 80))
                );
            }

            // Draw vertical altitude line
            let missile_ground = proj(missile_current.0, missile_current.1, 0.0);
            painter.line_segment([missile_ground, missile_pos],
                egui::Stroke::new(1.0, egui::Color32::from_rgba_unmultiplied(255, 100, 100, quality_alpha_half.max(60))));

            // Label with track quality indicator
            let quality_indicator = if *track_quality > 0.7 { "" }
                else if *track_quality > 0.3 { " [FADING]" }
                else { " [STALE]" };
            let label_color = egui::Color32::from_rgba_unmultiplied(255, 150, 150, quality_alpha.max(120));
            painter.text(missile_pos + egui::vec2(12.0, 0.0), egui::Align2::LEFT_CENTER,
                format!("{}{}\n{:.0}km alt", missile.name, quality_indicator, missile.altitude_km),
                egui::FontId::proportional(11.0), label_color);
        }

        // Draw interceptors
        for interceptor in &unit_interceptors {
            let interceptor_color = egui::Color32::from_rgb(100, 200, 255);
            let interceptor_future_color = egui::Color32::from_rgba_unmultiplied(100, 200, 255, 120);
            let interceptor_current = to_local(interceptor.position, interceptor.altitude_km);
            let intercept_point = to_local(interceptor.target_position, interceptor.target_altitude_km);

            // Draw PAST path: from origin (0,0,0) to current position - solid line
            let num_past_points = 15;
            let mut past_points: Vec<egui::Pos2> = Vec::new();
            for i in 0..=num_past_points {
                let t = i as f64 / num_past_points as f64;

                // Linear interpolation from origin to current position
                let x = interceptor_current.0 * t;
                let y = interceptor_current.1 * t;
                let z = interceptor_current.2 * t;

                past_points.push(proj(x, y, z));
            }

            if past_points.len() >= 2 {
                painter.add(egui::Shape::line(past_points, egui::Stroke::new(2.5, interceptor_color)));
            }

            // Draw interceptor current position
            let int_pos = proj(interceptor_current.0, interceptor_current.1, interceptor_current.2);
            painter.circle_filled(int_pos, 6.0, egui::Color32::from_rgb(100, 200, 255));

            // Draw FUTURE path: from current position to predicted intercept point - dashed/faded
            let intercept_screen = proj(intercept_point.0, intercept_point.1, intercept_point.2);
            painter.line_segment([int_pos, intercept_screen], egui::Stroke::new(1.5, interceptor_future_color));

            // X marker at intercept point
            let x_size = 10.0;
            painter.line_segment(
                [intercept_screen + egui::vec2(-x_size, -x_size), intercept_screen + egui::vec2(x_size, x_size)],
                egui::Stroke::new(3.0, egui::Color32::from_rgb(255, 200, 50))
            );
            painter.line_segment(
                [intercept_screen + egui::vec2(x_size, -x_size), intercept_screen + egui::vec2(-x_size, x_size)],
                egui::Stroke::new(3.0, egui::Color32::from_rgb(255, 200, 50))
            );

            // Vertical line from intercept to ground
            let intercept_ground = proj(intercept_point.0, intercept_point.1, 0.0);
            painter.line_segment([intercept_ground, intercept_screen],
                egui::Stroke::new(1.0, egui::Color32::from_rgba_unmultiplied(255, 200, 50, 80)));

            // Vertical line from interceptor to ground
            let int_ground = proj(interceptor_current.0, interceptor_current.1, 0.0);
            painter.line_segment([int_ground, int_pos],
                egui::Stroke::new(1.0, egui::Color32::from_rgba_unmultiplied(100, 200, 255, 80)));

            // Stats label near intercept point
            let time_to_intercept = (interceptor.intercept_time - self.simulation.sim_time).max(0.0);
            painter.text(intercept_screen + egui::vec2(15.0, -10.0), egui::Align2::LEFT_CENTER,
                format!("ToI: {:.1}s\nPk: {:.0}%", time_to_intercept, interceptor.hit_probability * 100.0),
                egui::FontId::proportional(11.0), egui::Color32::from_rgb(255, 220, 100));
        }

        // Draw overlay stats panel
        let stats_rect = egui::Rect::from_min_size(
            rect.min + egui::vec2(10.0, 10.0),
            egui::vec2(220.0, 160.0)
        );
        painter.rect_filled(stats_rect, 6.0, egui::Color32::from_rgba_unmultiplied(20, 25, 35, 200));
        painter.rect_stroke(stats_rect, 6.0, egui::Stroke::new(1.0, egui::Color32::from_rgb(60, 70, 90)), egui::StrokeKind::Inside);

        let text_start = stats_rect.min + egui::vec2(10.0, 10.0);
        painter.text(text_start, egui::Align2::LEFT_TOP,
            "INTERCEPT VIEW",
            egui::FontId::proportional(14.0), egui::Color32::from_rgb(200, 200, 200));

        painter.text(text_start + egui::vec2(0.0, 22.0), egui::Align2::LEFT_TOP,
            format!("Unit: {}", unit_name),
            egui::FontId::proportional(12.0), egui::Color32::from_rgb(150, 220, 150));

        painter.text(text_start + egui::vec2(0.0, 40.0), egui::Align2::LEFT_TOP,
            format!("Active Interceptors: {}", unit_interceptors.len()),
            egui::FontId::proportional(12.0), egui::Color32::from_rgb(100, 200, 255));

        painter.text(text_start + egui::vec2(0.0, 58.0), egui::Align2::LEFT_TOP,
            format!("Tracked Threats: {}", tracked_missiles.len()),
            egui::FontId::proportional(12.0), egui::Color32::from_rgb(255, 150, 150));

        // Engagement envelope info
        painter.text(text_start + egui::vec2(0.0, 80.0), egui::Align2::LEFT_TOP,
            format!("Envelope: {:.0}km range", engagement_range),
            egui::FontId::proportional(10.0), egui::Color32::from_rgb(255, 220, 100));

        painter.text(text_start + egui::vec2(0.0, 95.0), egui::Align2::LEFT_TOP,
            format!("Alt: {:.0}-{:.0}km", min_engage_alt, max_engage_alt),
            egui::FontId::proportional(10.0), egui::Color32::from_rgb(255, 220, 100));

        painter.text(text_start + egui::vec2(0.0, 115.0), egui::Align2::LEFT_TOP,
            format!("Azimuth: {:.0}°  Elev: {:.0}°", self.isometric_state.azimuth_deg, self.isometric_state.elevation_deg),
            egui::FontId::proportional(10.0), egui::Color32::from_rgb(120, 120, 140));

        painter.text(text_start + egui::vec2(0.0, 130.0), egui::Align2::LEFT_TOP,
            format!("Zoom: {:.1}x", self.isometric_state.zoom),
            egui::FontId::proportional(10.0), egui::Color32::from_rgb(120, 120, 140));

        // Legend in bottom right
        let legend_rect = egui::Rect::from_min_size(
            egui::pos2(rect.max.x - 180.0, rect.max.y - 110.0),
            egui::vec2(170.0, 100.0)
        );
        painter.rect_filled(legend_rect, 6.0, egui::Color32::from_rgba_unmultiplied(20, 25, 35, 200));
        painter.rect_stroke(legend_rect, 6.0, egui::Stroke::new(1.0, egui::Color32::from_rgb(60, 70, 90)), egui::StrokeKind::Inside);

        let legend_start = legend_rect.min + egui::vec2(10.0, 10.0);

        // Missile legend
        painter.circle_filled(legend_start + egui::vec2(5.0, 5.0), 5.0, egui::Color32::from_rgb(255, 80, 80));
        painter.text(legend_start + egui::vec2(15.0, 5.0), egui::Align2::LEFT_CENTER, "Tracked Missile",
            egui::FontId::proportional(11.0), egui::Color32::from_rgb(200, 200, 200));

        // Interceptor legend
        painter.circle_filled(legend_start + egui::vec2(5.0, 25.0), 5.0, egui::Color32::from_rgb(100, 200, 255));
        painter.text(legend_start + egui::vec2(15.0, 25.0), egui::Align2::LEFT_CENTER, "Interceptor",
            egui::FontId::proportional(11.0), egui::Color32::from_rgb(200, 200, 200));

        // Intercept point legend
        let x_pos = legend_start + egui::vec2(5.0, 45.0);
        painter.line_segment([x_pos + egui::vec2(-4.0, -4.0), x_pos + egui::vec2(4.0, 4.0)],
            egui::Stroke::new(2.0, egui::Color32::from_rgb(255, 200, 50)));
        painter.line_segment([x_pos + egui::vec2(4.0, -4.0), x_pos + egui::vec2(-4.0, 4.0)],
            egui::Stroke::new(2.0, egui::Color32::from_rgb(255, 200, 50)));
        painter.text(legend_start + egui::vec2(15.0, 45.0), egui::Align2::LEFT_CENTER, "Intercept Point",
            egui::FontId::proportional(11.0), egui::Color32::from_rgb(200, 200, 200));

        // Engagement envelope legend
        painter.rect_filled(
            egui::Rect::from_min_size(legend_start + egui::vec2(2.0, 62.0), egui::vec2(8.0, 8.0)),
            2.0,
            egui::Color32::from_rgba_unmultiplied(255, 220, 100, 60)
        );
        painter.rect_stroke(
            egui::Rect::from_min_size(legend_start + egui::vec2(2.0, 62.0), egui::vec2(8.0, 8.0)),
            2.0,
            egui::Stroke::new(1.0, egui::Color32::from_rgb(255, 200, 50)),
            egui::StrokeKind::Inside
        );
        painter.text(legend_start + egui::vec2(15.0, 66.0), egui::Align2::LEFT_CENTER, "Engagement Envelope",
            egui::FontId::proportional(11.0), egui::Color32::from_rgb(200, 200, 200));

        // Defense unit legend
        painter.circle_filled(legend_start + egui::vec2(5.0, 85.0), 5.0, egui::Color32::from_rgb(80, 180, 80));
        painter.text(legend_start + egui::vec2(15.0, 85.0), egui::Align2::LEFT_CENTER, "Defense Unit",
            egui::FontId::proportional(11.0), egui::Color32::from_rgb(200, 200, 200));

        // Controls hint at bottom
        let hint_pos = egui::pos2(rect.center().x, rect.max.y - 20.0);
        painter.text(hint_pos, egui::Align2::CENTER_BOTTOM,
            "Drag to rotate • Scroll to zoom",
            egui::FontId::proportional(11.0), egui::Color32::from_rgb(100, 100, 120));

        // If no tracked missiles, show a message
        if tracked_missiles.is_empty() {
            painter.text(rect.center() + egui::vec2(0.0, 50.0), egui::Align2::CENTER_CENTER,
                "No threats currently being tracked",
                egui::FontId::proportional(16.0), egui::Color32::from_rgb(120, 120, 140));
        }
    }

    fn render_satellite_info(&self, ui: &mut egui::Ui, satellite: &Satellite) {
        ui.colored_label(
            match satellite.affiliation {
                Affiliation::Friendly => egui::Color32::from_rgb(100, 200, 255),
                Affiliation::Hostile => egui::Color32::from_rgb(255, 100, 100),
                Affiliation::Neutral => egui::Color32::GRAY,
            },
            format!("🛰 {}", satellite.name),
        );
        ui.add_space(8.0);

        egui::Grid::new("satellite_info_grid")
            .num_columns(2)
            .spacing([10.0, 4.0])
            .show(ui, |ui| {
                ui.label("Affiliation:");
                ui.label(format!("{:?}", satellite.affiliation));
                ui.end_row();

                ui.label("Position:");
                ui.label(format!("{:.2}°, {:.2}°", satellite.position.lat, satellite.position.lon));
                ui.end_row();

                ui.label("Altitude:");
                ui.label(format!("{:.0} km", satellite.altitude_km));
                ui.end_row();

                ui.label("Sensor Type:");
                ui.label(match satellite.sensor_type {
                    SensorType::Radar => "Radar",
                    SensorType::Infrared => "Infrared",
                    SensorType::Both => "Radar + IR",
                });
                ui.end_row();

                ui.label("Coverage Angle:");
                ui.label(format!("{:.1}°", satellite.coverage_angle_deg));
                ui.end_row();

                ui.label("Coverage Radius:");
                ui.label(format!("{:.0} km", satellite.coverage_radius_km()));
                ui.end_row();

                ui.label("Orbital Period:");
                ui.label(format!("{:.1} hours", satellite.orbital_period_hours));
                ui.end_row();
            });
    }

    fn render_radar_station_info(&self, ui: &mut egui::Ui, station: &RadarStation) {
        ui.colored_label(
            match station.affiliation {
                Affiliation::Friendly => egui::Color32::from_rgb(100, 220, 150),
                Affiliation::Hostile => egui::Color32::from_rgb(255, 150, 50),
                Affiliation::Neutral => egui::Color32::GRAY,
            },
            format!("📡 {}", station.name),
        );
        ui.add_space(8.0);

        egui::Grid::new("radar_station_info_grid")
            .num_columns(2)
            .spacing([10.0, 4.0])
            .show(ui, |ui| {
                ui.label("Affiliation:");
                ui.label(format!("{:?}", station.affiliation));
                ui.end_row();

                ui.label("Position:");
                ui.label(format!("{:.2}°, {:.2}°", station.position.lat, station.position.lon));
                ui.end_row();

                ui.label("Detection Range:");
                ui.label(format!("{:.0} km", station.detection_range_km));
                ui.end_row();

                ui.label("Azimuth Coverage:");
                ui.label(format!("{:.0}°", station.azimuth_coverage_deg));
                ui.end_row();

                ui.label("Elevation Range:");
                ui.label(format!("{:.0}° - {:.0}°", station.elevation_min_deg, station.elevation_max_deg));
                ui.end_row();

                ui.label("Sensor Type:");
                ui.label(match station.sensor_type {
                    SensorType::Radar => "Radar",
                    SensorType::Infrared => "Infrared",
                    SensorType::Both => "Radar + IR",
                });
                ui.end_row();

                ui.label("Facing:");
                ui.label(format!("{:.0}°", station.facing_deg));
                ui.end_row();

                // Get sensor config for additional info
                let config = self.simulation.sensor_configs.get_by_name(&station.sensor_config_name);

                ui.label("Radar Type:");
                ui.label(format!("{:?}", config.detection.radar_type));
                ui.end_row();

                ui.label("Radar Band:");
                ui.label(format!("{:?}", config.detection.radar_band));
                ui.end_row();
            });

        // Tracking Status Section
        ui.add_space(12.0);
        ui.separator();
        ui.heading("Tracking Status");
        ui.add_space(4.0);

        let config = self.simulation.sensor_configs.get_by_name(&station.sensor_config_name);

        if let Some(mode_state) = self.simulation.detection.radar_mode_states.get(&station.id) {
            egui::Grid::new("radar_tracking_grid")
                .num_columns(2)
                .spacing([10.0, 4.0])
                .show(ui, |ui| {
                    // Mode breakdown
                    ui.label("Fire Control:");
                    let fc = mode_state.stats.last_fc_count;
                    let fc_max = config.tracking.max_fire_control_tracks;
                    let fc_color = if fc >= fc_max {
                        egui::Color32::from_rgb(255, 100, 100)
                    } else if fc > 0 {
                        egui::Color32::from_rgb(255, 200, 100)
                    } else {
                        egui::Color32::GRAY
                    };
                    ui.colored_label(fc_color, format!("{}/{}", fc, fc_max));
                    ui.end_row();

                    ui.label("Tracking:");
                    ui.label(format!("{}", mode_state.stats.last_track_count));
                    ui.end_row();

                    ui.label("Search:");
                    ui.label(format!("{}", mode_state.stats.last_search_count));
                    ui.end_row();

                    ui.label("Total Tracks:");
                    let total = mode_state.target_assignments.len();
                    let max = config.tracking.max_simultaneous_tracks;
                    let capacity_pct = (total as f64 / max as f64) * 100.0;
                    let cap_color = if capacity_pct > 90.0 {
                        egui::Color32::from_rgb(255, 100, 100)
                    } else if capacity_pct > 70.0 {
                        egui::Color32::from_rgb(255, 200, 100)
                    } else {
                        egui::Color32::from_rgb(100, 200, 100)
                    };
                    ui.colored_label(cap_color, format!("{}/{} ({:.0}%)", total, max, capacity_pct));
                    ui.end_row();

                    // Time budget
                    ui.label("Time Budget:");
                    let util = mode_state.stats.last_time_budget_utilization;
                    let time_color = if util > 0.95 {
                        egui::Color32::from_rgb(255, 100, 100)
                    } else if util > 0.75 {
                        egui::Color32::from_rgb(255, 200, 100)
                    } else {
                        egui::Color32::from_rgb(100, 200, 100)
                    };
                    ui.colored_label(time_color, format!("{:.0}%", util * 100.0));
                    ui.end_row();
                });

            // List tracked targets
            if !mode_state.target_assignments.is_empty() {
                ui.add_space(8.0);
                ui.label(egui::RichText::new("Tracked Targets:").strong());

                let mut targets: Vec<_> = mode_state.target_assignments.iter().collect();
                targets.sort_by_key(|(_, (mode, _))| match mode {
                    RadarMode::FireControl => 0,
                    RadarMode::Track => 1,
                    RadarMode::Search => 2,
                });

                for (target_id, (mode, _band)) in targets.iter().take(8) {
                    let target_name = self.simulation.missiles.iter()
                        .find(|m| m.id == **target_id)
                        .map(|m| m.name.as_str())
                        .unwrap_or("Unknown");

                    let mode_str = match mode {
                        RadarMode::FireControl => "FC",
                        RadarMode::Track => "TK",
                        RadarMode::Search => "SR",
                    };

                    let mode_color = match mode {
                        RadarMode::FireControl => egui::Color32::from_rgb(255, 150, 50),
                        RadarMode::Track => egui::Color32::from_rgb(255, 255, 100),
                        RadarMode::Search => egui::Color32::from_rgb(100, 220, 255),
                    };

                    ui.horizontal(|ui| {
                        ui.colored_label(mode_color, format!("[{}]", mode_str));
                        ui.label(target_name);
                    });
                }

                if targets.len() > 8 {
                    ui.label(format!("  ...and {} more", targets.len() - 8));
                }
            }
        } else {
            // No mode state - simple stats
            let detection_count = self.simulation.detection.detections_for_sensor(station.id).len();
            let track_count = self.simulation.detection.active_tracks.iter()
                .filter(|t| t.tracker_id == station.id)
                .count();

            ui.label(format!("Detections: {}", detection_count));
            ui.label(format!("Tracks: {}", track_count));
        }
    }

    fn render_radar_stats_window(&self, ctx: &egui::Context) {
        egui::Window::new("Radar Mode Statistics")
            .default_pos([20.0, 100.0])
            .default_width(300.0)
            .show(ctx, |ui| {
                ui.heading("Phased Array Radars");
                ui.separator();

                // Collect phased array radars with statistics
                for unit in &self.simulation.defense_units {
                    let config = self.simulation.sensor_configs.get_by_name(&unit.sensor_config_name);
                    if config.detection.radar_type != RadarType::PhasedArray {
                        continue;
                    }

                    if let Some(mode_state) = self.simulation.detection.radar_mode_states.get(&unit.id) {
                        ui.group(|ui| {
                            ui.label(egui::RichText::new(&unit.name).strong());

                            ui.horizontal(|ui| {
                                ui.label("Mode:");
                                ui.label(format!(
                                    "FC:{} T:{} S:{}",
                                    mode_state.stats.last_fc_count,
                                    mode_state.stats.last_track_count,
                                    mode_state.stats.last_search_count
                                ));
                            });

                            ui.horizontal(|ui| {
                                ui.label("Capacity:");
                                ui.label(format!(
                                    "{}/{} ({:.0}%)",
                                    mode_state.target_assignments.len(),
                                    config.tracking.max_simultaneous_tracks,
                                    (mode_state.target_assignments.len() as f64 / config.tracking.max_simultaneous_tracks as f64) * 100.0
                                ));
                            });

                            ui.horizontal(|ui| {
                                ui.label("Time Budget:");
                                let util = mode_state.stats.last_time_budget_utilization;
                                let color = if util > 0.95 {
                                    egui::Color32::RED
                                } else if util > 0.8 {
                                    egui::Color32::YELLOW
                                } else {
                                    egui::Color32::GREEN
                                };
                                ui.colored_label(color, format!("{:.1}%", util * 100.0));
                            });

                            ui.horizontal(|ui| {
                                ui.label("Mode Switches:");
                                ui.label(format!("{}", mode_state.stats.mode_switch_count));
                            });

                            ui.horizontal(|ui| {
                                ui.label("Targets Dropped:");
                                let color = if mode_state.stats.targets_dropped_count > 0 {
                                    egui::Color32::from_rgb(255, 150, 50)
                                } else {
                                    egui::Color32::GRAY
                                };
                                ui.colored_label(color, format!("{}", mode_state.stats.targets_dropped_count));
                            });
                        });
                        ui.add_space(5.0);
                    }
                }

                // Radar stations
                for station in &self.simulation.radar_stations {
                    let config = self.simulation.sensor_configs.get_by_name(&station.sensor_config_name);
                    if config.detection.radar_type != RadarType::PhasedArray {
                        continue;
                    }

                    if let Some(mode_state) = self.simulation.detection.radar_mode_states.get(&station.id) {
                        ui.group(|ui| {
                            ui.label(egui::RichText::new(&station.name).strong());

                            ui.horizontal(|ui| {
                                ui.label("Mode:");
                                ui.label(format!(
                                    "FC:{} T:{} S:{}",
                                    mode_state.stats.last_fc_count,
                                    mode_state.stats.last_track_count,
                                    mode_state.stats.last_search_count
                                ));
                            });

                            ui.horizontal(|ui| {
                                ui.label("Capacity:");
                                ui.label(format!(
                                    "{}/{} ({:.0}%)",
                                    mode_state.target_assignments.len(),
                                    config.tracking.max_simultaneous_tracks,
                                    (mode_state.target_assignments.len() as f64 / config.tracking.max_simultaneous_tracks as f64) * 100.0
                                ));
                            });

                            ui.horizontal(|ui| {
                                ui.label("Time Budget:");
                                let util = mode_state.stats.last_time_budget_utilization;
                                let color = if util > 0.95 {
                                    egui::Color32::RED
                                } else if util > 0.8 {
                                    egui::Color32::YELLOW
                                } else {
                                    egui::Color32::GREEN
                                };
                                ui.colored_label(color, format!("{:.1}%", util * 100.0));
                            });

                            ui.horizontal(|ui| {
                                ui.label("Mode Switches:");
                                ui.label(format!("{}", mode_state.stats.mode_switch_count));
                            });

                            ui.horizontal(|ui| {
                                ui.label("Targets Dropped:");
                                let color = if mode_state.stats.targets_dropped_count > 0 {
                                    egui::Color32::from_rgb(255, 150, 50)
                                } else {
                                    egui::Color32::GRAY
                                };
                                ui.colored_label(color, format!("{}", mode_state.stats.targets_dropped_count));
                            });
                        });
                        ui.add_space(5.0);
                    }
                }
            });
    }
}

impl eframe::App for App {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        // Update simulation
        let now = Instant::now();
        let dt = now.duration_since(self.last_update).as_secs_f64();
        self.last_update = now;
        self.simulation.update(dt);

        // Generate events based on state changes
        self.generate_events();

        // Update visual effects (remove finished ones)
        self.update_visual_effects();

        // Request continuous repainting when simulation is running
        if self.simulation.time_scale != crate::simulation::TimeScale::Paused {
            ctx.request_repaint();
        }

        // Top panel with controls
        egui::TopBottomPanel::top("top_panel").show(ctx, |ui| {
            ui.horizontal(|ui| {
                ui.heading("Global Thermonuclear War");
                ui.separator();
                self.render_time_controls(ui);
            });
        });

        // Bottom panel with options
        egui::TopBottomPanel::bottom("bottom_panel").show(ctx, |ui| {
            ui.horizontal(|ui| {
                // View mode toggle (Map / Globe)
                ui.label("Mode:");
                if ui.selectable_label(self.view_mode == ViewMode::Map2D, "Map").clicked() {
                    self.view_mode = ViewMode::Map2D;
                }
                if ui.selectable_label(self.view_mode == ViewMode::Globe, "Globe").clicked() {
                    self.view_mode = ViewMode::Globe;
                    // Sync globe center with current map view
                    self.globe_state.center_lat = self.viewport.center.lat;
                    self.globe_state.center_lon = self.viewport.center.lon;
                }
                if ui.selectable_label(self.view_mode == ViewMode::Intercept3D, "Intercept").clicked() {
                    self.view_mode = ViewMode::Intercept3D;
                    // Set focus to selected defense unit if any
                    if let Some(Selection::DefenseUnit(id)) = self.selection {
                        self.isometric_state.focus_unit_id = Some(id);
                    }
                }
                ui.separator();

                // View presets dropdown
                ui.label("View:");
                egui::ComboBox::from_id_salt("view_preset")
                    .selected_text("Select Region")
                    .width(120.0)
                    .show_ui(ui, |ui| {
                        for preset in get_view_presets() {
                            if ui.selectable_label(false, preset.name).clicked() {
                                self.viewport.center = preset.center;
                                self.viewport.zoom = preset.zoom;
                                // Also update globe view
                                self.globe_state.center_lat = preset.center.lat;
                                self.globe_state.center_lon = preset.center.lon;
                            }
                        }
                    });

                ui.separator();

                // Scenario button
                if ui
                    .selectable_label(self.show_scenario_panel, "Scenarios")
                    .clicked()
                {
                    self.show_scenario_panel = !self.show_scenario_panel;
                }

                ui.separator();

                // Track view mode toggle
                ui.label("Track:");
                if ui
                    .selectable_label(self.track_view_mode == TrackViewMode::TrueTrack, "True")
                    .clicked()
                {
                    self.track_view_mode = TrackViewMode::TrueTrack;
                }
                if ui
                    .selectable_label(self.track_view_mode == TrackViewMode::DetectedTrack, "Detected")
                    .clicked()
                {
                    self.track_view_mode = TrackViewMode::DetectedTrack;
                }
                // Only show false alarm toggle when in detected mode
                if self.track_view_mode == TrackViewMode::DetectedTrack {
                    ui.checkbox(&mut self.show_false_alarms, "Clutter");
                }

                ui.separator();

                ui.checkbox(&mut self.show_debug_info, "Debug");
                ui.checkbox(&mut self.show_detection_ranges, "Ranges");
                ui.checkbox(&mut self.show_trajectories, "Paths");
                ui.checkbox(&mut self.show_tracking_lines, "Tracks");
                ui.checkbox(&mut self.show_radar_stats, "Radar Stats");

                ui.separator();

                // Map style selector
                ui.horizontal(|ui| {
                    ui.label("Map:");
                    let current_style = self.tile_cache.style();
                    egui::ComboBox::from_id_salt("map_style")
                        .selected_text(current_style.display_name())
                        .show_ui(ui, |ui| {
                            for style in [
                                MapStyle::StreetsLight,
                                MapStyle::StreetsDark,
                                MapStyle::Basic,
                                MapStyle::BasicDark,
                                MapStyle::Toner,
                                MapStyle::Satellite,
                            ] {
                                if ui.selectable_label(current_style == style, style.display_name()).clicked() {
                                    self.tile_cache.set_style(style);
                                }
                            }
                        });
                });

                ui.separator();
                if ui.button("Reset View").clicked() {
                    self.viewport = Viewport::default();
                }
            });
        });

        // Engagement status panel (always visible, left side)
        self.render_engagement_panel(ctx);

        // Scenario panel (left side, toggleable)
        if self.show_scenario_panel {
            self.render_scenario_panel(ctx);
        }

        // Info panel for selected entity (must be before CentralPanel)
        self.render_info_panel(ctx);

        // Radar statistics window
        if self.show_radar_stats {
            self.render_radar_stats_window(ctx);
        }

        // Main map panel
        egui::CentralPanel::default()
            .frame(egui::Frame::NONE)
            .show(ctx, |ui| {
                self.render_map(ui);

                if self.show_debug_info {
                    egui::Area::new(egui::Id::new("debug_overlay"))
                        .fixed_pos(egui::pos2(10.0, 80.0))
                        .show(ctx, |ui| {
                            self.render_debug_overlay(ui);
                        });
                }
            });
    }
}
