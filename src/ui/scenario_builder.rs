//! Scenario builder UI: egui panels, placement tools, and draft overlay.
//!
//! This module is compiled into the binary only (like `app.rs`) — it holds
//! all egui state for the builder while the pure draft model lives in
//! `src/scenario/builder.rs` (library target, unit-tested headless).
//!
//! The `ScenarioBuilder` struct is owned by `App` as an `Option` (builder
//! active ⇔ `Some`). App delegates:
//!   - left-panel rendering (`render_panel`)
//!   - map click handling (`handle_map_click`)
//!   - draft overlay drawing (`draw_overlay`)
//!   - hover position updates (`set_hover_pos`) — rubber-band preview

use eframe::egui;

use crate::map::Viewport;
use crate::scenario::{
    classify_missile_range, BuilderTool, DraftCategory, ScenarioDefinition, ScenarioDraft,
};
use crate::types::GeoCoord;

/// Colors for draft overlay rendering (distinct from live-entity styling).
mod draft_colors {
    use eframe::egui::Color32;
    /// Defense units: cyan outline
    pub const DEFENSE: Color32 = Color32::from_rgb(0, 200, 255);
    /// Radars: purple outline
    pub const RADAR: Color32 = Color32::from_rgb(200, 130, 255);
    /// Satellites: light blue outline
    pub const SATELLITE: Color32 = Color32::from_rgb(130, 180, 255);
    /// Missiles: red
    pub const MISSILE: Color32 = Color32::from_rgb(255, 90, 90);
    /// Missile target impact: red with X marker
    pub const MISSILE_TARGET: Color32 = Color32::from_rgb(255, 140, 60);
    /// Selection highlight: yellow ring
    pub const SELECTED: Color32 = Color32::from_rgb(255, 220, 60);
    /// Pending missile origin + rubber band: dashed orange
    pub const PENDING: Color32 = Color32::from_rgb(255, 170, 40);
}

/// The action App should perform after the panel renders.
#[derive(Clone, Debug, PartialEq)]
pub enum BuilderAction {
    /// Load the draft into the live engine and center the view
    TestRun { center: GeoCoord, zoom: f64 },
    /// A scenario file was saved under this id — refresh caches and
    /// select it in the scenario panel
    Saved { id: String },
    /// None (no action)
    None,
}

/// UI state for the scenario builder. The draft data itself is in `draft`.
pub struct ScenarioBuilder {
    /// The scenario under construction
    pub draft: ScenarioDraft,
    /// Currently active placement tool
    tool: BuilderTool,
    /// First click of the two-click missile flow (origin), if pending
    pending_missile_origin: Option<GeoCoord>,
    /// Currently selected draft entity
    selected: Option<DraftCategory>,
    /// Last known hover position in geo coords (for rubber band)
    hover_geo: Option<GeoCoord>,
    /// Cached validation results (recomputed on demand, not every frame)
    cached_errors: Vec<String>,
    cached_warnings: Vec<String>,
    /// Set when validation results are stale and should be recomputed
    validation_dirty: bool,
    /// Import combo state: index into the scenario list being imported
    import_open: bool,
    /// Status message shown after save/test-run actions
    status_message: Option<(String, bool)>, // (text, is_success)
    /// Defense type chosen for the next defense-unit placement
    next_defense_type: &'static str,
}

impl ScenarioBuilder {
    pub fn new() -> Self {
        Self {
            draft: ScenarioDraft::new(),
            tool: BuilderTool::Select,
            pending_missile_origin: None,
            selected: None,
            hover_geo: None,
            cached_errors: Vec::new(),
            cached_warnings: Vec::new(),
            validation_dirty: true,
            import_open: false,
            status_message: None,
            next_defense_type: "THAAD",
        }
    }

    /// Start editing an existing scenario (import flow).
    pub fn import_scenario(&mut self, def: &ScenarioDefinition) {
        self.draft = ScenarioDraft::from_scenario(def.file.clone(), def.id.clone());
        self.selected = None;
        self.pending_missile_origin = None;
        self.validation_dirty = true;
        self.status_message = Some((format!("Imported '{}'", def.name), true));
    }

    /// Escape key: cancel pending missile origin, else switch to Select.
    pub fn handle_escape(&mut self) {
        if self.pending_missile_origin.is_some() {
            self.pending_missile_origin = None;
        } else if self.tool != BuilderTool::Select {
            self.tool = BuilderTool::Select;
        }
    }

    /// Test-only accessors (keeps fields private otherwise).
    #[cfg(test)]
    pub fn set_tool_for_test(&mut self, tool: BuilderTool) {
        self.tool = tool;
        self.pending_missile_origin = None;
    }

    #[cfg(test)]
    pub fn pending_missile_origin_is_some_for_test(&self) -> bool {
        self.pending_missile_origin.is_some()
    }

    /// Called every frame with the current hover position (geo coords).
    pub fn set_hover_pos(&mut self, geo: Option<GeoCoord>) {
        self.hover_geo = geo;
    }

    // ------------------------------------------------------------------
    // Map interaction
    // ------------------------------------------------------------------

    /// Handle a map click in geo coordinates. Returns true if the builder
    /// consumed the click (App should not run its normal selection logic).
    pub fn handle_map_click(&mut self, geo: GeoCoord) -> bool {
        match self.tool {
            BuilderTool::Select => {
                // Hit-test draft entities; select if hit
                self.selected = self.hit_test(geo);
                self.selected.is_some()
            }
            BuilderTool::DefenseUnit => {
                let idx = self.draft.add_defense_unit(geo, self.next_defense_type);
                self.selected = Some(DraftCategory::DefenseUnit(idx));
                self.validation_dirty = true;
                true
            }
            BuilderTool::RadarStation => {
                let idx = self.draft.add_radar_station(geo);
                self.selected = Some(DraftCategory::RadarStation(idx));
                self.validation_dirty = true;
                true
            }
            BuilderTool::Satellite => {
                let idx = self.draft.add_satellite(geo);
                self.selected = Some(DraftCategory::Satellite(idx));
                self.validation_dirty = true;
                true
            }
            BuilderTool::Missile => {
                match self.pending_missile_origin {
                    None => {
                        // First click: origin
                        self.pending_missile_origin = Some(geo);
                        true
                    }
                    Some(origin) => {
                        // Second click: target — create the missile
                        let idx = self.draft.add_missile(origin, geo);
                        self.selected = Some(DraftCategory::Missile(idx));
                        self.pending_missile_origin = None;
                        self.validation_dirty = true;
                        true
                    }
                }
            }
        }
    }

    /// Hit-test draft entities at a geo position. App calls this via
    /// `handle_map_click` in Select mode; the threshold is in approximate
    /// degrees (pixel-accurate hit-testing would need screen coords, but the
    /// draft markers are drawn with generous 10-14 px hit areas that this
    /// approximates well at typical zoom levels).
    fn hit_test(&self, geo: GeoCoord) -> Option<DraftCategory> {
        let threshold_deg = 0.8;
        let near = |pos: GeoCoord| -> bool {
            (geo.lat - pos.lat).abs() < threshold_deg && (geo.lon - pos.lon).abs() < threshold_deg
        };
        self.draft
            .placed_positions()
            .into_iter()
            .find(|(pos, _)| near(*pos))
            .map(|(_, sel)| sel)
    }

    // ------------------------------------------------------------------
    // Actions requested by the panel; App performs the engine-side effects
    // ------------------------------------------------------------------

    // (BuilderAction is at module scope — see above.)

    // ------------------------------------------------------------------
    // Panel
    // ------------------------------------------------------------------

    /// Render the builder side panel. Returns an action for App to perform.
    pub fn render_panel(
        &mut self,
        ctx: &egui::Context,
        importable: &[ScenarioDefinition],
        existing_ids: &[String],
    ) -> BuilderAction {
        let mut action = BuilderAction::None;

        egui::SidePanel::left("scenario_builder_panel")
            .resizable(true)
            .default_width(330.0)
            .show(ctx, |ui| {
                egui::ScrollArea::vertical().show(ui, |ui| {
                    ui.heading("Scenario Builder");
                    ui.label(
                        egui::RichText::new(format!(
                            "{} entities, file: {}.toml",
                            self.draft.entity_count(),
                            self.draft.filename
                        ))
                        .small()
                        .weak(),
                    );
                    if let Some((msg, ok)) = &self.status_message {
                        let color = if *ok {
                            egui::Color32::from_rgb(150, 200, 150)
                        } else {
                            egui::Color32::from_rgb(230, 120, 120)
                        };
                        ui.label(egui::RichText::new(msg).small().color(color));
                    }
                    ui.separator();

                    self.render_tools(ui);
                    ui.add_space(6.0);

                    self.render_import(ui, importable);
                    ui.add_space(6.0);

                    self.render_metadata(ui);
                    ui.add_space(6.0);

                    self.render_entity_lists(ui);
                    ui.add_space(6.0);

                    self.render_property_editor(ui);
                    ui.add_space(6.0);

                    self.render_validation(ui);
                    ui.add_space(6.0);

                    action = self.render_actions(ui, existing_ids);
                });
            });

        action
    }

    fn render_tools(&mut self, ui: &mut egui::Ui) {
        ui.horizontal(|ui| {
            ui.label("Tool:");
            tool_button(ui, BuilderTool::Select, "Select", &mut self.tool);
            tool_button(ui, BuilderTool::DefenseUnit, "Defense", &mut self.tool);
            tool_button(ui, BuilderTool::RadarStation, "Radar", &mut self.tool);
            tool_button(ui, BuilderTool::Satellite, "Satellite", &mut self.tool);
            tool_button(ui, BuilderTool::Missile, "Missile", &mut self.tool);
        });
        if self.tool == BuilderTool::DefenseUnit {
            ui.horizontal(|ui| {
                ui.label("Type:");
                egui::ComboBox::from_id_salt("builder_next_defense_type")
                    .selected_text(self.next_defense_type)
                    .width(140.0)
                    .show_ui(ui, |ui| {
                        for t in crate::scenario::builder::DEFENSE_TYPES {
                            ui.selectable_value(&mut self.next_defense_type, t, t);
                        }
                    });
            });
        }
        if self.tool == BuilderTool::Missile {
            if self.pending_missile_origin.is_some() {
                ui.label(
                    egui::RichText::new("Click the TARGET point (Esc cancels)")
                        .color(draft_colors::PENDING),
                );
            } else {
                ui.label("Click the missile's LAUNCH point, then its target.");
            }
        }
    }

    fn render_import(&mut self, ui: &mut egui::Ui, importable: &[ScenarioDefinition]) {
        if importable.is_empty() {
            return;
        }
        egui::CollapsingHeader::new("Import Existing Scenario")
            .default_open(false)
            .show(ui, |ui| {
                if ui.button("Choose scenario to edit...").clicked() {
                    self.import_open = !self.import_open;
                }
                if self.import_open {
                    egui::ScrollArea::vertical()
                        .max_height(180.0)
                        .show(ui, |ui| {
                            for def in importable {
                                if ui.selectable_label(false, &def.name).clicked() {
                                    self.import_scenario(def);
                                    self.import_open = false;
                                }
                            }
                        });
                }
            });
    }

    fn render_metadata(&mut self, ui: &mut egui::Ui) {
        egui::CollapsingHeader::new("Metadata")
            .default_open(true)
            .show(ui, |ui| {
                egui::Grid::new("builder_metadata")
                    .num_columns(2)
                    .spacing([8.0, 4.0])
                    .show(ui, |ui| {
                        ui.label("Name:");
                        ui.text_edit_singleline(&mut self.draft.file.metadata.name);
                        ui.end_row();

                        ui.label("Description:");
                        ui.text_edit_singleline(&mut self.draft.file.metadata.description);
                        ui.end_row();

                        ui.label("Region:");
                        ui.text_edit_singleline(&mut self.draft.file.metadata.region);
                        ui.end_row();

                        ui.label("Filename:");
                        let mut fname = self.draft.filename.clone();
                        ui.text_edit_singleline(&mut fname);
                        if fname != self.draft.filename {
                            self.draft.filename = fname;
                            self.validation_dirty = true;
                        }
                        ui.end_row();

                        ui.label("Auto center:");
                        if ui
                            .checkbox(&mut self.draft.auto_center, "from entities")
                            .changed()
                        {
                            self.validation_dirty = true;
                        }
                        ui.end_row();
                    });
            });
    }

    fn render_entity_lists(&mut self, ui: &mut egui::Ui) {
        let mut changed = false;

        egui::CollapsingHeader::new(format!(
            "Defense Units ({})",
            self.draft.file.defense_units.len()
        ))
        .default_open(false)
        .show(ui, |ui| {
            for (i, u) in self.draft.file.defense_units.iter().enumerate() {
                let is_sel = self.selected == Some(DraftCategory::DefenseUnit(i));
                if ui
                    .selectable_label(is_sel, format!("{} [{}]", u.name, u.defense_type))
                    .clicked()
                {
                    self.selected = Some(DraftCategory::DefenseUnit(i));
                }
            }
            ui.horizontal(|ui| {
                if ui.small_button("+ at cursor").clicked() {
                    if let Some(g) = self.hover_geo {
                        let idx = self.draft.add_defense_unit(g, self.next_defense_type);
                        self.selected = Some(DraftCategory::DefenseUnit(idx));
                        changed = true;
                    } else {
                        self.status_message = Some((
                            "Hover the map first, then use + at cursor".to_string(),
                            false,
                        ));
                    }
                }
            });
        });

        egui::CollapsingHeader::new(format!("Radars ({})", self.draft.file.radar_stations.len()))
            .default_open(false)
            .show(ui, |ui| {
                for (i, r) in self.draft.file.radar_stations.iter().enumerate() {
                    let is_sel = self.selected == Some(DraftCategory::RadarStation(i));
                    if ui
                        .selectable_label(is_sel, format!("{} [{:.0} km]", r.name, r.range_km))
                        .clicked()
                    {
                        self.selected = Some(DraftCategory::RadarStation(i));
                    }
                }
            });

        egui::CollapsingHeader::new(format!("Satellites ({})", self.draft.file.satellites.len()))
            .default_open(false)
            .show(ui, |ui| {
                for (i, s) in self.draft.file.satellites.iter().enumerate() {
                    let is_sel = self.selected == Some(DraftCategory::Satellite(i));
                    if ui
                        .selectable_label(
                            is_sel,
                            format!("{} [{:.0} km, {}]", s.name, s.altitude_km, s.sensor_type),
                        )
                        .clicked()
                    {
                        self.selected = Some(DraftCategory::Satellite(i));
                    }
                }
            });

        egui::CollapsingHeader::new(format!("Missiles ({})", self.draft.file.missiles.len()))
            .default_open(true)
            .show(ui, |ui| {
                for (i, m) in self.draft.file.missiles.iter().enumerate() {
                    let range = haversine_of_missile(m);
                    let class = classify_missile_range(range);
                    let is_sel = self.selected == Some(DraftCategory::Missile(i));
                    if ui
                        .selectable_label(
                            is_sel,
                            format!(
                                "{} [{}, {:.0} km, delay {:.0}s]",
                                m.name, class, range, m.launch_delay_sec
                            ),
                        )
                        .clicked()
                    {
                        self.selected = Some(DraftCategory::Missile(i));
                    }
                }
            });

        if changed {
            self.validation_dirty = true;
        }
    }

    /// Small helper: delete/duplicate buttons that act on the current selection.
    fn render_property_editor(&mut self, ui: &mut egui::Ui) {
        let sel = match self.selected {
            Some(s) => s,
            None => {
                ui.label(
                    egui::RichText::new("Select an entity to edit its properties")
                        .weak()
                        .italics(),
                );
                return;
            }
        };

        ui.separator();
        let mut changed = false;

        egui::Grid::new("builder_property_editor")
            .num_columns(2)
            .spacing([8.0, 4.0])
            .show(ui, |ui| {
                match sel {
                    DraftCategory::DefenseUnit(i) => {
                        let Some(u) = self.draft.file.defense_units.get_mut(i) else {
                            ui.label("(index out of range)");
                            return;
                        };
                        ui.label("Defense Unit:");
                        ui.strong(&u.name);
                        ui.end_row();

                        changed |= text_field(ui, "Name:", &mut u.name);
                        changed |= combo_field(
                            ui,
                            "Affiliation:",
                            &mut u.affiliation,
                            &crate::scenario::builder::AFFILIATIONS,
                        );
                        changed |= combo_field(
                            ui,
                            "Type:",
                            &mut u.defense_type,
                            &crate::scenario::builder::DEFENSE_TYPES,
                        );
                        changed |= u32_field(ui, "Interceptors:", &mut u.interceptors, 0, 500);
                    }
                    DraftCategory::RadarStation(i) => {
                        let Some(r) = self.draft.file.radar_stations.get_mut(i) else {
                            ui.label("(index out of range)");
                            return;
                        };
                        ui.label("Radar Station:");
                        ui.strong(&r.name);
                        ui.end_row();

                        changed |= text_field(ui, "Name:", &mut r.name);
                        changed |= combo_field(
                            ui,
                            "Affiliation:",
                            &mut r.affiliation,
                            &crate::scenario::builder::AFFILIATIONS,
                        );
                        changed |= f64_field(ui, "Range km:", &mut r.range_km, 10.0, 6000.0);
                        // sensor_config: free text referencing config/sensors/<name>.toml
                        let mut cfg_text = r.sensor_config.clone().unwrap_or_default();
                        ui.label("Sensor cfg:");
                        if ui.text_edit_singleline(&mut cfg_text).changed() {
                            r.sensor_config = if cfg_text.trim().is_empty() {
                                None
                            } else {
                                Some(cfg_text.trim().to_string())
                            };
                            changed = true;
                        }
                        ui.end_row();
                        let mut facing = r.facing_deg.unwrap_or(0.0);
                        if f64_field(ui, "Facing deg:", &mut facing, 0.0, 360.0) {
                            r.facing_deg = Some(facing);
                            changed = true;
                        }
                    }
                    DraftCategory::Satellite(i) => {
                        let Some(s) = self.draft.file.satellites.get_mut(i) else {
                            ui.label("(index out of range)");
                            return;
                        };
                        ui.label("Satellite:");
                        ui.strong(&s.name);
                        ui.end_row();

                        changed |= text_field(ui, "Name:", &mut s.name);
                        changed |= combo_field(
                            ui,
                            "Affiliation:",
                            &mut s.affiliation,
                            &crate::scenario::builder::AFFILIATIONS,
                        );
                        changed |=
                            f64_field(ui, "Altitude km:", &mut s.altitude_km, 100.0, 40000.0);
                        changed |= combo_field(
                            ui,
                            "Sensor:",
                            &mut s.sensor_type,
                            &crate::scenario::builder::SATELLITE_SENSOR_TYPES,
                        );
                    }
                    DraftCategory::Missile(i) => {
                        let Some(m) = self.draft.file.missiles.get_mut(i) else {
                            ui.label("(index out of range)");
                            return;
                        };
                        let range = haversine_of_missile(m);
                        ui.label("Missile:");
                        ui.strong(format!(
                            "{}  ({}, {:.0} km)",
                            m.name,
                            classify_missile_range(range),
                            range
                        ));
                        ui.end_row();

                        changed |= text_field(ui, "Name:", &mut m.name);
                        changed |= combo_field(
                            ui,
                            "Affiliation:",
                            &mut m.affiliation,
                            &crate::scenario::builder::AFFILIATIONS,
                        );
                        changed |= f64_field(ui, "Origin lat:", &mut m.origin_lat, -90.0, 90.0);
                        changed |= f64_field(ui, "Origin lon:", &mut m.origin_lon, -180.0, 180.0);
                        changed |= f64_field(ui, "Target lat:", &mut m.target_lat, -90.0, 90.0);
                        changed |= f64_field(ui, "Target lon:", &mut m.target_lon, -180.0, 180.0);
                        changed |=
                            f64_field(ui, "Delay sec:", &mut m.launch_delay_sec, 0.0, 3600.0);
                    }
                }
            });

        ui.add_space(4.0);
        ui.horizontal(|ui| {
            if ui.button("Duplicate").clicked() {
                if let Some(new_sel) = self.draft.duplicate(sel) {
                    self.selected = Some(new_sel);
                    changed = true;
                }
            }
            if ui.button("Delete").clicked() {
                self.draft.remove(sel);
                self.selected = None;
                changed = true;
            }
        });

        if changed {
            self.validation_dirty = true;
        }
    }

    fn render_validation(&mut self, ui: &mut egui::Ui) {
        if self.validation_dirty {
            let (errors, warnings) = self.draft.validate();
            self.cached_errors = errors;
            self.cached_warnings = warnings;
            self.validation_dirty = false;
        }

        ui.separator();
        ui.label(format!(
            "Validation: {} error(s), {} warning(s)",
            self.cached_errors.len(),
            self.cached_warnings.len()
        ));
        for e in &self.cached_errors {
            ui.label(
                egui::RichText::new(format!("✗ {e}"))
                    .small()
                    .color(egui::Color32::from_rgb(230, 120, 120)),
            );
        }
        for w in &self.cached_warnings {
            ui.label(
                egui::RichText::new(format!("⚠ {w}"))
                    .small()
                    .color(egui::Color32::from_rgb(220, 180, 90)),
            );
        }
    }

    fn render_actions(&mut self, ui: &mut egui::Ui, existing_ids: &[String]) -> BuilderAction {
        ui.separator();
        let is_valid = self.cached_errors.is_empty();
        let would_overwrite = existing_ids
            .iter()
            .any(|id| id == &ScenarioDraft::sanitize_filename(&self.draft.filename));

        // The ui.horizontal closure can't early-return the outer function, so
        // the action is accumulated in `action` instead.
        let mut action = BuilderAction::None;

        ui.horizontal(|ui| {
            if ui
                .add_enabled(is_valid, egui::Button::new("Test Run"))
                .on_disabled_hover_text("Fix validation errors first")
                .clicked()
            {
                self.draft.sync_metadata();
                self.validation_dirty = true;
                let center = GeoCoord::new(
                    self.draft.file.metadata.center_lat,
                    self.draft.file.metadata.center_lon,
                );
                action = BuilderAction::TestRun {
                    center,
                    zoom: self.draft.file.metadata.zoom,
                };
            }

            if ui
                .add_enabled(is_valid, egui::Button::new("Save"))
                .on_disabled_hover_text("Fix validation errors first")
                .clicked()
            {
                self.draft.sync_metadata();
                match self.draft.save() {
                    Ok(path) => {
                        self.status_message = Some((format!("Saved {}", path.display()), true));
                        action = BuilderAction::Saved {
                            id: self.draft.file.metadata.id.clone(),
                        };
                    }
                    Err(e) => {
                        self.status_message = Some((format!("Save failed: {e}"), false));
                    }
                }
            }
        });

        if would_overwrite {
            ui.label(
                egui::RichText::new("Note: saving will overwrite the existing scenario file")
                    .small()
                    .color(egui::Color32::from_rgb(220, 180, 90)),
            );
        }

        action
    }

    // ------------------------------------------------------------------
    // Overlay drawing
    // ------------------------------------------------------------------

    /// Draw all draft entities on the 2D map.
    pub fn draw_overlay(
        &self,
        painter: &egui::Painter,
        screen_rect: egui::Rect,
        viewport: &Viewport,
    ) {
        // Defense units
        for (i, u) in self.draft.file.defense_units.iter().enumerate() {
            let pos = GeoCoord::new(u.lat, u.lon);
            {
                let screen = viewport.geo_to_screen(pos, screen_rect);
                let selected = self.selected == Some(DraftCategory::DefenseUnit(i));
                draw_marker(painter, screen, draft_colors::DEFENSE, selected, &u.name);
                // Engagement ring (config-based range is engine-side; show a
                // nominal 200 km ring for spatial context)
                draw_range_ring(
                    painter,
                    screen,
                    viewport,
                    screen_rect,
                    pos,
                    200.0,
                    draft_colors::DEFENSE,
                );
            }
        }

        // Radars
        for (i, r) in self.draft.file.radar_stations.iter().enumerate() {
            let pos = GeoCoord::new(r.lat, r.lon);
            {
                let screen = viewport.geo_to_screen(pos, screen_rect);
                let selected = self.selected == Some(DraftCategory::RadarStation(i));
                draw_marker(painter, screen, draft_colors::RADAR, selected, &r.name);
                draw_range_ring(
                    painter,
                    screen,
                    viewport,
                    screen_rect,
                    pos,
                    r.range_km,
                    draft_colors::RADAR,
                );
            }
        }

        // Satellites (positions on the map plane, labeled with altitude)
        for (i, s) in self.draft.file.satellites.iter().enumerate() {
            let pos = GeoCoord::new(s.lat, s.lon);
            {
                let screen = viewport.geo_to_screen(pos, screen_rect);
                let selected = self.selected == Some(DraftCategory::Satellite(i));
                draw_marker(painter, screen, draft_colors::SATELLITE, selected, &s.name);
            }
        }

        // Missiles: origin → target line + endpoints
        for (i, m) in self.draft.file.missiles.iter().enumerate() {
            let origin = GeoCoord::new(m.origin_lat, m.origin_lon);
            let target = GeoCoord::new(m.target_lat, m.target_lon);
            let selected = self.selected == Some(DraftCategory::Missile(i));
            let line_color = if selected {
                draft_colors::SELECTED
            } else {
                draft_colors::MISSILE
            };

            {
                let o = viewport.geo_to_screen(origin, screen_rect);
                let t = viewport.geo_to_screen(target, screen_rect);
                painter.line_segment([o, t], (2.0, line_color));
                // Origin marker
                painter.circle_filled(o, 5.0, line_color);
                // Target X marker
                draw_x(painter, t, 6.0, draft_colors::MISSILE_TARGET);
                if selected {
                    painter.text(
                        t + egui::vec2(8.0, 0.0),
                        egui::Align2::LEFT_CENTER,
                        format!("{} → impact", m.name),
                        egui::FontId::proportional(11.0),
                        draft_colors::MISSILE_TARGET,
                    );
                }
            }
        }

        // Pending missile rubber band
        if let (Some(origin), Some(hover)) = (self.pending_missile_origin, self.hover_geo) {
            {
                let o = viewport.geo_to_screen(origin, screen_rect);
                let h = viewport.geo_to_screen(hover, screen_rect);
                // Dashed effect: draw short segments along the line
                let dist = o.distance(h);
                let segments = ((dist / 12.0).ceil() as usize).max(1);
                let lerp_pos = |a: egui::Pos2, b: egui::Pos2, t: f32| a + (b - a) * t;
                for s in 0..segments {
                    let t0 = s as f32 / segments as f32;
                    let t1 = (s + 1) as f32 / segments as f32;
                    let a = lerp_pos(o, h, t0);
                    let b = lerp_pos(o, h, t1);
                    if s % 2 == 0 {
                        painter.line_segment([a, b], (2.0, draft_colors::PENDING));
                    }
                }
                painter.circle_filled(o, 5.0, draft_colors::PENDING);
            }
        }
    }
}

impl Default for ScenarioBuilder {
    fn default() -> Self {
        Self::new()
    }
}

// ============================================================================
// Small drawing and widget helpers
// ============================================================================

fn tool_button(ui: &mut egui::Ui, tool: BuilderTool, label: &str, current: &mut BuilderTool) {
    if ui.selectable_label(*current == tool, label).clicked() {
        *current = tool;
    }
}

/// Round marker with optional selection ring and label.
fn draw_marker(
    painter: &egui::Painter,
    pos: egui::Pos2,
    color: egui::Color32,
    selected: bool,
    label: &str,
) {
    painter.circle_stroke(pos, 7.0, (2.0, color));
    painter.circle_filled(pos, 2.5, color);
    if selected {
        painter.circle_stroke(pos, 10.0, (2.0, draft_colors::SELECTED));
    }
    painter.text(
        pos + egui::vec2(10.0, -10.0),
        egui::Align2::LEFT_BOTTOM,
        label,
        egui::FontId::proportional(11.0),
        color,
    );
}

/// X marker for missile impact points.
fn draw_x(painter: &egui::Painter, pos: egui::Pos2, r: f32, color: egui::Color32) {
    let d = egui::vec2(r, r);
    painter.line_segment([pos - d, pos + d], (2.0, color));
    painter.line_segment(
        [pos - egui::vec2(r, -r), pos + egui::vec2(r, -r)],
        (2.0, color),
    );
}

/// Draw a ground-range ring (km) around a position.
fn draw_range_ring(
    painter: &egui::Painter,
    _screen: egui::Pos2,
    viewport: &Viewport,
    screen_rect: egui::Rect,
    center: GeoCoord,
    radius_km: f64,
    color: egui::Color32,
) {
    // Sample the circle in geo space, project to screen, draw as polyline
    let points: Vec<egui::Pos2> = (0..=32)
        .map(|k| {
            let bearing = k as f64 / 32.0 * 360.0;
            let pos = crate::simulation::calculate_position_from_bearing_range(
                center, bearing, radius_km,
            );
            viewport.geo_to_screen(pos, screen_rect)
        })
        .collect();
    if points.len() > 1 {
        painter.add(egui::Shape::line(points, (1.0, color.gamma_multiply(0.4))));
    }
}

fn haversine_of_missile(m: &crate::scenario::loader::MissileConfig) -> f64 {
    crate::simulation::haversine_distance(
        GeoCoord::new(m.origin_lat, m.origin_lon),
        GeoCoord::new(m.target_lat, m.target_lon),
    )
}

// --- Grid field widget helpers ---

fn text_field(ui: &mut egui::Ui, label: &str, value: &mut String) -> bool {
    ui.label(label);
    let changed = ui.text_edit_singleline(value).changed();
    ui.end_row();
    changed
}

fn combo_field(ui: &mut egui::Ui, label: &str, value: &mut String, options: &[&str]) -> bool {
    ui.label(label);
    let id = format!("combo_{}", label);
    let mut changed = false;
    egui::ComboBox::from_id_salt(id)
        .selected_text(value.clone())
        .show_ui(ui, |ui| {
            for opt in options {
                if ui.selectable_value(value, opt.to_string(), *opt).changed() {
                    changed = true;
                }
            }
        });
    ui.end_row();
    changed
}

fn f64_field(ui: &mut egui::Ui, label: &str, value: &mut f64, min: f64, max: f64) -> bool {
    ui.label(label);
    let mut temp = *value;
    let changed = ui
        .add(egui::DragValue::new(&mut temp).range(min..=max).speed(0.1))
        .changed();
    if changed {
        *value = temp;
    }
    ui.end_row();
    changed
}

fn u32_field(ui: &mut egui::Ui, label: &str, value: &mut u32, min: u32, max: u32) -> bool {
    ui.label(label);
    let mut temp = *value;
    let changed = ui
        .add(egui::DragValue::new(&mut temp).range(min..=max).speed(0.1))
        .changed();
    if changed {
        *value = temp;
    }
    ui.end_row();
    changed
}

// ============================================================================
// Headless render tests (bin target #[cfg(test)])
//
// These drive the panel and overlay through a real egui::Context without a
// window. Compilation can't catch egui runtime panics (duplicate IDs, bad
// drag-value ranges, painter misuse), so rendering once per state variant is
// the cheapest guard. Run with: cargo test --bin global-thermonuclear-war
// ============================================================================

#[cfg(test)]
mod render_tests {
    use super::*;
    use crate::map::Viewport;
    use crate::types::GeoCoord;

    /// One headless frame: run the UI closure inside a fresh context pass.
    fn run_headless(ctx: &egui::Context, mut ui_closure: impl FnMut(&egui::Context)) {
        // A single run pass is enough for the render assertions; egui's
        // internal state resets with the default context each test anyway.
        let _ = ctx.run(egui::RawInput::default(), |ctx| ui_closure(ctx));
    }

    fn sample_definitions() -> Vec<ScenarioDefinition> {
        crate::scenario::get_scenarios()
    }

    #[test]
    fn test_render_panel_fresh_draft() {
        let ctx = egui::Context::default();
        let mut builder = ScenarioBuilder::new();
        let defs = sample_definitions();
        let ids: Vec<String> = defs.iter().map(|d| d.id.clone()).collect();

        run_headless(&ctx, |ctx| {
            let _ = builder.render_panel(ctx, &defs, &ids);
        });

        // Fresh draft: all entity lists are empty (no errors possible), but
        // the composition warnings ("No missiles", "No defense units") fire.
        assert!(
            builder.cached_errors.is_empty(),
            "fresh draft must have no errors, got: {:?}",
            builder.cached_errors
        );
        assert!(
            !builder.cached_warnings.is_empty(),
            "fresh draft must warn about missing missiles/defenders"
        );
    }

    #[test]
    fn test_render_panel_populated_and_selected() {
        let ctx = egui::Context::default();
        let mut builder = ScenarioBuilder::new();

        // Populate: unit, radar, satellite, missile
        builder
            .draft
            .add_defense_unit(GeoCoord::new(37.0, 132.0), "Aegis");
        builder.draft.add_radar_station(GeoCoord::new(37.5, 133.0));
        builder.draft.add_satellite(GeoCoord::new(0.0, 130.0));
        builder
            .draft
            .add_missile(GeoCoord::new(39.0, 125.5), GeoCoord::new(35.0, 139.0));
        builder.selected = Some(DraftCategory::Missile(0));
        builder.validation_dirty = true;

        let defs = sample_definitions();
        let ids: Vec<String> = defs.iter().map(|d| d.id.clone()).collect();

        run_headless(&ctx, |ctx| {
            let _ = builder.render_panel(ctx, &defs, &ids);
        });

        // Populated draft with distinct entities should validate cleanly
        assert!(
            builder.cached_errors.is_empty(),
            "expected clean validation, got errors: {:?}",
            builder.cached_errors
        );

        // Every entity category rendered without panic; counts intact
        assert_eq!(builder.draft.entity_count(), 4);
    }

    #[test]
    fn test_render_panel_each_selection() {
        let ctx = egui::Context::default();
        let mut builder = ScenarioBuilder::new();
        builder
            .draft
            .add_defense_unit(GeoCoord::new(37.0, 132.0), "THAAD");
        builder.draft.add_radar_station(GeoCoord::new(37.5, 133.0));
        builder.draft.add_satellite(GeoCoord::new(0.0, 130.0));
        builder
            .draft
            .add_missile(GeoCoord::new(39.0, 125.5), GeoCoord::new(35.0, 139.0));

        let defs = sample_definitions();
        let ids: Vec<String> = defs.iter().map(|d| d.id.clone()).collect();

        // Render once per selection — each property editor variant must
        // survive a frame (grid layout, drag values, combos)
        for sel in [
            DraftCategory::DefenseUnit(0),
            DraftCategory::RadarStation(0),
            DraftCategory::Satellite(0),
            DraftCategory::Missile(0),
        ] {
            builder.selected = Some(sel);
            builder.validation_dirty = true;
            run_headless(&ctx, |ctx| {
                let _ = builder.render_panel(ctx, &defs, &ids);
            });
        }
    }

    #[test]
    fn test_draw_overlay_entities_visible() {
        let ctx = egui::Context::default();
        let mut builder = ScenarioBuilder::new();
        builder
            .draft
            .add_defense_unit(GeoCoord::new(37.0, 132.0), "Aegis");
        builder.draft.add_radar_station(GeoCoord::new(37.5, 133.0));
        builder
            .draft
            .add_missile(GeoCoord::new(39.0, 125.5), GeoCoord::new(35.0, 139.0));
        builder.selected = Some(DraftCategory::DefenseUnit(0));

        let viewport = Viewport::default();
        let screen_rect =
            egui::Rect::from_min_size(egui::pos2(0.0, 0.0), egui::vec2(1280.0, 800.0));

        run_headless(&ctx, |ctx| {
            egui::CentralPanel::default()
                .frame(egui::Frame::NONE)
                .show(ctx, |ui| {
                    let painter = ui.painter_at(screen_rect);
                    builder.draw_overlay(&painter, screen_rect, &viewport);
                });
        });
        // No panic = pass. The painter exercises line/marker/range-ring paths.
    }

    #[test]
    fn test_map_click_place_flow() {
        let mut builder = ScenarioBuilder::new();

        // Select tool on empty draft: click hits nothing
        assert!(!builder.handle_map_click(GeoCoord::new(10.0, 10.0)));

        // Defense tool: click places
        builder.set_tool_for_test(BuilderTool::DefenseUnit);
        assert!(builder.handle_map_click(GeoCoord::new(37.0, 132.0)));
        assert_eq!(builder.draft.file.defense_units.len(), 1);

        // Missile tool: two clicks complete the flow
        builder.set_tool_for_test(BuilderTool::Missile);
        assert!(builder.handle_map_click(GeoCoord::new(39.0, 125.5)));
        assert!(builder.pending_missile_origin_is_some_for_test());
        assert!(builder.handle_map_click(GeoCoord::new(35.0, 139.0)));
        assert!(!builder.pending_missile_origin_is_some_for_test());
        assert_eq!(builder.draft.file.missiles.len(), 1);

        // Escape cancels a pending missile
        builder.set_tool_for_test(BuilderTool::Missile);
        builder.handle_map_click(GeoCoord::new(39.0, 125.5));
        builder.handle_escape();
        assert!(!builder.pending_missile_origin_is_some_for_test());
    }
}
