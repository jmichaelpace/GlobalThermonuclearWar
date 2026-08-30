use crate::simulation::{Affiliation, DefenseType, MissileStatus, SensorType, UnitStatus};
use eframe::egui::{self, Color32, Pos2, Stroke};

/// APP-6 inspired military symbology colors
pub struct SymbolColors;

impl SymbolColors {
    /// Frame and fill colors based on affiliation
    pub fn for_affiliation(affiliation: Affiliation) -> (Color32, Color32) {
        match affiliation {
            Affiliation::Friendly => (
                Color32::from_rgb(0, 120, 215),   // Blue frame
                Color32::from_rgb(128, 179, 255), // Light blue fill
            ),
            Affiliation::Hostile => (
                Color32::from_rgb(200, 0, 0),     // Red frame
                Color32::from_rgb(255, 128, 128), // Light red fill
            ),
            Affiliation::Neutral => (
                Color32::from_rgb(0, 160, 0),     // Green frame
                Color32::from_rgb(128, 255, 128), // Light green fill
            ),
        }
    }

    /// Status indicator colors
    pub fn for_status_indicator(_engaged: bool, _tracking: bool) -> Option<Color32> {
        // No color change for status - keep it clean
        None
    }

    /// Missile phase colors
    pub fn for_missile_phase(status: MissileStatus) -> Color32 {
        match status {
            MissileStatus::PreLaunch => Color32::from_rgb(128, 128, 128),
            MissileStatus::Boost => Color32::from_rgb(255, 100, 0),
            MissileStatus::Midcourse => Color32::from_rgb(255, 50, 50),
            MissileStatus::Terminal => Color32::from_rgb(255, 0, 0),
            MissileStatus::Intercepted => Color32::from_rgb(255, 200, 0),
            MissileStatus::Impacted => Color32::from_rgb(128, 0, 0),
        }
    }
}

/// Draw military symbols on the map
pub struct MilitarySymbols;

impl MilitarySymbols {
    /// Draw a ground-based defense unit symbol (APP-6 style)
    pub fn draw_defense_unit(
        painter: &egui::Painter,
        pos: Pos2,
        affiliation: Affiliation,
        defense_type: DefenseType,
        status: UnitStatus,
        size: f32,
    ) {
        let (frame_color, fill_color) = SymbolColors::for_affiliation(affiliation);

        // Draw frame based on affiliation
        Self::draw_ground_frame(painter, pos, affiliation, fill_color, frame_color, size);

        // Draw unit type icon inside
        Self::draw_air_defense_icon(painter, pos, defense_type, frame_color, size * 0.6);

        // Draw status indicator
        Self::draw_status_indicator(painter, pos, status, size);
    }

    /// Draw a missile symbol
    /// track_quality: Optional track quality (0.0-1.0), affects transparency
    /// fire_control_locked: If true, changes from light red to dark red
    pub fn draw_missile(
        painter: &egui::Painter,
        pos: Pos2,
        affiliation: Affiliation,
        status: MissileStatus,
        heading: f32, // radians
        size: f32,
        track_quality: Option<f64>,
        fire_control_locked: bool,
    ) {
        let (mut frame_color, _fill_color) = SymbolColors::for_affiliation(affiliation);
        let mut phase_color = SymbolColors::for_missile_phase(status);

        // Apply transparency based on track quality (if provided)
        // Lower quality = more transparent
        if let Some(quality) = track_quality {
            // Map quality (0.0-1.0) to alpha (50-255)
            // Quality 0.0 = very transparent (alpha 50)
            // Quality 1.0 = fully opaque (alpha 255)
            let alpha = (quality * 205.0 + 50.0).clamp(50.0, 255.0) as u8;
            phase_color = Color32::from_rgba_unmultiplied(
                phase_color.r(),
                phase_color.g(),
                phase_color.b(),
                alpha,
            );
        }

        // Change color from light red to dark red when under fire control lock
        if fire_control_locked && affiliation == Affiliation::Hostile {
            // Much darker red/maroon for fire control locked targets
            phase_color = Color32::from_rgba_unmultiplied(120, 0, 0, phase_color.a());
            frame_color = Color32::from_rgba_unmultiplied(100, 0, 0, 255); // Dark maroon frame
        }

        match status {
            MissileStatus::PreLaunch => {
                // Small dot for pre-launch
                let mut pre_color = frame_color.gamma_multiply(0.5);
                if let Some(quality) = track_quality {
                    let alpha = (quality * 205.0 + 50.0).clamp(50.0, 255.0) as u8;
                    pre_color = Color32::from_rgba_unmultiplied(
                        pre_color.r(),
                        pre_color.g(),
                        pre_color.b(),
                        alpha,
                    );
                }
                painter.circle_filled(pos, size * 0.3, pre_color);
            }
            MissileStatus::Boost | MissileStatus::Midcourse | MissileStatus::Terminal => {
                // Fire control lock indicator - bright targeting ring
                if fire_control_locked {
                    painter.circle_stroke(
                        pos,
                        size * 2.0,
                        Stroke::new(2.5, Color32::from_rgb(255, 100, 0)), // Bright orange ring
                    );
                    painter.circle_stroke(
                        pos,
                        size * 2.3,
                        Stroke::new(1.5, Color32::from_rgba_unmultiplied(255, 100, 0, 120)), // Outer glow
                    );
                }

                // Rotated missile shape
                Self::draw_missile_icon(painter, pos, heading, phase_color, frame_color, size);

                // Glow effect with same transparency
                let glow_color = Color32::from_rgba_unmultiplied(
                    phase_color.r(),
                    phase_color.g(),
                    phase_color.b(),
                    (phase_color.a() as f32 * 0.2) as u8,
                );
                painter.circle_filled(pos, size * 1.5, glow_color);

                // Boost flame for boost phase
                if status == MissileStatus::Boost {
                    Self::draw_boost_flame(painter, pos, heading, size);
                }
            }
            MissileStatus::Intercepted => {
                // Explosion symbol
                Self::draw_explosion(painter, pos, size);
            }
            MissileStatus::Impacted => {
                // Impact marker
                Self::draw_impact_marker(painter, pos, size);
            }
        }
    }

    /// Draw a satellite symbol
    pub fn draw_satellite(
        painter: &egui::Painter,
        pos: Pos2,
        affiliation: Affiliation,
        sensor_type: SensorType,
        size: f32,
    ) {
        let (frame_color, fill_color) = SymbolColors::for_affiliation(affiliation);

        // Satellite body (hexagon)
        let points: Vec<Pos2> = (0..6)
            .map(|i| {
                let angle = (i as f32) * std::f32::consts::PI / 3.0 - std::f32::consts::PI / 6.0;
                Pos2::new(
                    pos.x + angle.cos() * size * 0.6,
                    pos.y + angle.sin() * size * 0.6,
                )
            })
            .collect();

        painter.add(egui::Shape::convex_polygon(
            points,
            fill_color,
            Stroke::new(2.0, frame_color),
        ));

        // Solar panels
        let panel_width = size * 0.3;
        let panel_height = size * 0.8;

        // Left panel
        painter.rect_filled(
            egui::Rect::from_center_size(
                Pos2::new(pos.x - size * 0.9, pos.y),
                egui::vec2(panel_width, panel_height),
            ),
            0.0,
            fill_color,
        );
        painter.rect_stroke(
            egui::Rect::from_center_size(
                Pos2::new(pos.x - size * 0.9, pos.y),
                egui::vec2(panel_width, panel_height),
            ),
            0.0,
            Stroke::new(1.0, frame_color),
            egui::StrokeKind::Outside,
        );

        // Right panel
        painter.rect_filled(
            egui::Rect::from_center_size(
                Pos2::new(pos.x + size * 0.9, pos.y),
                egui::vec2(panel_width, panel_height),
            ),
            0.0,
            fill_color,
        );
        painter.rect_stroke(
            egui::Rect::from_center_size(
                Pos2::new(pos.x + size * 0.9, pos.y),
                egui::vec2(panel_width, panel_height),
            ),
            0.0,
            Stroke::new(1.0, frame_color),
            egui::StrokeKind::Outside,
        );

        // Sensor indicator
        let sensor_color = match sensor_type {
            SensorType::Infrared => Color32::from_rgb(255, 100, 100),
            SensorType::Radar => Color32::from_rgb(100, 255, 100),
            SensorType::Both => Color32::from_rgb(255, 255, 100),
        };
        painter.circle_filled(pos, size * 0.25, sensor_color);
    }

    /// Draw a radar station symbol
    pub fn draw_radar_station(
        painter: &egui::Painter,
        pos: Pos2,
        affiliation: Affiliation,
        size: f32,
    ) {
        let (frame_color, fill_color) = SymbolColors::for_affiliation(affiliation);

        // Base (trapezoid)
        let base_points = vec![
            Pos2::new(pos.x - size * 0.6, pos.y + size * 0.3),
            Pos2::new(pos.x + size * 0.6, pos.y + size * 0.3),
            Pos2::new(pos.x + size * 0.4, pos.y - size * 0.1),
            Pos2::new(pos.x - size * 0.4, pos.y - size * 0.1),
        ];
        painter.add(egui::Shape::convex_polygon(
            base_points,
            fill_color,
            Stroke::new(1.5, frame_color),
        ));

        // Radar dish (arc)
        let dish_center = Pos2::new(pos.x, pos.y - size * 0.3);
        let dish_radius = size * 0.5;

        // Draw dish as curved line segments
        let segments = 12;
        let start_angle = -std::f32::consts::PI * 0.8;
        let end_angle = -std::f32::consts::PI * 0.2;

        let dish_points: Vec<Pos2> = (0..=segments)
            .map(|i| {
                let t = i as f32 / segments as f32;
                let angle = start_angle + (end_angle - start_angle) * t;
                Pos2::new(
                    dish_center.x + angle.cos() * dish_radius,
                    dish_center.y + angle.sin() * dish_radius,
                )
            })
            .collect();

        painter.add(egui::Shape::line(
            dish_points.clone(),
            Stroke::new(3.0, frame_color),
        ));

        // Radar waves emanating
        for i in 1..=3 {
            let wave_radius = dish_radius + (i as f32) * size * 0.15;
            let wave_points: Vec<Pos2> = (0..=8)
                .map(|j| {
                    let t = j as f32 / 8.0;
                    let angle = start_angle * 0.7 + (end_angle - start_angle) * 0.7 * t + 0.15;
                    Pos2::new(
                        dish_center.x + angle.cos() * wave_radius,
                        dish_center.y + angle.sin() * wave_radius,
                    )
                })
                .collect();

            painter.add(egui::Shape::line(
                wave_points,
                Stroke::new(1.0, frame_color.gamma_multiply(0.5 - i as f32 * 0.1)),
            ));
        }
    }

    /// Draw ground unit frame based on affiliation
    fn draw_ground_frame(
        painter: &egui::Painter,
        pos: Pos2,
        affiliation: Affiliation,
        fill: Color32,
        stroke_color: Color32,
        size: f32,
    ) {
        match affiliation {
            Affiliation::Friendly => {
                // Rounded rectangle for friendly
                painter.rect_filled(
                    egui::Rect::from_center_size(pos, egui::vec2(size * 2.0, size * 1.4)),
                    size * 0.3,
                    fill,
                );
                painter.rect_stroke(
                    egui::Rect::from_center_size(pos, egui::vec2(size * 2.0, size * 1.4)),
                    size * 0.3,
                    Stroke::new(2.0, stroke_color),
                    egui::StrokeKind::Outside,
                );
            }
            Affiliation::Hostile => {
                // Diamond for hostile
                let points = vec![
                    Pos2::new(pos.x, pos.y - size),
                    Pos2::new(pos.x + size * 1.2, pos.y),
                    Pos2::new(pos.x, pos.y + size),
                    Pos2::new(pos.x - size * 1.2, pos.y),
                ];
                painter.add(egui::Shape::convex_polygon(
                    points,
                    fill,
                    Stroke::new(2.0, stroke_color),
                ));
            }
            Affiliation::Neutral => {
                // Square for neutral
                painter.rect_filled(
                    egui::Rect::from_center_size(pos, egui::vec2(size * 1.6, size * 1.6)),
                    0.0,
                    fill,
                );
                painter.rect_stroke(
                    egui::Rect::from_center_size(pos, egui::vec2(size * 1.6, size * 1.6)),
                    0.0,
                    Stroke::new(2.0, stroke_color),
                    egui::StrokeKind::Outside,
                );
            }
        }
    }

    /// Draw air defense icon inside frame
    fn draw_air_defense_icon(
        painter: &egui::Painter,
        pos: Pos2,
        defense_type: DefenseType,
        color: Color32,
        size: f32,
    ) {
        // Common air defense symbol: upward pointing arrow/chevron
        let chevron_points = vec![
            Pos2::new(pos.x - size * 0.5, pos.y + size * 0.3),
            Pos2::new(pos.x, pos.y - size * 0.4),
            Pos2::new(pos.x + size * 0.5, pos.y + size * 0.3),
        ];

        painter.add(egui::Shape::line(chevron_points, Stroke::new(2.5, color)));

        // Add dots or lines based on defense type capability
        match defense_type {
            DefenseType::GBI | DefenseType::THAAD | DefenseType::Arrow3 => {
                // Long-range/exo-atmospheric: double chevron
                let chevron2_points = vec![
                    Pos2::new(pos.x - size * 0.35, pos.y + size * 0.5),
                    Pos2::new(pos.x, pos.y - size * 0.1),
                    Pos2::new(pos.x + size * 0.35, pos.y + size * 0.5),
                ];
                painter.add(egui::Shape::line(chevron2_points, Stroke::new(2.0, color)));
            }
            DefenseType::Aegis => {
                // Naval: add wave underneath
                painter.line_segment(
                    [
                        Pos2::new(pos.x - size * 0.5, pos.y + size * 0.5),
                        Pos2::new(pos.x + size * 0.5, pos.y + size * 0.5),
                    ],
                    Stroke::new(2.0, color),
                );
            }
            DefenseType::DavidsSling => {
                // Mid-tier: chevron with dot (sling reference)
                painter.circle_filled(Pos2::new(pos.x, pos.y + size * 0.5), size * 0.15, color);
            }
            DefenseType::Patriot | DefenseType::S400 | DefenseType::IronDome => {
                // Standard air defense: single chevron (already drawn)
            }
        }
    }

    /// Draw missile icon
    fn draw_missile_icon(
        painter: &egui::Painter,
        pos: Pos2,
        heading: f32,
        fill: Color32,
        stroke: Color32,
        size: f32,
    ) {
        // Missile body - pointed ellipse shape
        let cos_h = heading.cos();
        let sin_h = heading.sin();

        // Points for missile shape (nose, body sides, tail fins)
        let nose = Pos2::new(pos.x + cos_h * size, pos.y + sin_h * size);
        let left_body = Pos2::new(
            pos.x - sin_h * size * 0.3 - cos_h * size * 0.3,
            pos.y + cos_h * size * 0.3 - sin_h * size * 0.3,
        );
        let right_body = Pos2::new(
            pos.x + sin_h * size * 0.3 - cos_h * size * 0.3,
            pos.y - cos_h * size * 0.3 - sin_h * size * 0.3,
        );
        let tail_center = Pos2::new(pos.x - cos_h * size * 0.6, pos.y - sin_h * size * 0.6);
        let left_fin = Pos2::new(
            pos.x - sin_h * size * 0.5 - cos_h * size * 0.8,
            pos.y + cos_h * size * 0.5 - sin_h * size * 0.8,
        );
        let right_fin = Pos2::new(
            pos.x + sin_h * size * 0.5 - cos_h * size * 0.8,
            pos.y - cos_h * size * 0.5 - sin_h * size * 0.8,
        );

        // Draw body
        painter.add(egui::Shape::convex_polygon(
            vec![nose, right_body, tail_center, left_body],
            fill,
            Stroke::new(1.5, stroke),
        ));

        // Draw fins
        painter.add(egui::Shape::line(
            vec![left_body, left_fin, tail_center],
            Stroke::new(1.5, stroke),
        ));
        painter.add(egui::Shape::line(
            vec![right_body, right_fin, tail_center],
            Stroke::new(1.5, stroke),
        ));
    }

    /// Draw boost flame effect
    fn draw_boost_flame(painter: &egui::Painter, pos: Pos2, heading: f32, size: f32) {
        let cos_h = heading.cos();
        let sin_h = heading.sin();

        // Flame behind missile
        let flame_base = Pos2::new(pos.x - cos_h * size * 0.8, pos.y - sin_h * size * 0.8);
        let flame_tip = Pos2::new(pos.x - cos_h * size * 1.8, pos.y - sin_h * size * 1.8);
        let flame_left = Pos2::new(
            pos.x - sin_h * size * 0.25 - cos_h * size * 0.9,
            pos.y + cos_h * size * 0.25 - sin_h * size * 0.9,
        );
        let flame_right = Pos2::new(
            pos.x + sin_h * size * 0.25 - cos_h * size * 0.9,
            pos.y - cos_h * size * 0.25 - sin_h * size * 0.9,
        );

        painter.add(egui::Shape::convex_polygon(
            vec![flame_base, flame_left, flame_tip, flame_right],
            Color32::from_rgb(255, 150, 0),
            Stroke::new(1.0, Color32::from_rgb(255, 100, 0)),
        ));

        // Inner flame
        let inner_tip = Pos2::new(pos.x - cos_h * size * 1.4, pos.y - sin_h * size * 1.4);
        painter.add(egui::Shape::convex_polygon(
            vec![
                flame_base,
                Pos2::new(
                    flame_left.x * 0.5 + flame_base.x * 0.5,
                    flame_left.y * 0.5 + flame_base.y * 0.5,
                ),
                inner_tip,
                Pos2::new(
                    flame_right.x * 0.5 + flame_base.x * 0.5,
                    flame_right.y * 0.5 + flame_base.y * 0.5,
                ),
            ],
            Color32::from_rgb(255, 255, 100),
            Stroke::NONE,
        ));
    }

    /// Draw explosion effect
    fn draw_explosion(painter: &egui::Painter, pos: Pos2, size: f32) {
        // Outer glow
        painter.circle_filled(
            pos,
            size * 2.0,
            Color32::from_rgba_unmultiplied(255, 100, 0, 50),
        );

        // Explosion rays
        for i in 0..8 {
            let angle = (i as f32) * std::f32::consts::PI / 4.0;
            let inner_r = size * 0.5;
            let outer_r = size * 1.5;

            painter.line_segment(
                [
                    Pos2::new(pos.x + angle.cos() * inner_r, pos.y + angle.sin() * inner_r),
                    Pos2::new(pos.x + angle.cos() * outer_r, pos.y + angle.sin() * outer_r),
                ],
                Stroke::new(3.0, Color32::from_rgb(255, 200, 0)),
            );
        }

        // Center
        painter.circle_filled(pos, size * 0.8, Color32::from_rgb(255, 200, 0));
        painter.circle_filled(pos, size * 0.4, Color32::from_rgb(255, 255, 200));
    }

    /// Draw impact marker
    fn draw_impact_marker(painter: &egui::Painter, pos: Pos2, size: f32) {
        // Red circle with X
        painter.circle_filled(pos, size, Color32::from_rgb(180, 0, 0));
        painter.circle_stroke(pos, size, Stroke::new(2.0, Color32::from_rgb(255, 0, 0)));

        // X mark
        let offset = size * 0.6;
        painter.line_segment(
            [
                Pos2::new(pos.x - offset, pos.y - offset),
                Pos2::new(pos.x + offset, pos.y + offset),
            ],
            Stroke::new(2.5, Color32::WHITE),
        );
        painter.line_segment(
            [
                Pos2::new(pos.x + offset, pos.y - offset),
                Pos2::new(pos.x - offset, pos.y + offset),
            ],
            Stroke::new(2.5, Color32::WHITE),
        );
    }

    /// Draw status indicator ring
    fn draw_status_indicator(painter: &egui::Painter, pos: Pos2, status: UnitStatus, size: f32) {
        let (tracking, engaged) = match status {
            UnitStatus::Idle => (false, false),
            UnitStatus::Tracking => (true, false),
            UnitStatus::Engaged => (true, true),
            UnitStatus::Disabled => (false, false),
        };

        if let Some(color) = SymbolColors::for_status_indicator(engaged, tracking) {
            // Subtle ring effect
            painter.circle_stroke(pos, size * 1.4, Stroke::new(1.0, color));

            if engaged {
                // Second ring for engaged (subtle)
                painter.circle_stroke(
                    pos,
                    size * 1.6,
                    Stroke::new(0.75, color.gamma_multiply(0.6)),
                );
            }
        }

        // Disabled indicator
        if status == UnitStatus::Disabled {
            painter.line_segment(
                [
                    Pos2::new(pos.x - size, pos.y - size),
                    Pos2::new(pos.x + size, pos.y + size),
                ],
                Stroke::new(3.0, Color32::from_rgb(128, 128, 128)),
            );
        }
    }

    /// Draw target designator (for impact points)
    pub fn draw_target_designator(painter: &egui::Painter, pos: Pos2, size: f32) {
        let color = Color32::from_rgb(255, 50, 50);

        // Outer circle
        painter.circle_stroke(pos, size, Stroke::new(2.0, color));

        // Cross hairs extending beyond circle
        let inner = size * 0.4;
        let outer = size * 1.4;

        // Horizontal
        painter.line_segment(
            [
                Pos2::new(pos.x - outer, pos.y),
                Pos2::new(pos.x - inner, pos.y),
            ],
            Stroke::new(2.0, color),
        );
        painter.line_segment(
            [
                Pos2::new(pos.x + inner, pos.y),
                Pos2::new(pos.x + outer, pos.y),
            ],
            Stroke::new(2.0, color),
        );

        // Vertical
        painter.line_segment(
            [
                Pos2::new(pos.x, pos.y - outer),
                Pos2::new(pos.x, pos.y - inner),
            ],
            Stroke::new(2.0, color),
        );
        painter.line_segment(
            [
                Pos2::new(pos.x, pos.y + inner),
                Pos2::new(pos.x, pos.y + outer),
            ],
            Stroke::new(2.0, color),
        );

        // Center dot
        painter.circle_filled(pos, 2.0, color);
    }
}
