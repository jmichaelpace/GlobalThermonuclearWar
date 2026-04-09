use crate::map::GeoCoord;
use eframe::egui;

/// Type of visual effect
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EffectType {
    Impact,      // Missile impact explosion
    Intercept,   // Successful intercept
    Debris,      // Debris cloud from intercept
}

/// A visual effect to render
#[derive(Clone, Debug)]
pub struct VisualEffect {
    pub position: GeoCoord,
    pub effect_type: EffectType,
    pub start_time: f64,
    pub duration: f64,
}

impl VisualEffect {
    pub fn new(position: GeoCoord, effect_type: EffectType, start_time: f64) -> Self {
        let duration = match effect_type {
            EffectType::Impact => 3.0,     // 3 seconds
            EffectType::Intercept => 2.0,  // 2 seconds
            EffectType::Debris => 2.5,     // 2.5 seconds
        };
        Self { position, effect_type, start_time, duration }
    }

    pub fn progress(&self, current_time: f64) -> f64 {
        ((current_time - self.start_time) / self.duration).clamp(0.0, 1.0)
    }

    pub fn is_finished(&self, current_time: f64) -> bool {
        current_time > self.start_time + self.duration
    }
}

/// Manages visual effects (explosions, intercepts, debris)
pub struct EffectsManager {
    effects: Vec<VisualEffect>,
}

impl Default for EffectsManager {
    fn default() -> Self {
        Self::new()
    }
}

impl EffectsManager {
    pub fn new() -> Self {
        Self {
            effects: Vec::new(),
        }
    }

    /// Spawn a new effect at the given position
    pub fn spawn(&mut self, position: GeoCoord, effect_type: EffectType, start_time: f64) {
        self.effects.push(VisualEffect::new(position, effect_type, start_time));
    }

    /// Update effects, removing finished ones
    pub fn update(&mut self, current_time: f64) {
        self.effects.retain(|effect| !effect.is_finished(current_time));
    }

    /// Clear all effects
    pub fn clear(&mut self) {
        self.effects.clear();
    }

    /// Get all active effects (for rendering)
    pub fn effects(&self) -> &[VisualEffect] {
        &self.effects
    }

    /// Render an impact explosion effect with debris
    pub fn render_impact_effect(painter: &egui::Painter, pos: egui::Pos2, progress: f64) {
        let progress = progress as f32;

        // Initial bright flash (first 10%)
        if progress < 0.1 {
            let flash_progress = progress / 0.1;
            let flash_size = 30.0 + flash_progress * 20.0;
            let flash_alpha = ((1.0 - flash_progress) * 255.0) as u8;
            painter.circle_filled(
                pos,
                flash_size,
                egui::Color32::from_rgba_unmultiplied(255, 255, 255, flash_alpha),
            );
        }

        // Fireball (first 40%)
        if progress < 0.4 {
            let fireball_progress = progress / 0.4;
            let fireball_size = 20.0 + fireball_progress * 30.0;
            let alpha = ((1.0 - fireball_progress) * 200.0) as u8;

            // Orange/red gradient
            painter.circle_filled(
                pos,
                fireball_size,
                egui::Color32::from_rgba_unmultiplied(255, 150, 50, alpha),
            );
            painter.circle_filled(
                pos,
                fireball_size * 0.6,
                egui::Color32::from_rgba_unmultiplied(255, 200, 100, alpha),
            );
        }

        // Expanding shockwave rings
        let num_rings = 4;
        for i in 0..num_rings {
            let ring_delay = i as f32 * 0.1;
            let ring_progress = ((progress - ring_delay) / 0.6).clamp(0.0, 1.0);

            if ring_progress > 0.0 {
                let radius = 15.0 + ring_progress * 60.0;
                let alpha = ((1.0 - ring_progress) * 180.0) as u8;
                let width = 4.0 - ring_progress * 3.0;

                let color = egui::Color32::from_rgba_unmultiplied(
                    255,
                    (100.0 + i as f32 * 30.0) as u8,
                    0,
                    alpha,
                );
                painter.circle_stroke(pos, radius, egui::Stroke::new(width.max(0.5), color));
            }
        }

        // Debris particles (flying outward)
        let num_debris = 12;
        for i in 0..num_debris {
            let angle = (i as f32 / num_debris as f32) * std::f32::consts::TAU;
            // Add some variation using the index
            let angle = angle + (i as f32 * 0.7).sin() * 0.3;
            let speed = 40.0 + (i as f32 * 1.3).sin() * 20.0;
            let debris_progress = (progress * 1.2).clamp(0.0, 1.0);

            let distance = speed * debris_progress;
            let debris_x = pos.x + angle.cos() * distance;
            let debris_y = pos.y + angle.sin() * distance;

            let alpha = ((1.0 - debris_progress) * 255.0) as u8;
            let size = 2.0 + (i as f32 * 0.5).sin().abs() * 2.0;

            // Debris color (orange to gray)
            let gray = (debris_progress * 150.0) as u8;
            let color = egui::Color32::from_rgba_unmultiplied(
                255 - gray,
                150 - gray.min(150),
                gray / 2,
                alpha,
            );

            painter.circle_filled(egui::pos2(debris_x, debris_y), size * (1.0 - debris_progress * 0.5), color);
        }

        // Smoke cloud (fades in as explosion fades)
        if progress > 0.3 {
            let smoke_progress = ((progress - 0.3) / 0.7).clamp(0.0, 1.0);
            let smoke_alpha = ((1.0 - smoke_progress * 0.5) * 80.0) as u8;
            let smoke_size = 25.0 + smoke_progress * 15.0;

            painter.circle_filled(
                pos,
                smoke_size,
                egui::Color32::from_rgba_unmultiplied(80, 80, 80, smoke_alpha),
            );
        }

        // Ground scar marker (persists)
        if progress > 0.5 {
            let marker_alpha = (((progress - 0.5) / 0.5) * 200.0) as u8;

            // Crater circle
            painter.circle_stroke(
                pos,
                10.0,
                egui::Stroke::new(2.5, egui::Color32::from_rgba_unmultiplied(100, 50, 30, marker_alpha)),
            );

            // X mark
            let x_size = 7.0;
            let x_color = egui::Color32::from_rgba_unmultiplied(200, 50, 50, marker_alpha);
            painter.line_segment(
                [egui::pos2(pos.x - x_size, pos.y - x_size), egui::pos2(pos.x + x_size, pos.y + x_size)],
                egui::Stroke::new(2.5, x_color),
            );
            painter.line_segment(
                [egui::pos2(pos.x + x_size, pos.y - x_size), egui::pos2(pos.x - x_size, pos.y + x_size)],
                egui::Stroke::new(2.5, x_color),
            );
        }
    }

    /// Render an intercept success effect with debris
    pub fn render_intercept_effect(painter: &egui::Painter, pos: egui::Pos2, progress: f64) {
        let progress = progress as f32;

        // Initial bright flash
        if progress < 0.15 {
            let flash_progress = progress / 0.15;
            let flash_size = 20.0 + flash_progress * 10.0;
            let flash_alpha = ((1.0 - flash_progress) * 255.0) as u8;
            painter.circle_filled(
                pos,
                flash_size,
                egui::Color32::from_rgba_unmultiplied(200, 255, 200, flash_alpha),
            );
        }

        // Green/white expanding shockwave
        let num_rings = 3;
        for i in 0..num_rings {
            let ring_delay = i as f32 * 0.12;
            let ring_progress = ((progress - ring_delay) / 0.5).clamp(0.0, 1.0);

            if ring_progress > 0.0 {
                let radius = 12.0 + ring_progress * 40.0;
                let alpha = ((1.0 - ring_progress) * 200.0) as u8;
                let width = 3.0 - ring_progress * 2.0;

                let color = if i == 0 {
                    egui::Color32::from_rgba_unmultiplied(150, 255, 150, alpha)
                } else {
                    egui::Color32::from_rgba_unmultiplied(100, 255, 100, alpha)
                };
                painter.circle_stroke(pos, radius, egui::Stroke::new(width.max(0.5), color));
            }
        }

        // Debris particles (destroyed missile fragments)
        let num_debris = 10;
        for i in 0..num_debris {
            let angle = (i as f32 / num_debris as f32) * std::f32::consts::TAU;
            let angle = angle + (i as f32 * 1.1).sin() * 0.4;
            let speed = 30.0 + (i as f32 * 0.9).sin() * 15.0;
            let debris_progress = (progress * 1.5).clamp(0.0, 1.0);

            let distance = speed * debris_progress;
            // Add gravity effect (debris falls)
            let gravity = debris_progress * debris_progress * 20.0;
            let debris_x = pos.x + angle.cos() * distance;
            let debris_y = pos.y + angle.sin() * distance + gravity;

            let alpha = ((1.0 - debris_progress) * 220.0) as u8;
            let size = 1.5 + (i as f32 * 0.4).sin().abs() * 1.5;

            // Debris color (white-hot to dark)
            let heat = 1.0 - debris_progress;
            let color = egui::Color32::from_rgba_unmultiplied(
                (100.0 + heat * 155.0) as u8,
                (200.0 + heat * 55.0) as u8,
                (100.0 + heat * 100.0) as u8,
                alpha,
            );

            painter.circle_filled(egui::pos2(debris_x, debris_y), size * (1.0 - debris_progress * 0.3), color);

            // Small trail behind each debris piece
            if debris_progress < 0.7 {
                let trail_alpha = (alpha as f32 * 0.5) as u8;
                let trail_x = debris_x - angle.cos() * 5.0;
                let trail_y = debris_y - angle.sin() * 5.0 - 3.0;
                painter.line_segment(
                    [egui::pos2(debris_x, debris_y), egui::pos2(trail_x, trail_y)],
                    egui::Stroke::new(1.0, egui::Color32::from_rgba_unmultiplied(150, 255, 150, trail_alpha)),
                );
            }
        }

        // Success checkmark (fades in)
        if progress > 0.6 {
            let check_progress = ((progress - 0.6) / 0.4).clamp(0.0, 1.0);
            let check_alpha = (check_progress * 200.0) as u8;
            let check_color = egui::Color32::from_rgba_unmultiplied(50, 255, 50, check_alpha);

            // Checkmark
            let check_size = 8.0;
            painter.line_segment(
                [
                    egui::pos2(pos.x - check_size, pos.y),
                    egui::pos2(pos.x - check_size * 0.3, pos.y + check_size * 0.7),
                ],
                egui::Stroke::new(3.0, check_color),
            );
            painter.line_segment(
                [
                    egui::pos2(pos.x - check_size * 0.3, pos.y + check_size * 0.7),
                    egui::pos2(pos.x + check_size, pos.y - check_size * 0.5),
                ],
                egui::Stroke::new(3.0, check_color),
            );
        }
    }

    /// Render a debris cloud effect (scattered fragments)
    pub fn render_debris_effect(painter: &egui::Painter, pos: egui::Pos2, progress: f64) {
        let progress = progress as f32;

        // Expanding debris cloud
        let num_particles = 16;
        for i in 0..num_particles {
            let base_angle = (i as f32 / num_particles as f32) * std::f32::consts::TAU;
            let angle_offset = (i as f32 * 2.3 + 0.5).sin() * 0.3;
            let angle = base_angle + angle_offset;

            let speed = 20.0 + (i as f32 * 1.7).sin().abs() * 25.0;
            let distance = speed * progress;

            // Gravity effect
            let gravity = progress * progress * 30.0;
            let debris_x = pos.x + angle.cos() * distance;
            let debris_y = pos.y + angle.sin() * distance + gravity;

            let alpha = ((1.0 - progress * 0.8) * 200.0) as u8;
            let size = 1.0 + (i as f32 * 0.6).sin().abs() * 2.0;

            // Gray debris color
            let brightness = 80 + ((i as f32 * 1.2).sin().abs() * 100.0) as u8;
            let color = egui::Color32::from_rgba_unmultiplied(
                brightness,
                brightness,
                brightness,
                alpha,
            );

            painter.circle_filled(
                egui::pos2(debris_x, debris_y),
                size * (1.0 - progress * 0.4),
                color,
            );
        }

        // Smoke cloud expanding
        let smoke_alpha = ((1.0 - progress) * 80.0) as u8;
        let smoke_radius = 10.0 + progress * 30.0;
        painter.circle_filled(
            pos,
            smoke_radius,
            egui::Color32::from_rgba_unmultiplied(100, 100, 100, smoke_alpha),
        );
    }
}
