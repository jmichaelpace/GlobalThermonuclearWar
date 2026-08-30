use crate::map::{GeoCoord, Viewport};
use crate::simulation::{
    Affiliation, BallisticTrajectory, Interceptor, InterceptorPhase, InterceptorStatus, Missile,
    MissileStatus, SensorKind,
};
use eframe::egui::{self, Color32, FontId, Pos2, Stroke};

/// Render detection and tracking overlays
pub struct DetectionOverlays;

impl DetectionOverlays {
    /// Draw a tracking line from sensor to target
    pub fn draw_tracking_line(
        painter: &egui::Painter,
        viewport: &Viewport,
        screen_rect: egui::Rect,
        sensor_pos: GeoCoord,
        target_pos: GeoCoord,
        affiliation: Affiliation,
        track_quality: f64,
    ) {
        let sensor_screen = viewport.geo_to_screen(sensor_pos, screen_rect);
        let target_screen = viewport.geo_to_screen(target_pos, screen_rect);

        // No culling - let egui's painter handle clipping
        // This ensures lines are visible at all zoom levels

        let base_color = match affiliation {
            Affiliation::Friendly => Color32::from_rgb(100, 200, 255),
            Affiliation::Hostile => Color32::from_rgb(255, 150, 100),
            Affiliation::Neutral => Color32::from_rgb(200, 200, 100),
        };

        // Fade based on track quality
        let alpha = (track_quality * 180.0) as u8 + 50;
        let color =
            Color32::from_rgba_unmultiplied(base_color.r(), base_color.g(), base_color.b(), alpha);

        // Draw dashed line
        Self::draw_dashed_line(painter, sensor_screen, target_screen, color, 2.0, 8.0, 4.0);

        // Draw small diamond at sensor end
        Self::draw_track_indicator(painter, sensor_screen, color, 4.0);
    }

    /// Draw a dashed line between two points
    fn draw_dashed_line(
        painter: &egui::Painter,
        start: Pos2,
        end: Pos2,
        color: Color32,
        width: f32,
        dash_length: f32,
        gap_length: f32,
    ) {
        let total_length = start.distance(end);
        if total_length < 0.1 {
            return; // Skip only if extremely short (same point)
        }

        // If line is very short, just draw a solid line instead of dashed
        if total_length < dash_length * 2.0 {
            painter.line_segment([start, end], Stroke::new(width, color));
            return;
        }

        let direction = (end - start) / total_length;
        let mut current = 0.0;
        let mut drawing = true;

        while current < total_length {
            let segment_length = if drawing { dash_length } else { gap_length };
            let segment_end = (current + segment_length).min(total_length);

            if drawing {
                let p1 = start + direction * current;
                let p2 = start + direction * segment_end;
                painter.line_segment([p1, p2], Stroke::new(width, color));
            }

            current = segment_end;
            drawing = !drawing;
        }
    }

    /// Draw a small indicator at the sensor position
    fn draw_track_indicator(painter: &egui::Painter, pos: Pos2, color: Color32, size: f32) {
        let points = vec![
            Pos2::new(pos.x, pos.y - size),
            Pos2::new(pos.x + size, pos.y),
            Pos2::new(pos.x, pos.y + size),
            Pos2::new(pos.x - size, pos.y),
        ];
        painter.add(egui::Shape::convex_polygon(points, color, Stroke::NONE));
    }

    /// Draw a radar cone/sector for a directional radar
    pub fn draw_radar_cone(
        painter: &egui::Painter,
        viewport: &Viewport,
        screen_rect: egui::Rect,
        position: GeoCoord,
        facing_deg: f64,   // Direction the radar faces (0 = North)
        coverage_deg: f64, // Width of coverage arc
        range_km: f64,
        affiliation: Affiliation,
    ) {
        let center = viewport.geo_to_screen(position, screen_rect);

        // Calculate range in screen pixels
        let range_deg_geo = range_km / 111.32;
        let edge_pos = GeoCoord::new(position.lat + range_deg_geo, position.lon);
        let edge_screen = viewport.geo_to_screen(edge_pos, screen_rect);
        let radius = (center.y - edge_screen.y).abs();

        if radius < 5.0 {
            return; // Too small to render
        }

        let color = match affiliation {
            Affiliation::Friendly => Color32::from_rgba_unmultiplied(0, 150, 100, 25),
            Affiliation::Hostile => Color32::from_rgba_unmultiplied(200, 100, 50, 25),
            Affiliation::Neutral => Color32::from_rgba_unmultiplied(150, 150, 50, 25),
        };

        let stroke_color = match affiliation {
            Affiliation::Friendly => Color32::from_rgba_unmultiplied(0, 200, 150, 80),
            Affiliation::Hostile => Color32::from_rgba_unmultiplied(255, 150, 100, 80),
            Affiliation::Neutral => Color32::from_rgba_unmultiplied(200, 200, 100, 80),
        };

        // Draw the cone as a filled arc
        let half_coverage = coverage_deg / 2.0;
        // Convert facing from geographic (0=North) to screen angle (0=East, counter-clockwise)
        let screen_facing = 90.0 - facing_deg;
        let start_angle = (screen_facing - half_coverage).to_radians() as f32;
        let end_angle = (screen_facing + half_coverage).to_radians() as f32;

        // Build arc points
        let segments = 32;
        let mut points = vec![center];

        for i in 0..=segments {
            let t = i as f32 / segments as f32;
            let angle = start_angle + (end_angle - start_angle) * t;
            points.push(Pos2::new(
                center.x + angle.cos() * radius,
                center.y - angle.sin() * radius, // Negative because screen Y is inverted
            ));
        }

        // Draw filled arc
        if points.len() >= 3 {
            painter.add(egui::Shape::convex_polygon(
                points.clone(),
                color,
                Stroke::NONE,
            ));
        }

        // Draw arc outline
        if points.len() >= 2 {
            let arc_points: Vec<Pos2> = points[1..].to_vec();
            painter.add(egui::Shape::line(
                arc_points,
                Stroke::new(1.5, stroke_color),
            ));

            // Draw radial lines
            painter.line_segment([center, points[1]], Stroke::new(1.0, stroke_color));
            painter.line_segment(
                [center, *points.last().unwrap()],
                Stroke::new(1.0, stroke_color),
            );
        }
    }

    /// Draw detection quality indicator on a target
    pub fn draw_detection_indicator(
        painter: &egui::Painter,
        pos: Pos2,
        quality: f64,
        sensor_kind: SensorKind,
    ) {
        let size = 14.0 + (quality * 4.0) as f32;

        let color = match sensor_kind {
            SensorKind::SatelliteIR => Color32::from_rgba_unmultiplied(255, 100, 100, 150),
            SensorKind::SatelliteRadar => Color32::from_rgba_unmultiplied(100, 255, 100, 150),
            SensorKind::GroundRadar | SensorKind::DefenseUnitRadar | SensorKind::ShipRadar => {
                Color32::from_rgba_unmultiplied(100, 200, 255, 150)
            }
        };

        // Draw pulsing detection ring
        painter.circle_stroke(pos, size, Stroke::new(2.0, color));

        // Inner ring for high quality detections
        if quality > 0.6 {
            painter.circle_stroke(pos, size * 0.7, Stroke::new(1.5, color.gamma_multiply(0.7)));
        }
    }

    /// Draw satellite coverage swath
    pub fn draw_satellite_swath(
        painter: &egui::Painter,
        viewport: &Viewport,
        screen_rect: egui::Rect,
        position: GeoCoord,
        coverage_radius_km: f64,
        affiliation: Affiliation,
        is_detecting: bool,
    ) {
        let positions = viewport.geo_to_screen_wrapped(position, screen_rect);

        // Calculate radius in screen pixels
        let range_deg = coverage_radius_km / 111.32;
        let edge_pos = GeoCoord::new(position.lat + range_deg, position.lon);
        let base_screen = viewport.geo_to_screen(position, screen_rect);
        let edge_screen = viewport.geo_to_screen(edge_pos, screen_rect);
        let radius = (base_screen.y - edge_screen.y).abs();

        if radius < 5.0 {
            return;
        }

        let (fill_color, stroke_color) = if is_detecting {
            // Brighter when actively detecting
            match affiliation {
                Affiliation::Friendly => (
                    Color32::from_rgba_unmultiplied(100, 255, 200, 40),
                    Color32::from_rgba_unmultiplied(100, 255, 200, 120),
                ),
                Affiliation::Hostile => (
                    Color32::from_rgba_unmultiplied(255, 150, 100, 40),
                    Color32::from_rgba_unmultiplied(255, 150, 100, 120),
                ),
                Affiliation::Neutral => (
                    Color32::from_rgba_unmultiplied(200, 200, 100, 40),
                    Color32::from_rgba_unmultiplied(200, 200, 100, 120),
                ),
            }
        } else {
            match affiliation {
                Affiliation::Friendly => (
                    Color32::from_rgba_unmultiplied(100, 200, 255, 20),
                    Color32::from_rgba_unmultiplied(100, 200, 255, 60),
                ),
                Affiliation::Hostile => (
                    Color32::from_rgba_unmultiplied(255, 100, 100, 20),
                    Color32::from_rgba_unmultiplied(255, 100, 100, 60),
                ),
                Affiliation::Neutral => (
                    Color32::from_rgba_unmultiplied(200, 200, 200, 20),
                    Color32::from_rgba_unmultiplied(200, 200, 200, 60),
                ),
            }
        };

        for center in positions {
            painter.circle_filled(center, radius, fill_color);
            painter.circle_stroke(center, radius, Stroke::new(1.5, stroke_color));
        }
    }

    /// Draw an uncertainty ellipse around a tracked target
    /// Color changes from green (good track) to red (poor track)
    pub fn draw_uncertainty_ellipse(
        painter: &egui::Painter,
        center: Pos2,
        radius_pixels: f32,
        quality: f64,
    ) {
        // Color fades from green (good track) to yellow to red (poor track)
        let r = ((1.0 - quality) * 255.0) as u8;
        let g = (quality * 200.0 + (1.0 - quality) * 100.0) as u8;
        let fill_alpha = 30;
        let stroke_alpha = 150;

        let fill_color = Color32::from_rgba_unmultiplied(r, g, 50, fill_alpha);
        let stroke_color = Color32::from_rgba_unmultiplied(r, g, 50, stroke_alpha);

        // Draw filled ellipse (using circle for simplicity - ellipse would need bearing info)
        painter.circle_filled(center, radius_pixels, fill_color);
        painter.circle_stroke(center, radius_pixels, Stroke::new(2.0, stroke_color));

        // Draw pulsing outer ring for low quality tracks
        if quality < 0.5 {
            let pulse_color =
                Color32::from_rgba_unmultiplied(r, g, 50, (stroke_alpha as f64 * 0.4) as u8);
            painter.circle_stroke(center, radius_pixels * 1.15, Stroke::new(1.0, pulse_color));
        }

        // Draw crosshairs for very low quality (uncertain position)
        if quality < 0.3 {
            let cross_size = radius_pixels * 0.4;
            let cross_color = Color32::from_rgba_unmultiplied(r, g, 50, 100);
            painter.line_segment(
                [
                    Pos2::new(center.x - cross_size, center.y),
                    Pos2::new(center.x + cross_size, center.y),
                ],
                Stroke::new(1.0, cross_color),
            );
            painter.line_segment(
                [
                    Pos2::new(center.x, center.y - cross_size),
                    Pos2::new(center.x, center.y + cross_size),
                ],
                Stroke::new(1.0, cross_color),
            );
        }
    }

    /// Draw a false alarm marker (clutter/noise detection)
    /// Shows as yellow/orange X marker to distinguish from real tracks
    pub fn draw_false_alarm_marker(painter: &egui::Painter, pos: Pos2, quality: f64) {
        let size = 6.0 + (quality * 4.0) as f32;

        // Use yellow/orange color for false alarms
        let color = Color32::from_rgba_unmultiplied(255, 180, 50, 180);
        let dim_color = Color32::from_rgba_unmultiplied(255, 180, 50, 80);

        // Draw X shape for "unknown contact"
        let offset = size * 0.7;
        painter.line_segment(
            [
                Pos2::new(pos.x - offset, pos.y - offset),
                Pos2::new(pos.x + offset, pos.y + offset),
            ],
            Stroke::new(2.5, color),
        );
        painter.line_segment(
            [
                Pos2::new(pos.x + offset, pos.y - offset),
                Pos2::new(pos.x - offset, pos.y + offset),
            ],
            Stroke::new(2.5, color),
        );

        // Dashed circle around it
        painter.circle_stroke(pos, size * 1.3, Stroke::new(1.0, dim_color));

        // Question mark indicator? No, keep it simple with just the X
    }
}

/// Trajectory visualization for missiles and interceptors
pub struct TrajectoryOverlays;

impl TrajectoryOverlays {
    /// Draw a missile trajectory arc with time markers
    pub fn draw_missile_trajectory(
        painter: &egui::Painter,
        viewport: &Viewport,
        screen_rect: egui::Rect,
        missile: &Missile,
        trajectory: &BallisticTrajectory,
        show_time_markers: bool,
    ) {
        // Only draw for in-flight missiles
        match missile.status {
            MissileStatus::Boost | MissileStatus::Midcourse | MissileStatus::Terminal => {}
            _ => return,
        }

        let progress = missile.flight_progress();

        // Draw remaining trajectory (from current position to target)
        let segments = 30;
        let mut path_points: Vec<Pos2> = Vec::new();

        for i in 0..=segments {
            let t = progress + (1.0 - progress) * (i as f64 / segments as f64);
            let (pos, _alt) = trajectory.position_at(t);
            let screen_pos = viewport.geo_to_screen(pos, screen_rect);
            path_points.push(screen_pos);
        }

        // Skip if path crosses screen (date line issue)
        let valid_path = path_points
            .windows(2)
            .all(|w| w[0].distance(w[1]) < screen_rect.width() * 0.3);
        if !valid_path || path_points.len() < 2 {
            return;
        }

        // Color based on affiliation
        let path_color = match missile.affiliation {
            Affiliation::Hostile => Color32::from_rgba_unmultiplied(255, 80, 80, 180),
            Affiliation::Friendly => Color32::from_rgba_unmultiplied(80, 180, 255, 180),
            Affiliation::Neutral => Color32::from_rgba_unmultiplied(200, 200, 80, 180),
        };

        // Draw trajectory arc
        painter.add(egui::Shape::line(
            path_points.clone(),
            Stroke::new(2.0, path_color),
        ));

        // Draw impact marker at end
        if let Some(last) = path_points.last() {
            Self::draw_impact_marker(painter, *last, path_color);
        }

        // Draw time markers
        if show_time_markers {
            let time_remaining = missile.flight_time - missile.current_flight_time;
            let marker_interval = if time_remaining > 300.0 { 60.0 } else { 30.0 }; // 1 min or 30 sec intervals

            let mut marker_time = marker_interval;
            while marker_time < time_remaining - 10.0 {
                let marker_progress = progress + marker_time / missile.flight_time;
                if marker_progress < 1.0 {
                    let (pos, _alt) = trajectory.position_at(marker_progress);
                    let screen_pos = viewport.geo_to_screen(pos, screen_rect);

                    // Draw time marker
                    Self::draw_time_marker(
                        painter,
                        screen_pos,
                        time_remaining - marker_time,
                        path_color,
                    );
                }
                marker_time += marker_interval;
            }
        }
    }

    /// Draw an interceptor trajectory
    pub fn draw_interceptor_trajectory(
        painter: &egui::Painter,
        viewport: &Viewport,
        screen_rect: egui::Rect,
        interceptor: &Interceptor,
    ) {
        // Only draw for in-flight interceptors
        if interceptor.status != InterceptorStatus::InFlight {
            return;
        }

        let progress = interceptor.flight_progress();

        // Draw from current position to predicted intercept point
        let segments = 20;
        let mut path_points: Vec<Pos2> = Vec::new();

        for i in 0..=segments {
            let t = progress + (1.0 - progress) * (i as f64 / segments as f64);
            // Simple linear interpolation for interceptor path
            let lat =
                interceptor.launch_position.lat * (1.0 - t) + interceptor.target_position.lat * t;
            let lon =
                interceptor.launch_position.lon * (1.0 - t) + interceptor.target_position.lon * t;
            let pos = GeoCoord::new(lat, lon);
            let screen_pos = viewport.geo_to_screen(pos, screen_rect);
            path_points.push(screen_pos);
        }

        // Skip if path crosses screen
        let valid_path = path_points
            .windows(2)
            .all(|w| w[0].distance(w[1]) < screen_rect.width() * 0.3);
        if !valid_path || path_points.len() < 2 {
            return;
        }

        // Color based on flight phase
        let path_color = match interceptor.phase {
            InterceptorPhase::Boost => Color32::from_rgba_unmultiplied(255, 200, 50, 200), // Yellow/orange - thrusting
            InterceptorPhase::Coast => Color32::from_rgba_unmultiplied(100, 200, 255, 180), // Light blue - coasting
            InterceptorPhase::Terminal => Color32::from_rgba_unmultiplied(50, 255, 150, 220), // Green - homing
        };

        // Draw trajectory line
        painter.add(egui::Shape::line(
            path_points.clone(),
            Stroke::new(1.5, path_color),
        ));

        // Draw intercept point marker
        if let Some(last) = path_points.last() {
            Self::draw_intercept_marker(painter, *last, path_color);
        }
    }

    /// Draw engagement line between defense unit and its target
    pub fn draw_engagement_line(
        painter: &egui::Painter,
        viewport: &Viewport,
        screen_rect: egui::Rect,
        unit_pos: GeoCoord,
        target_pos: GeoCoord,
        interceptor_count: u32,
    ) {
        let start = viewport.geo_to_screen(unit_pos, screen_rect);
        let end = viewport.geo_to_screen(target_pos, screen_rect);

        // Skip if crossing date line
        if start.distance(end) > screen_rect.width() * 0.5 {
            return;
        }

        // Dashed line color
        let color = Color32::from_rgba_unmultiplied(255, 255, 100, 120);

        // Draw dashed engagement line
        Self::draw_dashed_engagement_line(painter, start, end, color, 1.5, 6.0, 4.0);

        // Draw count indicator at midpoint
        if interceptor_count > 0 {
            let mid = Pos2::new((start.x + end.x) / 2.0, (start.y + end.y) / 2.0);
            let text = format!("{}", interceptor_count);
            painter.text(
                mid,
                egui::Align2::CENTER_CENTER,
                text,
                FontId::proportional(10.0),
                color,
            );
        }
    }

    /// Draw a dashed line for engagement
    fn draw_dashed_engagement_line(
        painter: &egui::Painter,
        start: Pos2,
        end: Pos2,
        color: Color32,
        width: f32,
        dash_length: f32,
        gap_length: f32,
    ) {
        let total_length = start.distance(end);
        if total_length < 1.0 {
            return;
        }

        let direction = (end - start) / total_length;
        let mut current = 0.0;
        let mut drawing = true;

        while current < total_length {
            let segment_length = if drawing { dash_length } else { gap_length };
            let segment_end = (current + segment_length).min(total_length);

            if drawing {
                let p1 = start + direction * current;
                let p2 = start + direction * segment_end;
                painter.line_segment([p1, p2], Stroke::new(width, color));
            }

            current = segment_end;
            drawing = !drawing;
        }
    }

    /// Draw impact marker (X symbol)
    fn draw_impact_marker(painter: &egui::Painter, pos: Pos2, color: Color32) {
        let size = 6.0;
        let stroke = Stroke::new(2.0, color);

        // Draw X
        painter.line_segment(
            [
                Pos2::new(pos.x - size, pos.y - size),
                Pos2::new(pos.x + size, pos.y + size),
            ],
            stroke,
        );
        painter.line_segment(
            [
                Pos2::new(pos.x + size, pos.y - size),
                Pos2::new(pos.x - size, pos.y + size),
            ],
            stroke,
        );
    }

    /// Draw intercept point marker (diamond)
    fn draw_intercept_marker(painter: &egui::Painter, pos: Pos2, color: Color32) {
        let size = 5.0;
        let points = vec![
            Pos2::new(pos.x, pos.y - size),
            Pos2::new(pos.x + size, pos.y),
            Pos2::new(pos.x, pos.y + size),
            Pos2::new(pos.x - size, pos.y),
        ];
        painter.add(egui::Shape::convex_polygon(
            points,
            Color32::TRANSPARENT,
            Stroke::new(1.5, color),
        ));
    }

    /// Draw time marker with label
    fn draw_time_marker(painter: &egui::Painter, pos: Pos2, time_sec: f64, color: Color32) {
        let size = 3.0;

        // Draw small tick
        painter.circle_filled(pos, size, color);

        // Format time
        let time_str = if time_sec >= 60.0 {
            format!("{}m", (time_sec / 60.0) as i32)
        } else {
            format!("{}s", time_sec as i32)
        };

        // Draw label offset to avoid overlapping the line
        let label_pos = Pos2::new(pos.x + 8.0, pos.y - 8.0);
        painter.text(
            label_pos,
            egui::Align2::LEFT_BOTTOM,
            time_str,
            FontId::proportional(9.0),
            color.gamma_multiply(0.8),
        );
    }

    /// Draw altitude profile line (optional vertical representation)
    pub fn draw_altitude_indicator(
        painter: &egui::Painter,
        pos: Pos2,
        altitude_km: f64,
        max_altitude_km: f64,
        color: Color32,
    ) {
        // Normalize altitude for visual representation
        let height = (altitude_km / max_altitude_km * 30.0).min(30.0) as f32;

        if height < 2.0 {
            return;
        }

        // Draw vertical line indicating altitude
        let stroke = Stroke::new(1.5, color.gamma_multiply(0.6));
        painter.line_segment([pos, Pos2::new(pos.x, pos.y - height)], stroke);

        // Draw small tick at top
        let tick_width = 3.0;
        painter.line_segment(
            [
                Pos2::new(pos.x - tick_width, pos.y - height),
                Pos2::new(pos.x + tick_width, pos.y - height),
            ],
            stroke,
        );
    }
}
