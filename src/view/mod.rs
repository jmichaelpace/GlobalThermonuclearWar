use crate::map::GeoCoord;
use eframe::egui::{Pos2, Rect};

/// Trait for map projections that convert geographic coordinates to screen positions.
/// Implementations include Mercator (2D map), Orthographic (globe), and Isometric (3D view).
pub trait MapProjection {
    /// Convert geographic coordinates to screen position.
    /// Returns None if the point is not visible (e.g., on the back of the globe).
    fn geo_to_screen(&self, coord: GeoCoord, screen_rect: Rect) -> Option<Pos2>;

    /// Check if a geographic point is visible in this projection.
    fn is_visible(&self, coord: GeoCoord) -> bool;

    /// Handle wrapped coordinates (for 2D map where the world wraps).
    /// Returns all visible positions for a coordinate (typically 1, but can be 2-3 for wrap-around).
    /// The default implementation returns a single position if visible.
    fn geo_to_screen_wrapped(&self, coord: GeoCoord, screen_rect: Rect) -> Vec<Pos2> {
        self.geo_to_screen(coord, screen_rect).into_iter().collect()
    }

    /// Get the visible bounds of this projection in geographic coordinates.
    /// Returns (min_lat, max_lat, min_lon, max_lon).
    fn visible_bounds(&self) -> (f64, f64, f64, f64);

    /// Get a scale factor for rendering (pixels per degree or similar).
    /// Used to determine appropriate detail levels for rendering.
    fn scale_factor(&self) -> f64;
}

/// Wrapper around Viewport to implement MapProjection for 2D Mercator view.
pub struct MercatorProjection<'a> {
    pub viewport: &'a crate::map::Viewport,
}

impl<'a> MapProjection for MercatorProjection<'a> {
    fn geo_to_screen(&self, coord: GeoCoord, screen_rect: Rect) -> Option<Pos2> {
        Some(self.viewport.geo_to_screen(coord, screen_rect))
    }

    fn is_visible(&self, _coord: GeoCoord) -> bool {
        // In 2D Mercator, all points are technically visible (with wrapping)
        true
    }

    fn geo_to_screen_wrapped(&self, coord: GeoCoord, screen_rect: Rect) -> Vec<Pos2> {
        self.viewport.geo_to_screen_wrapped(coord, screen_rect)
    }

    fn visible_bounds(&self) -> (f64, f64, f64, f64) {
        // For now, return full world bounds
        // Could be improved to return actual viewport bounds
        (-85.0, 85.0, -180.0, 180.0)
    }

    fn scale_factor(&self) -> f64 {
        self.viewport.zoom as f64
    }
}

/// Wrapper around GlobeState to implement MapProjection for 3D globe view.
pub struct GlobeProjection<'a> {
    pub globe_state: &'a crate::app::GlobeState,
    pub screen_center: Pos2,
}

impl<'a> MapProjection for GlobeProjection<'a> {
    fn geo_to_screen(&self, coord: GeoCoord, _screen_rect: Rect) -> Option<Pos2> {
        self.globe_state.geo_to_screen(coord, self.screen_center)
    }

    fn is_visible(&self, coord: GeoCoord) -> bool {
        self.globe_state.is_visible(coord)
    }

    fn visible_bounds(&self) -> (f64, f64, f64, f64) {
        // Approximate visible hemisphere
        let center = self.globe_state.center();
        let range = 90.0; // Visible range from center
        (
            (center.lat - range).max(-90.0),
            (center.lat + range).min(90.0),
            center.lon - range,
            center.lon + range,
        )
    }

    fn scale_factor(&self) -> f64 {
        self.globe_state.radius as f64
    }
}

/// Unified rendering functions that work with any projection.
/// These can replace duplicate Map2D/Globe rendering code.
pub mod rendering {
    use super::*;
    use eframe::egui::{Color32, Painter, Stroke};

    /// Draw a marker at a geographic position.
    pub fn draw_marker(
        painter: &Painter,
        projection: &dyn MapProjection,
        screen_rect: Rect,
        coord: GeoCoord,
        radius: f32,
        fill: Color32,
        stroke: Option<Stroke>,
    ) {
        for pos in projection.geo_to_screen_wrapped(coord, screen_rect) {
            painter.circle_filled(pos, radius, fill);
            if let Some(stroke) = stroke {
                painter.circle_stroke(pos, radius, stroke);
            }
        }
    }

    /// Draw a line between two geographic positions.
    pub fn draw_line(
        painter: &Painter,
        projection: &dyn MapProjection,
        screen_rect: Rect,
        from: GeoCoord,
        to: GeoCoord,
        stroke: Stroke,
    ) {
        // For wrapped projections, this is simplified and may not handle cross-antimeridian lines
        if let (Some(from_pos), Some(to_pos)) = (
            projection.geo_to_screen(from, screen_rect),
            projection.geo_to_screen(to, screen_rect),
        ) {
            painter.line_segment([from_pos, to_pos], stroke);
        }
    }

    /// Draw a great circle arc between two points.
    /// Uses interpolation for smooth curves on any projection.
    pub fn draw_great_circle(
        painter: &Painter,
        projection: &dyn MapProjection,
        screen_rect: Rect,
        from: GeoCoord,
        to: GeoCoord,
        stroke: Stroke,
        segments: usize,
    ) {
        let mut last_pos: Option<Pos2> = None;

        for i in 0..=segments {
            let t = i as f64 / segments as f64;
            let point = crate::simulation::physics::interpolate_great_circle(from, to, t);

            if let Some(pos) = projection.geo_to_screen(point, screen_rect) {
                if let Some(last) = last_pos {
                    painter.line_segment([last, pos], stroke);
                }
                last_pos = Some(pos);
            } else {
                last_pos = None; // Reset if point not visible
            }
        }
    }
}
