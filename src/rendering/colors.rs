use eframe::egui::Color32;
use crate::simulation::Affiliation;

/// Color utilities for consistent rendering across the application.
/// Eliminates duplicate `match affiliation` blocks throughout the codebase.

// ============================================================================
// Base Affiliation Colors
// ============================================================================

/// Get the base color for an affiliation (full opacity)
pub fn affiliation_base(affiliation: Affiliation) -> Color32 {
    match affiliation {
        Affiliation::Friendly => Color32::from_rgb(80, 180, 255),  // Blue
        Affiliation::Hostile => Color32::from_rgb(255, 80, 80),    // Red
        Affiliation::Neutral => Color32::from_rgb(100, 255, 100),  // Green
    }
}

/// Get affiliation color with custom alpha
pub fn affiliation_with_alpha(affiliation: Affiliation, alpha: u8) -> Color32 {
    match affiliation {
        Affiliation::Friendly => Color32::from_rgba_unmultiplied(80, 180, 255, alpha),
        Affiliation::Hostile => Color32::from_rgba_unmultiplied(255, 80, 80, alpha),
        Affiliation::Neutral => Color32::from_rgba_unmultiplied(100, 255, 100, alpha),
    }
}

// ============================================================================
// Radar Mode Colors (by affiliation and radar mode)
// ============================================================================

/// Fire control radar color (orange-tinted, high visibility)
pub fn fire_control_color(affiliation: Affiliation) -> Color32 {
    match affiliation {
        Affiliation::Friendly => Color32::from_rgba_unmultiplied(255, 150, 50, 120),
        Affiliation::Hostile => Color32::from_rgba_unmultiplied(255, 50, 50, 120),
        Affiliation::Neutral => Color32::from_rgba_unmultiplied(255, 200, 50, 120),
    }
}

/// Track mode radar color (yellow-tinted)
pub fn track_mode_color(affiliation: Affiliation) -> Color32 {
    match affiliation {
        Affiliation::Friendly => Color32::from_rgba_unmultiplied(255, 255, 100, 90),
        Affiliation::Hostile => Color32::from_rgba_unmultiplied(255, 200, 80, 90),
        Affiliation::Neutral => Color32::from_rgba_unmultiplied(200, 255, 100, 90),
    }
}

/// Track mode fill color (lower alpha for filled areas)
pub fn track_mode_fill(affiliation: Affiliation) -> Color32 {
    match affiliation {
        Affiliation::Friendly => Color32::from_rgba_unmultiplied(255, 255, 100, 35),
        Affiliation::Hostile => Color32::from_rgba_unmultiplied(255, 200, 80, 35),
        Affiliation::Neutral => Color32::from_rgba_unmultiplied(200, 255, 100, 35),
    }
}

/// Search mode radar color (cyan-tinted, lower visibility)
pub fn search_mode_color(affiliation: Affiliation) -> Color32 {
    match affiliation {
        Affiliation::Friendly => Color32::from_rgba_unmultiplied(100, 220, 255, 70),
        Affiliation::Hostile => Color32::from_rgba_unmultiplied(255, 150, 100, 70),
        Affiliation::Neutral => Color32::from_rgba_unmultiplied(150, 255, 150, 70),
    }
}

/// Search mode fill color (very low alpha for background areas)
pub fn search_mode_fill(affiliation: Affiliation) -> Color32 {
    match affiliation {
        Affiliation::Friendly => Color32::from_rgba_unmultiplied(100, 220, 255, 25),
        Affiliation::Hostile => Color32::from_rgba_unmultiplied(255, 150, 100, 25),
        Affiliation::Neutral => Color32::from_rgba_unmultiplied(150, 255, 150, 25),
    }
}

// ============================================================================
// Defense Unit Colors
// ============================================================================

/// Detection range stroke color
pub fn detection_range_stroke(affiliation: Affiliation) -> Color32 {
    match affiliation {
        Affiliation::Friendly => Color32::from_rgba_unmultiplied(80, 180, 255, 100),
        Affiliation::Hostile => Color32::from_rgba_unmultiplied(255, 80, 80, 100),
        Affiliation::Neutral => Color32::from_rgba_unmultiplied(100, 255, 100, 100),
    }
}

/// Engagement envelope color
pub fn engagement_envelope_color(affiliation: Affiliation) -> Color32 {
    match affiliation {
        Affiliation::Friendly => Color32::from_rgba_unmultiplied(100, 255, 150, 80),
        Affiliation::Hostile => Color32::from_rgba_unmultiplied(255, 150, 50, 80),
        Affiliation::Neutral => Color32::from_rgba_unmultiplied(255, 255, 100, 80),
    }
}

// ============================================================================
// Entity Colors
// ============================================================================

/// Missile path color
pub fn missile_path_color(affiliation: Affiliation) -> Color32 {
    match affiliation {
        Affiliation::Friendly => Color32::from_rgba_unmultiplied(100, 200, 255, 180),
        Affiliation::Hostile => Color32::from_rgba_unmultiplied(255, 100, 100, 180),
        Affiliation::Neutral => Color32::from_rgba_unmultiplied(200, 255, 100, 180),
    }
}

/// Interceptor path color (typically friendly, green-tinted)
pub fn interceptor_path_color() -> Color32 {
    Color32::from_rgba_unmultiplied(100, 255, 100, 200)
}

/// Defense unit marker color
pub fn defense_unit_marker(affiliation: Affiliation) -> Color32 {
    match affiliation {
        Affiliation::Friendly => Color32::from_rgb(80, 200, 120),   // Green
        Affiliation::Hostile => Color32::from_rgb(255, 100, 100),   // Red
        Affiliation::Neutral => Color32::from_rgb(200, 200, 100),   // Yellow
    }
}

/// Radar station marker color
pub fn radar_station_marker(affiliation: Affiliation) -> Color32 {
    match affiliation {
        Affiliation::Friendly => Color32::from_rgb(100, 180, 255),  // Blue
        Affiliation::Hostile => Color32::from_rgb(255, 120, 120),   // Red
        Affiliation::Neutral => Color32::from_rgb(180, 180, 180),   // Gray
    }
}

// ============================================================================
// Tracking and Quality Colors
// ============================================================================

/// Track quality color (red=low, yellow=medium, green=high)
pub fn track_quality_color(quality: f64) -> Color32 {
    let clamped = quality.clamp(0.0, 1.0);
    let r = ((1.0 - clamped) * 255.0) as u8;
    let g = (clamped * 255.0) as u8;
    Color32::from_rgb(r, g, 100)
}

/// Uncertainty indicator color (faded based on quality)
pub fn uncertainty_color(quality: f64) -> Color32 {
    let alpha = ((1.0 - quality) * 150.0 + 50.0) as u8;
    Color32::from_rgba_unmultiplied(255, 200, 100, alpha)
}

// ============================================================================
// Fixed Colors
// ============================================================================

/// Color for visual effects (explosions, intercepts)
pub mod effects {
    use super::Color32;

    pub fn explosion_outer() -> Color32 {
        Color32::from_rgba_unmultiplied(255, 200, 50, 150)
    }

    pub fn explosion_inner() -> Color32 {
        Color32::from_rgba_unmultiplied(255, 255, 200, 200)
    }

    pub fn intercept_flash() -> Color32 {
        Color32::from_rgba_unmultiplied(100, 255, 100, 200)
    }
}

/// Isometric 3D view colors
pub mod isometric {
    use super::Color32;

    pub fn engagement_envelope() -> Color32 {
        Color32::from_rgba_unmultiplied(255, 255, 100, 30) // Lightly shaded yellow
    }

    pub fn engagement_envelope_stroke() -> Color32 {
        Color32::from_rgba_unmultiplied(255, 255, 100, 100)
    }

    pub fn grid_line() -> Color32 {
        Color32::from_rgba_unmultiplied(100, 100, 100, 60)
    }

    pub fn altitude_line() -> Color32 {
        Color32::from_rgba_unmultiplied(150, 150, 150, 80)
    }
}
