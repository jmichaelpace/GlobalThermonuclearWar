use eframe::egui;
use serde::{Deserialize, Serialize};

/// Geographic coordinate (latitude, longitude)
#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize)]
pub struct GeoCoord {
    pub lat: f64,
    pub lon: f64,
}

impl GeoCoord {
    pub fn new(lat: f64, lon: f64) -> Self {
        Self { lat, lon }
    }
}

/// Map viewport handling pan, zoom, and coordinate transformations
pub struct Viewport {
    /// Center of the viewport in geographic coordinates
    pub center: GeoCoord,
    /// Zoom level (0 = world view, higher = more zoomed in)
    pub zoom: f64,
    /// Minimum zoom level
    pub min_zoom: f64,
    /// Maximum zoom level
    pub max_zoom: f64,
}

impl Default for Viewport {
    fn default() -> Self {
        Self {
            center: GeoCoord::new(20.0, 0.0), // Centered on Atlantic
            zoom: 2.0,
            min_zoom: 0.5,
            max_zoom: 18.0,
        }
    }
}

impl Viewport {
    /// Convert geographic coordinates to Web Mercator projection (EPSG:3857)
    pub fn geo_to_mercator(coord: GeoCoord) -> (f64, f64) {
        let x = coord.lon;
        let lat_rad = coord.lat.to_radians();
        let y = lat_rad.tan().asinh().to_degrees();
        (x, y)
    }

    /// Convert Web Mercator back to geographic coordinates
    pub fn mercator_to_geo(x: f64, y: f64) -> GeoCoord {
        let lon = x;
        let lat = y.to_radians().sinh().atan().to_degrees();
        GeoCoord::new(lat, lon)
    }

    /// Convert geographic coordinates to screen pixels
    /// Handles longitude wrapping to show entities at the closest position
    pub fn geo_to_screen(&self, coord: GeoCoord, screen_rect: egui::Rect) -> egui::Pos2 {
        let (mx, my) = Self::geo_to_mercator(coord);
        let (cx, cy) = Self::geo_to_mercator(self.center);

        // Handle longitude wrapping - find the closest representation
        let mut delta_x = mx - cx;
        if delta_x > 180.0 {
            delta_x -= 360.0;
        } else if delta_x < -180.0 {
            delta_x += 360.0;
        }

        let scale = self.pixels_per_degree(screen_rect);

        let screen_x = screen_rect.center().x as f64 + delta_x * scale;
        let screen_y = screen_rect.center().y as f64 - (my - cy) * scale; // Y is inverted

        egui::pos2(screen_x as f32, screen_y as f32)
    }

    /// Convert geographic coordinates to screen pixels WITHOUT longitude wrapping
    /// Used for tile positioning where we need exact unwrapped coordinates
    fn geo_to_screen_unwrapped(&self, coord: GeoCoord, screen_rect: egui::Rect) -> egui::Pos2 {
        let (mx, my) = Self::geo_to_mercator(coord);
        let (cx, cy) = Self::geo_to_mercator(self.center);

        // No wrapping - use raw delta for tile positioning
        let delta_x = mx - cx;

        let scale = self.pixels_per_degree(screen_rect);

        let screen_x = screen_rect.center().x as f64 + delta_x * scale;
        let screen_y = screen_rect.center().y as f64 - (my - cy) * scale;

        egui::pos2(screen_x as f32, screen_y as f32)
    }

    /// Convert geographic coordinates to screen pixels for all wrapped instances
    /// Returns up to 3 positions (main + left wrap + right wrap) that are visible
    pub fn geo_to_screen_wrapped(&self, coord: GeoCoord, screen_rect: egui::Rect) -> Vec<egui::Pos2> {
        let scale = self.pixels_per_degree(screen_rect);
        let world_width_pixels = 360.0 * scale;

        let base_pos = self.geo_to_screen(coord, screen_rect);
        let mut positions = vec![base_pos];

        // Check if wrapped versions are visible
        let left_wrap = egui::pos2(base_pos.x - world_width_pixels as f32, base_pos.y);
        let right_wrap = egui::pos2(base_pos.x + world_width_pixels as f32, base_pos.y);

        // Add wrapped positions if they're close to being visible
        let margin = 100.0; // pixels
        let expanded_rect = screen_rect.expand(margin);

        if expanded_rect.contains(left_wrap) {
            positions.push(left_wrap);
        }
        if expanded_rect.contains(right_wrap) {
            positions.push(right_wrap);
        }

        positions
    }

    /// Convert screen pixels to geographic coordinates
    pub fn screen_to_geo(&self, pos: egui::Pos2, screen_rect: egui::Rect) -> GeoCoord {
        let (cx, cy) = Self::geo_to_mercator(self.center);
        let scale = self.pixels_per_degree(screen_rect);

        let mx = cx + (pos.x as f64 - screen_rect.center().x as f64) / scale;
        let my = cy - (pos.y as f64 - screen_rect.center().y as f64) / scale;

        Self::mercator_to_geo(mx, my)
    }

    /// Get pixels per degree at current zoom level
    fn pixels_per_degree(&self, screen_rect: egui::Rect) -> f64 {
        let base_scale = screen_rect.width() as f64 / 360.0;
        base_scale * 2.0_f64.powf(self.zoom)
    }

    /// Pan the viewport by a screen delta
    pub fn pan(&mut self, delta: egui::Vec2, screen_rect: egui::Rect) {
        let scale = self.pixels_per_degree(screen_rect);
        let (cx, cy) = Self::geo_to_mercator(self.center);

        let new_mx = cx - delta.x as f64 / scale;
        let new_my = cy + delta.y as f64 / scale;

        self.center = Self::mercator_to_geo(new_mx, new_my);

        // Clamp latitude to valid Mercator range
        self.center.lat = self.center.lat.clamp(-85.0, 85.0);

        // Wrap longitude
        while self.center.lon > 180.0 {
            self.center.lon -= 360.0;
        }
        while self.center.lon < -180.0 {
            self.center.lon += 360.0;
        }
    }

    /// Zoom the viewport, keeping a screen point fixed
    pub fn zoom_at(&mut self, delta: f64, screen_pos: egui::Pos2, screen_rect: egui::Rect) {
        let geo_before = self.screen_to_geo(screen_pos, screen_rect);

        self.zoom = (self.zoom + delta).clamp(self.min_zoom, self.max_zoom);

        let geo_after = self.screen_to_geo(screen_pos, screen_rect);

        // Adjust center to keep the point under the cursor fixed
        self.center.lat += geo_before.lat - geo_after.lat;
        self.center.lon += geo_before.lon - geo_after.lon;

        self.center.lat = self.center.lat.clamp(-85.0, 85.0);
    }

    /// Get the tile coordinates for the current view (with wrapping support)
    pub fn get_visible_tiles(&self, screen_rect: egui::Rect) -> Vec<VisibleTile> {
        let tile_zoom = self.tile_zoom_level();
        let num_tiles = 2_u32.pow(tile_zoom);

        // Get corners in geo coordinates
        let top_left = self.screen_to_geo(screen_rect.left_top(), screen_rect);
        let bottom_right = self.screen_to_geo(screen_rect.right_bottom(), screen_rect);

        // Convert to tile coordinates (these can be outside 0..num_tiles range)
        let (min_x, min_y) = Self::geo_to_tile(top_left, tile_zoom);
        let (max_x, max_y) = Self::geo_to_tile(bottom_right, tile_zoom);

        let mut tiles = Vec::new();
        for y in min_y..=max_y {
            for x in min_x..=max_x {
                // Wrap X to valid tile coordinate, but keep original for positioning
                let wrapped_x = ((x % num_tiles as i32) + num_tiles as i32) as u32 % num_tiles;
                if y >= 0 && (y as u32) < num_tiles {
                    tiles.push(VisibleTile {
                        coord: TileCoord {
                            x: wrapped_x,
                            y: y as u32,
                            z: tile_zoom,
                        },
                        unwrapped_x: x,
                    });
                }
            }
        }

        tiles
    }

    /// Get the appropriate tile zoom level for the current viewport zoom
    pub fn tile_zoom_level(&self) -> u32 {
        (self.zoom.round() as u32).clamp(0, 18)
    }

    /// Convert geographic coordinates to tile coordinates
    fn geo_to_tile(coord: GeoCoord, zoom: u32) -> (i32, i32) {
        let n = 2.0_f64.powi(zoom as i32);
        let x = ((coord.lon + 180.0) / 360.0 * n).floor() as i32;
        let lat_rad = coord.lat.to_radians();
        let y = ((1.0 - lat_rad.tan().asinh() / std::f64::consts::PI) / 2.0 * n).floor() as i32;
        (x, y)
    }

    /// Get the screen rect for a tile (using unwrapped coordinates for proper positioning)
    pub fn visible_tile_screen_rect(&self, tile: &VisibleTile, screen_rect: egui::Rect) -> egui::Rect {
        // Use unwrapped_x for screen position calculation to handle world wrapping
        let top_left = Self::tile_to_geo_unwrapped(tile.unwrapped_x, tile.coord.y, tile.coord.z);
        let bottom_right = Self::tile_to_geo_unwrapped(tile.unwrapped_x + 1, tile.coord.y + 1, tile.coord.z);

        // Use unwrapped geo_to_screen to avoid longitude wrapping issues
        let screen_tl = self.geo_to_screen_unwrapped(top_left, screen_rect);
        let screen_br = self.geo_to_screen_unwrapped(bottom_right, screen_rect);

        egui::Rect::from_two_pos(screen_tl, screen_br)
    }

    /// Convert tile coordinates to geographic coordinates (top-left corner of tile)
    /// Uses unwrapped X coordinate to handle positions beyond -180/+180
    fn tile_to_geo_unwrapped(x: i32, y: u32, z: u32) -> GeoCoord {
        let n = 2.0_f64.powi(z as i32);
        let lon = x as f64 / n * 360.0 - 180.0;
        let lat_rad = (std::f64::consts::PI * (1.0 - 2.0 * y as f64 / n)).sinh().atan();
        GeoCoord::new(lat_rad.to_degrees(), lon)
    }

    /// Convert tile coordinates to geographic coordinates (top-left corner of tile)
    fn tile_to_geo(x: u32, y: u32, z: u32) -> GeoCoord {
        let n = 2.0_f64.powi(z as i32);
        let lon = x as f64 / n * 360.0 - 180.0;
        let lat_rad = (std::f64::consts::PI * (1.0 - 2.0 * y as f64 / n)).sinh().atan();
        GeoCoord::new(lat_rad.to_degrees(), lon)
    }
}

/// Tile coordinate in the standard XYZ tile scheme
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct TileCoord {
    pub x: u32,
    pub y: u32,
    pub z: u32,
}

impl TileCoord {
    pub fn url(&self, base_url: &str) -> String {
        base_url
            .replace("{z}", &self.z.to_string())
            .replace("{x}", &self.x.to_string())
            .replace("{y}", &self.y.to_string())
    }
}

/// Tile with position info for rendering (handles world wrapping)
#[derive(Clone, Copy, Debug)]
pub struct VisibleTile {
    /// The actual tile coordinate (wrapped to valid range)
    pub coord: TileCoord,
    /// The unwrapped X position for screen coordinate calculation
    pub unwrapped_x: i32,
}
