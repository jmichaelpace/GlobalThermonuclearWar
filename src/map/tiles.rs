use crate::map::TileCoord;
use eframe::egui;
use std::collections::HashMap;
use std::sync::mpsc::{channel, Receiver, Sender};
use std::thread;

/// Map style for MapTiler tiles
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MapStyle {
    StreetsLight,
    StreetsDark,
    Basic,
    BasicDark,
    Toner,
    Satellite,
}

impl MapStyle {
    /// Get the MapTiler style ID for URL construction
    pub fn style_id(&self) -> &'static str {
        match self {
            MapStyle::StreetsLight => "streets-v2",
            MapStyle::StreetsDark => "streets-v2-dark",
            MapStyle::Basic => "basic-v2",
            MapStyle::BasicDark => "basic-v2-dark",
            MapStyle::Toner => "toner-v2",
            MapStyle::Satellite => "satellite",
        }
    }

    /// Display name for UI
    pub fn display_name(&self) -> &'static str {
        match self {
            MapStyle::StreetsLight => "Streets (Light)",
            MapStyle::StreetsDark => "Streets (Dark)",
            MapStyle::Basic => "Basic (Light)",
            MapStyle::BasicDark => "Basic (Dark)",
            MapStyle::Toner => "Toner (B&W)",
            MapStyle::Satellite => "Satellite",
        }
    }
}

impl Default for MapStyle {
    fn default() -> Self {
        MapStyle::BasicDark
    }
}

/// Status of a tile in the cache
enum TileStatus {
    Loading,
    Loaded(egui::TextureHandle),
    Failed,
}

/// Request to load a tile
struct TileRequest {
    coord: TileCoord,
    url: String,
}

/// Message from tile loading thread
struct TileLoadResult {
    coord: TileCoord,
    data: Result<Vec<u8>, String>,
}

/// Tile coordinate for EPSG:4326 (geographic projection) tiles
#[derive(Clone, Copy, Debug, Hash, Eq, PartialEq)]
pub struct GibsTileCoord {
    pub z: u32,   // Zoom level (0-8 for Blue Marble)
    pub row: u32, // Tile row
    pub col: u32, // Tile column
}

/// Request to load a GIBS tile
struct GibsTileRequest {
    coord: GibsTileCoord,
    url: String,
}

/// Message from GIBS tile loading thread
struct GibsTileLoadResult {
    coord: GibsTileCoord,
    data: Result<Vec<u8>, String>,
}

/// Status of a GIBS tile in the cache
enum GibsTileStatus {
    Loading,
    Loaded(egui::TextureHandle),
    Failed,
}

/// Cache for NASA GIBS tiles (Blue Marble satellite imagery)
pub struct GibsTileCache {
    tiles: HashMap<GibsTileCoord, GibsTileStatus>,
    request_tx: Sender<GibsTileRequest>,
    result_rx: Receiver<GibsTileLoadResult>,
    max_cache_size: usize,
    access_order: Vec<GibsTileCoord>,
}

impl GibsTileCache {
    pub fn new() -> Self {
        let (request_tx, request_rx) = channel::<GibsTileRequest>();
        let (result_tx, result_rx) = channel::<GibsTileLoadResult>();

        // Spawn background thread for GIBS tile loading
        thread::spawn(move || {
            let client = reqwest::blocking::Client::builder()
                .user_agent("GlobalThermonuclearWar/0.1 (Educational Simulation)")
                .timeout(std::time::Duration::from_secs(30))
                .build()
                .expect("Failed to create HTTP client");

            while let Ok(request) = request_rx.recv() {
                let result = client
                    .get(&request.url)
                    .send()
                    .and_then(|r| r.bytes())
                    .map(|b| b.to_vec())
                    .map_err(|e| e.to_string());

                let _ = result_tx.send(GibsTileLoadResult {
                    coord: request.coord,
                    data: result,
                });
            }
        });

        Self {
            tiles: HashMap::new(),
            request_tx,
            result_rx,
            max_cache_size: 512, // Large cache to prevent tile eviction during rotation
            access_order: Vec::new(),
        }
    }

    /// Build the NASA GIBS URL for Blue Marble tiles (EPSG:4326)
    fn build_url(&self, coord: &GibsTileCoord) -> String {
        // NASA GIBS WMTS URL for Blue Marble with shaded relief and bathymetry
        // Using the 500m TileMatrixSet for EPSG:4326
        // Note: The double slash after "default" is for the empty time parameter (Blue Marble is static)
        format!(
            "https://gibs.earthdata.nasa.gov/wmts/epsg4326/best/BlueMarble_ShadedRelief_Bathymetry/default//500m/{}/{}/{}.jpeg",
            coord.z, coord.row, coord.col
        )
    }

    /// Process any completed tile loads, returns true if any tiles were processed
    pub fn process_pending(&mut self, ctx: &egui::Context) -> bool {
        let mut processed_any = false;
        while let Ok(result) = self.result_rx.try_recv() {
            processed_any = true;
            match result.data {
                Ok(bytes) => {
                    if let Ok(image) = image::load_from_memory(&bytes) {
                        let rgba = image.to_rgba8();
                        let size = [rgba.width() as usize, rgba.height() as usize];
                        let pixels = rgba.into_raw();

                        let color_image = egui::ColorImage::from_rgba_unmultiplied(size, &pixels);
                        let texture = ctx.load_texture(
                            format!(
                                "gibs_tile_{}_{}_{}",
                                result.coord.z, result.coord.row, result.coord.col
                            ),
                            color_image,
                            egui::TextureOptions::LINEAR,
                        );

                        self.tiles
                            .insert(result.coord, GibsTileStatus::Loaded(texture));
                    } else {
                        self.tiles.insert(result.coord, GibsTileStatus::Failed);
                    }
                }
                Err(_) => {
                    self.tiles.insert(result.coord, GibsTileStatus::Failed);
                }
            }
        }
        processed_any
    }

    /// Get a tile, requesting it if not cached
    pub fn get_tile(&mut self, coord: GibsTileCoord) -> Option<&egui::TextureHandle> {
        // Update access order for LRU
        self.access_order.retain(|c| *c != coord);
        self.access_order.push(coord);

        // Evict old tiles if cache is full (but never evict loading tiles)
        while self.tiles.len() > self.max_cache_size && !self.access_order.is_empty() {
            let old = self.access_order.remove(0);
            // Only evict if not loading (to prevent re-requesting in-flight tiles)
            if !matches!(self.tiles.get(&old), Some(GibsTileStatus::Loading)) {
                self.tiles.remove(&old);
            }
        }

        // Check if tile is already cached or loading
        if !self.tiles.contains_key(&coord) {
            // Request the tile
            self.tiles.insert(coord, GibsTileStatus::Loading);
            let url = self.build_url(&coord);
            let _ = self.request_tx.send(GibsTileRequest { coord, url });
        }

        // Return the tile if loaded
        match self.tiles.get(&coord) {
            Some(GibsTileStatus::Loaded(texture)) => Some(texture),
            _ => None,
        }
    }

    /// Check if any tiles are currently loading
    pub fn has_loading_tiles(&self) -> bool {
        self.tiles
            .values()
            .any(|status| matches!(status, GibsTileStatus::Loading))
    }

    /// Convert lat/lon to NASA GIBS EPSG:4326 tile coordinates
    /// GIBS uses 288 degrees per tile at level 0, NOT 180 degrees
    /// Formula: row = ((90 - lat) * 2^level) / 288
    ///          col = ((180 + lon) * 2^level) / 288
    pub fn geo_to_tile(lat: f64, lon: f64, zoom: u32) -> GibsTileCoord {
        // Clamp zoom to valid range (0-8 for Blue Marble)
        let zoom = zoom.min(8);
        let scale = (1u32 << zoom) as f64; // 2^zoom

        // Clamp lat/lon to valid ranges
        let lat = lat.clamp(-90.0, 90.0);
        let lon = lon.clamp(-180.0, 180.0);

        // GIBS EPSG:4326 uses 288 degrees per tile at zoom 0
        let row = (((90.0 - lat) * scale) / 288.0).floor().max(0.0) as u32;
        let col = (((180.0 + lon) * scale) / 288.0).floor().max(0.0) as u32;

        // Calculate max valid tile indices for this zoom
        // At zoom z, there are ceil(180 * 2^z / 288) rows and ceil(360 * 2^z / 288) cols
        let max_row = ((180.0 * scale) / 288.0).ceil() as u32;
        let max_col = ((360.0 * scale) / 288.0).ceil() as u32;

        GibsTileCoord {
            z: zoom,
            row: row.min(max_row.saturating_sub(1)),
            col: col.min(max_col.saturating_sub(1)),
        }
    }

    /// Get the lat/lon bounds for a GIBS tile
    /// Uses 288 degrees per tile at zoom 0
    pub fn tile_bounds(coord: &GibsTileCoord) -> (f64, f64, f64, f64) {
        let scale = (1u32 << coord.z) as f64; // 2^zoom
        let tile_size = 288.0 / scale; // Degrees per tile at this zoom

        // Calculate bounds
        let lat_max = 90.0 - (coord.row as f64 * tile_size);
        let lat_min = lat_max - tile_size;

        let lon_min = (coord.col as f64 * tile_size) - 180.0;
        let lon_max = lon_min + tile_size;

        (lat_min, lat_max, lon_min, lon_max)
    }
}

/// Cache for map tiles with async loading
pub struct TileCache {
    tiles: HashMap<TileCoord, TileStatus>,
    request_tx: Sender<TileRequest>,
    result_rx: Receiver<TileLoadResult>,
    max_cache_size: usize,
    access_order: Vec<TileCoord>,
    api_key: String,
    style: MapStyle,
}

impl TileCache {
    pub fn new(api_key: String) -> Self {
        let (request_tx, request_rx) = channel::<TileRequest>();
        let (result_tx, result_rx) = channel::<TileLoadResult>();

        // Spawn background thread for tile loading
        thread::spawn(move || {
            let client = reqwest::blocking::Client::builder()
                .user_agent("GlobalThermonuclearWar/0.1")
                .build()
                .expect("Failed to create HTTP client");

            while let Ok(request) = request_rx.recv() {
                let result = client
                    .get(&request.url)
                    .send()
                    .and_then(|r| r.bytes())
                    .map(|b| b.to_vec())
                    .map_err(|e| e.to_string());

                let _ = result_tx.send(TileLoadResult {
                    coord: request.coord,
                    data: result,
                });
            }
        });

        Self {
            tiles: HashMap::new(),
            request_tx,
            result_rx,
            max_cache_size: 256,
            access_order: Vec::new(),
            api_key,
            style: MapStyle::default(),
        }
    }

    /// Get current map style
    pub fn style(&self) -> MapStyle {
        self.style
    }

    /// Set map style and clear cache to load new tiles
    pub fn set_style(&mut self, style: MapStyle) {
        if self.style != style {
            self.style = style;
            // Clear the cache so tiles reload with new style
            self.tiles.clear();
            self.access_order.clear();
        }
    }

    /// Build the tile URL for MapTiler with current style
    fn build_url(&self, coord: &TileCoord) -> String {
        format!(
            "https://api.maptiler.com/maps/{}/{}/{}/{}@2x.png?key={}",
            self.style.style_id(),
            coord.z,
            coord.x,
            coord.y,
            self.api_key
        )
    }

    /// Process any completed tile loads, returns true if any tiles were processed
    pub fn process_pending(&mut self, ctx: &egui::Context) -> bool {
        let mut processed_any = false;
        while let Ok(result) = self.result_rx.try_recv() {
            processed_any = true;
            match result.data {
                Ok(bytes) => {
                    if let Ok(image) = image::load_from_memory(&bytes) {
                        let rgba = image.to_rgba8();
                        let size = [rgba.width() as usize, rgba.height() as usize];
                        let pixels = rgba.into_raw();

                        let color_image = egui::ColorImage::from_rgba_unmultiplied(size, &pixels);
                        let texture = ctx.load_texture(
                            format!(
                                "tile_{}_{}_{}",
                                result.coord.z, result.coord.x, result.coord.y
                            ),
                            color_image,
                            egui::TextureOptions::LINEAR,
                        );

                        self.tiles.insert(result.coord, TileStatus::Loaded(texture));
                    } else {
                        self.tiles.insert(result.coord, TileStatus::Failed);
                    }
                }
                Err(_) => {
                    self.tiles.insert(result.coord, TileStatus::Failed);
                }
            }
        }
        processed_any
    }

    /// Get a tile, requesting it if not cached
    pub fn get_tile(&mut self, coord: TileCoord) -> Option<&egui::TextureHandle> {
        // Update access order for LRU
        self.access_order.retain(|c| *c != coord);
        self.access_order.push(coord);

        // Evict old tiles if cache is full
        while self.tiles.len() > self.max_cache_size && !self.access_order.is_empty() {
            let old = self.access_order.remove(0);
            self.tiles.remove(&old);
        }

        // Check if tile is already cached or loading
        if !self.tiles.contains_key(&coord) {
            // Request the tile
            self.tiles.insert(coord, TileStatus::Loading);
            let url = self.build_url(&coord);
            let _ = self.request_tx.send(TileRequest { coord, url });
        }

        // Return the tile if loaded
        match self.tiles.get(&coord) {
            Some(TileStatus::Loaded(texture)) => Some(texture),
            _ => None,
        }
    }

    /// Check if a tile is currently loading
    pub fn is_loading(&self, coord: &TileCoord) -> bool {
        matches!(self.tiles.get(coord), Some(TileStatus::Loading))
    }

    /// Check if any tiles in the cache are currently loading
    pub fn has_loading_tiles(&self) -> bool {
        self.tiles
            .values()
            .any(|status| matches!(status, TileStatus::Loading))
    }
}
