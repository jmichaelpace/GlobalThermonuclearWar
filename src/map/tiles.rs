use crate::map::TileCoord;
use eframe::egui;
use std::collections::{HashMap, HashSet};
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::mpsc::{channel, Receiver, Sender};
use std::thread;
use std::time::SystemTime;

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
    /// Failed to load; carries the wall-clock time of the failure so a
    /// transient network error can be retried after TILE_RETRY_SEC
    /// instead of blanking the tile until a style change.
    Failed(SystemTime),
}

/// How long a failed tile stays failed before a retry is attempted
const TILE_RETRY_SEC: f64 = 30.0;

/// Root path of the on-disk tile cache (resolved once; None if the home
/// directory cannot be determined — disk caching degrades to disabled).
fn disk_cache_root() -> Option<PathBuf> {
    // macOS convention; matches the project's no-new-deps approach.
    // The API key is never part of any cache path.
    std::env::var("HOME")
        .ok()
        .map(|home| Path::new(&home).join("Library/Caches/GlobalThermonuclearWar/tiles"))
}

/// Per-style cache directory for a tile, e.g. .../tiles/streets-v2/7/64/43.png
fn disk_cache_path(style_id: &str, coord: &TileCoord) -> Option<PathBuf> {
    disk_cache_root().map(|root| {
        root.join(style_id)
            .join(coord.z.to_string())
            .join(coord.x.to_string())
            .join(format!("{}.png", coord.y))
    })
}

/// Approximate on-disk cache budget. Tiles average ~150-300 KB @2x.
const DISK_CACHE_MAX_BYTES: u64 = 512 * 1024 * 1024;

/// Files written since the last prune check; the loader thread budgets
/// against the average tile size to decide when to run a prune pass.
static DISK_WRITES_SINCE_CHECK: AtomicUsize = AtomicUsize::new(0);

/// Run a prune when roughly this many writes have accumulated
/// (512 MB / ~200 KB average tile ≈ 2,600 writes; check well before that).
const DISK_WRITES_PER_PRUNE: usize = 512;

/// Prune the on-disk cache: delete oldest-mtime tile files until the
/// tree is under ~75% of the cap. Runs on the loader thread (async from
/// the UI). Best-effort: any I/O error just skips that file.
fn prune_disk_cache() {
    let Some(root) = disk_cache_root() else {
        return;
    };

    // Collect (mtime, path, size) for every cached tile file
    let mut entries: Vec<(SystemTime, PathBuf, u64)> = Vec::new();
    let mut total_bytes: u64 = 0;

    let style_dirs = match std::fs::read_dir(&root) {
        Ok(iter) => iter,
        Err(_) => return,
    };
    for style_dir in style_dirs.flatten() {
        let z_dirs = match std::fs::read_dir(style_dir.path()) {
            Ok(iter) => iter,
            Err(_) => continue,
        };
        for z_dir in z_dirs.flatten() {
            let x_dirs = match std::fs::read_dir(z_dir.path()) {
                Ok(iter) => iter,
                Err(_) => continue,
            };
            for x_dir in x_dirs.flatten() {
                let files = match std::fs::read_dir(x_dir.path()) {
                    Ok(iter) => iter,
                    Err(_) => continue,
                };
                for file in files.flatten() {
                    if let Ok(meta) = file.metadata() {
                        if meta.is_file() {
                            let mtime = meta.modified().unwrap_or(SystemTime::UNIX_EPOCH);
                            total_bytes += meta.len();
                            entries.push((mtime, file.path(), meta.len()));
                        }
                    }
                }
            }
        }
    }

    let target = DISK_CACHE_MAX_BYTES / 4 * 3; // 75% of cap
    if total_bytes <= target {
        return;
    }

    // Oldest first
    entries.sort_by_key(|(mtime, _, _)| *mtime);
    for (_, path, size) in entries {
        if total_bytes <= target {
            break;
        }
        if std::fs::remove_file(&path).is_ok() {
            total_bytes = total_bytes.saturating_sub(size);
        }
    }
}

/// Read a tile's bytes from the disk cache, if present.
fn disk_cache_read(style_id: &str, coord: &TileCoord) -> Option<Vec<u8>> {
    let path = disk_cache_path(style_id, coord)?;
    std::fs::read(path).ok().filter(|b| !b.is_empty())
}

/// Atomically write tile bytes to the disk cache (temp file + rename).
/// Never called for failed loads. Creates parent dirs as needed.
fn disk_cache_write(style_id: &str, coord: &TileCoord, bytes: &[u8]) {
    let Some(path) = disk_cache_path(style_id, coord) else {
        return;
    };
    if let Some(parent) = path.parent() {
        if std::fs::create_dir_all(parent).is_err() {
            return;
        }
    }
    let tmp = path.with_extension("tmp");
    if std::fs::write(&tmp, bytes).is_ok() && std::fs::rename(&tmp, &path).is_ok() {
        DISK_WRITES_SINCE_CHECK.fetch_add(1, Ordering::Relaxed);
        if DISK_WRITES_SINCE_CHECK.load(Ordering::Relaxed) >= DISK_WRITES_PER_PRUNE {
            DISK_WRITES_SINCE_CHECK.store(0, Ordering::Relaxed);
            prune_disk_cache();
        }
    }
}

/// Request to load a tile
struct TileRequest {
    coord: TileCoord,
    url: String,
    /// MapTiler style id — used for the disk cache path (never the API key)
    style_id: String,
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

/// Ancestor lookup math for the multi-zoom fallback: which ancestor
/// coordinate (up to `levels_up`) and which UV sub-rect of that ancestor's
/// texture covers `coord`. Pure function (unit-tested).
///
/// Returns None if no ancestor exists within the level budget or k would
/// underflow zoom 0. The caller checks its cache for each ancestor in
/// order and uses the first hit.
pub fn ancestor_fallback(coord: TileCoord, levels_up: u8) -> Option<Vec<(TileCoord, egui::Rect)>> {
    let mut out = Vec::new();
    for k in 1..=levels_up {
        let Some(z) = coord.z.checked_sub(k as u32) else {
            break;
        };
        let shift = k as u32;
        let frac = 1.0 / (1u32 << shift) as f32;
        let u0 = (coord.x & ((1 << shift) - 1)) as f32 * frac;
        let v0 = (coord.y & ((1 << shift) - 1)) as f32 * frac;
        let uv = egui::Rect::from_min_max(egui::pos2(u0, v0), egui::pos2(u0 + frac, v0 + frac));
        out.push((
            TileCoord {
                x: coord.x >> shift,
                y: coord.y >> shift,
                z,
            },
            uv,
        ));
    }
    Some(out)
}

/// Choose a tile zoom level whose native ground resolution best matches a
/// requested meters-per-pixel target (pure, unit-tested). Web-tile ground
/// resolution at zoom z is 156543.03 * cos(lat) / 2^z meters/pixel
/// (256-pixel tiles); pick the z whose resolution is closest without
/// being much finer than needed (oversampling wastes tiles under a
/// perspective-squashed plane).
pub fn zoom_for_resolution(meters_per_pixel: f64, lat_deg: f64) -> u32 {
    const BASE_RES_M: f64 = 156_543.03; // equatorial, zoom 0, 256px tiles
    let target = meters_per_pixel.max(1.0);
    let cos_lat = lat_deg.to_radians().cos().max(0.01);
    // Exact fractional zoom: BASE*cos/2^z = target -> z = log2(BASE*cos/target)
    let z_exact = (BASE_RES_M * cos_lat / target).log2();
    // Round to the nearest integer level; clamp to the standard range
    (z_exact.round().clamp(0.0, 18.0)) as u32
}

impl TileCache {
    pub fn new(api_key: String) -> Self {
        let (request_tx, request_rx) = channel::<TileRequest>();
        let (result_tx, result_rx) = channel::<TileLoadResult>();

        // Spawn background thread for tile loading. Disk cache is checked
        // before the network; successful fetches are persisted atomically.
        // Failures are returned as errors and never written to disk.
        thread::spawn(move || {
            let client = reqwest::blocking::Client::builder()
                .user_agent("GlobalThermonuclearWar/0.1")
                .build()
                .expect("Failed to create HTTP client");

            while let Ok(request) = request_rx.recv() {
                // Disk cache first (async from the UI; tile bytes decode
                // identically regardless of origin)
                if let Some(bytes) = disk_cache_read(&request.style_id, &request.coord) {
                    let _ = result_tx.send(TileLoadResult {
                        coord: request.coord,
                        data: Ok(bytes),
                    });
                    continue;
                }

                let result = client
                    .get(&request.url)
                    .send()
                    .and_then(|r| r.bytes())
                    .map(|b| b.to_vec())
                    .map_err(|e| e.to_string());

                // Persist successful fetches to the disk cache
                if let Ok(bytes) = &result {
                    disk_cache_write(&request.style_id, &request.coord, bytes);
                }

                let _ = result_tx.send(TileLoadResult {
                    coord: request.coord,
                    data: result,
                });
            }
        });

        // One-shot startup prune: bring a cache left over from previous
        // sessions back under the size cap (async; never blocks startup)
        thread::spawn(prune_disk_cache);

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
                        self.tiles
                            .insert(result.coord, TileStatus::Failed(SystemTime::now()));
                    }
                }
                Err(_) => {
                    self.tiles
                        .insert(result.coord, TileStatus::Failed(SystemTime::now()));
                }
            }
        }
        processed_any
    }

    /// Get a tile, requesting it if not cached. Failed tiles are retried
    /// after TILE_RETRY_SEC; Loading and fresh failures are left alone.
    pub fn get_tile(&mut self, coord: TileCoord) -> Option<&egui::TextureHandle> {
        // Update access order for LRU
        self.access_order.retain(|c| *c != coord);
        self.access_order.push(coord);

        // Evict old tiles if cache is full
        while self.tiles.len() > self.max_cache_size && !self.access_order.is_empty() {
            let old = self.access_order.remove(0);
            self.tiles.remove(&old);
        }

        // Decide whether a (new) request is needed
        let needs_request = match self.tiles.get(&coord) {
            None => true,
            Some(TileStatus::Failed(failed_at)) => {
                // Retry transient failures after the retry window
                let age = failed_at.elapsed().unwrap_or_default().as_secs_f64();
                age >= TILE_RETRY_SEC
            }
            _ => false, // Loading or Loaded
        };
        if needs_request {
            self.tiles.insert(coord, TileStatus::Loading);
            let url = self.build_url(&coord);
            let style_id = self.style.style_id().to_string();
            let _ = self.request_tx.send(TileRequest {
                coord,
                url,
                style_id,
            });
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

    /// Non-mutating lookup: Some(texture) only if this exact tile is
    /// loaded, None otherwise. Unlike get_tile this never triggers a
    /// request — callers use it to read cache state (e.g. the 3D
    /// intercept view's ground plane, which requests asynchronously
    /// elsewhere).
    pub fn peek_loaded(&self, coord: &TileCoord) -> Option<&egui::TextureHandle> {
        match self.tiles.get(coord) {
            Some(TileStatus::Loaded(texture)) => Some(texture),
            _ => None,
        }
    }

    /// Look up the closest LOADED ancestor for `coord` without mutating:
    /// returns (texture, uv) for the nearest cached ancestor within
    /// `levels_up` (UV selects the quadrant of the ancestor covering
    /// `coord`). Non-requesting companion to peek_loaded.
    pub fn peek_loaded_with_fallback(
        &self,
        coord: &TileCoord,
        levels_up: u8,
    ) -> Option<(&egui::TextureHandle, egui::Rect)> {
        if let Some(texture) = self.peek_loaded(coord) {
            return Some((
                texture,
                egui::Rect::from_min_max(egui::pos2(0.0, 0.0), egui::pos2(1.0, 1.0)),
            ));
        }
        for (ancestor, uv) in ancestor_fallback(*coord, levels_up)? {
            if let Some(texture) = self.peek_loaded(&ancestor) {
                return Some((texture, uv));
            }
        }
        None
    }

    /// Check if any tiles in the cache are currently loading
    pub fn has_loading_tiles(&self) -> bool {
        self.tiles
            .values()
            .any(|status| matches!(status, TileStatus::Loading))
    }

    /// Get a tile with a multi-zoom fallback: if the exact tile is not
    /// loaded, walk up to `levels_up` ancestor zoom levels (each ancestor
    /// covers 2^k children) and return the closest loaded ancestor along
    /// with the UV sub-rect selecting the quadrant of the ancestor
    /// texture that this tile occupies. The renderer draws the ancestor
    /// into the tile's screen rect with that UV — an upscaled, slightly
    /// blurry stand-in (standard map-application behavior) instead of a
    /// blank rectangle while the exact tile loads.
    ///
    /// NOTE: does NOT trigger a request for the exact tile by itself —
    /// pair with `get_tile` for that (the render loop already calls
    /// both paths via get_tile then fallback).
    pub fn get_tile_with_fallback(
        &self,
        coord: TileCoord,
        levels_up: u8,
    ) -> Option<(&egui::TextureHandle, egui::Rect)> {
        // Exact hit: full texture
        if let Some(TileStatus::Loaded(texture)) = self.tiles.get(&coord) {
            return Some((
                texture,
                egui::Rect::from_min_max(egui::pos2(0.0, 0.0), egui::pos2(1.0, 1.0)),
            ));
        }

        // Ancestor candidates, nearest first (shared math with unit tests)
        for (ancestor, uv) in ancestor_fallback(coord, levels_up)? {
            if let Some(TileStatus::Loaded(texture)) = self.tiles.get(&ancestor) {
                return Some((texture, uv));
            }
        }
        None
    }

    /// Prefetch the immediate parents (z-1) of the given tiles so a
    /// fallback always exists while zooming. Only requests ancestors
    /// that are neither cached nor loading; returns how many requests
    /// were issued (callers can rate-limit). No-op at zoom 0.
    pub fn prefetch_parents(&mut self, coords: &[TileCoord]) -> usize {
        let mut requested = 0;
        let mut seen: HashSet<TileCoord> = HashSet::new();
        let style_id = self.style.style_id().to_string();

        for coord in coords {
            if coord.z == 0 {
                continue;
            }
            let parent = TileCoord {
                x: coord.x >> 1,
                y: coord.y >> 1,
                z: coord.z - 1,
            };
            if !seen.insert(parent) {
                continue; // dedupe within the batch
            }
            match self.tiles.get(&parent) {
                Some(TileStatus::Loaded(_)) | Some(TileStatus::Loading) => {}
                Some(TileStatus::Failed(failed_at)) => {
                    // Don't hammer retries from the prefetch path: only
                    // re-request if past the retry window
                    let age = failed_at.elapsed().unwrap_or_default().as_secs_f64();
                    if age < TILE_RETRY_SEC {
                        continue;
                    }
                }
                None => {}
            }
            self.tiles.insert(parent, TileStatus::Loading);
            let url = self.build_url(&parent);
            let _ = self.request_tx.send(TileRequest {
                coord: parent,
                url,
                style_id: style_id.clone(),
            });
            requested += 1;
        }
        requested
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // ---- Ancestor fallback math ----

    #[test]
    fn test_ancestor_fallback_immediate_parent() {
        // Child (x=5, y=3, z=4) -> parent (2, 1, 3); child occupies the
        // quadrant u=[0.5,1.0], v=[0.5,1.0] (low bits 1,1)
        let cands = ancestor_fallback(TileCoord { x: 5, y: 3, z: 4 }, 3).unwrap();
        assert_eq!(cands.len(), 3);
        let (parent, uv) = &cands[0];
        assert_eq!(*parent, TileCoord { x: 2, y: 1, z: 3 });
        assert!((uv.left() - 0.5).abs() < 1e-6 && (uv.top() - 0.5).abs() < 1e-6);
        assert!((uv.right() - 1.0).abs() < 1e-6 && (uv.bottom() - 1.0).abs() < 1e-6);
    }

    #[test]
    fn test_ancestor_fallback_quadrant_zero() {
        // Child with low bits 0,0 occupies the top-left quadrant
        let cands = ancestor_fallback(TileCoord { x: 4, y: 2, z: 4 }, 1).unwrap();
        let (parent, uv) = &cands[0];
        assert_eq!(*parent, TileCoord { x: 2, y: 1, z: 3 });
        assert!(uv.left() < 1e-6 && uv.top() < 1e-6);
        assert!((uv.right() - 0.5).abs() < 1e-6 && (uv.bottom() - 0.5).abs() < 1e-6);
    }

    #[test]
    fn test_ancestor_fallback_two_levels_up() {
        // Grandparent (k=2): child (7, 5, 6) -> (1, 1, 4); frac = 1/4;
        // low bits (7&3, 5&3) = (3, 1) -> u=[0.75, 1.0], v=[0.25, 0.5]
        let cands = ancestor_fallback(TileCoord { x: 7, y: 5, z: 6 }, 2).unwrap();
        assert_eq!(cands.len(), 2);
        let (gp, uv) = &cands[1];
        assert_eq!(*gp, TileCoord { x: 1, y: 1, z: 4 });
        assert!((uv.left() - 0.75).abs() < 1e-6);
        assert!((uv.top() - 0.25).abs() < 1e-6);
        assert!((uv.right() - 1.0).abs() < 1e-6);
        assert!((uv.bottom() - 0.5).abs() < 1e-6);
    }

    #[test]
    fn test_ancestor_fallback_zoom_floor() {
        // z=1 with 3 levels budgeted: only the z=0 parent exists
        let cands = ancestor_fallback(TileCoord { x: 1, y: 1, z: 1 }, 3).unwrap();
        assert_eq!(cands.len(), 1);
        assert_eq!(cands[0].0, TileCoord { x: 0, y: 0, z: 0 });

        // z=0: no ancestors at all
        let cands = ancestor_fallback(TileCoord { x: 0, y: 0, z: 0 }, 3).unwrap();
        assert!(cands.is_empty());
    }

    #[test]
    fn test_ancestor_fallback_wraps_uv_coverage() {
        // The four children of one parent must cover the full UV square
        // without gaps or overlap
        for y in 0..2u32 {
            for x in 0..2u32 {
                let cands = ancestor_fallback(TileCoord { x, y, z: 3 }, 1).unwrap();
                let uv = cands[0].1;
                assert!((uv.left() - x as f32 * 0.5).abs() < 1e-6);
                assert!((uv.top() - y as f32 * 0.5).abs() < 1e-6);
            }
        }
    }

    // ---- Disk cache paths ----

    #[test]
    fn test_disk_cache_path_layout() {
        let coord = TileCoord { x: 64, y: 43, z: 7 };
        let path = disk_cache_path("streets-v2", &coord).unwrap();
        let s = path.to_string_lossy();
        assert!(
            s.contains("Library/Caches/GlobalThermonuclearWar/tiles"),
            "{s}"
        );
        assert!(s.contains("streets-v2/7/64/43.png"), "{s}");
        // The API key can never appear (it's not part of any input here,
        // but pin the layout so future edits don't add it)
        assert!(!s.contains("key="));
    }

    #[test]
    fn test_disk_cache_path_style_isolation() {
        // Different styles map to different directories (no clobbering)
        let coord = TileCoord { x: 1, y: 2, z: 3 };
        let a = disk_cache_path("basic-v2", &coord).unwrap();
        let b = disk_cache_path("streets-v2", &coord).unwrap();
        assert_ne!(a, b);
    }

    // ---- Disk round trip (uses the real cache root; small temp tile) ----

    #[test]
    fn test_disk_cache_write_read_round_trip() {
        let coord = TileCoord {
            x: 99999,
            y: 99999,
            z: 18,
        };
        let bytes = b"PNG-not-really-but-bytes".to_vec();
        disk_cache_write("test-style", &coord, &bytes);
        let read = disk_cache_read("test-style", &coord);
        assert_eq!(read, Some(bytes));
        // Clean up the test artifact
        if let Some(p) = disk_cache_path("test-style", &coord) {
            let _ = std::fs::remove_file(p);
        }
    }

    #[test]
    fn test_disk_cache_read_missing_is_none() {
        // A coordinate that was never written
        let coord = TileCoord {
            x: 123456,
            y: 654321,
            z: 17,
        };
        assert!(disk_cache_read("test-style-missing", &coord).is_none());
    }

    // ---- Retry window ----

    #[test]
    fn test_retry_window_constant() {
        // Pin the retry window so a future edit that sets it below the
        // tile-request round trip doesn't create a retry storm
        assert!(TILE_RETRY_SEC >= 10.0);
    }

    // ---- Prune selection ----

    #[test]
    // ---- Zoom selection for the 3D ground plane ----
    #[test]
    fn test_zoom_for_resolution_basics() {
        // Zoom-0 ground resolution at the equator is 156543 m/px
        // (40075017 m equatorial circumference / 256 px tile).
        assert_eq!(zoom_for_resolution(200_000.0, 0.0), 0); // coarser than z0
        assert_eq!(zoom_for_resolution(70_000.0, 0.0), 1);
        assert_eq!(zoom_for_resolution(1000.0, 0.0), 7);
        assert_eq!(zoom_for_resolution(500.0, 0.0), 8);
        // Fine resolution (city scale, ~10 m/px) -> z 14
        assert_eq!(zoom_for_resolution(10.0, 0.0), 14);
        // Monotone: finer targets choose higher zooms
        let z1 = zoom_for_resolution(1000.0, 45.0);
        let z2 = zoom_for_resolution(500.0, 45.0);
        assert!(z2 >= z1, "finer resolution must not lower the zoom");
    }

    #[test]
    fn test_zoom_for_resolution_latitude_and_clamps() {
        // At latitude, ground resolution is finer by cos(lat): a given
        // target resolution needs a LOWER zoom than at the equator
        let z_eq = zoom_for_resolution(200.0, 0.0);
        let z_pole = zoom_for_resolution(200.0, 80.0);
        assert!(z_pole <= z_eq);
        // Clamped to the valid range
        assert_eq!(zoom_for_resolution(f64::MAX, 0.0), 0);
        // The 1 m/px floor inside the function bounds the finest request
        // (log2(156543) = 17.25 -> 17; finer targets cannot exceed it)
        assert_eq!(zoom_for_resolution(0.001, 0.0), 17);
        assert!(zoom_for_resolution(-5.0, 0.0) <= 18); // degenerate input stays finite
    }

    fn test_prune_targets_75_percent() {
        // Pin the budget: 512 MB cap, prune to 75%
        assert_eq!(DISK_CACHE_MAX_BYTES, 512 * 1024 * 1024);
        assert_eq!(DISK_CACHE_MAX_BYTES / 4 * 3, 384 * 1024 * 1024);
        // Write-trigger cadence is below the ~2600-write capacity of the cap
        assert!(DISK_WRITES_PER_PRUNE < 2600);
    }
}
