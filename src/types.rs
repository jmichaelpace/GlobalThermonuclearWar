//! Shared types that don't depend on UI frameworks
//! These types are available to both the library and binary crates

use serde::{Deserialize, Serialize};

/// Geographic coordinate (latitude, longitude)
#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize, PartialEq)]
pub struct GeoCoord {
    pub lat: f64,
    pub lon: f64,
}

impl GeoCoord {
    pub fn new(lat: f64, lon: f64) -> Self {
        Self { lat, lon }
    }
}
