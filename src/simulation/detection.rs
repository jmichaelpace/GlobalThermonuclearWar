use crate::map::GeoCoord;
use crate::simulation::{
    haversine_distance, Affiliation, DefenseUnit, EntityId, Missile, MissileStatus, RadarStation,
    Satellite, SensorType,
};

/// Detection event - a sensor detecting a threat
#[derive(Clone, Debug)]
pub struct Detection {
    pub sensor_id: EntityId,
    pub sensor_type: SensorKind,
    pub target_id: EntityId,
    pub detection_quality: f64, // 0.0 to 1.0
    pub bearing_deg: f64,
    pub range_km: f64,
    pub altitude_km: f64,
}

/// Kind of sensor that made the detection
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SensorKind {
    GroundRadar,
    ShipRadar,
    SatelliteIR,
    SatelliteRadar,
    DefenseUnitRadar,
}

/// Tracking state for a defense unit
#[derive(Clone, Debug)]
pub struct TrackingState {
    pub tracker_id: EntityId,
    pub target_id: EntityId,
    pub track_quality: f64,    // 0.0 to 1.0, degrades without updates
    pub time_since_update: f64,
    pub predicted_position: GeoCoord,
    pub predicted_altitude: f64,
}

/// Detection system that manages all sensor-target relationships
pub struct DetectionSystem {
    pub active_detections: Vec<Detection>,
    pub active_tracks: Vec<TrackingState>,
}

impl Default for DetectionSystem {
    fn default() -> Self {
        Self::new()
    }
}

impl DetectionSystem {
    pub fn new() -> Self {
        Self {
            active_detections: Vec::new(),
            active_tracks: Vec::new(),
        }
    }

    /// Update all detections based on current entity positions
    pub fn update(
        &mut self,
        missiles: &[Missile],
        defense_units: &[DefenseUnit],
        radar_stations: &[RadarStation],
        satellites: &[Satellite],
        dt: f64,
    ) {
        self.active_detections.clear();

        // Check each missile against each sensor
        for missile in missiles {
            // Only detect active missiles
            if !matches!(
                missile.status,
                MissileStatus::Boost | MissileStatus::Midcourse | MissileStatus::Terminal
            ) {
                continue;
            }

            // Check defense unit sensors
            for unit in defense_units {
                if unit.affiliation == missile.affiliation {
                    continue; // Don't detect friendly missiles
                }

                if let Some(detection) = Self::check_defense_unit_detection(unit, missile) {
                    self.active_detections.push(detection);
                }
            }

            // Check radar stations
            for station in radar_stations {
                if station.affiliation == missile.affiliation {
                    continue;
                }

                if let Some(detection) = Self::check_radar_station_detection(station, missile) {
                    self.active_detections.push(detection);
                }
            }

            // Check satellites
            for satellite in satellites {
                if satellite.affiliation == missile.affiliation {
                    continue;
                }

                if let Some(detection) = Self::check_satellite_detection(satellite, missile) {
                    self.active_detections.push(detection);
                }
            }
        }

        // Update tracking states
        self.update_tracks(dt);
    }

    /// Check if a defense unit can detect a missile
    fn check_defense_unit_detection(unit: &DefenseUnit, missile: &Missile) -> Option<Detection> {
        let range_km = haversine_distance(unit.position, missile.position);
        let detection_range = unit.detection_range_km();

        if range_km > detection_range {
            return None;
        }

        // Calculate bearing
        let bearing = calculate_bearing(unit.position, missile.position);

        // Check altitude constraints (simplified - ground radars have horizon limits)
        let horizon_angle = calculate_horizon_angle(range_km, missile.altitude_km);
        if horizon_angle < 2.0 {
            // Below radar horizon (minimum elevation ~2 degrees)
            return None;
        }

        // Detection quality based on range (closer = better)
        let quality = 1.0 - (range_km / detection_range).powi(2);

        Some(Detection {
            sensor_id: unit.id,
            sensor_type: SensorKind::DefenseUnitRadar,
            target_id: missile.id,
            detection_quality: quality.max(0.1),
            bearing_deg: bearing,
            range_km,
            altitude_km: missile.altitude_km,
        })
    }

    /// Check if a radar station can detect a missile
    fn check_radar_station_detection(station: &RadarStation, missile: &Missile) -> Option<Detection> {
        let range_km = haversine_distance(station.position, missile.position);

        if range_km > station.detection_range_km {
            return None;
        }

        // Calculate bearing
        let bearing = calculate_bearing(station.position, missile.position);

        // Check azimuth coverage (if not 360 degrees)
        if station.azimuth_coverage_deg < 360.0 {
            // For now, assume station faces north (0 degrees)
            // A real implementation would have a station.facing_deg field
            let half_coverage = station.azimuth_coverage_deg / 2.0;
            if bearing > half_coverage && bearing < (360.0 - half_coverage) {
                return None;
            }
        }

        // Check elevation constraints
        let elevation = calculate_elevation_angle(range_km, missile.altitude_km);
        if elevation < station.elevation_min_deg || elevation > station.elevation_max_deg {
            return None;
        }

        // Check horizon
        let horizon_angle = calculate_horizon_angle(range_km, missile.altitude_km);
        if horizon_angle < station.elevation_min_deg {
            return None;
        }

        let quality = 1.0 - (range_km / station.detection_range_km).powi(2);

        Some(Detection {
            sensor_id: station.id,
            sensor_type: SensorKind::GroundRadar,
            target_id: missile.id,
            detection_quality: quality.max(0.1),
            bearing_deg: bearing,
            range_km,
            altitude_km: missile.altitude_km,
        })
    }

    /// Check if a satellite can detect a missile
    fn check_satellite_detection(satellite: &Satellite, missile: &Missile) -> Option<Detection> {
        let ground_range = haversine_distance(satellite.position, missile.position);
        let coverage_radius = satellite.coverage_radius_km();

        if ground_range > coverage_radius {
            return None;
        }

        // IR satellites are particularly good at detecting boost phase
        let phase_bonus = match (satellite.sensor_type, missile.status) {
            (SensorType::Infrared | SensorType::Both, MissileStatus::Boost) => 0.3,
            _ => 0.0,
        };

        let quality = (1.0 - (ground_range / coverage_radius).powi(2) + phase_bonus).min(1.0);

        let bearing = calculate_bearing(satellite.position, missile.position);

        Some(Detection {
            sensor_id: satellite.id,
            sensor_type: match satellite.sensor_type {
                SensorType::Infrared => SensorKind::SatelliteIR,
                SensorType::Radar => SensorKind::SatelliteRadar,
                SensorType::Both => SensorKind::SatelliteIR, // Primary is IR
            },
            target_id: missile.id,
            detection_quality: quality.max(0.1),
            bearing_deg: bearing,
            range_km: ground_range,
            altitude_km: missile.altitude_km,
        })
    }

    /// Update tracking states - degrade quality over time
    fn update_tracks(&mut self, dt: f64) {
        // Update existing tracks
        for track in &mut self.active_tracks {
            track.time_since_update += dt;
            // Degrade quality over time (lose track after ~10 seconds without update)
            track.track_quality -= dt * 0.1;
        }

        // Remove dead tracks
        self.active_tracks.retain(|t| t.track_quality > 0.0);

        // Promote detections to tracks or update existing tracks
        for detection in &self.active_detections {
            if let Some(track) = self
                .active_tracks
                .iter_mut()
                .find(|t| t.tracker_id == detection.sensor_id && t.target_id == detection.target_id)
            {
                // Update existing track
                track.track_quality = (track.track_quality + detection.detection_quality * 0.5).min(1.0);
                track.time_since_update = 0.0;
            } else if detection.detection_quality > 0.3 {
                // Create new track for good detections
                self.active_tracks.push(TrackingState {
                    tracker_id: detection.sensor_id,
                    target_id: detection.target_id,
                    track_quality: detection.detection_quality,
                    time_since_update: 0.0,
                    predicted_position: GeoCoord::default(), // Would be calculated
                    predicted_altitude: detection.altitude_km,
                });
            }
        }
    }

    /// Get all detections for a specific sensor
    pub fn detections_for_sensor(&self, sensor_id: EntityId) -> Vec<&Detection> {
        self.active_detections
            .iter()
            .filter(|d| d.sensor_id == sensor_id)
            .collect()
    }

    /// Get all detections of a specific target
    pub fn detections_of_target(&self, target_id: EntityId) -> Vec<&Detection> {
        self.active_detections
            .iter()
            .filter(|d| d.target_id == target_id)
            .collect()
    }

    /// Check if a target is being tracked by any friendly sensor
    pub fn is_target_tracked(&self, target_id: EntityId) -> bool {
        self.active_tracks.iter().any(|t| t.target_id == target_id)
    }

    /// Get the best track quality for a target
    pub fn best_track_quality(&self, target_id: EntityId) -> f64 {
        self.active_tracks
            .iter()
            .filter(|t| t.target_id == target_id)
            .map(|t| t.track_quality)
            .fold(0.0, f64::max)
    }
}

/// Calculate bearing from one point to another (degrees, 0 = North)
fn calculate_bearing(from: GeoCoord, to: GeoCoord) -> f64 {
    let lat1 = from.lat.to_radians();
    let lat2 = to.lat.to_radians();
    let delta_lon = (to.lon - from.lon).to_radians();

    let y = delta_lon.sin() * lat2.cos();
    let x = lat1.cos() * lat2.sin() - lat1.sin() * lat2.cos() * delta_lon.cos();

    y.atan2(x).to_degrees().rem_euclid(360.0)
}

/// Calculate the elevation angle to a target at given range and altitude
fn calculate_elevation_angle(range_km: f64, altitude_km: f64) -> f64 {
    if range_km <= 0.0 {
        return 90.0;
    }
    (altitude_km / range_km).atan().to_degrees()
}

/// Calculate the angle above the horizon for a target
/// Takes Earth curvature into account
fn calculate_horizon_angle(range_km: f64, altitude_km: f64) -> f64 {
    const EARTH_RADIUS_KM: f64 = 6371.0;

    if range_km <= 0.0 {
        return 90.0;
    }

    // Angle that the ground dips below horizontal at this range
    let earth_dip = (range_km / EARTH_RADIUS_KM).asin().to_degrees();

    // Simple elevation angle
    let elevation = (altitude_km / range_km).atan().to_degrees();

    // The apparent elevation above horizon
    elevation + earth_dip
}

/// Calculate line-of-sight distance considering Earth curvature
pub fn line_of_sight_range(observer_altitude_km: f64, target_altitude_km: f64) -> f64 {
    const EARTH_RADIUS_KM: f64 = 6371.0;

    // Distance to horizon for observer
    let d1 = (2.0 * EARTH_RADIUS_KM * observer_altitude_km + observer_altitude_km.powi(2)).sqrt();

    // Distance to horizon for target
    let d2 = (2.0 * EARTH_RADIUS_KM * target_altitude_km + target_altitude_km.powi(2)).sqrt();

    d1 + d2
}
