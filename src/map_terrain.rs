use bevy::prelude::*;

#[derive(Resource, Clone)]
pub struct MapConfig {
    pub reference_lat: f64,
    pub reference_lon: f64,
    pub min_zoom: u8,
    pub max_zoom: u8,
    pub tile_source_url: String,
    pub cache_dir: String,
    pub tile_radius: u32,
}

pub struct MapTerrainPlugin {
    pub config: MapConfig,
}

impl MapTerrainPlugin {
    pub fn new(config: MapConfig) -> Self {
        Self { config }
    }
}

impl Plugin for MapTerrainPlugin {
    fn build(&self, app: &mut App) {
        app.insert_resource(self.config.clone());
    }
}

// -----------------------------------------------------------------------------
// Web Mercator helper functions
// -----------------------------------------------------------------------------
const EARTH_RADIUS_M: f64 = 6_378_137.0;

/// Convert geographic coordinates to Web Mercator meters.
fn geo_to_mercator(lat: f64, lon: f64) -> Vec2 {
    let x = EARTH_RADIUS_M * lon.to_radians();
    let y = EARTH_RADIUS_M * (f64::tan(std::f64::consts::FRAC_PI_4 + lat.to_radians() / 2.0)).ln();
    Vec2::new(x as f32, y as f32)
}

/// Convert Web Mercator meters back to geographic coordinates.
#[allow(dead_code)]
fn mercator_to_geo(x: f64, y: f64) -> (f64, f64) {
    let lon = x / EARTH_RADIUS_M;
    let lat = (2.0 * f64::atan(f64::exp(y / EARTH_RADIUS_M)) - std::f64::consts::FRAC_PI_2);
    (lat.to_degrees(), lon.to_degrees())
}

/// Convert geographic coordinates to offsets in meters relative to the
/// configured reference point.
pub fn geo_to_world(lat: f64, lon: f64, config: &MapConfig) -> Vec2 {
    let reference = geo_to_mercator(config.reference_lat, config.reference_lon);
    let point = geo_to_mercator(lat, lon);
    point - reference
}

#[cfg(test)]
mod tests {
    use super::*;

    fn test_config() -> MapConfig {
        MapConfig {
            reference_lat: 45.0,
            reference_lon: 10.0,
            min_zoom: 0,
            max_zoom: 0,
            tile_source_url: String::new(),
            cache_dir: String::new(),
            tile_radius: 0,
        }
    }

    #[test]
    fn zero_offset() {
        let cfg = test_config();
        let delta = geo_to_world(45.0, 10.0, &cfg);
        assert!(delta.length() < 1e-6);
    }

    #[test]
    fn latitude_offset() {
        let cfg = test_config();
        let delta = geo_to_world(45.001, 10.0, &cfg);
        assert!((delta.y as f64 - 157.43).abs() < 0.5);
    }

    #[test]
    fn longitude_offset() {
        let cfg = test_config();
        let delta = geo_to_world(45.0, 10.001, &cfg);
        assert!((delta.x as f64 - 111.32).abs() < 0.5);
    }
}

