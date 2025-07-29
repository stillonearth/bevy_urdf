use bevy::prelude::*;
use crossbeam_channel::{Receiver, Sender};
use ehttp::Request;
use std::{collections::HashSet, path::Path};

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

#[derive(Event)]
pub struct TileRequest {
    pub z: u8,
    pub x: u32,
    pub y: u32,
}

#[derive(Resource)]
pub struct TileCache {
    pub in_flight: HashSet<(u8, u32, u32)>,
    pub sender: Sender<TileResponse>,
    pub receiver: Receiver<TileResponse>,
}

impl Default for TileCache {
    fn default() -> Self {
        let (tx, rx) = crossbeam_channel::unbounded();
        Self {
            in_flight: HashSet::new(),
            sender: tx,
            receiver: rx,
        }
    }
}

pub struct TileResponse {
    pub z: u8,
    pub x: u32,
    pub y: u32,
    pub bytes: Option<Vec<u8>>,
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
        app.insert_resource(self.config.clone())
            .init_resource::<TileCache>()
            .add_event::<TileRequest>()
            .add_systems(Update, (download_tiles, process_downloads));
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

fn tile_path(z: u8, x: u32, y: u32) -> String {
    format!("assets/tiles/{z}/{x}/{y}.png")
}

fn download_tiles(
    mut requests: EventReader<TileRequest>,
    mut cache: ResMut<TileCache>,
    config: Res<MapConfig>,
) {
    for req in requests.read() {
        let key = (req.z, req.x, req.y);

        #[cfg(not(target_arch = "wasm32"))]
        if Path::new(&tile_path(req.z, req.x, req.y)).exists() {
            continue;
        }

        if cache.in_flight.contains(&key) {
            continue;
        }

        cache.in_flight.insert(key);
        let url = config
            .tile_source_url
            .replace("{z}", &req.z.to_string())
            .replace("{x}", &req.x.to_string())
            .replace("{y}", &req.y.to_string());

        let tx = cache.sender.clone();
        let z = req.z;
        let x = req.x;
        let y = req.y;

        ehttp::fetch(Request::get(url), move |result| {
            let bytes = result.ok().map(|r| r.bytes);
            let _ = tx.send(TileResponse { z, x, y, bytes });
        });
    }
}

fn process_downloads(mut cache: ResMut<TileCache>) {
    while let Ok(res) = cache.receiver.try_recv() {
        cache.in_flight.remove(&(res.z, res.x, res.y));
        if let Some(bytes) = res.bytes {
            #[cfg(not(target_arch = "wasm32"))]
            {
                let path = tile_path(res.z, res.x, res.y);
                if let Some(dir) = Path::new(&path).parent() {
                    if std::fs::create_dir_all(dir).is_ok() {
                        let _ = std::fs::write(path, &bytes);
                    }
                }
            }
        }
    }
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
