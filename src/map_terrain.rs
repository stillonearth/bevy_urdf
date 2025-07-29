use bevy::prelude::Plane3d;
use bevy::prelude::*;
use bevy::render::render_asset::RenderAssetUsages;
use crossbeam_channel::{Receiver, Sender};
use ehttp::Request;
use image;
use std::{
    collections::{HashMap, HashSet},
    path::Path,
};

#[derive(Resource, Clone)]
pub struct MapConfig {
    pub reference_lat: f64,
    pub reference_lon: f64,
    /// List of `(zoom, radius)` pairs sorted from highest zoom to lowest
    pub zoom_levels: Vec<(u8, u32)>,
    pub tile_source_url: String,
    /// Optional source for per-tile heightmaps. `{z}`, `{x}` and `{y}` tokens
    /// will be replaced with tile coordinates. Can be a local file path or a
    /// remote URL.
    pub heightmap_source_url: Option<String>,
    /// Scale factor applied to heightmap values when generating terrain.
    pub height_scale: f32,
    pub cache_dir: String,
    /// distance between zoom layers along the Y axis
    pub z_layer: f32,
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
    pub spawned: HashSet<(u8, u32, u32)>,
    pub sender: Sender<TileResponse>,
    pub receiver: Receiver<TileResponse>,
}

impl Default for TileCache {
    fn default() -> Self {
        let (tx, rx) = crossbeam_channel::unbounded();
        Self {
            in_flight: HashSet::new(),
            spawned: HashSet::new(),
            sender: tx,
            receiver: rx,
        }
    }
}

pub struct TileResponse {
    pub z: u8,
    pub x: u32,
    pub y: u32,
    pub image_bytes: Option<Vec<u8>>,
    pub height_bytes: Option<Vec<u8>>,
}

#[derive(Resource, Default)]
pub struct ActiveTiles {
    pub tiles: HashMap<(u8, u32, u32), Entity>,
}

#[derive(Component)]
struct MapTile {
    z: u8,
    x: u32,
    y: u32,
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
            .init_resource::<ActiveTiles>()
            .add_event::<TileRequest>()
            .add_systems(
                Update,
                (download_tiles, process_downloads, update_tiles).chain(),
            );
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
    let lat = 2.0 * f64::atan(f64::exp(y / EARTH_RADIUS_M)) - std::f64::consts::FRAC_PI_2;
    (lat.to_degrees(), lon.to_degrees())
}

/// Convert geographic coordinates to offsets in meters relative to the
/// configured reference point.
pub fn geo_to_world(lat: f64, lon: f64, config: &MapConfig) -> Vec2 {
    let reference = geo_to_mercator(config.reference_lat, config.reference_lon);
    let point = geo_to_mercator(lat, lon);
    point - reference
}

fn world_to_geo(x: f32, y: f32, config: &MapConfig) -> (f64, f64) {
    let reference = geo_to_mercator(config.reference_lat, config.reference_lon);
    let point = Vec2::new(x, y) + reference;
    mercator_to_geo(point.x as f64, point.y as f64)
}

fn lon_to_tile_x(z: u8, lon: f64) -> u32 {
    let n = 2f64.powi(z as i32);
    ((lon + 180.0) / 360.0 * n).floor() as u32
}

fn lat_to_tile_y(z: u8, lat: f64) -> u32 {
    let n = 2f64.powi(z as i32);
    let lat_rad = lat.to_radians();
    ((1.0 - (lat_rad.tan() + 1.0 / lat_rad.cos()).ln() / std::f64::consts::PI) / 2.0 * n).floor()
        as u32
}

fn world_to_tile(z: u8, world: Vec2, config: &MapConfig) -> (u32, u32) {
    let (lat, lon) = world_to_geo(world.x, world.y, config);
    (lon_to_tile_x(z, lon), lat_to_tile_y(z, lat))
}

fn tile_path(z: u8, x: u32, y: u32) -> String {
    format!("assets/tiles/{z}/{x}/{y}.png")
}

fn heightmap_path(z: u8, x: u32, y: u32) -> String {
    format!("assets/heightmaps/{z}/{x}/{y}.png")
}

fn tile_to_lon(z: u8, x: u32) -> f64 {
    let n = 2f64.powi(z as i32);
    x as f64 / n * 360.0 - 180.0
}

fn tile_to_lat(z: u8, y: u32) -> f64 {
    let n = 2f64.powi(z as i32);
    let lat_rad = f64::atan(f64::sinh(std::f64::consts::PI * (1.0 - 2.0 * y as f64 / n)));
    lat_rad.to_degrees()
}

fn heightmap_to_mesh(
    img: &image::GrayImage,
    width: f32,
    height: f32,
    scale: f32,
) -> Mesh {
    use bevy::render::mesh::{Indices, PrimitiveTopology};

    let w = img.width() as usize;
    let h = img.height() as usize;
    let dx = width / (w as f32 - 1.0);
    let dz = height / (h as f32 - 1.0);

    let mut positions = Vec::with_capacity(w * h);
    let mut normals = Vec::with_capacity(w * h);
    let mut uvs = Vec::with_capacity(w * h);

    let get_height = |i: usize, j: usize| -> f32 {
        let pixel = img.get_pixel(i as u32, j as u32)[0] as f32 / 255.0;
        pixel * scale
    };

    for j in 0..h {
        for i in 0..w {
            let x = i as f32 * dx - width / 2.0;
            let z = j as f32 * dz - height / 2.0;
            let y = get_height(i, j);

            let left = if i > 0 { get_height(i - 1, j) } else { y };
            let right = if i + 1 < w { get_height(i + 1, j) } else { y };
            let down = if j > 0 { get_height(i, j - 1) } else { y };
            let up = if j + 1 < h { get_height(i, j + 1) } else { y };

            let normal = Vec3::new(left - right, 2.0, down - up).normalize();

            positions.push([x, y, z]);
            normals.push(normal.to_array());
            uvs.push([i as f32 / (w as f32 - 1.0), 1.0 - j as f32 / (h as f32 - 1.0)]);
        }
    }

    let mut indices = Vec::with_capacity((w - 1) * (h - 1) * 6);
    for j in 0..h - 1 {
        for i in 0..w - 1 {
            let a = (j * w + i) as u32;
            let b = a + 1;
            let c = a + w as u32;
            let d = c + 1;
            indices.extend_from_slice(&[a, b, c, b, d, c]);
        }
    }

    let mut mesh = Mesh::new(PrimitiveTopology::TriangleList, RenderAssetUsages::default());
    mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, positions);
    mesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL, normals);
    mesh.insert_attribute(Mesh::ATTRIBUTE_UV_0, uvs);
    mesh.insert_indices(Indices::U32(indices));
    mesh
}

fn spawn_tile(
    commands: &mut Commands,
    asset_server: &Res<AssetServer>,
    meshes: &mut ResMut<Assets<Mesh>>,
    materials: &mut ResMut<Assets<StandardMaterial>>,
    config: &MapConfig,
    z: u8,
    x: u32,
    y: u32,
) {
    let west = tile_to_lon(z, x);
    let east = tile_to_lon(z, x + 1);
    let north = tile_to_lat(z, y);
    let south = tile_to_lat(z, y + 1);

    let center_lat = (north + south) / 2.0;
    let center_lon = (west + east) / 2.0;

    let center = geo_to_world(center_lat, center_lon, config);
    let west_world = geo_to_world(center_lat, west, config);
    let east_world = geo_to_world(center_lat, east, config);
    let north_world = geo_to_world(north, center_lon, config);
    let south_world = geo_to_world(south, center_lon, config);

    let width = (east_world.x - west_world.x).abs();
    let height = (north_world.y - south_world.y).abs();
    let mut mesh = None;
    if let Some(template) = &config.heightmap_source_url {
        let path = template
            .replace("{z}", &z.to_string())
            .replace("{x}", &x.to_string())
            .replace("{y}", &y.to_string());
        #[cfg(not(target_arch = "wasm32"))]
        if Path::new(&path).exists() {
            if let Ok(img) = image::open(&path) {
                mesh = Some(heightmap_to_mesh(
                    &img.into_luma8(),
                    width,
                    height,
                    config.height_scale,
                ));
            }
        }
    }

    let mesh_handle = meshes.add(
        mesh.unwrap_or_else(|| Plane3d::default().mesh().size(width, height).into()),
    );
    let path = tile_path(z, x, y);
    let handle: Handle<Image> = asset_server.load(path.as_str());
    let material = materials.add(StandardMaterial {
        base_color_texture: Some(handle.clone()),
        unlit: true,
        cull_mode: None,
        ..default()
    });

    commands.spawn((
        Mesh3d(mesh_handle.clone()),
        MeshMaterial3d(material),
        Transform::from_xyz(center.x, config.z_layer * z as f32, center.y),
        MapTile { z, x, y },
    ));
}

fn download_tiles(
    mut commands: Commands,
    mut requests: EventReader<TileRequest>,
    mut cache: ResMut<TileCache>,
    config: Res<MapConfig>,
    asset_server: Res<AssetServer>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    for req in requests.read() {
        let key = (req.z, req.x, req.y);

        if cache.spawned.contains(&key) {
            continue;
        }

        #[cfg(not(target_arch = "wasm32"))]
        if Path::new(&tile_path(req.z, req.x, req.y)).exists() {
            spawn_tile(
                &mut commands,
                &asset_server,
                &mut meshes,
                &mut materials,
                &config,
                req.z,
                req.x,
                req.y,
            );
            cache.spawned.insert(key);
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

        let hm_url = config
            .heightmap_source_url
            .as_ref()
            .map(|u| {
                u.replace("{z}", &req.z.to_string())
                    .replace("{x}", &req.x.to_string())
                    .replace("{y}", &req.y.to_string())
            });

        let tx = cache.sender.clone();
        let z = req.z;
        let x = req.x;
        let y = req.y;

        ehttp::fetch(Request::get(url), move |result| {
            let image_bytes = result.ok().map(|r| r.bytes);
            if let Some(hurl) = hm_url {
                let tx2 = tx.clone();
                ehttp::fetch(Request::get(hurl), move |hres| {
                    let height_bytes = hres.ok().map(|r| r.bytes);
                    let _ = tx2.send(TileResponse {
                        z,
                        x,
                        y,
                        image_bytes: image_bytes.clone(),
                        height_bytes,
                    });
                });
            } else {
                let _ = tx.send(TileResponse {
                    z,
                    x,
                    y,
                    image_bytes,
                    height_bytes: None,
                });
            }
        });
    }
}

fn process_downloads(
    mut commands: Commands,
    mut cache: ResMut<TileCache>,
    asset_server: Res<AssetServer>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    config: Res<MapConfig>,
) {
    while let Ok(res) = cache.receiver.try_recv() {
        let key = (res.z, res.x, res.y);
        cache.in_flight.remove(&key);
        if let Some(bytes) = res.image_bytes {
            #[cfg(not(target_arch = "wasm32"))]
            {
                let path = tile_path(res.z, res.x, res.y);
                if let Some(dir) = Path::new(&path).parent() {
                    if std::fs::create_dir_all(dir).is_ok() {
                        let _ = std::fs::write(&path, &bytes);
                    }
                }
            }
        }

        if let Some(bytes) = res.height_bytes {
            if let Some(template) = &config.heightmap_source_url {
                #[cfg(not(target_arch = "wasm32"))]
                {
                    let path = template
                        .replace("{z}", &res.z.to_string())
                        .replace("{x}", &res.x.to_string())
                        .replace("{y}", &res.y.to_string());
                    if let Some(dir) = Path::new(&path).parent() {
                        if std::fs::create_dir_all(dir).is_ok() {
                            let _ = std::fs::write(&path, &bytes);
                        }
                    }
                }
            }
        }

        spawn_tile(
            &mut commands,
            &asset_server,
            &mut meshes,
            &mut materials,
            &config,
            res.z,
            res.x,
            res.y,
        );
        cache.spawned.insert(key);
    }
}

fn update_tiles(
    mut commands: Commands,
    q_uuv: Query<&GlobalTransform, With<crate::uuv::UuvDescriptor>>,
    mut ew_requests: EventWriter<TileRequest>,
    mut cache: ResMut<TileCache>,
    config: Res<MapConfig>,
    mut active: ResMut<ActiveTiles>,
    q_tiles: Query<(Entity, &MapTile)>,
) {
    let Ok(transform) = q_uuv.get_single() else {
        return;
    };
    let pos = transform.translation();
    let center_world = Vec2::new(pos.x, pos.z);

    let mut needed: HashSet<(u8, u32, u32)> = HashSet::new();
    let mut all_tiles: Vec<(u8, u32, u32)> = Vec::new();
    for (z, radius) in config.zoom_levels.iter() {
        let (cx, cy) = world_to_tile(*z, center_world, &config);
        let radius = *radius as i32;
        let max_index = (1u32 << *z) as i32 - 1;
        for dx in -radius..=radius {
            for dy in -radius..=radius {
                let tx = (cx as i32 + dx).clamp(0, max_index) as u32;
                let ty = (cy as i32 + dy).clamp(0, max_index) as u32;
                needed.insert((*z, tx, ty));
                all_tiles.push((*z, tx, ty));
            }
        }
    }

    // Remove lower-zoom tiles that are covered by higher-zoom tiles
    all_tiles.sort_by_key(|&(z, _, _)| std::cmp::Reverse(z));
    for &(z_high, x_high, y_high) in &all_tiles {
        for (z_low, _) in config.zoom_levels.iter().filter(|(zl, _)| *zl < z_high) {
            let shift = z_high - *z_low;
            let parent = (*z_low, x_high >> shift, y_high >> shift);
            needed.remove(&parent);
        }
    }

    active.tiles.clear();
    for (entity, tile) in q_tiles.iter() {
        let key = (tile.z, tile.x, tile.y);
        if !needed.contains(&key) {
            commands.entity(entity).despawn_recursive();
            cache.spawned.remove(&key);
        } else {
            active.tiles.insert(key, entity);
        }
    }

    for key in needed {
        if !cache.spawned.contains(&key) && !cache.in_flight.contains(&key) {
            ew_requests.send(TileRequest {
                z: key.0,
                x: key.1,
                y: key.2,
            });
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
            zoom_levels: vec![(0, 0)],
            tile_source_url: String::new(),
            heightmap_source_url: None,
            height_scale: 1.0,
            cache_dir: String::new(),
            z_layer: 0.0,
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
