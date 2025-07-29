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
