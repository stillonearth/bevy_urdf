use std::path::Path;

use bevy::{
    asset::{io::Reader, AssetLoader, LoadContext},
    prelude::*,
};
use rapier3d::prelude::*;
use rapier3d_urdf::{UrdfLoaderOptions, UrdfRobot};
use serde::{Deserialize, Serialize};
use thiserror::Error;
use urdf_rs::Robot;

#[derive(Default)]
pub struct RpyAssetLoader;

#[derive(Asset, TypePath, Debug)]
pub struct UrdfAsset {
    pub robot: Robot,
    pub urdf_robot: UrdfRobot,
}

#[derive(Default, Debug, Clone, Deserialize, Serialize)]
pub struct RpyAssetLoaderSettings {
    pub mesh_dir: Option<String>,
}

#[non_exhaustive]
#[derive(Debug, Error)]
pub enum UrdfAssetLoaderError {
    #[error("Could not load file: {0}")]
    Io(#[from] std::io::Error),
}

impl AssetLoader for RpyAssetLoader {
    type Asset = UrdfAsset;
    type Settings = RpyAssetLoaderSettings;
    type Error = UrdfAssetLoaderError;

    async fn load(
        &self,
        reader: &mut dyn Reader,
        settings: &RpyAssetLoaderSettings,
        _load_context: &mut LoadContext<'_>,
    ) -> Result<Self::Asset, Self::Error> {
        let mut bytes = Vec::new();
        reader.read_to_end(&mut bytes).await?;
        let content = std::str::from_utf8(&bytes).unwrap();

        let options = UrdfLoaderOptions {
            create_colliders_from_visual_shapes: true,
            create_colliders_from_collision_shapes: false,
            enable_joint_collisions: true,
            apply_imported_mass_props: true,
            make_roots_fixed: true,
            // Z-up to Y-up.
            shift: Isometry::rotation(-Vector::x() * std::f32::consts::FRAC_PI_2),
            ..Default::default()
        };

        let mesh_dir = settings.clone().mesh_dir.unwrap_or("./".to_string());
        let mesh_dir = Path::new(&mesh_dir);

        let (urdf_robot, robot) = UrdfRobot::from_str(content, options, mesh_dir).unwrap();

        Ok(UrdfAsset { robot, urdf_robot })
    }
}
