use std::path::Path;

use bevy::{
    asset::{io::Reader, AssetLoader, LoadContext},
    prelude::*,
};
use rapier3d::prelude::*;
use rapier3d_urdf::{UrdfLoaderOptions, UrdfRobot};
use thiserror::Error;
use urdf_rs::Robot;

#[derive(Default)]
pub struct RpyAssetLoader;

#[derive(Asset, TypePath, Debug)]
pub struct UrdfAsset {
    pub robot: Robot,
    pub urdf_robot: UrdfRobot,
}

#[non_exhaustive]
#[derive(Debug, Error)]
pub enum UrdfAssetLoaderError {
    #[error("Could not load file: {0}")]
    Io(#[from] std::io::Error),
}

impl AssetLoader for RpyAssetLoader {
    type Asset = UrdfAsset;
    type Settings = ();
    type Error = UrdfAssetLoaderError;

    async fn load(
        &self,
        reader: &mut dyn Reader,
        _settings: &(),
        _load_context: &mut LoadContext<'_>,
    ) -> Result<Self::Asset, Self::Error> {
        // read urdf file to memory
        let mut bytes = Vec::new();
        reader.read_to_end(&mut bytes).await?;
        // urdf file content
        let content = std::str::from_utf8(&bytes).unwrap();

        let options = UrdfLoaderOptions {
            create_colliders_from_visual_shapes: true,
            create_colliders_from_collision_shapes: true,
            make_roots_fixed: true,
            // Z-up to Y-up.
            shift: Isometry::rotation(Vector::x() * std::f32::consts::FRAC_PI_2),
            ..Default::default()
        };

        let mesh_dir = Path::new("./");
        let (urdf_robot, robot) = UrdfRobot::from_str(content, options, mesh_dir).unwrap();

        Ok(UrdfAsset { robot, urdf_robot })
    }
}
