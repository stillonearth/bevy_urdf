use bevy::prelude::*;
use urdf_rs::{Geometry, Pose};

use crate::{
    events::SpawnRobot,
    urdf_asset_loader::{self, UrdfAsset},
};
pub struct UrdfPlugin;

impl Plugin for UrdfPlugin {
    fn build(&self, app: &mut App) {
        app.init_asset_loader::<urdf_asset_loader::RpyAssetLoader>()
            .add_event::<SpawnRobot>()
            .init_asset::<urdf_asset_loader::UrdfAsset>();
    }
}

pub fn extract_robot_geometry(robot: &UrdfAsset) -> Vec<(usize, Geometry, Pose, Pose)> {
    let mut result: Vec<(usize, Geometry, Pose, Pose)> = Vec::new();
    for (i, link) in robot.robot.links.iter().enumerate() {
        let visual_len = link.visual.len();
        assert_eq!(visual_len, 1);

        let geometry = &link.visual[0].geometry;
        let geometry_origin = &link.visual[0].origin;
        let inertia_origin = link.inertial.origin.clone();

        println!("{} name: {}", i, link.name);

        result.push((
            i,
            geometry.clone(),
            geometry_origin.clone(),
            inertia_origin.clone(),
        ));
    }

    result
}
