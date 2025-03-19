use bevy::prelude::*;
use rapier3d::prelude::Collider;
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

pub fn extract_robot_geometry(
    robot: &UrdfAsset,
) -> Vec<(usize, Geometry, Pose, Pose, Option<Collider>)> {
    let mut result: Vec<(usize, Geometry, Pose, Pose, Option<Collider>)> = Vec::new();
    for (i, link) in robot.robot.links.iter().enumerate() {
        let visual_len = link.visual.len();
        assert_eq!(visual_len, 1);

        let colliders = robot.urdf_robot.links[i].colliders.clone();
        assert!(colliders.len() <= 1);
        let collider = if colliders.len() == 1 {
            Some(colliders[0].clone())
        } else {
            None
        };

        let geometry = &link.visual[0].geometry;
        let geometry_origin = &link.visual[0].origin;
        let inertia_origin = link.inertial.origin.clone();

        result.push((
            i,
            geometry.clone(),
            geometry_origin.clone(),
            inertia_origin.clone(),
            collider,
        ));
    }

    result
}
