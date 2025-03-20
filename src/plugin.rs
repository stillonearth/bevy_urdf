use bevy::prelude::*;
use bevy_rapier3d::{
    plugin::RapierConfiguration,
    prelude::{RapierContextSimulation, RapierRigidBodySet},
};
use rapier3d::prelude::Collider;
use urdf_rs::{Geometry, Pose};

use crate::{
    events::{SpawnRobot, UrdfRobotRigidBodyHandle},
    urdf_asset_loader::{self, UrdfAsset},
};
pub struct UrdfPlugin;

impl Plugin for UrdfPlugin {
    fn build(&self, app: &mut App) {
        app.init_asset_loader::<urdf_asset_loader::RpyAssetLoader>()
            .add_event::<SpawnRobot>()
            .add_systems(Update, sync_robot_geometry)
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

fn sync_robot_geometry(
    mut q_rapier_robot_bodies: Query<(Entity, &mut Transform, &mut UrdfRobotRigidBodyHandle)>,
    q_rapier_rigid_body_set: Query<(&RapierRigidBodySet,)>,
) {
    for (rapier_rigid_body_set) in q_rapier_rigid_body_set.iter() {
        for (_, mut transform, body_handle) in q_rapier_robot_bodies.iter_mut() {
            if let Some(robot_body) = rapier_rigid_body_set.0.bodies.get(body_handle.0) {
                let rapier_pos = robot_body.position();

                let rapier_rot = rapier_pos.rotation;

                let quat_fix = Quat::from_rotation_z(std::f32::consts::PI);
                let bevy_quat = quat_fix
                    * Quat::from_array([rapier_rot.i, rapier_rot.j, rapier_rot.k, rapier_rot.w]);

                let rapier_vec = Vec3::new(
                    rapier_pos.translation.x,
                    rapier_pos.translation.y,
                    rapier_pos.translation.z,
                );
                let bevy_vec = quat_fix.mul_vec3(rapier_vec);

                *transform = Transform::from_translation(bevy_vec).with_rotation(bevy_quat);
            }
        }
    }
}
