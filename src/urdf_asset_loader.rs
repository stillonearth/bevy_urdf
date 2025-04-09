use std::path::Path;

use bevy::{
    asset::{io::Reader, AssetLoader, LoadContext},
    prelude::*,
};
use rapier3d::{na::Translation3, prelude::*};
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
    pub interaction_groups: Option<InteractionGroups>,
    pub translation_shift: Option<Vec3>,
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

        let mut isometry: nalgebra::Isometry<f32, nalgebra::Unit<nalgebra::Quaternion<f32>>, 3> =
            Isometry::rotation(Vector::x() * std::f32::consts::FRAC_PI_2);
        if let Some(translaction_shift) = settings.translation_shift {
            isometry.append_translation_mut(&Translation3::new(
                translaction_shift.x,
                translaction_shift.y,
                translaction_shift.z,
            ));
        }

        let options = UrdfLoaderOptions {
            create_colliders_from_visual_shapes: true,
            create_colliders_from_collision_shapes: false,
            enable_joint_collisions: false,
            apply_imported_mass_props: true,
            make_roots_fixed: false,
            // Z-up to Y-up.
            shift: isometry,
            ..Default::default()
        };

        let mesh_dir = settings.clone().mesh_dir.unwrap_or("./".to_string());
        let mesh_dir = Path::new(&mesh_dir);

        let (mut urdf_robot, robot) = UrdfRobot::from_str(content, options, mesh_dir).unwrap();
        let mut robot_joints = urdf_robot.joints.clone();

        // hotfix robot revolute joints motors
        for (index, urdf_joint) in robot_joints.clone().iter().enumerate() {
            let mut joint = urdf_joint.joint.clone();
            if let Some(_) = joint.as_revolute() {
                joint.set_motor_velocity(JointAxis::AngX, 0.0, 1.0);
                robot_joints[index].joint = joint;
            }
        }
        urdf_robot.joints = robot_joints;

        // apply custom collision groups
        if let Some(mut adjusted_interaction_groups) = settings.interaction_groups {
            let mut robot_links = urdf_robot.links.clone();
            for (body_index, urdf_link) in robot_links.clone().iter().enumerate() {
                for (collider_index, collider) in urdf_link.colliders.clone().iter().enumerate() {
                    let mut collider = collider.clone();
                    let urdf_interactions_groups = collider.collision_groups();

                    adjusted_interaction_groups.filter = urdf_interactions_groups.filter;

                    collider.set_collision_groups(adjusted_interaction_groups);
                    robot_links[body_index].colliders[collider_index] = collider;
                }
            }
            urdf_robot.links = robot_links;
        }

        Ok(UrdfAsset { robot, urdf_robot })
    }
}
