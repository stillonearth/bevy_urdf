use bevy::{prelude::*, utils::hashbrown::HashMap};
use bevy_rapier3d::prelude::{RapierContextJoints, RapierRigidBodySet};
use rapier3d::prelude::Collider;
use urdf_rs::{Geometry, Pose};

use crate::{
    events::{
        handle_control_motors, handle_load_robot, handle_spawn_robot, handle_wait_robot_loaded,
        ControlMotors, LoadRobot, RobotLoaded, RobotSpawned, SensorsRead, SpawnRobot, URDFRobot,
        UrdfRobotRigidBodyHandle, WaitRobotLoaded,
    },
    urdf_asset_loader::{self, UrdfAsset},
};
pub struct UrdfPlugin;

impl Plugin for UrdfPlugin {
    fn build(&self, app: &mut App) {
        app.init_asset_loader::<urdf_asset_loader::RpyAssetLoader>()
            .add_event::<SpawnRobot>()
            .add_event::<RobotSpawned>()
            .add_event::<WaitRobotLoaded>()
            .add_event::<LoadRobot>()
            .add_event::<RobotLoaded>()
            .add_event::<SensorsRead>()
            .add_event::<ControlMotors>()
            .add_systems(Update, sync_robot_geometry)
            .add_systems(
                Update,
                (
                    handle_spawn_robot,
                    handle_load_robot,
                    handle_wait_robot_loaded,
                    read_sensors,
                    handle_control_motors,
                ),
            )
            .init_asset::<urdf_asset_loader::UrdfAsset>();
    }
}

pub fn extract_robot_geometry(
    robot: &UrdfAsset,
) -> Vec<(usize, Option<Geometry>, Pose, Option<Collider>)> {
    let mut result: Vec<(usize, Option<Geometry>, Pose, Option<Collider>)> = Vec::new();
    for (i, link) in robot.robot.links.iter().enumerate() {
        let colliders = robot.urdf_robot.links[i].colliders.clone();
        let collider = if colliders.len() == 1 {
            Some(colliders[0].clone())
        } else {
            None
        };

        let geometry = if !link.visual.is_empty() {
            Some(link.visual[0].geometry.clone())
        } else {
            None
        };
        let inertia_origin = link.inertial.origin.clone();

        result.push((i, geometry.clone(), inertia_origin.clone(), collider));
    }

    result
}

fn sync_robot_geometry(
    mut q_rapier_robot_bodies: Query<(Entity, &mut Transform, &mut UrdfRobotRigidBodyHandle)>,
    q_rapier_rigid_body_set: Query<(&RapierRigidBodySet,)>,
) {
    for rapier_rigid_body_set in q_rapier_rigid_body_set.iter() {
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

fn read_sensors(
    q_urdf_robots: Query<(Entity, &URDFRobot)>,
    q_urdf_rigid_bodies: Query<(Entity, &Parent, &Transform, &UrdfRobotRigidBodyHandle)>,
    mut ew_sensors_read: EventWriter<SensorsRead>,
    q_rapier_joints: Query<(&RapierContextJoints, &RapierRigidBodySet)>,
) {
    let mut readings_hashmap: HashMap<Handle<UrdfAsset>, Vec<Transform>> = HashMap::new();
    let mut joint_angles: HashMap<Handle<UrdfAsset>, Vec<f32>> = HashMap::new();

    for (parent_entity, urdf_robot) in &mut q_urdf_robots.iter() {
        for (_, parent, transform, _) in q_urdf_rigid_bodies.iter() {
            if parent_entity.index() == parent.get().index() {
                readings_hashmap
                    .entry(urdf_robot.handle.clone())
                    .or_insert_with(Vec::new)
                    .push(transform.clone());
            }
        }

        for (rapier_context_joints, rapier_rigid_bodies) in q_rapier_joints.iter() {
            for joint_link_handle in urdf_robot.rapier_handles.joints.iter() {
                if let Some(handle) = joint_link_handle.joint {
                    if let Some((multibody, index)) =
                        rapier_context_joints.multibody_joints.get(handle)
                    {
                        if let Some(link) = multibody.link(index) {
                            let joint = link.joint.data;

                            let body_1_link = joint_link_handle.link1;
                            let body_2_link = joint_link_handle.link2;

                            if let Some(revolute) = joint.as_revolute() {
                                let rb1 = rapier_rigid_bodies.bodies.get(body_1_link).unwrap();
                                let rb2 = rapier_rigid_bodies.bodies.get(body_2_link).unwrap();

                                let angle = revolute.angle(rb1.rotation(), rb2.rotation());

                                joint_angles
                                    .entry(urdf_robot.handle.clone())
                                    .or_insert_with(Vec::new)
                                    .push(angle);
                            }
                        }
                    }
                }
            }
        }
    }

    for (key, transforms) in readings_hashmap.iter() {
        ew_sensors_read.send(SensorsRead {
            transforms: transforms.clone(),
            handle: key.clone(),
            joint_angles: joint_angles.entry(key.clone()).or_default().clone(),
        });
    }
}
