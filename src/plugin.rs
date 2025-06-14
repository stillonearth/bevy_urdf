use std::collections::HashMap;

use bevy::prelude::*;
use bevy_rapier3d::prelude::{RapierContextJoints, RapierRigidBodySet};
use rapier3d::prelude::{Collider, MultibodyJointHandle, RigidBodyHandle};
use rapier3d_urdf::UrdfRobotHandles;
use urdf_rs::{Geometry, Pose};

use crate::{
    drones::{handle_control_thrusts, simulate_drone, ControlThrusts},
    events::{
        handle_control_motors, handle_despawn_robot, handle_load_robot, handle_spawn_robot,
        handle_wait_robot_loaded, ControlMotors, DespawnRobot, LoadRobot, RobotLoaded,
        RobotSpawned, RobotType, SensorsRead, SpawnRobot, WaitRobotLoaded,
    },
    urdf_asset_loader::{self, UrdfAsset},
};
pub struct UrdfPlugin;

// Components

#[derive(Component)]
pub struct URDFRobot {
    pub handle: Handle<UrdfAsset>,
    pub rapier_handles: UrdfRobotHandles<Option<MultibodyJointHandle>>,
    pub robot_type: RobotType,
}

// impl Component for URDFRobot {
//     const STORAGE_TYPE: StorageType = StorageType::Table;

//     fn register_component_hooks(hooks: &mut ComponentHooks) {
//         hooks.on_remove(|mut world, entity: Entity, _component| {
//             let world_entity = world.entity(entity);
//             let component = world_entity.get::<Self>().unwrap();
//             let handle = component.handle.clone();

//             world.commands().queue(move |world: &mut World| {
//                 world.send_event(DespawnRobot { handle });
//                 world.commands().entity(entity).despawn_recursive();
//             })
//         });
//     }
// }

#[derive(Component, Default, Deref)]
pub struct URDFRobotRigidBodyHandle(pub RigidBodyHandle);

#[derive(Component, Default, Deref)]
pub struct VisualPositionShift(pub Vec3);

// Plugin

impl Plugin for UrdfPlugin {
    fn build(&self, app: &mut App) {
        app.init_asset_loader::<urdf_asset_loader::RpyAssetLoader>()
            .add_event::<ControlMotors>()
            .add_event::<ControlThrusts>()
            .add_event::<DespawnRobot>()
            .add_event::<LoadRobot>()
            .add_event::<RobotLoaded>()
            .add_event::<RobotSpawned>()
            .add_event::<SensorsRead>()
            .add_event::<SpawnRobot>()
            .add_event::<WaitRobotLoaded>()
            .add_systems(
                Update,
                (
                    simulate_drone,
                    sync_robot_geometry,
                    adjust_urdf_robot_mean_position,
                )
                    .chain(),
            )
            .add_systems(
                Update,
                (
                    handle_control_motors,
                    handle_control_thrusts,
                    handle_despawn_robot,
                    handle_load_robot,
                    handle_spawn_robot,
                    handle_wait_robot_loaded,
                    read_sensors,
                )
                    .chain(),
            )
            .init_asset::<urdf_asset_loader::UrdfAsset>();
    }
}

pub fn extract_robot_geometry(
    robot: &UrdfAsset,
) -> Vec<(
    usize,
    Option<Geometry>,
    Pose,
    Option<Pose>,
    Option<Collider>,
)> {
    let mut result: Vec<(
        usize,
        Option<Geometry>,
        Pose,
        Option<Pose>,
        Option<Collider>,
    )> = Vec::new();

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
        let inertia_pose = link.inertial.origin.clone();
        let mut visual_pose: Option<Pose> = None;

        if let Some(vp) = link.visual.last() {
            visual_pose = Some(vp.origin.clone());
        }

        result.push((
            i,
            geometry.clone(),
            inertia_pose.clone(),
            visual_pose.clone(),
            collider,
        ));
    }

    result
}

fn sync_robot_geometry(
    mut q_rapier_robot_bodies: Query<(
        Entity,
        &mut Transform,
        &mut URDFRobotRigidBodyHandle,
        &VisualPositionShift,
    )>,
    q_rapier_rigid_body_set: Query<(&RapierRigidBodySet,)>,
) {
    for rapier_rigid_body_set in q_rapier_rigid_body_set.iter() {
        for (j, (_, mut transform, body_handle, shift)) in
            q_rapier_robot_bodies.iter_mut().enumerate()
        {
            if let Some(robot_body) = rapier_rigid_body_set.0.bodies.get(body_handle.0) {
                let rapier_pos = robot_body.position();
                let rapier_rot = rapier_pos.rotation;

                let quat_fix = Quat::from_rotation_z(std::f32::consts::PI)
                    * Quat::from_rotation_y(std::f32::consts::PI);
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

/// move parent entity of robot to the ceenter of robot's parts, and adjust robot's parts positions accordingly
fn adjust_urdf_robot_mean_position(
    mut q_rapier_robot_bodies: Query<(Entity, &URDFRobotRigidBodyHandle, &mut Transform, &ChildOf)>,
    mut q_urdf_robots: Query<
        (Entity, &mut Transform, &URDFRobot),
        Without<URDFRobotRigidBodyHandle>,
    >,
) {
    let mut robot_parts: HashMap<Handle<UrdfAsset>, Vec<Transform>> = HashMap::new();
    for (_, _, transform, child_od) in q_rapier_robot_bodies.iter() {
        let urdf_robot_result = q_urdf_robots
            .get(child_od.parent())
            .map(|(_, _, urdf)| urdf.handle.clone());

        if urdf_robot_result.is_ok() {
            robot_parts
                .entry(urdf_robot_result.unwrap())
                .or_default()
                .push(transform.clone());
        }
    }

    let quat_fix =
        Quat::from_rotation_z(std::f32::consts::PI) * Quat::from_rotation_y(std::f32::consts::PI);

    let mut mean_translations: HashMap<Handle<UrdfAsset>, Vec3> = HashMap::new();

    // calculate mean transfrom for each URDF asset
    for (urdf_handle, transforms) in robot_parts.iter() {
        let mut mean_translation = Vec3::ZERO;
        for transform in transforms {
            mean_translation += transform.translation;
        }

        mean_translation /= transforms.len() as f32;
        mean_translation = quat_fix.mul_vec3(mean_translation);

        mean_translations
            .entry(urdf_handle.clone())
            .or_insert(mean_translation);
    }

    // set urdf_robots translation to mean transform
    for (_, mut transform, urdf_robot) in q_urdf_robots.iter_mut() {
        if let Some(mean_translation) = mean_translations.get(&urdf_robot.handle.clone()) {
            transform.translation = mean_translation.clone();
        }
    }

    // subtract mean translation from each URDF asset
    for (_, _, mut transform, child_of) in q_rapier_robot_bodies.iter_mut() {
        let parent = q_urdf_robots.get(child_of.parent());
        if parent.is_err() {
            continue;
        }
        let handle = parent.unwrap().2.handle.clone();
        if let Some(mean_translation) = mean_translations.get(&handle) {
            let transform_fix = quat_fix.mul_vec3(mean_translation.clone());
            transform.translation -= transform_fix;
        }
    }
}

fn read_sensors(
    q_urdf_robots: Query<(Entity, &URDFRobot)>,
    q_urdf_rigid_bodies: Query<(Entity, &ChildOf, &Transform, &URDFRobotRigidBodyHandle)>,
    mut ew_sensors_read: EventWriter<SensorsRead>,
    q_rapier_joints: Query<(&RapierContextJoints, &RapierRigidBodySet)>,
) {
    let mut readings_hashmap: HashMap<Handle<UrdfAsset>, Vec<Transform>> = HashMap::new();
    let mut joint_angles: HashMap<Handle<UrdfAsset>, Vec<f32>> = HashMap::new();

    for (parent_entity, urdf_robot) in &mut q_urdf_robots.iter() {
        for (_, child_of, transform, _) in q_urdf_rigid_bodies.iter() {
            if parent_entity.index() == child_of.parent().index() {
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
                                let rb1 = rapier_rigid_bodies.bodies.get(body_1_link);
                                let rb2 = rapier_rigid_bodies.bodies.get(body_2_link);

                                if rb1.is_none() || rb2.is_none() {
                                    continue;
                                }

                                let rb1 = rb1.unwrap();
                                let rb2 = rb2.unwrap();

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
        ew_sensors_read.write(SensorsRead {
            transforms: transforms.clone(),
            handle: key.clone(),
            joint_angles: joint_angles.entry(key.clone()).or_default().clone(),
        });
    }
}
