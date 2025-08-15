use std::{collections::HashMap, marker::PhantomData, str::FromStr};

use bevy::{
    ecs::{
        intern::Interned,
        schedule::{ScheduleConfigs, ScheduleLabel},
        system::{ScheduleSystem, SystemParamItem},
    },
    prelude::*,
};
use bevy_rapier3d::{
    plugin::{NoUserData, PhysicsSet},
    prelude::{BevyPhysicsHooks, RapierContextJoints, RapierRigidBodySet},
};
use nalgebra::UnitQuaternion;
use rapier3d::prelude::{Collider, MultibodyJointHandle, RigidBodyHandle};
use rapier3d_urdf::UrdfRobotHandles;
use urdf_rs::{Geometry, Link, Pose};

use crate::{
    drones::{
        handle_control_thrusts, render_drone_rotors, simulate_drone, switch_drone_physics,
        ControlThrusts,
    },
    events::{
        handle_control_motors, handle_despawn_robot, handle_load_robot, handle_spawn_robot,
        handle_wait_robot_loaded, ControlMotors, DespawnRobot, LoadRobot, RobotLoaded,
        RobotSpawned, SensorsRead, SpawnRobot, UAVStateUpdate, WaitRobotLoaded,
    },
    urdf_asset_loader::{self, UrdfAsset},
};
pub struct UrdfPlugin<PhysicsHooks = ()> {
    default_system_setup: bool,
    schedule: Interned<dyn ScheduleLabel>,
    _phantom: PhantomData<PhysicsHooks>,
}

impl<PhysicsHooks> UrdfPlugin<PhysicsHooks>
where
    PhysicsHooks: 'static + BevyPhysicsHooks,
    for<'w, 's> SystemParamItem<'w, 's, PhysicsHooks>: BevyPhysicsHooks,
{
    pub fn with_default_system_setup(mut self, default_system_setup: bool) -> Self {
        self.default_system_setup = default_system_setup;
        self
    }

    /// Provided for use when staging systems outside of this plugin using
    /// [`with_default_system_setup(false)`](Self::with_default_system_setup).
    pub fn get_systems(set: PhysicsSet) -> ScheduleConfigs<ScheduleSystem> {
        match set {
            PhysicsSet::StepSimulation => ((switch_drone_physics, simulate_drone).chain())
                .in_set(PhysicsSet::StepSimulation)
                .into_configs(),
            PhysicsSet::SyncBackend => (
                handle_control_motors,
                handle_control_thrusts,
                handle_despawn_robot,
                handle_load_robot,
                handle_spawn_robot,
                handle_wait_robot_loaded,
                read_sensors,
            )
                .into_configs(),
            PhysicsSet::Writeback => ((
                sync_robot_geometry,
                render_drone_rotors,
                // adjust_urdf_robot_mean_position,
            )
                .chain())
            .in_set(PhysicsSet::Writeback)
            .into_configs(),
        }
    }
}

impl<PhysicsHooksSystemParam> Default for UrdfPlugin<PhysicsHooksSystemParam> {
    fn default() -> Self {
        Self {
            schedule: PostUpdate.intern(),
            default_system_setup: true,
            _phantom: PhantomData,
        }
    }
}

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
            .add_event::<UAVStateUpdate>()
            .add_event::<SpawnRobot>()
            .add_event::<WaitRobotLoaded>()
            .init_asset::<urdf_asset_loader::UrdfAsset>()
            .init_resource::<SyncGeometryCounter>();

        if self.default_system_setup {
            app.add_systems(
                self.schedule,
                (
                    UrdfPlugin::<NoUserData>::get_systems(PhysicsSet::SyncBackend)
                        .in_set(PhysicsSet::SyncBackend),
                    UrdfPlugin::<NoUserData>::get_systems(PhysicsSet::StepSimulation)
                        .in_set(PhysicsSet::StepSimulation),
                    UrdfPlugin::<NoUserData>::get_systems(PhysicsSet::Writeback)
                        .in_set(PhysicsSet::Writeback),
                ),
            );
        }
    }
}

pub(crate) fn rapier_to_bevy_rotation() -> Quat {
    Quat::IDENTITY
}

// Components

#[derive(Component)]
pub struct URDFRobot {
    pub handle: Handle<UrdfAsset>,
    pub rapier_handles: UrdfRobotHandles<Option<MultibodyJointHandle>>,
    pub robot_type: RobotType,
}

#[derive(Clone, PartialEq, PartialOrd, Copy, Debug, Reflect)]
pub enum RobotType {
    Drone,
    NotDrone,
}

impl FromStr for RobotType {
    type Err = String;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        match s.to_lowercase().as_str() {
            "drone" => Ok(RobotType::Drone),
            "notdrone" | "not_drone" | "not-drone" => Ok(RobotType::NotDrone),
            _ => Err(format!("Invalid robot type: '{s}'")),
        }
    }
}

impl From<String> for RobotType {
    fn from(s: String) -> Self {
        s.parse()
            .unwrap_or_else(|_| panic!("Invalid robot type: '{s}'"))
    }
}

impl From<&str> for RobotType {
    fn from(s: &str) -> Self {
        s.parse()
            .unwrap_or_else(|_| panic!("Invalid robot type: '{s}'"))
    }
}

#[derive(Component)]
pub struct URDFRobotRigidBodyHandle {
    pub rigid_body_handle: RigidBodyHandle,
    pub visual_pose: Pose,
}

#[derive(Resource, Default)]
pub struct SyncGeometryCounter {
    pub count: usize,
}

// Plugin

pub struct ExtractedGeometry {
    pub index: usize,
    pub geometries: Vec<Geometry>,
    pub inertia_pose: Pose,
    pub visual_poses: Vec<Pose>,
    pub colliders: Vec<Collider>,
    pub link: Link,
}

#[allow(clippy::type_complexity)]
pub fn extract_robot_geometry(robot: &UrdfAsset) -> Vec<ExtractedGeometry> {
    robot
        .robot
        .links
        .iter()
        .enumerate()
        .map(|(index, link)| {
            let colliders = robot.urdf_robot.links[index].colliders.clone();
            let geometries = link
                .visual
                .iter()
                .map(|visual| visual.geometry.clone())
                .collect();
            let visual_poses = link
                .visual
                .iter()
                .map(|visual| visual.origin.clone())
                .collect();

            ExtractedGeometry {
                index,
                geometries,
                inertia_pose: link.inertial.origin.clone(),
                visual_poses,
                link: link.clone(),
                colliders,
            }
        })
        .collect()
}

fn sync_robot_geometry(
    mut q_rapier_robot_bodies: Query<(Entity, &mut Transform, &mut URDFRobotRigidBodyHandle)>,
    q_rapier_rigid_body_set: Query<(&RapierRigidBodySet,)>,
    mut counter: ResMut<SyncGeometryCounter>,
) {
    // return;

    for rapier_rigid_body_set in q_rapier_rigid_body_set.iter() {
        for (_, mut transform, body_handle) in q_rapier_robot_bodies.iter_mut() {
            if let Some(robot_body) = rapier_rigid_body_set
                .0
                .bodies
                .get(body_handle.rigid_body_handle)
            {
                // Only run for exactly 10 iterations
                if counter.count >= 20 {
                    return;
                }
                counter.count += 1;
                let visual_pose = body_handle.visual_pose.clone();

                // Get body transform from Rapier
                let nalgebra_body_translation = robot_body.position().translation.vector;
                let mut bevy_body_translation = Vec3::new(
                    nalgebra_body_translation.x,
                    nalgebra_body_translation.y,
                    nalgebra_body_translation.z,
                );
                let nalgebra_body_rotation = robot_body.position().rotation;

                // Convert body rotation to Bevy quaternion (correct component order)
                let bevy_body_rotation = Quat::from_xyzw(
                    nalgebra_body_rotation.i,
                    nalgebra_body_rotation.j,
                    nalgebra_body_rotation.k,
                    nalgebra_body_rotation.w,
                );

                // Get visual pose offset
                let bevy_pose_translation = Vec3::new(
                    visual_pose.xyz.0[0] as f32,
                    visual_pose.xyz.0[1] as f32,
                    visual_pose.xyz.0[2] as f32,
                );
                let nalgebra_pose_rotation = UnitQuaternion::from_euler_angles(
                    visual_pose.rpy.0[0] as f32,
                    visual_pose.rpy.0[1] as f32,
                    visual_pose.rpy.0[2] as f32,
                );
                let bevy_pose_rotation = Quat::from_xyzw(
                    nalgebra_pose_rotation.i,
                    nalgebra_pose_rotation.j,
                    nalgebra_pose_rotation.k,
                    nalgebra_pose_rotation.w,
                );

                // Apply pose offset to body transform
                bevy_body_translation += bevy_body_rotation * bevy_pose_translation;
                let final_rotation = bevy_body_rotation * bevy_pose_rotation;

                // Convert to Bevy coordinate system
                let bevy_translation = rapier_to_bevy_rotation().mul_vec3(bevy_body_translation);
                let bevy_rotation = rapier_to_bevy_rotation() * final_rotation;

                *transform =
                    Transform::from_translation(bevy_translation).with_rotation(bevy_rotation);
            }
        }
    }
}

/// move parent entity of robot to the center of robot's parts, and adjust robot's parts positions accordingly
fn adjust_urdf_robot_mean_position(
    mut q_rapier_robot_bodies: Query<(Entity, &URDFRobotRigidBodyHandle, &mut Transform, &ChildOf)>,
    mut q_urdf_robots: Query<
        (Entity, &mut Transform, &URDFRobot),
        Without<URDFRobotRigidBodyHandle>,
    >,
) {
    let mut robot_parts: HashMap<Handle<UrdfAsset>, Vec<Transform>> = HashMap::new();

    // Collect all transforms for each robot
    for (_, _, transform, child_of) in q_rapier_robot_bodies.iter() {
        let urdf_robot_result = q_urdf_robots
            .get(child_of.parent())
            .map(|(_, _, urdf)| urdf.handle.clone());
        if let Ok(handle) = urdf_robot_result {
            robot_parts.entry(handle).or_default().push(*transform);
        }
    }

    let quat_fix = Quat::from_rotation_z(std::f32::consts::PI);
    let mut mean_translations: HashMap<Handle<UrdfAsset>, Vec3> = HashMap::new();

    // Calculate mean translation for each URDF asset
    for (urdf_handle, transforms) in robot_parts.iter() {
        if transforms.is_empty() {
            continue;
        }

        let mut mean_translation = Vec3::ZERO;
        for transform in transforms {
            mean_translation += transform.translation;
        }
        mean_translation /= transforms.len() as f32;

        // Apply coordinate system fix to the mean translation
        mean_translation = quat_fix.mul_vec3(mean_translation);
        mean_translations.insert(urdf_handle.clone(), mean_translation);
    }

    // Set urdf_robots translation to mean translation
    for (_, mut transform, urdf_robot) in q_urdf_robots.iter_mut() {
        if let Some(mean_translation) = mean_translations.get(&urdf_robot.handle) {
            transform.translation = *mean_translation;
        }
    }

    // Adjust child body transforms relative to the new parent position
    for (_, _, mut transform, child_of) in q_rapier_robot_bodies.iter_mut() {
        if let Ok((_, _, urdf_robot)) = q_urdf_robots.get(child_of.parent()) {
            if let Some(mean_translation) = mean_translations.get(&urdf_robot.handle) {
                // Convert mean translation back to world space for subtraction
                let world_mean_translation = quat_fix.mul_vec3(*mean_translation);
                transform.translation -= world_mean_translation;
            }
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
                    .or_default()
                    .push(*transform);
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
                                    .or_default()
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
