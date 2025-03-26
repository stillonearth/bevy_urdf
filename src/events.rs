use std::path::Path;

use bevy::prelude::*;
use bevy_rapier3d::prelude::*;
use rapier3d::prelude::{InteractionGroups, MultibodyJointHandle, RigidBodyHandle};
use rapier3d_urdf::{UrdfMultibodyOptions, UrdfRobotHandles};

use crate::{
    plugin::extract_robot_geometry,
    urdf_asset_loader::{RpyAssetLoaderSettings, UrdfAsset},
};

#[derive(Clone, Event)]
pub struct SpawnRobot {
    pub handle: Handle<UrdfAsset>,
    pub mesh_dir: String,
}

#[derive(Clone, Event)]
pub struct RobotSpawned {
    pub handle: Handle<UrdfAsset>,
}

#[derive(Clone, Event)]
pub struct WaitRobotLoaded {
    pub handle: Handle<UrdfAsset>,
    pub mesh_dir: String,
}

#[derive(Clone, Event)]
pub struct RobotLoaded {
    pub handle: Handle<UrdfAsset>,
    pub mesh_dir: String,
}

#[derive(Clone, Event)]
pub struct LoadRobot {
    pub urdf_path: String,
    pub mesh_dir: String,
    pub interaction_groups: Option<InteractionGroups>,
}

#[derive(Component)]
pub struct URDFRobot {
    pub handle: Handle<UrdfAsset>,
    pub rapier_handles: UrdfRobotHandles<Option<MultibodyJointHandle>>,
}

#[derive(Event)]
pub struct SensorsRead {
    pub handle: Handle<UrdfAsset>,
    pub transforms: Vec<Transform>,
    pub joint_angles: Vec<f32>,
}

#[derive(Event)]
pub struct ControlMotors {
    pub handle: Handle<UrdfAsset>,
    pub velocities: Vec<f32>,
}

#[derive(Component, Default, Deref)]
pub struct UrdfRobotRigidBodyHandle(pub RigidBodyHandle);

pub(crate) fn handle_spawn_robot(
    mut commands: Commands,
    asset_server: Res<AssetServer>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    urdf_assets: Res<Assets<UrdfAsset>>,
    mut q_rapier_context: Query<(
        Entity,
        &mut RapierRigidBodySet,
        &mut RapierContextColliders,
        &mut RapierContextJoints,
    )>,

    q_rapier_context_simulation: Query<(Entity, &RapierContextSimulation)>,
    mut er_spawn_robot: EventReader<SpawnRobot>,
    mut ew_wait_robot_loaded: EventWriter<WaitRobotLoaded>,
    mut ew_robot_spawned: EventWriter<RobotSpawned>,
) {
    for event in er_spawn_robot.read() {
        let rapier_context_simulation_entity = q_rapier_context_simulation.iter().next().unwrap().0;
        let robot_handle = event.handle.clone();
        if let Some(urdf) = urdf_assets.get(robot_handle.id()) {
            let mut maybe_rapier_handles: Option<UrdfRobotHandles<Option<MultibodyJointHandle>>> =
                None;
            // let mut handles: Option<UrdfRobotHandles<ImpulseJointHandle>> = None;
            for (_entity, mut rigid_body_set, mut collider_set, mut multibidy_joint_set) in
                q_rapier_context.iter_mut()
            {
                let urdf_robot = urdf.urdf_robot.clone();

                maybe_rapier_handles = Some(urdf_robot.clone().insert_using_multibody_joints(
                    &mut rigid_body_set.bodies,
                    &mut collider_set.colliders,
                    &mut multibidy_joint_set.multibody_joints,
                    UrdfMultibodyOptions::DISABLE_SELF_CONTACTS,
                ));
                break;
            }

            if maybe_rapier_handles.is_none() {
                panic!("couldn't initialize handles");
            }

            let rapier_handles = maybe_rapier_handles.unwrap();
            let body_handles: Vec<RigidBodyHandle> =
                rapier_handles.links.iter().map(|link| link.body).collect();
            let geoms = extract_robot_geometry(urdf);

            assert_eq!(body_handles.len(), geoms.len());

            commands
                .spawn((
                    URDFRobot {
                        handle: event.handle.clone(),
                        rapier_handles: rapier_handles,
                    },
                    Transform::IDENTITY.with_rotation(Quat::from_rotation_x(std::f32::consts::PI)),
                    Name::new("URDF Robot"),
                    InheritedVisibility::VISIBLE,
                ))
                .with_children(|children| {
                    for (index, geom, _inertia_pose, _collider) in geoms {
                        if geom.is_none() {
                            continue;
                        }
                        let mesh_3d: Mesh3d = match geom.unwrap() {
                            urdf_rs::Geometry::Box { size } => Mesh3d(meshes.add(Cuboid::new(
                                size[0] as f32 * 2.0,
                                size[2] as f32 * 2.0,
                                size[1] as f32 * 2.0,
                            ))),
                            urdf_rs::Geometry::Cylinder { radius: _, length: _ } => todo!(),
                            urdf_rs::Geometry::Capsule { radius: _, length: _ } => todo!(),
                            urdf_rs::Geometry::Sphere { radius } => {
                                Mesh3d(meshes.add(Sphere::new(radius as f32)))
                            }
                            urdf_rs::Geometry::Mesh { filename, scale: _ } => {
                                let base_path = event.mesh_dir.as_str();
                                let model_path = Path::new(base_path).join(filename);
                                let model_path = model_path.to_str().unwrap();

                                Mesh3d(asset_server.load(model_path))
                            }
                        };

                        let rapier_link = urdf.urdf_robot.links[index].clone();
                        let rapier_pos = rapier_link.body.position();
                        let rapier_rot = rapier_pos.rotation;

                        let quat_fix = Quat::from_rotation_z(std::f32::consts::PI);
                        let bevy_quat = quat_fix
                            * Quat::from_array([
                                rapier_rot.i,
                                rapier_rot.j,
                                rapier_rot.k,
                                rapier_rot.w,
                            ]);

                        let rapier_vec = Vec3::new(
                            rapier_pos.translation.x,
                            rapier_pos.translation.y,
                            rapier_pos.translation.z,
                        );
                        let bevy_vec = quat_fix.mul_vec3(rapier_vec);

                        let transform =
                            Transform::from_translation(bevy_vec).with_rotation(bevy_quat);

                        children.spawn((
                            mesh_3d,
                            MeshMaterial3d(materials.add(Color::srgb(0.3, 0.4, 0.3))),
                            UrdfRobotRigidBodyHandle(body_handles[index]),
                            RapierContextEntityLink(rapier_context_simulation_entity),
                            transform,
                        ));
                    }
                });

            ew_robot_spawned.send(RobotSpawned {
                handle: event.handle.clone(),
            });
        } else {
            ew_wait_robot_loaded.send(WaitRobotLoaded {
                handle: event.handle.clone(),
                mesh_dir: event.mesh_dir.clone(),
            });
        }
    }
}

pub(crate) fn handle_load_robot(
    asset_server: Res<AssetServer>,
    mut er_load_robot: EventReader<LoadRobot>,
    mut ew_robot_loaded: EventWriter<RobotLoaded>,
) {
    for event in er_load_robot.read() {
        let interaction_groups = event.interaction_groups.clone();
        let mesh_dir = Some(event.clone().mesh_dir);
        let robot_handle: Handle<UrdfAsset> =
            asset_server.load_with_settings(event.clone().urdf_path, move |s: &mut _| {
                *s = RpyAssetLoaderSettings {
                    mesh_dir: mesh_dir.clone(),
                    interaction_groups,
                }
            });

        ew_robot_loaded.send(RobotLoaded {
            handle: robot_handle,
            mesh_dir: event.mesh_dir.clone().replace("assets/", ""),
        });
    }
}
pub(crate) fn handle_wait_robot_loaded(
    mut er_wait_robot_loaded: EventReader<WaitRobotLoaded>,
    mut ew_spawn_robot: EventWriter<SpawnRobot>,
) {
    for event in er_wait_robot_loaded.read() {
        ew_spawn_robot.send(SpawnRobot {
            handle: event.handle.clone(),
            mesh_dir: event.mesh_dir.clone(),
        });
    }
}

pub(crate) fn handle_control_motors(
    mut er_control_motors: EventReader<ControlMotors>,
    q_urdf_robots: Query<(Entity, &URDFRobot)>,
    mut q_rapier_joints: Query<(&mut RapierContextJoints, &RapierRigidBodySet)>,
) {
    for event in er_control_motors.read() {
        for (_parent_entity, urdf_robot) in &mut q_urdf_robots.iter() {
            if urdf_robot.handle != event.handle {
                continue;
            }

            let mut actuator_index: usize = 0;

            for joint_link_handle in urdf_robot.rapier_handles.joints.iter() {
                for (mut rapier_context_joints, _rapier_rigid_bodies) in q_rapier_joints.iter_mut()
                {
                    if let Some(handle) = joint_link_handle.joint {
                        if let Some((multibody, index)) =
                            rapier_context_joints.multibody_joints.get_mut(handle)
                        {
                            if let Some(link) = multibody.link_mut(index) {
                                let mut joint = link.joint.data;

                                if let Some(revolute) = joint.as_revolute_mut() {
                                    if let Some(_) = revolute.motor() {
                                        if actuator_index < event.velocities.len() {
                                            let target_velocity = event.velocities[actuator_index];
                                            link.joint.data.set_motor_velocity(
                                                JointAxis::AngX,
                                                target_velocity,
                                                1.0,
                                            );

                                            actuator_index += 1;
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}
