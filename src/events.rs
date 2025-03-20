use std::path::Path;

use bevy::prelude::*;
use bevy_rapier3d::prelude::*;
use rapier3d::prelude::{Group, InteractionGroups, MultibodyJointHandle, RigidBodyHandle};
use rapier3d_urdf::{UrdfMultibodyOptions, UrdfRobotHandles};

use crate::{plugin::extract_robot_geometry, urdf_asset_loader::UrdfAsset};

#[derive(Clone, Event)]
pub struct SpawnRobot {
    pub handle: Handle<UrdfAsset>,
}

#[derive(Component, Default)]
pub struct UrdfRobot {}

#[derive(Component, Default, Deref)]
pub struct UrdfRobotRigidBodyHandle(pub RigidBodyHandle);

pub(crate) fn handle_spawn_robot(
    mut commands: Commands,
    mut er_spawn_robot: EventReader<SpawnRobot>,
    urdf_assets: Res<Assets<UrdfAsset>>,
    asset_server: Res<AssetServer>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut q_rapier_context: Query<(
        Entity,
        &mut RapierRigidBodySet,
        &mut RapierContextColliders,
        &mut RapierContextJoints,
    )>,
    q_rapier_context_simulation: Query<(Entity, &RapierContextSimulation)>,
) {
    for event in er_spawn_robot.read() {
        let rapier_context_simulation_entity = q_rapier_context_simulation.iter().next().unwrap().0;
        let robot_handle = event.handle.clone();
        if let Some(urdf) = urdf_assets.get(robot_handle.id()) {
            let mut handles: Option<UrdfRobotHandles<Option<MultibodyJointHandle>>> = None;
            for (_entity, mut rigid_body_set, mut collider_set, mut multibidy_joint_set) in
                q_rapier_context.iter_mut()
            {
                let mut urdf_robot = urdf.urdf_robot.clone();

                // for link in urdf_robot.links.iter() {
                //     for mut collider in link.colliders.iter_mut() {
                //         println!("collider id: {:?}", collider.collision_groups());
                //         collider.set_collision_groups(InteractionGroups::all());
                //     }
                // }

                for (i, _) in urdf_robot.links.clone().iter().enumerate() {
                    urdf_robot.links[i].body.set_gravity_scale(-1.0, true);
                    let mut colliders = urdf_robot.links[i].colliders.clone();

                    for (j, _) in colliders.clone().iter().enumerate() {
                        colliders[j].set_collision_groups(InteractionGroups::new(
                            Group::GROUP_1 | Group::GROUP_2,
                            Group::NONE,
                        ));
                    }
                    urdf_robot.links[i].colliders = colliders;
                }

                handles = Some(urdf_robot.clone().insert_using_multibody_joints(
                    &mut rigid_body_set.bodies,
                    &mut collider_set.colliders,
                    &mut multibidy_joint_set.multibody_joints,
                    UrdfMultibodyOptions::DISABLE_SELF_CONTACTS,
                ));
                break;
            }

            if handles.is_none() {
                panic!("couldn't initialize handles");
            }

            let body_handles: Vec<RigidBodyHandle> = handles
                .unwrap()
                .links
                .iter()
                .map(|link| link.body.clone())
                .collect();
            let geoms = extract_robot_geometry(urdf);

            assert_eq!(body_handles.len(), geoms.len());

            commands
                .spawn((
                    UrdfRobot {},
                    Transform::IDENTITY,
                    Name::new("robot"),
                    // Transform::IDENTITY.with_rotation(Quat::from_rotation_z(std::f32::consts::PI)),
                ))
                .with_children(|children| {
                    for (index, geom, _origin_pose, _inertia_pose, _collider) in geoms {
                        match geom {
                            urdf_rs::Geometry::Box { size } => todo!(),
                            urdf_rs::Geometry::Cylinder { radius, length } => todo!(),
                            urdf_rs::Geometry::Capsule { radius, length } => todo!(),
                            urdf_rs::Geometry::Sphere { radius } => todo!(),
                            urdf_rs::Geometry::Mesh { filename, scale } => {
                                let base_path = "robots/flamingo_edu/urdf/";
                                let model_path = Path::new(base_path).join(filename);
                                let model_path = model_path.to_str().unwrap();

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

                                let mesh_3d = Mesh3d(asset_server.load(model_path));

                                children.spawn((
                                    mesh_3d,
                                    MeshMaterial3d(materials.add(Color::srgb(0.3, 0.4, 0.3))),
                                    UrdfRobotRigidBodyHandle(body_handles[index]),
                                    RapierContextEntityLink(
                                        rapier_context_simulation_entity.clone(),
                                    ),
                                    transform,
                                ));
                            }
                        }
                    }
                });
        }
    }
}
