use std::path::Path;

use bevy::prelude::*;
use bevy_rapier3d::prelude::*;
use rapier3d::prelude::{
    ColliderBuilder, Group, ImpulseJointHandle, InteractionGroups, MultibodyJointHandle,
    RigidBodyHandle,
};
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
    mut meshes: ResMut<Assets<Mesh>>,
    q_rapier_context_simulation: Query<(Entity, &RapierContextSimulation)>,
) {
    for event in er_spawn_robot.read() {
        let rapier_context_simulation_entity = q_rapier_context_simulation.iter().next().unwrap().0;
        let robot_handle = event.handle.clone();
        if let Some(urdf) = urdf_assets.get(robot_handle.id()) {
            let mut handles: Option<UrdfRobotHandles<Option<MultibodyJointHandle>>> = None;
            // let mut handles: Option<UrdfRobotHandles<ImpulseJointHandle>> = None;
            for (_entity, mut rigid_body_set, mut collider_set, mut multibidy_joint_set) in
                q_rapier_context.iter_mut()
            {
                // insert colliders

                for (_, collider) in collider_set.colliders.iter() {
                    println!("collider {:?}", collider.collision_groups());
                    println!("collider {:?}", collider.active_collision_types());
                    println!("collider {:?}", collider.position());
                }

                let mut collider = ColliderBuilder::cuboid(100.0, 0.1, 100.0).build();
                // // println!("collider, {:?}", collider.);
                // // collider.set_position(Isometry::<Real>::ne);
                collider_set.colliders.insert(collider);

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
                    // }
                    for (_, collider) in colliders.clone().iter().enumerate() {
                        println!("");
                        println!("robot collider {:?}", collider.collision_groups());
                        println!("robot collider {:?}", collider.active_collision_types());
                        println!("robot collider {:?}", collider.position());
                    }
                    // urdf_robot.links[i].colliders = colliders;
                }

                // handles = Some(urdf_robot.clone().insert_using_impulse_joints(
                //     &mut rigid_body_set.bodies,
                //     &mut collider_set.colliders,
                //     &mut multibidy_joint_set.impulse_joints,
                //     // UrdfMultibodyOptions::DISABLE_SELF_CONTACTS,
                // ));

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
                    InheritedVisibility::VISIBLE,
                    // Transform::IDENTITY.with_rotation(Quat::from_rotation_z(std::f32::consts::PI)),
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
                            urdf_rs::Geometry::Cylinder { radius, length } => todo!(),
                            urdf_rs::Geometry::Capsule { radius, length } => todo!(),
                            urdf_rs::Geometry::Sphere { radius } => {
                                Mesh3d(meshes.add(Sphere::new(radius as f32)))
                            }
                            urdf_rs::Geometry::Mesh { filename, scale } => {
                                println!("filename: {}", filename);
                                let base_path = "robots/unitree_a1/urdf";
                                let model_path = Path::new(base_path).join(filename);
                                let model_path = model_path.to_str().unwrap();

                                let mesh_3d = Mesh3d(asset_server.load(model_path));

                                mesh_3d
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
                            RapierContextEntityLink(rapier_context_simulation_entity.clone()),
                            transform,
                        ));
                    }
                });
        }
    }
}
