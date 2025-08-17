use std::{collections::HashMap, path::Path};

use bevy::prelude::*;
use bevy_rapier3d::prelude::*;
use nalgebra::UnitQuaternion;
use rapier3d::prelude::{InteractionGroups, MultibodyJointHandle, RigidBodyHandle};
use rapier3d_urdf::{UrdfMultibodyOptions, UrdfRobot, UrdfRobotHandles};
use uav::dynamics::RotorState;

use crate::{
    drones::{
        try_extract_drone_aerodynamic_props, try_extract_drone_visual_and_dynamic_model_props,
        DroneDescriptor, DroneRotor,
    },
    kinematics::LinkTransform,
    plugin::{
        extract_robot_geometry, rapier_to_bevy_rotation, ExtractedGeometry, URDFRobot,
        URDFRobotRigidBodyHandle,
    },
    urdf_asset_loader::{RpyAssetLoaderSettings, UrdfAsset},
    RobotType,
};

#[derive(Component)]
pub struct Rotor {}

#[derive(Clone, Event)]
pub struct SpawnRobot {
    pub handle: Handle<UrdfAsset>,
    pub mesh_dir: String,
    pub parent_entity: Option<Entity>,
    pub robot_type: RobotType,
    pub drone_descriptor: Option<DroneDescriptor>,
}

#[derive(Clone, Event)]
pub struct DespawnRobot {
    pub handle: Handle<UrdfAsset>,
}

#[derive(Clone, Event)]
pub struct RobotSpawned {
    pub handle: Handle<UrdfAsset>,
}

#[derive(Clone, Event)]
pub struct WaitRobotLoaded {
    pub handle: Handle<UrdfAsset>,
    pub mesh_dir: String,
    pub parent_entity: Option<Entity>,
    pub robot_type: RobotType,
    pub drone_descriptor: Option<DroneDescriptor>,
}

#[derive(Clone, Event)]
pub struct RobotLoaded {
    pub handle: Handle<UrdfAsset>,
    pub mesh_dir: String,
    pub marker: Option<u32>,
    pub robot_type: RobotType,
    pub drone_descriptor: Option<DroneDescriptor>,
}

#[derive(Clone)]
pub struct RapierOption {
    pub interaction_groups: Option<InteractionGroups>,
    pub translation_shift: Option<Vec3>,
    pub create_colliders_from_visual_shapes: bool,
    pub create_colliders_from_collision_shapes: bool,
    pub make_roots_fixed: bool,
}

#[derive(Clone, Event)]
pub struct LoadRobot {
    pub robot_type: RobotType,
    pub urdf_path: String,
    pub mesh_dir: String,
    pub rapier_options: RapierOption,
    pub drone_descriptor: Option<DroneDescriptor>,
    /// this field can be used to keep causality of `LoadRobot -> RobotLoaded` event chain
    pub marker: Option<u32>,
}

fn try_create_drone_descriptor(
    robot_type: RobotType,
    drone_descriptor: Option<DroneDescriptor>,
    urdf_asset: &UrdfAsset,
) -> Option<DroneDescriptor> {
    if robot_type == RobotType::UAV && drone_descriptor.is_none() {
        let adp = try_extract_drone_aerodynamic_props(&urdf_asset.xml_string).unwrap();
        let (dmp, vbp) =
            try_extract_drone_visual_and_dynamic_model_props(&urdf_asset.xml_string).unwrap();

        Some(DroneDescriptor {
            aerodynamic_props: adp,
            dynamics_model_props: dmp,
            visual_body_properties: vbp,
            ..default()
        })
    } else {
        drone_descriptor
    }
}

fn initialize_rapier_handles(
    urdf_robot: UrdfRobot,
    q_rapier_context: &mut Query<(
        Entity,
        &mut RapierRigidBodySet,
        &mut RapierContextColliders,
        &mut RapierContextJoints,
    )>,
) -> UrdfRobotHandles<Option<MultibodyJointHandle>> {
    for (_entity, mut rigid_body_set, mut collider_set, mut multibidy_joint_set) in
        q_rapier_context.iter_mut()
    {
        return urdf_robot.insert_using_multibody_joints(
            &mut rigid_body_set.bodies,
            &mut collider_set.colliders,
            &mut multibidy_joint_set.multibody_joints,
            UrdfMultibodyOptions::DISABLE_SELF_CONTACTS,
        );
    }
    panic!("couldn't initialize handles");
}

fn create_mesh_from_geometry(
    geom: &urdf_rs::Geometry,
    mesh_dir: &str,
    meshes: &mut ResMut<Assets<Mesh>>,
    asset_server: &Res<AssetServer>,
) -> Mesh3d {
    match geom {
        urdf_rs::Geometry::Box { size } => Mesh3d(meshes.add(Cuboid::new(
            size[0] as f32 * 2.0,
            size[1] as f32 * 2.0,
            size[2] as f32 * 2.0,
        ))),
        urdf_rs::Geometry::Cylinder { .. } => todo!(),
        urdf_rs::Geometry::Capsule { .. } => todo!(),
        urdf_rs::Geometry::Sphere { radius } => Mesh3d(meshes.add(Sphere::new(*radius as f32))),
        urdf_rs::Geometry::Mesh { filename, .. } => {
            let model_path = Path::new(mesh_dir).join(filename);
            let model_path = model_path.to_str().unwrap();
            Mesh3d(asset_server.load(model_path))
        }
    }
}

fn calculate_transform_from_pose(
    link_transform: &LinkTransform,
    visual_pose: &urdf_rs::Pose,
) -> (Vec3, Quat) {
    let mut rapier_body_rotation = link_transform.rotation;

    let pose_translation = Vec3::new(
        visual_pose.xyz.0[0] as f32,
        visual_pose.xyz.0[1] as f32,
        visual_pose.xyz.0[2] as f32,
    );

    let pose_rotation = UnitQuaternion::from_euler_angles(
        visual_pose.rpy.0[0] as f32,
        visual_pose.rpy.0[1] as f32,
        visual_pose.rpy.0[2] as f32,
    );

    let mut rapier_body_translation = Vec3::new(
        link_transform.position[0],
        link_transform.position[1],
        link_transform.position[2],
    );

    let bevy_rapier_body_rotation = Quat::from_array([
        rapier_body_rotation.i,
        rapier_body_rotation.j,
        rapier_body_rotation.k,
        rapier_body_rotation.w,
    ]);

    rapier_body_translation += bevy_rapier_body_rotation * pose_translation;
    rapier_body_rotation = rapier_body_rotation * pose_rotation;

    let bevy_translation = rapier_to_bevy_rotation().mul_vec3(rapier_body_translation);
    let bevy_rotation = rapier_to_bevy_rotation()
        * Quat::from_array([
            rapier_body_rotation.i,
            rapier_body_rotation.j,
            rapier_body_rotation.k,
            rapier_body_rotation.w,
        ]);

    (bevy_translation, bevy_rotation)
}

fn setup_drone_rotor(
    ec: &mut EntityCommands,
    drone_descriptor: &DroneDescriptor,
    index: usize,
    rotor_index: &mut usize,
    bevy_translation: Vec3,
    bevy_rotation: Quat,
) {
    let adp = drone_descriptor.aerodynamic_props;
    let vbp = &drone_descriptor.visual_body_properties;
    let dmp = &drone_descriptor.dynamics_model_props;

    if vbp.rotor_body_indices.contains(&index) {
        let max_thrust = adp.thrust2weight * dmp.mass * 9.81;

        let rotor_state = RotorState::new(max_thrust, max_thrust * 2.0, adp.kf, adp.km);

        let transform = if let Some(visual_rotor_position) = vbp.rotor_positions.get(*rotor_index) {
            Transform::from_translation(*visual_rotor_position).with_rotation(bevy_rotation)
        } else {
            Transform::from_translation(bevy_translation).with_rotation(bevy_rotation)
        };

        ec.insert((
            DroneRotor {
                state: rotor_state,
                transform,
                rotor_index: *rotor_index,
            },
            transform,
        ));

        *rotor_index += 1;
    }
}

fn update_collider_user_data(
    entity_id: u32,
    body_handle: RigidBodyHandle,
    q_rapier_context: &mut Query<(
        Entity,
        &mut RapierRigidBodySet,
        &mut RapierContextColliders,
        &mut RapierContextJoints,
    )>,
) {
    for (_entity, mut rigid_body_set, mut collider_set, _) in q_rapier_context.iter_mut() {
        if let Some(rigid_body) = rigid_body_set.bodies.get_mut(body_handle) {
            let collider_handles = rigid_body.colliders();
            for collider_handle in collider_handles.iter() {
                let collider = collider_set.colliders.get_mut(*collider_handle).unwrap();
                collider.user_data = entity_id as u128;
            }
        }
    }
}

#[allow(clippy::too_many_arguments)]
fn spawn_robot_geometries(
    children: &mut ChildSpawnerCommands,
    extracted_geometries: &[ExtractedGeometry],
    kinematic_transforms: &HashMap<String, LinkTransform>,
    body_handles: &[RigidBodyHandle],
    drone_descriptor: Option<DroneDescriptor>,
    event: &SpawnRobot,
    rapier_context_simulation_entity: Entity,
    asset_server: &Res<AssetServer>,
    meshes: &mut ResMut<Assets<Mesh>>,
    materials: &mut ResMut<Assets<StandardMaterial>>,
    q_rapier_context: &mut Query<(
        Entity,
        &mut RapierRigidBodySet,
        &mut RapierContextColliders,
        &mut RapierContextJoints,
    )>,
) {
    let mut rotor_index = 0;

    for (_eg_index, extracted_geometry) in extracted_geometries.iter().enumerate() {
        let index = extracted_geometry.index;

        for (geom_index, geom) in extracted_geometry.geometries.iter().enumerate() {
            let mesh_3d = create_mesh_from_geometry(geom, &event.mesh_dir, meshes, asset_server);

            if let Some(link_transform) = kinematic_transforms.get(&extracted_geometry.link.name) {
                if let Some(visual_pose) = extracted_geometry.visual_poses.get(geom_index) {
                    let (bevy_translation, bevy_rotation) =
                        calculate_transform_from_pose(link_transform, visual_pose);

                    let mut ec = children.spawn((
                        mesh_3d,
                        MeshMaterial3d(materials.add(Color::srgb(0.2, 0.8, 0.2))),
                        URDFRobotRigidBodyHandle {
                            rigid_body_handle: body_handles[index],
                            visual_pose: visual_pose.clone(),
                        },
                        RapierContextEntityLink(rapier_context_simulation_entity),
                        Name::new(format!(
                            "{} part {} / {}",
                            extracted_geometry.link.name.clone(),
                            geom_index + 1,
                            extracted_geometry.geometries.len(),
                        )),
                    ));

                    if let Some(ref drone_descriptor) = drone_descriptor {
                        setup_drone_rotor(
                            &mut ec,
                            drone_descriptor,
                            index,
                            &mut rotor_index,
                            bevy_translation,
                            bevy_rotation,
                        );
                    } else {
                        let transform = Transform::from_translation(bevy_translation)
                            .with_rotation(bevy_rotation);
                        ec.insert(transform);
                    }

                    let entity_id = ec.id().index();
                    update_collider_user_data(entity_id, body_handles[index], q_rapier_context);
                }
            }
        }
    }
}

#[allow(clippy::too_many_arguments)]
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

        if let Some(urdf_asset) = urdf_assets.get(event.handle.id()) {
            // Create drone descriptor if needed
            let drone_descriptor = try_create_drone_descriptor(
                event.robot_type,
                event.drone_descriptor.clone(),
                urdf_asset,
            );

            // Initialize rapier handles

            let rapier_handles =
                initialize_rapier_handles(urdf_asset.urdf_robot.clone(), &mut q_rapier_context);
            let body_handles: Vec<RigidBodyHandle> =
                rapier_handles.links.iter().map(|link| link.body).collect();

            // Extract geometries and get kinematic transforms
            let extracted_geometries = extract_robot_geometry(urdf_asset);

            assert_eq!(body_handles.len(), extracted_geometries.len());

            // Create robot entity
            let mut ec = if let Some(parent_entity) = event.parent_entity {
                commands.entity(parent_entity)
            } else {
                commands.spawn(())
            };

            if event.robot_type == RobotType::UAV {
                ec.insert(drone_descriptor.clone().unwrap());
            }

            ec.insert((
                URDFRobot {
                    handle: event.handle.clone(),
                    rapier_handles,
                    robot_type: event.robot_type,
                },
                Transform::IDENTITY,
                InheritedVisibility::VISIBLE,
                Name::new("URDF Robot"),
            ))
            .with_children(|children| {
                spawn_robot_geometries(
                    children,
                    &extracted_geometries,
                    &urdf_asset.kinematic_transforms,
                    &body_handles,
                    drone_descriptor,
                    event,
                    rapier_context_simulation_entity,
                    &asset_server,
                    &mut meshes,
                    &mut materials,
                    &mut q_rapier_context,
                );
            });

            ew_robot_spawned.write(RobotSpawned {
                handle: event.handle.clone(),
            });
        } else {
            ew_wait_robot_loaded.write(WaitRobotLoaded {
                handle: event.handle.clone(),
                mesh_dir: event.mesh_dir.clone(),
                parent_entity: event.parent_entity,
                robot_type: event.robot_type,
                drone_descriptor: event.drone_descriptor.clone(),
            });
        }
    }
}

pub(crate) fn handle_despawn_robot(
    mut commands: Commands,
    mut q_rapier_context: Query<(
        Entity,
        &mut RapierContextSimulation,
        &mut RapierRigidBodySet,
        &mut RapierContextColliders,
        &mut RapierContextJoints,
    )>,
    q_urdf_robots: Query<(Entity, &URDFRobot)>,
    mut er_despawn_robot: EventReader<DespawnRobot>,
) {
    for event in er_despawn_robot.read() {
        let robot_handle = event.handle.clone();

        for (_, mut simulation, mut rigid_body_set, mut collider_set, mut rapier_context_joints) in
            q_rapier_context.iter_mut()
        {
            let mut impulse_joints = rapier_context_joints.impulse_joints.clone();
            let mut multibody_joint_set = rapier_context_joints.multibody_joints.clone();

            for (entity, urdf_robot) in q_urdf_robots.iter() {
                if urdf_robot.handle != robot_handle {
                    continue;
                }

                for rapier_handle in urdf_robot.rapier_handles.links.iter() {
                    multibody_joint_set.remove_multibody_articulations(rapier_handle.body, true);

                    // remove rigid bodies
                    rigid_body_set.bodies.remove(
                        rapier_handle.body,
                        &mut simulation.islands,
                        &mut collider_set.colliders,
                        &mut impulse_joints,
                        &mut multibody_joint_set,
                        true,
                    );
                }

                commands.entity(entity).despawn();
            }

            rapier_context_joints.impulse_joints = impulse_joints;
            rapier_context_joints.multibody_joints = multibody_joint_set;
        }
    }
}

pub(crate) fn handle_load_robot(
    asset_server: Res<AssetServer>,
    mut er_load_robot: EventReader<LoadRobot>,
    mut ew_robot_loaded: EventWriter<RobotLoaded>,
) {
    for event in er_load_robot.read() {
        let interaction_groups: Option<InteractionGroups> = event.rapier_options.interaction_groups;
        let create_colliders_from_collision_shapes =
            event.rapier_options.create_colliders_from_collision_shapes;
        let create_colliders_from_visual_shapes =
            event.rapier_options.create_colliders_from_visual_shapes;
        let make_roots_fixed = event.rapier_options.make_roots_fixed;
        let translation_shift = event.rapier_options.translation_shift;
        let mesh_dir = Some(event.clone().mesh_dir);
        let robot_handle: Handle<UrdfAsset> =
            asset_server.load_with_settings(event.clone().urdf_path, move |s: &mut _| {
                *s = RpyAssetLoaderSettings {
                    mesh_dir: mesh_dir.clone(),
                    translation_shift,
                    interaction_groups,
                    create_colliders_from_collision_shapes,
                    create_colliders_from_visual_shapes,
                    make_roots_fixed,
                }
            });

        ew_robot_loaded.write(RobotLoaded {
            handle: robot_handle,
            mesh_dir: event.mesh_dir.clone().replace("assets/", ""),
            marker: event.marker,
            robot_type: event.robot_type,
            drone_descriptor: event.drone_descriptor.clone(),
        });
    }
}

pub(crate) fn handle_wait_robot_loaded(
    mut er_wait_robot_loaded: EventReader<WaitRobotLoaded>,
    mut ew_spawn_robot: EventWriter<SpawnRobot>,
) {
    for event in er_wait_robot_loaded.read() {
        ew_spawn_robot.write(SpawnRobot {
            handle: event.handle.clone(),
            mesh_dir: event.mesh_dir.clone(),
            parent_entity: event.parent_entity,
            robot_type: event.robot_type,
            drone_descriptor: event.drone_descriptor.clone(),
        });
    }
}
