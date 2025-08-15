use std::path::Path;

use bevy::prelude::*;
use bevy_rapier3d::prelude::*;
use rapier3d::prelude::{InteractionGroups, MultibodyJointHandle, RigidBodyHandle};
use rapier3d_urdf::{UrdfMultibodyOptions, UrdfRobotHandles};
use uav::dynamics::RotorState;

use crate::{
    drones::{
        try_extract_drone_aerodynamics_properties,
        try_extract_drone_visual_and_dynamics_model_properties, DroneDescriptor, DroneRotor,
    },
    plugin::{extract_robot_geometry, URDFRobot, URDFRobotRigidBodyHandle},
    urdf_asset_loader::{RpyAssetLoaderSettings, UrdfAsset},
    uuv::{try_extract_uuv_thruster_positions, UuvDescriptor},
    RobotType,
};

#[derive(Component)]
pub struct Rotor {}

#[derive(Component)]
pub struct Thruster {
    pub index: usize,
}

#[derive(Clone, Event)]
pub struct SpawnRobot {
    pub handle: Handle<UrdfAsset>,
    pub mesh_dir: String,
    pub parent_entity: Option<Entity>,
    pub robot_type: RobotType,
    pub drone_descriptor: Option<DroneDescriptor>,
    pub uuv_descriptor: Option<UuvDescriptor>,
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
    pub uuv_descriptor: Option<UuvDescriptor>,
}

#[derive(Clone, Event)]
pub struct RobotLoaded {
    pub handle: Handle<UrdfAsset>,
    pub mesh_dir: String,
    pub marker: Option<u32>,
    pub robot_type: RobotType,
    pub drone_descriptor: Option<DroneDescriptor>,
    pub uuv_descriptor: Option<UuvDescriptor>,
}

#[derive(Clone)]
pub struct RapierOption {
    pub interaction_groups: Option<InteractionGroups>,
    pub translation_shift: Option<Vec3>,
    pub create_colliders_from_visual_shapes: bool,
    pub create_colliders_from_collision_shapes: bool,
}

#[derive(Clone, Event)]
pub struct LoadRobot {
    pub robot_type: RobotType,
    pub urdf_path: String,
    pub mesh_dir: String,
    pub rapier_options: RapierOption,
    pub drone_descriptor: Option<DroneDescriptor>,
    pub uuv_descriptor: Option<UuvDescriptor>,
    /// this field can be used to keep causality of `LoadRobot -> RobotLoaded` event chain
    pub marker: Option<u32>,
}

#[derive(Event)]
pub struct SensorsRead {
    pub handle: Handle<UrdfAsset>,
    pub transforms: Vec<Transform>,
    pub joint_angles: Vec<f32>,
}

#[derive(Event)]
pub struct UAVStateUpdate {
    pub handle: Handle<UrdfAsset>,
    pub drone_state: uav::dynamics::State,
}

#[derive(Event)]
pub struct UuvStateUpdate {
    pub handle: Handle<UrdfAsset>,
    pub uuv_state: crate::uuv::UuvState,
}

#[derive(Event)]
pub struct ControlMotors {
    pub handle: Handle<UrdfAsset>,
    pub velocities: Vec<f32>,
}

#[derive(Event)]
pub struct ControlThrusters {
    pub handle: Handle<UrdfAsset>,
    pub thrusts: Vec<f32>,
}

#[derive(Event)]
pub struct ControlFins {
    pub handle: Handle<UrdfAsset>,
    pub angles: Vec<f32>,
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
        let robot_handle = event.handle.clone();
        let mut drone_descriptor = event.drone_descriptor.clone();
        let mut uuv_descriptor = event.uuv_descriptor.clone();

        if let Some(urdf) = urdf_assets.get(robot_handle.id()) {
            let mut maybe_rapier_handles: Option<UrdfRobotHandles<Option<MultibodyJointHandle>>> =
                None;

            // let mut handles: Option<UrdfRobotHandles<ImpulseJointHandle>> = None;
            for (_entity, mut rigid_body_set, mut collider_set, mut multibidy_joint_set) in
                q_rapier_context.iter_mut()
            {
                let urdf_robot = urdf.urdf_robot.clone();

                // do stuff if robot is a copter
                if event.robot_type == RobotType::Drone && drone_descriptor.is_none() {
                    // extract model parameters automatically or fill manually if drone_descriptor is not none
                    let urdf_asset = urdf_assets.get(&event.handle).unwrap();
                    let adp =
                        try_extract_drone_aerodynamics_properties(&urdf_asset.xml_string).unwrap();
                    let (dmp, vbp) = try_extract_drone_visual_and_dynamics_model_properties(
                        &urdf_asset.xml_string,
                    )
                    .unwrap();

                    drone_descriptor = Some(DroneDescriptor {
                        aerodynamic_props: adp,
                        dynamics_model_props: dmp,
                        visual_body_properties: vbp,
                        ..default()
                    });
                }

                if event.robot_type == RobotType::Uuv && uuv_descriptor.is_none() {
                    let urdf_asset = urdf_assets.get(&event.handle).unwrap();
                    let thrusters =
                        try_extract_uuv_thruster_positions(&urdf_asset.xml_string).unwrap_or_default();
                    uuv_descriptor = Some(UuvDescriptor {
                        thruster_positions: thrusters,
                        ..default()
                    });
                }

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

            let mut ec = if let Some(parent_entity) = event.parent_entity {
                commands.entity(parent_entity)
            } else {
                commands.spawn(())
            };

            if event.robot_type == RobotType::Drone {
                ec.insert(drone_descriptor.clone().unwrap());
            }

            if event.robot_type == RobotType::Uuv {
                ec.insert(uuv_descriptor.clone().unwrap());
            }

            ec.insert((
                URDFRobot {
                    handle: event.handle.clone(),
                    rapier_handles,
                    robot_type: event.robot_type,
                },
                Transform::IDENTITY.with_rotation(Quat::from_rotation_x(std::f32::consts::PI)),
                InheritedVisibility::VISIBLE,
            ))
            .with_children(|children| {
                let mut rotor_index = 0;
                for (index, geom, _, _visual_pose, _collider) in geoms {
                    if geom.is_none() {
                        continue;
                    }

                    let mesh_3d: Mesh3d = match geom.unwrap() {
                        urdf_rs::Geometry::Box { size } => Mesh3d(meshes.add(Cuboid::new(
                            size[0] as f32 * 2.0,
                            size[2] as f32 * 2.0,
                            size[1] as f32 * 2.0,
                        ))),
                        urdf_rs::Geometry::Cylinder { .. } => todo!(),
                        urdf_rs::Geometry::Capsule { .. } => todo!(),
                        urdf_rs::Geometry::Sphere { radius } => {
                            Mesh3d(meshes.add(Sphere::new(radius as f32)))
                        }
                        urdf_rs::Geometry::Mesh { filename, .. } => {
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

                    let mut ec = children.spawn((
                        mesh_3d,
                        MeshMaterial3d(materials.add(Color::srgb(0.2, 0.8, 0.2))),
                        URDFRobotRigidBodyHandle(body_handles[index]),
                        RapierContextEntityLink(rapier_context_simulation_entity),
                    ));

                    if let Some(drone_descriptor) = drone_descriptor.clone() {
                        let adp = drone_descriptor.aerodynamic_props;
                        let vbp = drone_descriptor.visual_body_properties;
                        let dmp = drone_descriptor.dynamics_model_props;

                        if vbp.rotor_body_indices.contains(&index) {
                            let max_thrust = adp.thrust2weight * dmp.mass * 9.81;

                            let rotor_state = RotorState::new(
                                // todo: get 9.81 from config
                                max_thrust,
                                max_thrust * 2.0,
                                adp.kf,
                                adp.km,
                            );

                            let transform = if let Some(visual_rotor_position) =
                                vbp.rotor_positions.get(rotor_index)
                            {
                                Transform::from_translation(*visual_rotor_position)
                                    .with_rotation(bevy_quat)
                            } else {
                                Transform::from_translation(bevy_vec).with_rotation(bevy_quat)
                            };

                            ec.insert(DroneRotor {
                                state: rotor_state,
                                transform: transform,
                                rotor_index,
                            });
                            rotor_index += 1;

                            ec.insert(transform);
                        }
                    } else {
                        let transform =
                            Transform::from_translation(bevy_vec).with_rotation(bevy_quat);
                        ec.insert(transform);
                    }

                    // insert entity id to collider data, otherwise it breaks in debug mode
                    let entity_id = ec.id().index();
                for (_entity, mut rigid_body_set, mut collider_set, _) in
                        q_rapier_context.iter_mut()
                    {
                        if let Some(rigid_body) = rigid_body_set.bodies.get_mut(body_handles[index])
                        {
                            let collider_handles = rigid_body.colliders();
                            for collider_handle in collider_handles.iter() {
                                let collider =
                                    collider_set.colliders.get_mut(*collider_handle).unwrap();
                                collider.user_data = entity_id as u128;
                            }
                        }
                    }
                }

                if event.robot_type == RobotType::Uuv {
                    if let Some(desc) = uuv_descriptor.clone() {
                        for (thruster_index, pos) in desc.thruster_positions.iter().enumerate() {
                            children.spawn((
                                Thruster { index: thruster_index },
                                Transform::from_translation(*pos),
                            ));
                        }
                    }
                }
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
                uuv_descriptor: event.uuv_descriptor.clone(),
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
                }
            });

        ew_robot_loaded.write(RobotLoaded {
            handle: robot_handle,
            mesh_dir: event.mesh_dir.clone().replace("assets/", ""),
            marker: event.marker,
            robot_type: event.robot_type,
            drone_descriptor: event.drone_descriptor.clone(),
            uuv_descriptor: event.uuv_descriptor.clone(),
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
            uuv_descriptor: event.uuv_descriptor.clone(),
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
                                    if revolute.motor().is_some() {
                                        if event.velocities.len() < actuator_index {
                                            panic!("not enough control parameters provided");
                                        }

                                        if event.velocities.is_empty() {
                                            continue;
                                        }

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
