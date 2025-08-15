use bevy::prelude::*;
use bevy_rapier3d::prelude::*;

use crate::{plugin::URDFRobot, urdf_asset_loader::UrdfAsset};

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
pub struct ControlMotors {
    pub handle: Handle<UrdfAsset>,
    pub velocities: Vec<f32>,
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
