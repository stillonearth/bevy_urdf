use bevy::prelude::*;
use bevy_rapier3d::prelude::*;
use rapier3d::prelude::MultibodyJointHandle;
use rapier3d_urdf::UrdfJointHandle;
use serde::Deserialize;

use crate::{plugin::URDFRobot, urdf_asset_loader::UrdfAsset};

#[derive(Message)]
pub struct SensorsRead {
    pub handle: Handle<UrdfAsset>,
    pub transforms: Vec<Transform>,
    pub joint_angles: Vec<f32>,
}

#[derive(Message)]
pub struct UAVStateUpdate {
    pub handle: Handle<UrdfAsset>,
    pub drone_state: uav::dynamics::State,
}

#[derive(Message)]
pub struct UUVStateUpdate {
    pub handle: Handle<UrdfAsset>,
    pub uuv_state: crate::uuv::UuvState,
}

#[derive(Message)]
pub struct ControlMotorVelocities {
    pub handle: Handle<UrdfAsset>,
    pub velocities: Vec<f32>,
}

#[derive(Clone, Copy, Reflect, Deserialize)]
pub struct MotorProps {
    pub stiffness: f32,
    pub damping: f32,
}

#[derive(Message)]
pub struct ControlMotorPositions {
    pub handle: Handle<UrdfAsset>,
    pub positions: Vec<f32>,
    pub motor_props: Vec<MotorProps>,
}

#[derive(Message)]
pub struct ControlThrusters {
    pub handle: Handle<UrdfAsset>,
    pub thrusts: Vec<f32>,
}

#[derive(Message)]
pub struct ControlFins {
    pub handle: Handle<UrdfAsset>,
    pub angles: Vec<f32>,
}

pub(crate) fn handle_control_motor_velocities(
    mut er_control_motors: MessageReader<ControlMotorVelocities>,
    q_urdf_robots: Query<(Entity, &URDFRobot)>,
    mut q_rapier_joints: Query<(&mut RapierContextJoints, &mut RapierRigidBodySet)>,
) {
    for event in er_control_motors.read() {
        // Early exit conditions
        if event.velocities.is_empty() {
            continue;
        }

        // Find target robot
        let target_robot = q_urdf_robots
            .iter()
            .find(|(_, robot)| robot.handle == event.handle);

        let Some((_, urdf_robot)) = target_robot else {
            continue;
        };

        let mut actuator_index = 0;

        for (mut rapier_context_joints, mut rapier_rigid_bodies) in q_rapier_joints.iter_mut() {
            // Process each joint
            for joint_link_handle in &urdf_robot.rapier_handles.joints {
                let Some(joint_handle) = joint_link_handle.joint else {
                    continue;
                };

                // Process each rapier context

                let Some((multibody, index)) =
                    rapier_context_joints.multibody_joints.get_mut(joint_handle)
                else {
                    continue;
                };

                let Some(link) = multibody.link_mut(index) else {
                    continue;
                };

                let joint = &mut link.joint.data;

                let Some(revolute) = joint.as_revolute_mut() else {
                    continue;
                };

                // Only process if motor exists
                if revolute.motor().is_none() {
                    continue;
                }

                wakeup_links(&mut rapier_rigid_bodies, joint_link_handle);

                // Validate we have enough velocities
                if actuator_index >= event.velocities.len() {
                    panic!(
                        "Not enough control parameters provided. Required: {}, Available: {}",
                        actuator_index + 1,
                        event.velocities.len()
                    );
                }

                let target_velocity = event.velocities[actuator_index];
                joint.set_motor_velocity(rapier3d::prelude::JointAxis::AngX, target_velocity, 1.0);
                actuator_index += 1;
            }
        }
    }
}

pub(crate) fn handle_control_motor_positions(
    mut er_control_motors: MessageReader<ControlMotorPositions>,
    q_urdf_robots: Query<(Entity, &URDFRobot)>,
    mut q_rapier_joints: Query<(&mut RapierContextJoints, &mut RapierRigidBodySet)>,
) {
    for event in er_control_motors.read() {
        // Early exit conditions
        if event.positions.is_empty() {
            continue;
        }

        // Find target robot
        let target_robot = q_urdf_robots
            .iter()
            .find(|(_, robot)| robot.handle == event.handle);

        let Some((_, urdf_robot)) = target_robot else {
            continue;
        };

        let mut actuator_index = 0;

        for (mut rapier_context_joints, mut rapier_rigid_bodies) in q_rapier_joints.iter_mut() {
            for joint_link_handle in &urdf_robot.rapier_handles.joints {
                let Some(joint_handle) = joint_link_handle.joint else {
                    continue;
                };

                let Some((multibody, index)) =
                    rapier_context_joints.multibody_joints.get_mut(joint_handle)
                else {
                    continue;
                };

                let Some(link) = multibody.link_mut(index) else {
                    continue;
                };

                let joint = &mut link.joint.data;
                let Some(revolute) = joint.as_revolute_mut() else {
                    continue;
                };

                if revolute.motor().is_none() {
                    continue;
                }

                wakeup_links(&mut rapier_rigid_bodies, joint_link_handle);

                if actuator_index >= event.positions.len() {
                    panic!(
                        "Not enough positions parameters provided. Required: {}, Available: {}",
                        actuator_index + 1,
                        event.positions.len()
                    );
                }

                if actuator_index >= event.motor_props.len() {
                    panic!(
                        "Not enough motor_props parameters provided. Required: {}, Available: {}",
                        actuator_index + 1,
                        event.positions.len()
                    );
                }

                let target_position = event.positions[actuator_index];
                let motor_props = event.motor_props[actuator_index];

                joint.set_motor_position(
                    JointAxis::AngX,
                    target_position,
                    motor_props.stiffness,
                    motor_props.damping,
                );
                actuator_index += 1;
            }
        }
    }
}

fn wakeup_links(
    rapier_rigid_bodies: &mut Mut<RapierRigidBodySet>,
    joint_link_handle: &UrdfJointHandle<Option<MultibodyJointHandle>>,
) {
    for link in [joint_link_handle.link1, joint_link_handle.link2] {
        let Some(body) = rapier_rigid_bodies.bodies.get_mut(link) else {
            continue;
        };
        if body.is_sleeping() {
            body.wake_up(true)
        }
    }
}
