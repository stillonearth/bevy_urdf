use std::collections::VecDeque;

use anyhow::Error;

use bevy::prelude::*;

use crate::{
    control::{ControlFins, ControlThrusters, UUVStateUpdate},
    URDFRobot,
};

#[derive(Default, Debug, Clone, Copy)]
pub struct HydrodynamicProperties {
    pub mass: f32,
    pub buoyancy: f32,
    pub linear_drag: Vec3,
    pub angular_drag: Vec3,
}

#[derive(Default, Debug, Clone, Copy)]
pub struct UuvState {
    pub position: Vec3,
    pub orientation: Quat,
    pub linear_velocity: Vec3,
    pub angular_velocity: Vec3,
}

#[derive(Component, Debug, Clone)]
pub struct UUVDescriptor {
    pub hydrodynamic_props: HydrodynamicProperties,
    pub thrust_force: Vec3,
    pub thrust_torque: Vec3,
    pub thruster_forces: Vec<f32>,
    pub thruster_positions: Vec<Vec3>,
    pub fin_angles: Vec<f32>,
    pub state: UuvState,
    pub(crate) state_log: VecDeque<UuvState>,
}

impl Default for UUVDescriptor {
    fn default() -> Self {
        Self {
            hydrodynamic_props: HydrodynamicProperties {
                mass: 1.0,
                buoyancy: 0.0,
                linear_drag: Vec3::splat(0.1),
                angular_drag: Vec3::splat(0.1),
            },
            thrust_force: Vec3::ZERO,
            thrust_torque: Vec3::ZERO,
            thruster_forces: Vec::new(),
            thruster_positions: Vec::new(),
            fin_angles: Vec::new(),
            state: UuvState::default(),
            state_log: VecDeque::new(),
        }
    }
}

impl UUVDescriptor {
    fn push_state(&mut self, state: UuvState) {
        self.state = state;
        if self.state_log.len() >= 30 {
            self.state_log.pop_front();
        }
        self.state_log.push_back(state);
    }
}

pub fn try_extract_uuv_thruster_positions(xml_content: &str) -> Result<Vec<Vec3>, Error> {
    let urdf_robot = urdf_rs::read_from_string(xml_content)?;
    let mut thruster_positions = Vec::new();

    for link in urdf_robot.links.iter() {
        if link.name.to_lowercase().contains("thruster") {
            let origin = link.inertial.origin.xyz.0;
            thruster_positions.push(Vec3::new(
                origin[0] as f32,
                origin[1] as f32,
                origin[2] as f32,
            ));
        }
    }

    Ok(thruster_positions)
}

pub(crate) fn simulate_uuv(
    mut q_uuvs: Query<(&mut Transform, &URDFRobot, &mut UUVDescriptor)>,
    time: Res<Time>,
    mut ew_state: MessageWriter<UUVStateUpdate>,
) {
    for (mut transform, robot, mut descriptor) in q_uuvs.iter_mut() {
        let dt = time.delta_secs();
        let props = descriptor.hydrodynamic_props;
        let mut state = descriptor.state;

        // derive forces and torques from thruster and fin commands
        let thrust_total: f32 = descriptor.thruster_forces.iter().sum();
        descriptor.thrust_force = state.orientation * Vec3::new(thrust_total, 0.0, 0.0);

        let fin_torque: f32 = descriptor.fin_angles.iter().sum();
        descriptor.thrust_torque = Vec3::new(0.0, fin_torque, 0.0);

        let buoyancy_force = Vec3::new(0.0, props.buoyancy, 0.0);
        let drag_force = props.linear_drag * state.linear_velocity;
        let total_force = descriptor.thrust_force + buoyancy_force - drag_force;
        let acceleration = total_force / props.mass;
        state.linear_velocity += acceleration * dt;
        state.position += state.linear_velocity * dt;

        let drag_torque = props.angular_drag * state.angular_velocity;
        let total_torque = descriptor.thrust_torque - drag_torque;
        let angular_acc = total_torque / props.mass;
        state.angular_velocity += angular_acc * dt;
        state.orientation = state.orientation
            * Quat::from_rotation_x(state.angular_velocity.x * dt)
            * Quat::from_rotation_y(state.angular_velocity.y * dt)
            * Quat::from_rotation_z(state.angular_velocity.z * dt);

        descriptor.push_state(state);
        *transform = Transform::from_translation(state.position).with_rotation(state.orientation);

        ew_state.write(UUVStateUpdate {
            handle: robot.handle.clone(),
            uuv_state: state,
        });
    }
}

pub(crate) fn handle_control_thrusters(
    mut er_thrusters: MessageReader<ControlThrusters>,
    mut q_uuvs: Query<(&URDFRobot, &mut UUVDescriptor)>,
) {
    for event in er_thrusters.read() {
        for (robot, mut descriptor) in q_uuvs.iter_mut() {
            if robot.handle != event.handle {
                continue;
            }
            descriptor.thruster_forces = event.thrusts.clone();
        }
    }
}

pub(crate) fn handle_control_fins(
    mut er_fins: MessageReader<ControlFins>,
    mut q_uuvs: Query<(&URDFRobot, &mut UUVDescriptor)>,
) {
    for event in er_fins.read() {
        for (robot, mut descriptor) in q_uuvs.iter_mut() {
            if robot.handle != event.handle {
                continue;
            }
            descriptor.fin_angles = event.angles.clone();
        }
    }
}
