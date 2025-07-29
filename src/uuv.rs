use std::collections::VecDeque;

use bevy::prelude::*;

use crate::{events::UuvStateUpdate, URDFRobot};

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
pub struct UuvDescriptor {
    pub hydrodynamic_props: HydrodynamicProperties,
    pub thrust_force: Vec3,
    pub thrust_torque: Vec3,
    pub state: UuvState,
    pub(crate) state_log: VecDeque<UuvState>,
}

impl Default for UuvDescriptor {
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
            state: UuvState::default(),
            state_log: VecDeque::new(),
        }
    }
}

impl UuvDescriptor {
    fn push_state(&mut self, state: UuvState) {
        self.state = state;
        if self.state_log.len() >= 30 {
            self.state_log.pop_front();
        }
        self.state_log.push_back(state);
    }
}

pub(crate) fn simulate_uuv(
    mut q_uuvs: Query<(&mut Transform, &URDFRobot, &mut UuvDescriptor)>,
    time: Res<Time>,
    mut ew_state: EventWriter<UuvStateUpdate>,
) {
    for (mut transform, robot, mut descriptor) in q_uuvs.iter_mut() {
        let dt = time.delta_secs();
        let props = descriptor.hydrodynamic_props;
        let mut state = descriptor.state;

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

        ew_state.write(UuvStateUpdate {
            handle: robot.handle.clone(),
            uuv_state: state,
        });
    }
}

