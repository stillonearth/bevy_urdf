use std::collections::HashMap;

use bevy::prelude::*;
use bevy_rapier3d::prelude::*;
use nalgebra::vector;
use rapier3d::math::{Isometry, Vector};

use crate::{urdf_asset_loader::UrdfAsset, URDFRobot};

#[derive(Event)]
pub struct ControlThrusts {
    pub handle: Handle<UrdfAsset>,
    pub thrusts: Vec<f32>,
}

#[derive(Component)]
pub struct URDFDrone {
    pub propellers: HashMap<usize, PropellerDirection>,
}

pub enum PropellerDirection {
    CW,
    CCW,
}

pub(crate) fn handle_control_thrusts(
    mut er_control_thrusts: EventReader<ControlThrusts>,
    q_urdf_robots: Query<(Entity, &URDFRobot, &URDFDrone)>,
    mut q_rapier_joints: Query<(&mut RapierContextJoints, &mut RapierRigidBodySet)>,
) {
    let p1 = vector![0.28, 0.0, -0.28];
    let p2 = vector![-0.28, 0.0, -0.28];
    let p3 = vector![-0.28, 0.0, 0.28];
    let p4 = vector![0.28, 0.0, 0.28];

    let torque_to_thrust_ratio = 7.94e-12 / 3.16e-10;

    // let null_rotation = Rotation::

    for event in er_control_thrusts.read() {
        let f1 = vector![0.0, event.thrusts[0], 0.0];
        let f2 = vector![0.0, event.thrusts[1], 0.0];
        let f3 = vector![0.0, event.thrusts[2], 0.0];
        let f4 = vector![0.0, event.thrusts[3], 0.0];

        for (_parent_entity, urdf_robot, urdf_drone) in &mut q_urdf_robots.iter() {
            if urdf_robot.handle != event.handle {
                continue;
            }

            let drone_center_body_handle = urdf_robot.rapier_handles.links[0].body;

            for (_, mut rapier_rigid_bodies) in q_rapier_joints.iter_mut() {
                // ---
                if let Some(drone_center_body) =
                    rapier_rigid_bodies.bodies.get_mut(drone_center_body_handle)
                {
                    // DO DRONE DYNAMICS HERE
                }
            }
        }
    }
}
