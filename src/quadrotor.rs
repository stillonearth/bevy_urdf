use std::collections::HashMap;

use bevy::prelude::*;
use bevy_rapier3d::prelude::*;
use nalgebra::vector;

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
                    drone_center_body.reset_forces(false);
                    drone_center_body.reset_torques(false);

                    let translation = drone_center_body.translation();
                    println!("translation {:?}", translation);

                    let rotation = *drone_center_body.rotation();

                    let f1_eff = rotation * vector![0.0, event.thrusts[0], 0.0];
                    let f2_eff = rotation * vector![0.0, event.thrusts[1], 0.0];
                    let f3_eff = rotation * vector![0.0, event.thrusts[2], 0.0];
                    let f4_eff = rotation * vector![0.0, event.thrusts[3], 0.0];

                    let sum_force = f1_eff + f2_eff + f3_eff + f4_eff;
                    let sum_force = vector![sum_force[0], sum_force[2], sum_force[1]];

                    drone_center_body.add_force(sum_force, true);

                    let t1 = rotation * p1.cross(&f1);
                    let t1 = vector![t1[0], t1[2], t1[1]];

                    drone_center_body.add_torque(t1, true);

                    let t2 = rotation * p2.cross(&f2);
                    let t2 = vector![t2[0], t2[2], t2[1]];
                    drone_center_body.add_torque(t2, true);

                    let t3 = rotation * p3.cross(&f3);
                    let t3 = vector![t3[0], t3[2], t3[1]];
                    drone_center_body.add_torque(t3, true);

                    let t4 = rotation * p4.cross(&f4);
                    let t4 = vector![t4[0], t4[2], t4[1]];
                    drone_center_body.add_torque(t4, true);

                    println!("t1 {} t2 {} t3 {} t4 {}", t1, t2, t3, t4);
                }
            }
        }
    }
}
