use std::f32::consts::PI;

use bevy::prelude::*;
use bevy_rapier3d::{
    plugin::RapierConfiguration,
    prelude::{
        RapierContextColliders, RapierContextJoints, RapierContextSimulation, RapierRigidBodySet,
    },
};
use nalgebra::{vector, Isometry3, Rotation3, UnitQuaternion, Vector3};
use roxmltree::Document;

use crate::{events::UAVStateUpdate, urdf_asset_loader::UrdfAsset, URDFRobot};

#[derive(Component, Default, Debug)]
pub struct DroneDescriptor {
    pub mass: f32,
    pub ixx: f32,
    pub iyy: f32,
    pub izz: f32,
    pub rotor_positions: Vec<Vec3>,

    pub root_body_index: usize,
    pub rotor_indices: Vec<usize>,

    pub aerodynamics_props: DroneAerodynamicsProps,
    pub thrusts: Vec<f32>,
    pub uav_state: uav::dynamics::State,
}

#[derive(Default, Debug, Clone, Copy)]
pub struct DroneAerodynamicsProps {
    pub kf: f32,
    pub km: f32,
    pub thrust2weight: f32,
    pub max_speed_kmh: f32,
    pub gnd_eff_coeff: f32,
    pub prop_radius: f32,
    pub drag_coeff_xy: f32,
    pub drag_coeff_z: f32,
    pub dw_coeff_1: f32,
    pub dw_coeff_2: f32,
    pub dw_coeff_3: f32,
}

#[derive(Event)]
pub struct ControlThrusts {
    pub handle: Handle<UrdfAsset>,
    pub thrusts: Vec<f32>,
}

#[derive(Debug)]
pub enum ParseError {
    XmlError(roxmltree::Error),
    MissingProperties,
    InvalidFloat(String),
}

impl std::fmt::Display for ParseError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ParseError::XmlError(e) => write!(f, "XML parsing error: {}", e),
            ParseError::MissingProperties => write!(f, "Missing properties element"),
            ParseError::InvalidFloat(attr) => {
                write!(f, "Invalid float value for attribute: {}", attr)
            }
        }
    }
}

impl std::error::Error for ParseError {}

impl From<roxmltree::Error> for ParseError {
    fn from(error: roxmltree::Error) -> Self {
        ParseError::XmlError(error)
    }
}

pub fn parse_drone_aerodynamics(xml_content: &str) -> Result<DroneAerodynamicsProps, ParseError> {
    let doc = Document::parse(xml_content)?;

    // Find the properties element
    let properties = doc
        .descendants()
        .find(|n| n.has_tag_name("properties"))
        .ok_or(ParseError::MissingProperties)?;

    let mut props = DroneAerodynamicsProps::default();

    // Helper function to parse an attribute
    let parse_attr = |attr_name: &str| -> Result<Option<f32>, ParseError> {
        if let Some(value) = properties.attribute(attr_name) {
            let parsed = value
                .parse::<f32>()
                .map_err(|_| ParseError::InvalidFloat(attr_name.to_string()))?;
            Ok(Some(parsed))
        } else {
            Ok(None)
        }
    };

    // Parse each attribute
    if let Some(val) = parse_attr("kf")? {
        props.kf = val;
    }

    if let Some(val) = parse_attr("km")? {
        props.km = val;
    }

    if let Some(val) = parse_attr("thrust2weight")? {
        props.thrust2weight = val;
    }

    if let Some(val) = parse_attr("max_speed_kmh")? {
        props.max_speed_kmh = val;
    }

    if let Some(val) = parse_attr("gnd_eff_coeff")? {
        props.gnd_eff_coeff = val;
    }

    if let Some(val) = parse_attr("prop_radius")? {
        props.prop_radius = val;
    }

    if let Some(val) = parse_attr("drag_coeff_xy")? {
        props.drag_coeff_xy = val;
    }

    if let Some(val) = parse_attr("drag_coeff_z")? {
        props.drag_coeff_z = val;
    }

    if let Some(val) = parse_attr("dw_coeff_1")? {
        props.dw_coeff_1 = val;
    }

    if let Some(val) = parse_attr("dw_coeff_2")? {
        props.dw_coeff_2 = val;
    }

    if let Some(val) = parse_attr("dw_coeff_3")? {
        props.dw_coeff_3 = val;
    }

    Ok(props)
}

pub(crate) fn handle_control_thrusts(
    mut er_control_thrusts: EventReader<ControlThrusts>,
    mut q_urdf_robots: Query<(Entity, &URDFRobot, &mut DroneDescriptor)>,
) {
    // just copy thrusts to drone descriptor. uav will pick it and apply in dynamics model
    for event in er_control_thrusts.read() {
        for (_, robot, mut descriptor) in q_urdf_robots.iter_mut() {
            if event.handle != robot.handle {
                continue;
            }
            descriptor.thrusts = event.thrusts.clone();
        }
    }
}

pub(crate) fn simulate_drone(
    time: Res<Time>,
    mut q_drones: Query<(Entity, &URDFRobot, &mut DroneDescriptor)>,
    mut q_rapier_context: Query<(
        Entity,
        &mut RapierConfiguration,
        &mut RapierContextSimulation,
        &mut RapierRigidBodySet,
        &mut RapierContextColliders,
        &mut RapierContextJoints,
    )>,
    mut ew_uav_state_update: EventWriter<UAVStateUpdate>,
) {
    let start_time = 0.0;
    let end_time = time.delta_secs_f64() as f64;

    for (_, rapier_configuration, _, mut rigid_bodies, _, _) in q_rapier_context.iter_mut() {
        if !rapier_configuration.physics_pipeline_active {
            continue;
        }

        for (_, urdf_robot, mut drone) in q_drones.iter_mut() {
            let consts = uav::dynamics::Consts {
                g: 9.81, // todo: extract from rapier
                mass: drone.mass as f64,
                ixx: drone.ixx as f64,
                iyy: drone.iyy as f64,
                izz: drone.izz as f64,
            };

            if drone.thrusts.len() < 4 {
                continue;
            }

            let thrusts = [
                drone.thrusts[0],
                drone.thrusts[1],
                drone.thrusts[2],
                drone.thrusts[3],
            ];

            let (forces, torques) = quadrotor_dynamics(thrusts, drone.aerodynamics_props.clone());

            match uav::dynamics::simulate_drone(
                drone.uav_state,
                consts,
                Vector3::new(forces[0] as f64, forces[1] as f64, forces[2] as f64),
                Vector3::new(torques[0] as f64, torques[1] as f64, torques[2] as f64),
                (start_time, end_time),
                1e-6,
            ) {
                Ok(new_state) => {
                    // println!("new_state {:?}", new_state);

                    drone.uav_state = new_state;
                    // assume root body as at link 0
                    let root_body_handle = &urdf_robot.rapier_handles.links[0].body;
                    if let Some(root_body) = rigid_bodies.bodies.get_mut(*root_body_handle) {
                        let position = Isometry3::translation(
                            new_state.position_x as f32,
                            new_state.position_z as f32,
                            new_state.position_y as f32,
                        );

                        let rotation = Rotation3::from_euler_angles(
                            new_state.roll as f32,
                            new_state.pitch as f32,
                            new_state.yaw as f32,
                        );
                        let quat = UnitQuaternion::from_rotation_matrix(&rotation);
                        let quat_fix =
                            UnitQuaternion::from_scaled_axis(Vector3::new(-PI / 2., 0.0, 0.0));

                        root_body.set_position(position.into(), false);
                        root_body.set_rotation(quat_fix * quat, false);
                    }

                    ew_uav_state_update.write(UAVStateUpdate {
                        handle: urdf_robot.handle.clone(),
                        drone_state: new_state,
                    });
                }
                Err(_e) => {}
            }
        }
    }
}

fn quadrotor_dynamics(
    thrusts: [f32; 4],
    aerodynamics_props: DroneAerodynamicsProps,
) -> ([f32; 3], [f32; 3]) {
    // let torque_to_thrust_ratio = 7.94e-12 / 3.16e-10;
    let torque_to_thrust_ratio = aerodynamics_props.km / aerodynamics_props.kf;

    let rotor_1_position = vector![0.028, -0.028, 0.0];
    let rotor_2_position = vector![-0.028, -0.028, 0.0];
    let rotor_3_position = vector![-0.028, 0.028, 0.0];
    let rotor_4_position = vector![0.028, 0.028, 0.0];

    let f = vector![0.0, 0.0, 1.0];
    let f1 = f * thrusts[0];
    let f2 = f * thrusts[1];
    let f3 = f * thrusts[2];
    let f4 = f * thrusts[3];

    let full_force = f1 + f2 + f3 + f4;

    let t1_thrust = (rotor_1_position).cross(&(f1));
    let t1_torque = torque_to_thrust_ratio * (f1);

    let t2_thrust = (rotor_2_position).cross(&(f2));
    let t2_torque = torque_to_thrust_ratio * (f2);

    let t3_thrust = (rotor_3_position).cross(&(f3));
    let t3_torque = torque_to_thrust_ratio * (f3);

    let t4_thrust = (rotor_4_position).cross(&(f4));
    let t4_torque = torque_to_thrust_ratio * (f4);

    let t_thrust = t1_thrust + t2_thrust + t3_thrust + t4_thrust;
    let t_torque = (t1_torque - t4_torque) - (t2_torque - t3_torque);

    let torque = t_thrust - t_torque;

    return (
        [full_force.x, full_force.y, full_force.z],
        [torque.x, torque.y, torque.z],
    );
}
