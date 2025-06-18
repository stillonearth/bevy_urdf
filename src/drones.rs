use std::f32::consts::PI;

use anyhow::Error;
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

#[derive(Component, Default, Debug, Clone)]
pub struct DroneDescriptor {
    pub aerodynamic_props: AerodynamicsProperties,
    pub dynamics_model_props: DynamicsModelProperties,
    pub visual_body_properties: VisualBodyProperties,

    pub thrust_commands: Vec<f32>,
    pub uav_state: uav::dynamics::State,
}

#[derive(Default, Debug, Clone, Copy)]
pub struct AerodynamicsProperties {
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

#[derive(Default, Debug, Clone)]
pub struct DynamicsModelProperties {
    pub mass: f32,
    pub ixx: f32,
    pub iyy: f32,
    pub izz: f32,
    pub rotor_positions: Vec<Vec3>,
}

#[derive(Default, Debug, Clone)]
pub struct VisualBodyProperties {
    pub root_body_index: usize,
    pub rotor_body_indices: Vec<usize>,
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
            ParseError::XmlError(e) => write!(f, "XML parsing error: {e}"),
            ParseError::MissingProperties => write!(f, "Missing properties element"),
            ParseError::InvalidFloat(attr) => {
                write!(f, "Invalid float value for attribute: {attr}")
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

pub fn try_extract_drone_aerodynamics_properties(
    xml_content: &str,
) -> Result<AerodynamicsProperties, ParseError> {
    let doc = Document::parse(xml_content)?;

    // Find the properties element
    let properties = doc
        .descendants()
        .find(|n| n.has_tag_name("properties"))
        .ok_or(ParseError::MissingProperties)?;

    let mut props = AerodynamicsProperties::default();

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

pub fn try_extract_drone_visual_and_dynamics_model_properties(
    xml_content: &str,
) -> Result<(DynamicsModelProperties, VisualBodyProperties), Error> {
    let urdf_robot = urdf_rs::read_from_string(xml_content)?;
    let mut visual_body_props = VisualBodyProperties { ..default() };
    let mut dynamics_model_properties = DynamicsModelProperties { ..default() };

    for (index, link) in urdf_robot.links.iter().enumerate() {
        // probably it's a drone body
        if link.name.contains("base_") && link.inertial.mass.value != 0.0 {
            visual_body_props.root_body_index = index;
            dynamics_model_properties.ixx = link.inertial.inertia.ixx as f32;
            dynamics_model_properties.iyy = link.inertial.inertia.iyy as f32;
            dynamics_model_properties.izz = link.inertial.inertia.izz as f32;
            dynamics_model_properties.mass = link.inertial.mass.value as f32;
        }

        // probably it's a propeller
        if link.name.contains("prop") {
            let prop_origin = link.inertial.origin.xyz.0;
            visual_body_props.rotor_body_indices.push(index);
            dynamics_model_properties.rotor_positions.push(Vec3::new(
                prop_origin[0] as f32,
                prop_origin[1] as f32,
                prop_origin[2] as f32,
            ));
        }
    }

    Ok((dynamics_model_properties, visual_body_props))
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
            descriptor.thrust_commands = event.thrusts.clone();
        }
    }
}

pub(crate) fn simulate_drone(
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
    for (_, rapier_configuration, rapier_simulation, mut rigid_bodies, _, _) in
        q_rapier_context.iter_mut()
    {
        if !rapier_configuration.physics_pipeline_active {
            continue;
        }

        let start_time = 0.0;
        let end_time = rapier_simulation.integration_parameters.dt as f64;

        for (_, urdf_robot, mut drone) in q_drones.iter_mut() {
            let consts = uav::dynamics::Consts {
                g: rapier_configuration.gravity.length() as f64,
                mass: drone.dynamics_model_props.mass as f64,
                ixx: drone.dynamics_model_props.ixx as f64,
                iyy: drone.dynamics_model_props.iyy as f64,
                izz: drone.dynamics_model_props.izz as f64,
            };

            if drone.thrust_commands.len() == 0 {
                continue;
            }

            // let (forces, torques) = quadrotor_dynamics(thrusts, drone.aerodynamic_props);
            let (forces, torques) = multirotor_dynamics(
                &drone.thrust_commands,
                &drone.aerodynamic_props,
                &drone.dynamics_model_props,
            );

            match uav::dynamics::simulate_drone(
                drone.uav_state,
                consts,
                Vector3::new(forces[0] as f64, forces[1] as f64, forces[2] as f64),
                Vector3::new(torques[0] as f64, torques[1] as f64, torques[2] as f64),
                (start_time, end_time),
                1e-6,
            ) {
                Ok(mut new_state) => {
                    // no collisions yet, fix later

                    if new_state.position_z < 0.0 {
                        new_state.position_z = 0.0;
                        new_state.velocity_z = 0.0;
                    }

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

                        root_body.set_position(position, false);
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

fn multirotor_dynamics(
    thrusts: &[f32],
    aerodynamics_props: &AerodynamicsProperties,
    dynamics_props: &DynamicsModelProperties,
) -> ([f32; 3], [f32; 3]) {
    // Ensure we have the same number of thrusts as rotors
    assert_eq!(thrusts.len(), dynamics_props.rotor_positions.len());

    let torque_to_thrust_ratio = aerodynamics_props.km / aerodynamics_props.kf;
    let f_direction = vector![0.0, 0.0, 1.0];

    // Calculate total force and torque
    let mut total_force = vector![0.0, 0.0, 0.0];
    let mut total_torque_from_thrust = vector![0.0, 0.0, 0.0];
    let mut total_torque_from_drag = vector![0.0, 0.0, 0.0];

    for (i, &thrust) in thrusts.iter().enumerate() {
        let rp = dynamics_props.rotor_positions[i];
        let rotor_position = vector![rp.x, rp.y, rp.z];
        let force_vector = f_direction * thrust;

        // Accumulate total force
        total_force += force_vector;

        // Torque from thrust (position × force)
        let torque_from_thrust = rotor_position.cross(&force_vector);
        total_torque_from_thrust += torque_from_thrust;

        // Torque from rotor drag
        // Assuming alternating rotor directions: even indices CW, odd indices CCW
        let rotor_direction = if i % 2 == 0 { 1.0 } else { -1.0 };
        let torque_from_drag = f_direction * (torque_to_thrust_ratio * thrust * rotor_direction);
        total_torque_from_drag += torque_from_drag;
    }

    // Total torque is the sum of thrust-induced torque and drag torque
    let total_torque = total_torque_from_thrust + total_torque_from_drag;

    (
        [total_force.x, total_force.y, total_force.z],
        [total_torque.x, total_torque.y, total_torque.z],
    )
}
