use rapier3d::prelude::*;
use rapier3d_urdf::{UrdfLoaderOptions, UrdfMultibodyOptions, UrdfRobot};

pub fn main() {
    let gravity = vector![0.0, -9.81, 0.0];
    let mut integration_parameters = IntegrationParameters::default();
    let mut physics_pipeline = PhysicsPipeline::new();
    let mut island_manager = IslandManager::new();
    let mut broad_phase = DefaultBroadPhase::new();
    let mut narrow_phase = NarrowPhase::new();
    let mut impulse_joints = ImpulseJointSet::new();
    let mut multibody_joints = MultibodyJointSet::new();
    let mut ccd_solver = CCDSolver::new();
    let mut query_pipeline = QueryPipeline::new();
    let mut colliders = ColliderSet::new();
    let mut bodies = RigidBodySet::new();
    let physics_hooks = ();
    let event_handler = ();

    // set step size
    integration_parameters.dt = 1.0 / 60.0;

    // load robot
    let options = UrdfLoaderOptions {
        create_colliders_from_visual_shapes: false,
        create_colliders_from_collision_shapes: true,
        make_roots_fixed: false,
        // Z-up to Y-up.
        shift: Isometry::rotation(Vector::x() * std::f32::consts::FRAC_PI_2),
        ..Default::default()
    };

    let (drone, _) =
        UrdfRobot::from_file("assets/quadrotors/crazyflie/cf2x.urdf", options, None).unwrap();

    let robot_handle = drone.insert_using_multibody_joints(
        &mut bodies,
        &mut colliders,
        &mut multibody_joints,
        UrdfMultibodyOptions::DISABLE_SELF_CONTACTS,
    );
    let drone_handle = robot_handle.links[0].body;

    // return;
    /* Run the game loop, stepping the simulation once per frame. */
    for i in 0..999 {
        physics_pipeline.step(
            &gravity,
            &integration_parameters,
            &mut island_manager,
            &mut broad_phase,
            &mut narrow_phase,
            &mut bodies,
            &mut colliders,
            &mut impulse_joints,
            &mut multibody_joints,
            &mut ccd_solver,
            Some(&mut query_pipeline),
            &physics_hooks,
            &event_handler,
        );

        let drone_center_body = bodies.get_mut(drone_handle).unwrap();
        drone_dynamics(drone_center_body, [0.09, 0.09, 0.09, 0.09]);
    }
}

fn drone_dynamics(drone_center_body: &mut RigidBody, thrusts: [f32; 4]) {
    let torque_to_thrust_ratio = 7.94e-12 / 3.16e-10;

    drone_center_body.reset_forces(false);
    drone_center_body.reset_torques(false);

    let isometry = Isometry::rotation(Vector::x() * std::f32::consts::FRAC_PI_2);

    let rotation = isometry.rotation.inverse() * drone_center_body.rotation().clone();

    let rotor_1_position = vector![0.28, 0.0, -0.28];
    let rotor_2_position = vector![-0.28, 0.0, -0.28];
    let rotor_3_position = vector![-0.28, 0.0, 0.28];
    let rotor_4_position = vector![0.28, 0.0, 0.28];

    let f = vector![0.0, 1.0, 0.0];

    let f1 = rotation * f * thrusts[0];
    let f2 = rotation * f * thrusts[1];
    let f3 = rotation * f * thrusts[2];
    let f4 = rotation * f * thrusts[3];

    drone_center_body.add_force(f1 + f2 + f3 + f4, true);

    let mut t1 = (rotation * rotor_1_position).cross(&f1);
    t1 += (rotation * vector![0.0, 1.0, 0.0]) * (torque_to_thrust_ratio * thrusts[0]);
    drone_center_body.add_torque(t1, true);

    let mut t2 = (rotation * rotor_2_position).cross(&f2);
    t2 -= (rotation * vector![0.0, 1.0, 0.0]) * (torque_to_thrust_ratio * thrusts[1]);
    drone_center_body.add_torque(t2, true);

    let mut t3 = (rotation * rotor_3_position).cross(&f3);
    t3 += (rotation * vector![0.0, 1.0, 0.0]) * (torque_to_thrust_ratio * thrusts[2]);
    drone_center_body.add_torque(t3, true);

    let mut t4 = (rotation * rotor_4_position).cross(&f4);
    t4 -= (rotation * vector![0.0, 1.0, 0.0]) * (torque_to_thrust_ratio * thrusts[3]);
    drone_center_body.add_torque(t4, true);

    let position = drone_center_body.position().translation.clone();

    println!("{} {} {}", position.x, position.y, position.z);
}
