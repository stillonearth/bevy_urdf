use rapier3d::prelude::*;
use rapier3d_urdf::{UrdfLoaderOptions, UrdfMultibodyOptions, UrdfRobot};

pub fn main() {
    /*
     * World
     */
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let mut impulse_joints = ImpulseJointSet::new();
    let mut multibody_joints = MultibodyJointSet::new();

    /*
     * Ground
     */
    let options = UrdfLoaderOptions {
        create_colliders_from_visual_shapes: true,
        create_colliders_from_collision_shapes: false,
        make_roots_fixed: true,
        // Z-up to Y-up.
        shift: Isometry::rotation(Vector::x() * std::f32::consts::FRAC_PI_2),
        ..Default::default()
    };

    let (mut urdf_robot, robot) =
        UrdfRobot::from_file("assets/drones/crazyflie/cf2x.urdf", options, None).unwrap();

    // robot.

    println!("{:?}", urdf_robot);

    // // The robot can be inserted using impulse joints.
    // // (We clone because we want to insert the same robot once more afterward.)
    // robot
    //     .clone()
    //     .insert_using_impulse_joints(&mut bodies, &mut colliders, &mut impulse_joints);
    // // Insert the robot a second time, but using multibody joints this time.
    // robot.append_transform(&Isometry::translation(10.0, 0.0, 0.0));
    // robot.insert_using_multibody_joints(
    //     &mut bodies,
    //     &mut colliders,
    //     &mut multibody_joints,
    //     UrdfMultibodyOptions::DISABLE_SELF_CONTACTS,
    // );
}
