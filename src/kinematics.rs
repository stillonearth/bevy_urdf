use std::collections::HashMap;

#[derive(Debug, Clone)]
pub struct LinkTransform {
    pub position: Vector3<f32>,
    pub rotation: UnitQuaternion<f32>,
}

use nalgebra::{Point3, Quaternion, UnitQuaternion, Vector3};
use urdf_rs::Vec3;

// reinit_vector3 and reinit_quaternion reinitialize nalgebra types because k and urdf use different versions of nalgebra
pub fn reinit_vector3<T: Copy>(old_vec: &impl AsRef<[T; 3]>) -> Vector3<T> {
    let components = old_vec.as_ref();
    Vector3::new(components[0], components[1], components[2])
}

pub fn reinit_quaternion<T: Copy + nalgebra::RealField>(
    w: T,
    x: T,
    y: T,
    z: T,
) -> UnitQuaternion<T> {
    // Create quaternion from components (w, x, y, z)
    let quat = Quaternion::new(w, x, y, z);
    UnitQuaternion::from_quaternion(quat)
}

pub fn get_link_transforms(
    urdf_robot: &mut urdf_rs::Robot,
    isometry: nalgebra::Isometry<f32, nalgebra::Unit<nalgebra::Quaternion<f32>>, 3>,
) -> Result<HashMap<String, LinkTransform>, k::Error> {
    // Convert URDF to kinematic chain

    let robot: k::Chain<f32> = urdf_robot.clone().into();

    // Compute forward kinematics
    robot.update_transforms();

    let mut transforms = HashMap::new();

    // Get transform for each link
    for (i, link) in robot.iter().enumerate() {
        let world_transform = link.world_transform().unwrap();
        let link_name = &link.link().clone().unwrap().name;

        // Extract position
        let position = reinit_vector3(&world_transform.translation.vector);
        let position = Point3::from(position);
        let position = isometry.transform_point(&position).coords;

        let rotation = reinit_quaternion(
            world_transform.rotation.w,
            world_transform.rotation.i,
            world_transform.rotation.j,
            world_transform.rotation.k,
        );
        let rotation = isometry.rotation * rotation;

        urdf_robot.links[i].inertial.origin.rpy =
            Vec3([position.x as f64, position.y as f64, position.z as f64]);

        transforms.insert(link_name.clone(), LinkTransform { position, rotation });
    }

    Ok(transforms)
}
