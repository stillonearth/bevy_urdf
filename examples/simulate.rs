use bevy::{
    color::palettes::css::WHITE, input::common_conditions::input_toggle_active, prelude::*,
};
use bevy_flycam::*;
use bevy_inspector_egui::quick::WorldInspectorPlugin;
use bevy_rapier3d::prelude::*;
use bevy_stl::StlPlugin;

use bevy_urdf::events::{LoadRobot, RobotLoaded};
use bevy_urdf::events::{RobotSpawned, URDFRobot, UrdfRobotRigidBodyHandle};
use bevy_urdf::events::{SensorsRead, SpawnRobot};
use bevy_urdf::plugin::UrdfPlugin;
use bevy_urdf::urdf_asset_loader::UrdfAsset;
use rapier3d::prelude::JointEnabled;

fn main() {
    App::new()
        .add_plugins((
            DefaultPlugins,
            UrdfPlugin,
            StlPlugin,
            NoCameraPlayerPlugin,
            RapierPhysicsPlugin::<NoUserData>::default(),
            // RapierDebugRenderPlugin::default(),
            WorldInspectorPlugin::default().run_if(input_toggle_active(false, KeyCode::Escape)),
        ))
        .init_state::<AppState>()
        .insert_resource(MovementSettings {
            speed: 1.0,
            ..default()
        })
        .insert_resource(UrdfRobotHandle(None))
        .add_systems(Startup, setup)
        .add_systems(Update, (control_joint_motors, print_sensor_values))
        .add_systems(Update, start_simulation.run_if(in_state(AppState::Loading)))
        .run();
}

#[derive(Resource)]
struct UrdfRobotHandle(Option<Handle<UrdfAsset>>);

fn start_simulation(
    mut commands: Commands,
    mut er_robot_loaded: EventReader<RobotLoaded>,
    mut ew_spawn_robot: EventWriter<SpawnRobot>,
    mut state: ResMut<NextState<AppState>>,
) {
    for event in er_robot_loaded.read() {
        ew_spawn_robot.send(SpawnRobot {
            handle: event.handle.clone(),
            mesh_dir: event.mesh_dir.clone(),
        });
        state.set(AppState::Simulation);
        commands.insert_resource(UrdfRobotHandle(Some(event.handle.clone())));
    }
}

fn print_sensor_values(mut er_read_sensors: EventReader<SensorsRead>) {
    for event in er_read_sensors.read() {
        println!("Robot: {:?}", event.handle.id());
        println!("\transforms:");
        for transform in &event.transforms {
            let trans = transform.translation;
            let rot = transform.rotation;
            println!(
                "\t{} {} {} {} {} {} {}",
                trans.x, trans.y, trans.z, rot.x, rot.y, rot.z, rot.w
            );
        }

        let joint_angles_string: Vec<String> =
            event.joint_angles.iter().map(|a| a.to_string()).collect();
        println!("\tjoint_angles:");
        println!("\t{}", joint_angles_string.join(" "));
    }
}

fn control_joint_motors(
    mut q_rapier_joints: Query<(&mut RapierContextJoints,)>,
    urdf_assets: Res<Assets<UrdfAsset>>,
    robot_handle: Res<UrdfRobotHandle>,
) {
    if let Some(handle) = robot_handle.0.clone() {
        if let Some(urdf_asset) = urdf_assets.get(handle.id()) {
            for mut rapier_context_joints in q_rapier_joints.iter_mut() {
                let urdf_robot = urdf_asset.urdf_robot.clone();
                for joint in urdf_robot.joints {
                    if let Some(revolute) = joint.joint.as_revolute() {
                        if let Some(motor) = revolute.motor() {
                            println!("motor value {:?}", motor);
                        }
                        // println!("joint: {:?}",)
                    }

                    // println!("joint value {:?}", joint.as
                    // if joint.joint.enabled == JointEnabled::Enabled {
                    //     for (handle, link, _, _) in rapier_context_joints.0.multibody_joints.iter()
                    //     {
                    //         link.
                    //     }
                    // }
                }
            }
        }
    }
}

#[derive(Debug, Clone, Copy, Default, Eq, PartialEq, Hash, States)]
enum AppState {
    #[default]
    Loading,
    Simulation,
}

#[allow(deprecated)]
fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut ew_load_robot: EventWriter<LoadRobot>,
) {
    // Scene
    commands.insert_resource(AmbientLight {
        color: WHITE.into(),
        brightness: 300.0,
    });

    commands.spawn((
        Camera3dBundle {
            transform: Transform::from_xyz(2.0, 2.0, 2.0).looking_at(Vec3::ZERO, Vec3::Y),
            ..default()
        },
        FlyCam,
    ));

    // ground
    commands.spawn((
        Mesh3d(meshes.add(Cuboid::new(180., 1.8, 180.))),
        MeshMaterial3d(materials.add(Color::srgb_u8(124, 144, 255))),
        Collider::cuboid(90., 0.9, 90.),
        Transform::from_xyz(0.0, -2.5, 0.0),
        RigidBody::Fixed,
    ));

    // load robot
    // ew_load_robot.send(LoadRobot {
    //     urdf_path: "robots/unitree_a1/urdf/a1.urdf".to_string(),
    //     mesh_dir: "assets/robots/unitree_a1/urdf".to_string(),
    // });

    ew_load_robot.send(LoadRobot {
        urdf_path: "robots/flamingo_edu/urdf/Edu_v4.urdf".to_string(),
        mesh_dir: "assets/robots/flamingo_edu/urdf".to_string(),
    });
}
