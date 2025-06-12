use bevy::{
    color::palettes::css::WHITE, input::common_conditions::input_toggle_active, prelude::*,
};
use bevy_flycam::prelude::*;
use bevy_infinite_grid::{InfiniteGridBundle, InfiniteGridPlugin};
use bevy_inspector_egui::bevy_egui::EguiPlugin;
use bevy_inspector_egui::quick::WorldInspectorPlugin;
use bevy_obj::ObjPlugin;
use bevy_rapier3d::prelude::*;
use bevy_stl::StlPlugin;
use nalgebra::{UnitQuaternion, Vector3};

use bevy_urdf::drones::{ControlThrusts, DroneDescriptor};
use bevy_urdf::events::{LoadRobot, RobotLoaded};
use bevy_urdf::events::{RobotType, SpawnRobot};
use bevy_urdf::plugin::UrdfPlugin;
use bevy_urdf::urdf_asset_loader::UrdfAsset;

fn main() {
    let mut controller = uav::control::QuadcopterController::new(
        1. / 60.,
        0.027,
        1.4e-5,
        1.4e-5,
        2.17e-5,
        0.3,
        12.0,
        5.0,
        5.0,
        0.03,
        0.26477955 / 4. * 0.8,
        0.26477955 / 4. * 1.5,
    );

    controller.set_gains(
        Vector3::new(25., 25., 5.),
        2.0,
        6.,
        12.,
        2.0,
        8.0,
        1.0,
        8.0,
        1.0,
    );

    App::new()
        .add_plugins((
            DefaultPlugins,
            UrdfPlugin,
            StlPlugin,
            ObjPlugin,
            FlyCameraPlugin {
                spawn_camera: true,
                grab_cursor_on_startup: true,
            },
            RapierPhysicsPlugin::<NoUserData>::default(),
            EguiPlugin {
                enable_multipass_for_primary_context: true,
            },
            InfiniteGridPlugin,
            WorldInspectorPlugin::default().run_if(input_toggle_active(false, KeyCode::Escape)),
        ))
        .init_state::<AppState>()
        .insert_resource(MovementSettings {
            move_speed: Vec3::ONE * 3.0,
            ..default()
        })
        .insert_resource(MouseSettings {
            invert_horizontal: false,
            invert_vertical: false,
            mouse_sensitivity: 0.00012,
            lock_cursor_to_middle: false,
        })
        .insert_resource(QuadcopterController(controller))
        .insert_resource(ClearColor(Color::linear_rgb(1.0, 1.0, 1.0)))
        .insert_resource(UrdfRobotHandle(None))
        .add_systems(Startup, setup)
        .add_systems(Update, (control_thrusts))
        .add_systems(Update, start_simulation.run_if(in_state(AppState::Loading)))
        .run();
}

#[derive(Resource)]
struct QuadcopterController(uav::control::QuadcopterController);

#[derive(Resource)]
struct UrdfRobotHandle(Option<Handle<UrdfAsset>>);

#[derive(Component)]
struct Crazyflie;

fn start_simulation(
    mut commands: Commands,
    mut er_robot_loaded: EventReader<RobotLoaded>,
    mut ew_spawn_robot: EventWriter<SpawnRobot>,
    mut state: ResMut<NextState<AppState>>,
    q_crazflie: Query<(Entity, &Crazyflie)>,
) {
    for event in er_robot_loaded.read() {
        ew_spawn_robot.write(SpawnRobot {
            handle: event.handle.clone(),
            mesh_dir: event.mesh_dir.clone(),
            parent_entity: Some(q_crazflie.iter().last().unwrap().0),
            robot_type: RobotType::Drone,
        });
        state.set(AppState::Simulation);
        commands.insert_resource(UrdfRobotHandle(Some(event.handle.clone())));
    }
}

fn control_thrusts(
    robot_handle: Res<UrdfRobotHandle>,
    mut ew_control_motors: EventWriter<ControlThrusts>,
    mut controller: ResMut<QuadcopterController>,
    q_drone: Query<(Entity, &DroneDescriptor)>,
) {
    if let Some(handle) = robot_handle.0.clone() {
        for (_, drone_descriptor) in q_drone.iter() {
            let uav_state: uav::dynamics::State = drone_descriptor.uav_state;

            let t_pos = Vector3::new(-5., -5., 10.);
            let t_vel = Vector3::new(0.0, 0.0, 0.0);
            let t_acc = Vector3::new(0.0, 0.0, 0.0);
            let t_att = UnitQuaternion::identity();

            let est_pos = Vector3::new(
                uav_state.position_x,
                uav_state.position_y,
                uav_state.position_z,
            );
            let est_vel = Vector3::new(
                uav_state.velocity_x,
                uav_state.velocity_y,
                uav_state.velocity_z,
            );
            let est_omega = Vector3::new(
                -uav_state.roll_rate,
                -uav_state.pitch_rate,
                uav_state.yaw_rate,
            );
            let est_att =
                UnitQuaternion::from_euler_angles(uav_state.roll, uav_state.pitch, uav_state.yaw);

            let thrusts = controller.0.run_control(
                t_pos, t_vel, t_acc, t_att, est_pos, est_vel, est_omega, est_att,
            );

            // change order or rotors in commands
            ew_control_motors.write(ControlThrusts {
                handle: handle.clone(),
                thrusts: vec![
                    thrusts[0] as f32,
                    thrusts[2] as f32,
                    thrusts[3] as f32,
                    thrusts[1] as f32,
                ],
            });
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
fn setup(mut commands: Commands, mut ew_load_robot: EventWriter<LoadRobot>) {
    // scene
    commands.insert_resource(AmbientLight {
        color: WHITE.into(),
        brightness: 300.0,
        ..default()
    });

    commands.spawn((Crazyflie, Name::new("drone")));

    // ground
    commands.spawn((
        InfiniteGridBundle {
            transform: Transform::from_xyz(0.0, -1.0, 0.0),
            ..default()
        },
        RigidBody::Fixed,
        Collider::cuboid(900., 0.05, 900.),
        Name::new("ground"),
    ));

    // load robot
    ew_load_robot.send(LoadRobot {
        create_colliders_from_collision_shapes: true,
        create_colliders_from_visual_shapes: false,
        interaction_groups: None,
        marker: None,
        mesh_dir: "assets/quadrotors/crazyflie/".to_string(),
        translation_shift: None,
        urdf_path: "quadrotors/crazyflie/cf2x.urdf".to_string(),
    });
}
