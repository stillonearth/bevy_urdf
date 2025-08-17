use bevy::{
    color::palettes::css::WHITE, input::common_conditions::input_toggle_active, prelude::*,
};
use bevy_infinite_grid::{InfiniteGridBundle, InfiniteGridPlugin};
use bevy_inspector_egui::bevy_egui::EguiPlugin;
use bevy_inspector_egui::quick::WorldInspectorPlugin;
use bevy_panorbit_camera::*;
use bevy_rapier3d::prelude::*;
use bevy_stl::StlPlugin;
use bevy_urdf::control::ControlMotorPositions;
use bevy_urdf::control::MotorProps;
use bevy_urdf::plugin::RobotType;
use bevy_urdf::plugin::UrdfPlugin;
use bevy_urdf::spawn::{LoadRobot, RapierOption, RobotLoaded, SpawnRobot};
use bevy_urdf::urdf_asset_loader::UrdfAsset;

use rapier3d::prelude::Group;
use rapier3d::prelude::InteractionGroups;

use rand::Rng;

#[derive(Resource)]
struct UrdfRobotHandle(Option<Handle<UrdfAsset>>);

fn main() {
    App::new()
        .add_plugins((
            DefaultPlugins,
            UrdfPlugin::default(),
            StlPlugin,
            PanOrbitCameraPlugin,
            RapierPhysicsPlugin::<NoUserData>::default(),
            EguiPlugin {
                enable_multipass_for_primary_context: true,
            },
            InfiniteGridPlugin,
            WorldInspectorPlugin::default().run_if(input_toggle_active(false, KeyCode::Escape)),
        ))
        .init_state::<AppState>()
        .insert_resource(ClearColor(Color::linear_rgb(1.0, 1.0, 1.0)))
        .insert_resource(UrdfRobotHandle(None))
        .add_systems(Startup, setup)
        .add_systems(Update, start_simulation.run_if(in_state(AppState::Loading)))
        .add_systems(Update, (control_motors,))
        .run();
}

fn start_simulation(
    mut commands: Commands,
    mut er_robot_loaded: EventReader<RobotLoaded>,
    mut ew_spawn_robot: EventWriter<SpawnRobot>,
    mut state: ResMut<NextState<AppState>>,
) {
    for event in er_robot_loaded.read() {
        ew_spawn_robot.write(SpawnRobot {
            handle: event.handle.clone(),
            mesh_dir: event.mesh_dir.clone(),
            parent_entity: None,
            robot_type: RobotType::Manipulator,
            drone_descriptor: None,
        });
        state.set(AppState::Simulation);
        commands.insert_resource(UrdfRobotHandle(Some(event.handle.clone())));
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
    // Scene
    commands.insert_resource(AmbientLight {
        color: WHITE.into(),
        brightness: 300.0,
        ..default()
    });

    // ground
    commands.spawn((
        InfiniteGridBundle {
            transform: Transform::from_xyz(0.0, -1.0, 0.0),
            ..default()
        },
        RigidBody::Fixed,
        Collider::cuboid(900., 0.05, 900.),
    ));

    // camera
    commands.spawn((
        Camera3d { ..default() },
        Transform::from_xyz(-0.019, 0.32, -0.793).looking_at(Vec3::ZERO, Vec3::Y),
        PanOrbitCamera {
            focus: Vec3::new(0.0, 0.0, 0.0),
            ..Default::default()
        },
    ));

    // load robot
    ew_load_robot.send(LoadRobot {
        robot_type: RobotType::Manipulator,
        urdf_path: "manipulators/so-101/so101_new_calib.urdf".to_string(),
        mesh_dir: "assets/manipulators/so-101/".to_string(),
        rapier_options: RapierOption {
            interaction_groups: None,
            translation_shift: None,
            create_colliders_from_visual_shapes: false,
            create_colliders_from_collision_shapes: true,
            make_roots_fixed: true,
        },
        marker: None,
        drone_descriptor: None,
    });
}

fn control_motors(
    robot_handle: Res<UrdfRobotHandle>,
    mut ew_control_motors: EventWriter<ControlMotorPositions>,
) {
    if let Some(handle) = robot_handle.0.clone() {
        let mut rng = rand::rng();
        let mut positions: Vec<f32> = Vec::new();
        let mut motor_props: Vec<MotorProps> = Vec::new();

        for _ in 0..50 {
            positions.push(0.0); // rng.random_range(-5.0..5.0));
            motor_props.push(MotorProps {
                stiffness: 17.8,
                damping: 0.6,
            });
        }

        ew_control_motors.write(ControlMotorPositions {
            handle,
            positions,
            motor_props,
        });
    }
}
