use bevy::{
    color::palettes::css::WHITE, input::common_conditions::input_toggle_active, prelude::*,
};
use bevy_flycam::*;
use bevy_inspector_egui::quick::WorldInspectorPlugin;
use bevy_rapier3d::prelude::*;
use bevy_stl::StlPlugin;

use bevy_urdf_rapier::events::SpawnRobot;
use bevy_urdf_rapier::events::{LoadRobot, RobotLoaded};
use bevy_urdf_rapier::plugin::UrdfPlugin;

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
        .add_systems(Startup, setup)
        .add_systems(Update, start_simulation.run_if(in_state(AppState::Loading)))
        .run();
}

fn start_simulation(
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
        Mesh3d(meshes.add(Cuboid::new(1.8, 1.8, 1.8))),
        MeshMaterial3d(materials.add(Color::srgb_u8(124, 144, 255))),
        Collider::cuboid(0.9, 0.9, 0.9),
        Transform::from_xyz(0.0, -2.5, 0.0),
        RigidBody::Fixed,
    ));

    commands.spawn((
        Mesh3d(meshes.add(Cuboid::new(1.8, 1.8, 1.8))),
        MeshMaterial3d(materials.add(Color::srgba(0.2, 0.7, 0.1, 0.1))),
        Collider::cuboid(0.9, 0.9, 0.9),
        Transform::from_xyz(0.0, 2.5, 0.0),
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
