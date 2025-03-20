mod events;
mod plugin;
mod urdf_asset_loader;

use bevy::{
    color::palettes::css::WHITE, input::common_conditions::input_toggle_active, prelude::*,
};
use bevy_flycam::*;
use bevy_inspector_egui::quick::WorldInspectorPlugin;
use bevy_rapier3d::prelude::*;
use bevy_stl::StlPlugin;

use crate::events::{handle_spawn_robot, SpawnRobot};
use crate::plugin::UrdfPlugin;
use crate::urdf_asset_loader::UrdfAsset;

fn main() {
    App::new()
        .add_plugins((
            DefaultPlugins,
            UrdfPlugin,
            StlPlugin,
            NoCameraPlayerPlugin,
            RapierPhysicsPlugin::<NoUserData>::default(),
            WorldInspectorPlugin::default().run_if(input_toggle_active(false, KeyCode::Escape)),
        ))
        .init_state::<AppState>()
        .insert_resource(MovementSettings {
            speed: 1.0,
            ..default()
        })
        .add_systems(Startup, (load_robot, setup_scene))
        .add_systems(Update, handle_spawn_robot)
        .add_systems(Update, start_simulation.run_if(in_state(AppState::Loading)))
        .run();
}

fn load_robot(mut commands: Commands, asset_server: Res<AssetServer>) {
    let robot_handle = RobotHandle(asset_server.load("robots/flamingo_edu/urdf/Edu_v4.urdf"));
    commands.insert_resource(robot_handle);
}

fn start_simulation(
    robot_handle: Res<RobotHandle>,
    urdf_assets: Res<Assets<UrdfAsset>>,
    mut er_spawn_robot: EventWriter<SpawnRobot>,
    mut state: ResMut<NextState<AppState>>,
) {
    if urdf_assets.get(robot_handle.id()).is_some() {
        er_spawn_robot.send(SpawnRobot {
            handle: robot_handle.clone(),
        });
        state.set(AppState::Simulation);
    }
}

#[derive(Resource, Deref, DerefMut)]
struct RobotHandle(Handle<UrdfAsset>);

#[derive(Debug, Clone, Copy, Default, Eq, PartialEq, Hash, States)]
enum AppState {
    #[default]
    Loading,
    Simulation,
}

#[allow(deprecated)]
fn setup_scene(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    commands.insert_resource(AmbientLight {
        color: WHITE.into(),
        brightness: 300.0,
    });

    commands.spawn((
        Camera3dBundle {
            transform: Transform::from_xyz(0.0, 2.0, 2.0).looking_at(Vec3::ZERO, Vec3::Y),
            ..default()
        },
        FlyCam,
    ));

    commands.spawn((
        Mesh3d(meshes.add(Cuboid::new(1.0, 1.0, 1.0))),
        MeshMaterial3d(materials.add(Color::srgb_u8(124, 144, 255))),
        Collider::cuboid(0.5, 0.5, 0.5),
        CollisionGroups::new(Group::GROUP_1 | Group::GROUP_2, Group::NONE),
        Transform::from_xyz(0.0, -2.5, 0.0),
        RigidBody::Fixed,
    ));
}
