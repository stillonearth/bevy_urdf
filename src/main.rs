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
use events::{handle_load_robot, handle_wait_robot_loaded, LoadRobot, RobotLoaded};
use rapier3d::prelude::ColliderBuilder;

use crate::events::{handle_spawn_robot, SpawnRobot};
use crate::plugin::UrdfPlugin;

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
        .add_systems(Startup, (setup))
        .add_systems(
            Update,
            (
                handle_spawn_robot,
                handle_load_robot,
                handle_wait_robot_loaded,
            ),
        )
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
    mut q_rapier_context: Query<(
        Entity,
        &mut RapierRigidBodySet,
        &mut RapierContextColliders,
        &mut RapierContextJoints,
    )>,
) {
    // Scene
    commands.insert_resource(AmbientLight {
        color: WHITE.into(),
        brightness: 300.0,
    });

    commands.spawn((
        Camera3dBundle {
            transform: Transform::from_xyz(1.0, 1.0, 1.0).looking_at(Vec3::ZERO, Vec3::Y),
            ..default()
        },
        FlyCam,
    ));

    // ground
    // commands.spawn((
    //     Mesh3d(meshes.add(Cuboid::new(1.0, 1.0, 1.0))),
    //     MeshMaterial3d(materials.add(Color::srgb_u8(124, 144, 255))),
    //     Collider::cuboid(0.5, 0.5, 0.5),
    //     Transform::from_xyz(0.0, -2.5, 0.0),
    //     RigidBody::Fixed,
    // ));

    commands.spawn((
        Mesh3d(meshes.add(Cuboid::new(1.0, 1.0, 1.0))),
        MeshMaterial3d(materials.add(Color::srgba(0.2, 0.7, 0.1, 0.1))),
        Collider::cuboid(0.5, 0.5, 0.5),
        Transform::from_xyz(0.0, 2.5, 0.0),
        RigidBody::Dynamic,
    ));

    for (_entity, mut rigid_body_set, mut collider_set, mut multibidy_joint_set) in
        q_rapier_context.iter_mut()
    {
        let collider = ColliderBuilder::cuboid(100.0, 0.1, 100.0).build();
        // collider_set.colliders.insert(collider);
    }

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
