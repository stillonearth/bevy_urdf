use bevy::{
    color::palettes::css::WHITE, input::common_conditions::input_toggle_active, prelude::*,
};
use bevy_flycam::prelude::*;
use bevy_inspector_egui::bevy_egui::EguiPlugin;
use bevy_inspector_egui::quick::WorldInspectorPlugin;
use bevy_obj::ObjPlugin;
use bevy_rapier3d::prelude::*;
use bevy_stl::StlPlugin;
use bevy_urdf::drones::ControlThrusts;
use bevy_urdf::events::{LoadRobot, RobotLoaded};
use bevy_urdf::events::{RobotType, SpawnRobot};
use bevy_urdf::plugin::UrdfPlugin;
use bevy_urdf::urdf_asset_loader::UrdfAsset;

fn main() {
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
        .insert_resource(ClearColor(Color::linear_rgb(1.0, 1.0, 1.0)))
        .insert_resource(UrdfRobotHandle(None))
        .add_systems(Startup, setup)
        .add_systems(Update, control_thrusts)
        .add_systems(Update, start_simulation.run_if(in_state(AppState::Loading)))
        .run();
}

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
) {
    if let Some(handle) = robot_handle.0.clone() {
        let t1 = 0.027 * 9.81 / 4. * 1.0;
        let t2 = 0.027 * 9.81 / 4. * 1.0;
        let t3 = 0.027 * 9.81 / 4. * 1.0;
        let t4 = 0.027 * 9.81 / 4. * 1.0;

        ew_control_motors.write(ControlThrusts {
            handle,
            thrusts: vec![t1, t3, t4, t2],
        });
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
    // scene
    commands.insert_resource(AmbientLight {
        color: WHITE.into(),
        brightness: 300.0,
        ..default()
    });

    commands.spawn((Crazyflie));

    // ground
    commands.spawn((
        Mesh3d(meshes.add(Cuboid::new(180., 0.1, 180.))),
        MeshMaterial3d(materials.add(Color::srgb_u8(124, 144, 255))),
        Collider::cuboid(90., 0.05, 90.),
        Transform::from_xyz(0.0, 0.0, 0.0),
        RigidBody::Fixed,
    ));

    // load robot
    ew_load_robot.send(LoadRobot {
        urdf_path: "quadrotors/crazyflie/cf2x.urdf".to_string(),
        mesh_dir: "assets/quadrotors/crazyflie/".to_string(),
        interaction_groups: None,
        marker: None,
        translation_shift: None,
        create_colliders_from_visual_shapes: false,
        create_colliders_from_collision_shapes: true,
    });

    println!("end setup");
}
