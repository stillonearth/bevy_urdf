use bevy::{
    color::palettes::css::WHITE, input::common_conditions::input_toggle_active, prelude::*,
};
use bevy_flycam::*;
use bevy_inspector_egui::quick::WorldInspectorPlugin;
use bevy_rapier3d::prelude::*;
use bevy_stl::StlPlugin;
use rapier3d::prelude::InteractionGroups;

use bevy_urdf::events::{ControlMotors, DespawnRobot, LoadRobot, RobotLoaded};
use bevy_urdf::events::{SensorsRead, SpawnRobot};
use bevy_urdf::plugin::UrdfPlugin;
use bevy_urdf::urdf_asset_loader::UrdfAsset;

use rand::Rng;

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
        .insert_resource(SimulationStepCounter(0))
        .add_systems(Startup, setup)
        .add_systems(Update, (control_motors, print_sensor_values))
        .add_systems(Update, start_simulation.run_if(in_state(AppState::Loading)))
        .run();
}

#[derive(Resource)]
struct UrdfRobotHandle(Option<Handle<UrdfAsset>>);

#[derive(Resource)]
struct SimulationStepCounter(usize);

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
            parent_entity: None,
        });
        state.set(AppState::Simulation);
        commands.insert_resource(UrdfRobotHandle(Some(event.handle.clone())));
    }
}

fn print_sensor_values(
    mut er_read_sensors: EventReader<SensorsRead>,
    mut simulation_step_counter: ResMut<SimulationStepCounter>,
    robot_handle: Res<UrdfRobotHandle>,
    mut er_despawn_robot: EventWriter<DespawnRobot>,
) {
    for event in er_read_sensors.read() {
        println!("Step {}", simulation_step_counter.0);
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
        println!("------------------------------------");

        if let Some(_) = robot_handle.0.clone() {
            simulation_step_counter.0 += 1;

            if simulation_step_counter.0 == 300 {
                println!("despawn robot");
                er_despawn_robot.send(DespawnRobot {
                    handle: event.handle.clone(),
                });
            }
        }
    }
}

fn control_motors(
    robot_handle: Res<UrdfRobotHandle>,
    mut ew_control_motors: EventWriter<ControlMotors>,
) {
    if let Some(handle) = robot_handle.0.clone() {
        let mut rng = rand::rng();
        let mut velocities: Vec<f32> = Vec::new();

        for _ in 0..50 {
            velocities.push(rng.random_range(-5.0..5.0));
        }

        ew_control_motors.send(ControlMotors { handle, velocities });
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
            transform: Transform::from_xyz(3.0, 3.0, 3.0).looking_at(Vec3::ZERO, Vec3::Y),
            ..default()
        },
        FlyCam,
    ));

    // ground
    commands.spawn((
        Mesh3d(meshes.add(Cuboid::new(180., 0.1, 180.))),
        MeshMaterial3d(materials.add(Color::srgb_u8(124, 144, 255))),
        Collider::cuboid(90., 0.05, 90.),
        Transform::from_xyz(0.0, -1.0, 0.0),
        RigidBody::Fixed,
    ));

    // load robot

    ew_load_robot.send(LoadRobot {
        urdf_path: "robots/flamingo_edu/urdf/Edu_v4.urdf".to_string(),
        mesh_dir: "assets/robots/flamingo_edu/urdf/".to_string(),
        interaction_groups: None,
        marker: None,
        translation_shift: Some(Vec3::new(0.0, 5.0, 0.0)),
    });
}
