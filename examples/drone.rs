use bevy::{
    color::palettes::css::WHITE, input::common_conditions::input_toggle_active, prelude::*,
};
use bevy_flycam::prelude::*;
use bevy_inspector_egui::bevy_egui::EguiPlugin;
use bevy_inspector_egui::quick::WorldInspectorPlugin;
use bevy_obj::ObjPlugin;
use bevy_rapier3d::prelude::*;
use bevy_stl::StlPlugin;
use bevy_urdf::events::{ControlMotors, DespawnRobot, LoadRobot, RobotLoaded};
use bevy_urdf::events::{SensorsRead, SpawnRobot};
use bevy_urdf::plugin::UrdfPlugin;
use bevy_urdf::urdf_asset_loader::UrdfAsset;

use rand::Rng;

/// Simulates a URDF robot with Rapier physics in Bevy.
///
/// This example spawns a robot, controls it with random signals for 300 iterations,
/// then despawns a robot and checks that Rapier structures are clean after despawn.
fn main() {
    App::new()
        .add_plugins((
            DefaultPlugins,
            UrdfPlugin,
            StlPlugin,
            ObjPlugin,
            FlyCameraPlugin {
                spawn_camera: false,
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
        .insert_resource(UrdfRobotHandle(None))
        .insert_resource(SimulationStepCounter(0))
        .add_systems(Startup, setup)
        .add_systems(
            Update,
            (
                // control_motors,
                robot_lifecycle,
                // check_rapier_state.after(robot_lifecycle),
            ),
        )
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
        println!(
            "loaded {:?} {:?}",
            event.handle.clone(),
            event.mesh_dir.clone()
        );
        ew_spawn_robot.write(SpawnRobot {
            handle: event.handle.clone(),
            mesh_dir: event.mesh_dir.clone(),
            parent_entity: None,
        });
        state.set(AppState::Simulation);
        commands.insert_resource(UrdfRobotHandle(Some(event.handle.clone())));
    }
}

fn check_rapier_state(
    q_rapier_context: Query<(
        Entity,
        &mut RapierContextSimulation,
        &mut RapierRigidBodySet,
        &mut RapierContextColliders,
        &mut RapierContextJoints,
    )>,
    simulation_step_counter: Res<SimulationStepCounter>,
) {
    if simulation_step_counter.0 == 0
        || simulation_step_counter.0 == 100
        || simulation_step_counter.0 == 300
    {
        println!("----");
        println!("step {}", simulation_step_counter.0);
        for (_, _simulation, rigid_body_set, colliders, joints) in q_rapier_context.iter() {
            let rbl = rigid_body_set.bodies.len();
            let cl = colliders.colliders.len();
            let mbjl = joints.multibody_joints.iter().count();
            let ijl = joints.impulse_joints.len();

            println!("rigid bodies: {}", rbl);
            println!("colliders: {}", cl);
            println!("multibody joints: {}", mbjl);
            println!("impulse joints: {}", ijl);
        }
    }

    if simulation_step_counter.0 == 300 {
        std::process::exit(0x0100);
    }
}

fn robot_lifecycle(
    mut er_read_sensors: EventReader<SensorsRead>,
    mut simulation_step_counter: ResMut<SimulationStepCounter>,
    robot_handle: Res<UrdfRobotHandle>,
    mut er_despawn_robot: EventWriter<DespawnRobot>,
) {
    for event in er_read_sensors.read() {}
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

        ew_control_motors.write(ControlMotors { handle, velocities });
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

    // camera
    commands.spawn((
        Camera3d::default(),
        FlyCam,
        Transform::from_xyz(0.0, 0.18, 0.18).looking_at(Vec3::ZERO, Vec3::Y),
    ));

    // ground
    commands.spawn((
        Mesh3d(meshes.add(Cuboid::new(180., 0.1, 180.))),
        MeshMaterial3d(materials.add(Color::srgb_u8(124, 144, 255))),
        Collider::cuboid(90., 0.05, 90.),
        Transform::from_xyz(0.0, -1.0, 0.0),
        RigidBody::Fixed,
    ));

    // commands.spawn((
    //     Mesh3d(meshes.add(Cuboid::new(1.0, 1.0, 1.0))),
    //     MeshMaterial3d(materials.add(Color::srgb_u8(124, 144, 255))),
    //     Transform::from_xyz(0.0, 0.5, 0.0),
    // ));

    // load robot
    ew_load_robot.send(LoadRobot {
        urdf_path: "drones/crazyflie/cf2x.urdf".to_string(),
        mesh_dir: "assets/drones/crazyflie/".to_string(),
        interaction_groups: None,
        marker: None,
        translation_shift: Some(Vec3::new(0.0, 0.0, 0.0)),
        create_colliders_from_visual_shapes: false,
        create_colliders_from_collision_shapes: true,
    });
}
