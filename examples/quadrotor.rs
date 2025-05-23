use std::collections::HashMap;
use std::error::Error;

use bevy::{
    color::palettes::css::WHITE, input::common_conditions::input_toggle_active, prelude::*,
};
use bevy_flycam::prelude::*;
use bevy_inspector_egui::bevy_egui::EguiPlugin;
use bevy_inspector_egui::quick::WorldInspectorPlugin;
use bevy_obj::ObjPlugin;
use bevy_rapier3d::prelude::*;
use bevy_stl::StlPlugin;
use bevy_urdf::events::SpawnRobot;
use bevy_urdf::events::{LoadRobot, RobotLoaded};
use bevy_urdf::plugin::UrdfPlugin;
use bevy_urdf::urdf_asset_loader::UrdfAsset;

use bevy_urdf::quadrotor::{ControlThrusts, PropellerDirection, URDFDrone};

#[derive(Resource)]
pub struct MotorCommands(Vec<Vec<f32>>);

fn read_motor_commands() -> Result<Vec<Vec<f32>>, Box<dyn Error>> {
    // Build the CSV reader and iterate over each record.
    let rdr = csv::ReaderBuilder::new()
        .delimiter(b',')
        .from_path("assets/motors.csv");

    let mut recs: Vec<Vec<f32>> = Vec::new();
    for result in rdr.unwrap().records() {
        let record = result?;

        let mut rec: Vec<f32> = Vec::new();

        rec.push(record[0].parse::<f32>().unwrap());
        rec.push(record[1].parse::<f32>().unwrap());
        rec.push(record[2].parse::<f32>().unwrap());
        rec.push(record[3].parse::<f32>().unwrap());

        recs.push(rec);
    }
    Ok(recs)
}

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
        .insert_resource(MotorCommands(read_motor_commands().unwrap()))
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
        .insert_resource(SimulationStepCounter(0))
        .add_systems(Startup, setup)
        .add_systems(Update, (control_thrusts))
        .add_systems(Update, start_simulation.run_if(in_state(AppState::Loading)))
        .run();
}

#[derive(Resource)]
struct UrdfRobotHandle(Option<Handle<UrdfAsset>>);

#[derive(Resource)]
struct SimulationStepCounter(usize);

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
        });
        state.set(AppState::Simulation);
        commands.insert_resource(UrdfRobotHandle(Some(event.handle.clone())));
    }
}

fn control_thrusts(
    robot_handle: Res<UrdfRobotHandle>,
    mut ew_control_motors: EventWriter<ControlThrusts>,
    mut simulation_step_counter: ResMut<SimulationStepCounter>,
    motor_commands: Res<MotorCommands>,
) {
    if let Some(handle) = robot_handle.0.clone() {
        let step = simulation_step_counter.0 as usize;
        if motor_commands.0.len() <= step {
            std::process::exit(0x0100);
        }
        let motors = &motor_commands.0[step];

        let t1 = (motors[0] * 0.0 + 0.027 * 9.81 / 4.) * -1.0;
        let t2 = (motors[1] * 0.0 + 0.027 * 9.81 / 4.) * 1.0;
        let t3 = (motors[2] * 0.0 + 0.027 * 9.81 / 4.) * 1.0;
        let t4 = (motors[3] * 0.0 + 0.027 * 9.81 / 4.) * 1.0;

        // println!("total force {}", t1 + t2 + t3 + t4);

        ew_control_motors.write(ControlThrusts {
            handle,
            thrusts: vec![t1, t3, t4, t2],
        });

        simulation_step_counter.0 += 1;
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
        Transform::from_xyz(0.0, 3.0, 3.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));

    let mut drone_definition = URDFDrone {
        propellers: HashMap::new(),
    };
    drone_definition
        .propellers
        .insert(1, PropellerDirection::CW);
    drone_definition
        .propellers
        .insert(2, PropellerDirection::CCW);
    drone_definition
        .propellers
        .insert(3, PropellerDirection::CW);
    drone_definition
        .propellers
        .insert(4, PropellerDirection::CCW);

    // crazyflie parent entity
    commands.spawn((Crazyflie, drone_definition));

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
        translation_shift: Some(Vec3::new(0.0, 0.02, 0.0)),
        create_colliders_from_visual_shapes: false,
        create_colliders_from_collision_shapes: true,
    });
}
