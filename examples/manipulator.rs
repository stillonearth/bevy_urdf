use bevy::input::common_conditions::input_toggle_active;
use bevy::{color::palettes::css::WHITE, prelude::*};
use bevy_infinite_grid::{InfiniteGridBundle, InfiniteGridPlugin};
use bevy_inspector_egui::bevy_egui::EguiPlugin;
use bevy_inspector_egui::bevy_egui::{egui, EguiContextPass, EguiContexts};
use bevy_inspector_egui::quick::WorldInspectorPlugin;
use bevy_panorbit_camera::*;
use bevy_rapier3d::prelude::*;
use bevy_stl::StlPlugin;
use bevy_urdf::control::{ControlMotorPositions, MotorProps};
use bevy_urdf::plugin::{RobotType, URDFRobot};
use bevy_urdf::spawn::{LoadRobot, RapierOption, RobotLoaded, SpawnRobot};
use bevy_urdf::urdf_asset_loader::UrdfAsset;
use bevy_urdf::URDFPlugin;
use std::f32::consts::PI;

#[derive(Resource)]
struct UrdfRobotHandle(Option<Handle<UrdfAsset>>);

struct MotorAngle {
    angle: f32,
    upper_limit: f32,
    lower_limit: f32,
    name: String,
}

#[derive(Resource, Default)]
struct MotorAngles {
    angles: Vec<MotorAngle>,
    initialized: bool,
}

#[derive(Default, Resource, Clone, Debug, PartialEq)]
struct LastJointData {
    angles: Vec<f32>,
}

fn main() {
    App::new()
        .add_plugins((
            DefaultPlugins,
            URDFPlugin {
                default_system_setup: false,
                ..default()
            },
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
        .insert_resource(MotorAngles::default())
        .insert_resource(LastJointData::default())
        .add_systems(Startup, setup)
        .add_systems(Update, start_simulation.run_if(in_state(AppState::Loading)))
        .add_systems(
            Update,
            (
                URDFPlugin::<NoUserData>::get_systems(PhysicsSet::SyncBackend)
                    .in_set(PhysicsSet::SyncBackend),
                URDFPlugin::<NoUserData>::get_systems(PhysicsSet::StepSimulation)
                    .in_set(PhysicsSet::StepSimulation),
                URDFPlugin::<NoUserData>::get_systems(PhysicsSet::Writeback)
                    .in_set(PhysicsSet::Writeback),
            ),
        )
        .add_systems(
            Update,
            (
                initialize_motors,
                control_motors.after(PhysicsSet::Writeback),
            )
                .run_if(in_state(AppState::Simulation)),
        )
        .add_systems(EguiContextPass, motor_ui.before(control_motors))
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
            uav_descriptor: None,
            uuv_descriptor: None,
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
            button_orbit: MouseButton::Right,
            button_pan: MouseButton::Right,
            modifier_pan: Some(KeyCode::ShiftLeft),
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
        uav_descriptor: None,
        uuv_descriptor: None,
    });
}

fn initialize_motors(
    mut motor_angles: ResMut<MotorAngles>,
    q_urdf_robots: Query<&URDFRobot>,
    urdf_assets: Res<Assets<UrdfAsset>>,
) {
    if motor_angles.initialized {
        return;
    }

    for urdf_robot in q_urdf_robots.iter() {
        if let Some(urdf_asset) = urdf_assets.get(&urdf_robot.handle) {
            motor_angles.angles.clear();

            for (i, joint) in urdf_asset.robot.joints.iter().enumerate() {
                match &joint.joint_type {
                    urdf_rs::JointType::Revolute
                    | urdf_rs::JointType::Continuous
                    | urdf_rs::JointType::Prismatic => {
                        let limit = &joint.limit;
                        let mut lower = limit.lower as f32;
                        let mut upper = limit.upper as f32;

                        if (lower - upper).abs() < 0.001 {
                            lower = -PI;
                            upper = PI;
                        }

                        let angle: f32 = match i {
                            1 => 0.0, // (-0.174533 + 1.74533) / 2.0,
                            2 => (-2.74385 + 2.84121) / 2.0,
                            3 => (-1.65806 + 1.65806) / 2.0,
                            4 => (-1.69 + 1.69) / 2.0,
                            5 => (-1.74533 + 1.74533) / 2.0,
                            6 => 3.14, //(-1.41986 - 1.11986) / 2.0 + 1.0,
                            _ => 1.0,
                        };
                        info!("{} {} rad(type: {:?})", joint.name, angle, joint.joint_type);
                        motor_angles.angles.push(MotorAngle {
                            angle,
                            lower_limit: lower,
                            upper_limit: upper,
                            name: joint.name.clone(),
                        })
                    }
                    _ => {
                        info!(
                            "Skipping joint: {} (type: {:?})",
                            joint.name, joint.joint_type
                        );
                        continue;
                    }
                }
            }

            motor_angles.initialized = true;
            break;
        }
    }
}

fn motor_ui(mut contexts: EguiContexts, mut motor_angles: ResMut<MotorAngles>) {
    if !motor_angles.initialized || motor_angles.angles.is_empty() {
        return;
    }

    if let Some(ctx) = contexts.try_ctx_mut() {
        let mut angles_changed = false;

        egui::Window::new("Motor Control")
            .default_pos(egui::pos2(20.0, 20.0))
            .default_size(egui::vec2(400.0, 500.0))
            .resizable(true)
            .collapsible(true)
            .show(ctx, |ui| {
                let motor_count = motor_angles.angles.len();
                ui.heading(format!("Motor Control ({} motors)", motor_count));

                for MotorAngle {
                    name,
                    lower_limit,
                    upper_limit,
                    angle,
                } in motor_angles.angles.iter_mut()
                {
                    let min_limit = *lower_limit;
                    let max_limit = *upper_limit;
                    let old_value = *angle;

                    ui.horizontal(|ui| {
                        ui.label(format!("{}: ", name));
                        let response = ui.add(
                            egui::Slider::new(angle, min_limit..=max_limit)
                                .text("rad")
                                .step_by(0.01),
                        );
                        ui.label(format!("{:.3}", angle));

                        if response.changed() || (old_value - *angle).abs() > 0.001 {
                            angles_changed = true;
                        }
                    });
                }

                if angles_changed {
                    motor_angles.set_changed();
                }
            });
    }
}

fn control_motors(
    robot_handle: Res<UrdfRobotHandle>,
    mut ew_control_motors: EventWriter<ControlMotorPositions>,
    motor_angles: Res<MotorAngles>,
    mut last_joint_data: ResMut<LastJointData>,
) {
    if !motor_angles.initialized || motor_angles.angles.is_empty() {
        return;
    }

    if let Some(handle) = robot_handle.0.clone() {
        let current_angles = motor_angles
            .angles
            .iter()
            .map(|a| a.angle)
            .collect::<Vec<_>>();
        let should_send = {
            if last_joint_data.angles != current_angles {
                last_joint_data.angles = current_angles.clone();
                true
            } else {
                false
            }
        };

        if should_send {
            let positions = current_angles;
            let motor_props = vec![
                MotorProps {
                    stiffness: 17.8,
                    damping: 0.6,
                };
                positions.len()
            ];

            ew_control_motors.write(ControlMotorPositions {
                handle,
                positions,
                motor_props,
            });
        }
    }
}
