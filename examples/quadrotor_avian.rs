use avian3d::prelude::*;
use bevy::input::common_conditions::input_toggle_active;
use bevy::prelude::*;
use bevy_inspector_egui::bevy_egui::EguiPlugin;
use bevy_inspector_egui::quick::WorldInspectorPlugin;
use bevy_obj::ObjPlugin;
use bevy_rl::{
    AIGymPlugin, AIGymSettings, AIGymState, EventControl, EventPause, EventReset, SimulationState,
};
use serde::{Deserialize, Serialize};

#[derive(Component)]
struct Quadcopter;

#[derive(Event)]
struct ControlMotors {
    pub thrusts: [f32; 4],
}

fn spawn_quadcopter(
    commands: &mut Commands,
    meshes: &mut ResMut<Assets<Mesh>>,
    materials: &mut ResMut<Assets<StandardMaterial>>,
    asset_server: &Res<AssetServer>,
    q_quadcopter: Query<(Entity, &Quadcopter)>,
) {
    for (entity, _) in q_quadcopter.iter() {
        commands.entity(entity).despawn();
    }

    commands
        .spawn((
            RigidBody::Dynamic,
            Mass(0.027),
            Collider::cylinder(0.06, 0.025),
            Mesh3d(meshes.add(Cylinder::new(0.06, 0.025))),
            MeshMaterial3d(materials.add(Color::linear_rgba(1.0, 0.2, 0.3, 0.5))),
            Transform::from_xyz(0.0, 0.1, 0.0),
            Name::new("Quadcopter"),
            ExternalForce::new(Vec3::ZERO).with_persistence(false),
            ExternalTorque::new(Vec3::ZERO).with_persistence(false),
            CenterOfMass::new(0.0, 0.0, 0.0),
            Friction::new(1.0),
            // AngularInertia::new(Vec3::new(1. / 1.4e-5, 1. / 2.17e-15, 1. / 1.4e-5)),
            Quadcopter,
            NoAutoMass,
            NoAutoCenterOfMass,
        ))
        .with_children(|spawner| {
            spawner.spawn((
                Mesh3d(asset_server.load("quadrotors/crazyflie/body.obj")),
                MeshMaterial3d(materials.add(Color::srgb(0.3, 0.4, 0.3))),
                Transform::IDENTITY
                    .with_rotation(Quat::from_rotation_x(-std::f32::consts::FRAC_PI_2)),
            ));
        });
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    asset_server: Res<AssetServer>,
    mut simulation_state: ResMut<NextState<SimulationState>>,
    q_quadcopter: Query<(Entity, &Quadcopter)>,
) {
    // Static physics object with a collision shape
    commands.spawn((
        RigidBody::Static,
        Collider::cylinder(4.0, 0.1),
        Mesh3d(meshes.add(Cylinder::new(4.0, 0.1))),
        MeshMaterial3d(materials.add(Color::WHITE)),
    ));

    spawn_quadcopter(
        &mut commands,
        &mut meshes,
        &mut materials,
        &asset_server,
        q_quadcopter,
    );

    // Light
    commands.spawn((
        PointLight {
            shadows_enabled: true,
            ..default()
        },
        Transform::from_xyz(4.0, 8.0, 4.0),
    ));

    // Camera
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(-2.5, 4.5, 9.0).looking_at(Vec3::ZERO, Dir3::Y),
    ));

    simulation_state.set(SimulationState::Running);
}

// -------
// physics
// -------

fn quadcopter_dynamics(
    mut er_motor_thrusts: EventReader<ControlMotors>,
    mut q_quadcopter: Query<(
        Entity,
        &mut ExternalForce,
        &mut ExternalTorque,
        &Transform,
        &Quadcopter,
    )>,
) {
    for event in er_motor_thrusts.read() {
        for (_, mut external_force, mut external_torque, drone_transform, _) in
            q_quadcopter.iter_mut()
        {
            let torque_to_thrust_ratio = 7.94e-12 / 3.16e-10;

            let rotation = drone_transform.rotation.clone();

            let rotor_1_position = Vec3::new(0.028, 0.0, -0.028);
            let rotor_2_position = Vec3::new(-0.028, 0.0, -0.028);
            let rotor_3_position = Vec3::new(-0.028, 0.0, 0.028);
            let rotor_4_position = Vec3::new(0.028, 0.0, 0.028);

            let f = Vec3::new(0.0, 1.0, 0.0);
            let f1 = f * event.thrusts[0];
            let f2 = f * event.thrusts[1];
            let f3 = f * event.thrusts[2];
            let f4 = f * event.thrusts[3];

            let full_force = rotation * (f1 + f2 + f3 + f4);

            *external_force = ExternalForce::new(full_force).with_persistence(false);

            let t1_thrust = (rotor_1_position).cross(f1);
            let t1_torque = torque_to_thrust_ratio * (f1);

            let t2_thrust = (rotor_2_position).cross(f2);
            let t2_torque = torque_to_thrust_ratio * (f2);

            let t3_thrust = (rotor_3_position).cross(f3);
            let t3_torque = torque_to_thrust_ratio * (f3);

            let t4_thrust = (rotor_4_position).cross(f4);
            let t4_torque = torque_to_thrust_ratio * (f4);

            let t_thrust = rotation * (t1_thrust + t2_thrust + t3_thrust + t4_thrust);
            let t_torque = rotation * ((t1_torque - t4_torque) - (t2_torque - t3_torque));

            *external_torque = ExternalTorque::new(t_thrust - t_torque).with_persistence(false);
        }
    }
}

// -------
// bevy_rl
// -------

#[derive(Default, Deref, DerefMut, Clone, Deserialize)]
pub struct Actions([f32; 4]);

#[derive(Default, Serialize, Clone)]
pub struct EnvironmentState {
    pub position: Vec3,
    pub velocity: Vec3,
    pub omega: Vec3,
    pub attitude: Vec4,
}

fn sync_bevy_rl_and_avian(
    bevy_rl_state: Res<State<SimulationState>>,
    mut time: ResMut<Time<Physics>>,
) {
    match **bevy_rl_state {
        SimulationState::Initializing => {
            time.pause();
        }
        SimulationState::Running => {
            time.unpause();
        }
        SimulationState::PausedForControl => {
            time.pause();
        }
    };
}

fn handle_control_request(
    mut er_control: EventReader<EventControl>,
    mut simulation_state: ResMut<NextState<SimulationState>>,
    mut ew_control_motors: EventWriter<ControlMotors>,
) {
    for control in er_control.read() {
        let raw_actions = control.0.clone();

        for i in 0..raw_actions.len() {
            if let Some(unparsed_action) = raw_actions[i].clone() {
                let thrusts: [f32; 4] = serde_json::from_str(&unparsed_action).unwrap();
                ew_control_motors.write(ControlMotors { thrusts });

                break;
            }
        }

        simulation_state.set(SimulationState::Running);
    }
}

fn handle_pause_event(
    mut er_pause: EventReader<EventPause>,
    ai_gym_state: Res<AIGymState<Actions, EnvironmentState>>,
    q_quadcopter: Query<(
        Entity,
        &Transform,
        &LinearVelocity,
        &AngularVelocity,
        &Quadcopter,
    )>,
    mut simulation_state: ResMut<NextState<SimulationState>>,
) {
    for _ in er_pause.read() {
        simulation_state.set(SimulationState::PausedForControl);

        let mut ai_gym_state = ai_gym_state.lock().unwrap();

        for (_, transform, velocity, omega, _) in q_quadcopter.iter() {
            let environment_state = EnvironmentState {
                position: transform.translation,
                velocity: velocity.0,
                omega: omega.0,
                attitude: Vec4::new(
                    transform.rotation.x,
                    transform.rotation.y,
                    transform.rotation.z,
                    transform.rotation.w,
                ),
            };

            ai_gym_state.set_env_state(environment_state.clone());
            ai_gym_state.set_reward(0, 0.0);
            break;
        }

        ai_gym_state.send_step_result(vec![true]);
    }
}

fn handle_reset_event(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    asset_server: Res<AssetServer>,
    mut er_reset: EventReader<EventReset>,
    ai_gym_state: Res<AIGymState<Actions, EnvironmentState>>,
    q_quadcopter: Query<(Entity, &Quadcopter)>,
    mut simulation_state: ResMut<NextState<SimulationState>>,
) {
    for _ in er_reset.read() {
        spawn_quadcopter(
            &mut commands,
            &mut meshes,
            &mut materials,
            &asset_server,
            q_quadcopter,
        );

        let ai_gym_state = ai_gym_state.lock().unwrap();

        ai_gym_state.send_reset_result(true);
        simulation_state.set(SimulationState::Running);
    }
}

// ---
// app
// ---

fn main() {
    App::new()
        .insert_resource(AIGymState::<Actions, EnvironmentState>::new(
            AIGymSettings {
                num_agents: 1,
                render_to_buffer: false,
                pause_interval: 1. / 60.,
                ..default()
            },
        ))
        .add_plugins((
            DefaultPlugins,
            ObjPlugin,
            PhysicsPlugins::default(),
            EguiPlugin {
                enable_multipass_for_primary_context: true,
            },
            AIGymPlugin::<Actions, EnvironmentState>::default(),
            WorldInspectorPlugin::default().run_if(input_toggle_active(false, KeyCode::Escape)),
        ))
        .insert_resource(Gravity(Vec3::NEG_Y * 1.0))
        .add_systems(Startup, setup)
        .add_systems(Update, (quadcopter_dynamics))
        .add_systems(
            Update,
            (
                sync_bevy_rl_and_avian,
                handle_control_request,
                handle_pause_event,
                handle_reset_event,
            ),
        )
        .add_event::<ControlMotors>()
        .run();
}
