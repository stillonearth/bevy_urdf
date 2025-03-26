use bevy::{color::palettes::css::WHITE, input::common_conditions::input_toggle_active, prelude::*};
use bevy_flycam::*;
use bevy_inspector_egui::quick::WorldInspectorPlugin;
use bevy_rapier3d::prelude::*;
use bevy_stl::StlPlugin;
use bevy_urdf::{events::*, plugin::UrdfPlugin, urdf_asset_loader::UrdfAsset};
use rand::Rng;

const DBG: bool = false;

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
        .init_state::<S>()
        .insert_resource(MovementSettings { speed: 1.0, ..default() })
        .insert_resource(R(None))
        .add_systems(Startup, setup)
        .add_systems(Update, ctrl)
        .add_systems(Update, print.run_if(|| DBG))
        .add_systems(Update, start.run_if(in_state(S::L)))
        .run();
}

#[derive(Resource)]
struct R(Option<Handle<UrdfAsset>>);

fn start(
    mut c: Commands,
    mut er: EventReader<RobotLoaded>,
    mut ew: EventWriter<SpawnRobot>,
    mut s: ResMut<NextState<S>>,
    cams: Query<Entity, With<Camera3d>>
) {
    for e in er.read() {
        ew.send(SpawnRobot { handle: e.handle.clone(), mesh_dir: e.mesh_dir.clone() });
        s.set(S::Sim);
        c.insert_resource(R(Some(e.handle.clone())));
    }
    for cam in cams.iter() {
        c.entity(cam).despawn();
    }
    c.spawn((
        Camera3d::default(),
        Transform::from_xyz(0.0, 2.0, 5.0).looking_at(Vec3::ZERO, Vec3::Y),
        FlyCam
    ));
}

fn print(mut er: EventReader<SensorsRead>) {
    for e in er.read() {
        println!("Robot: {:?}", e.handle.id());
        println!("\transforms:");
        for t in &e.transforms {
            let tr = t.translation;
            let r = t.rotation;
            println!("\t{} {} {} {} {} {} {}", tr.x, tr.y, tr.z, r.x, r.y, r.z, r.w);
        }
        println!("\tjoint_angles:\n\t{}", e.joint_angles.iter().map(|a| a.to_string()).collect::<Vec<_>>().join(" "));
    }
}

fn ctrl(r: Res<R>, mut ew: EventWriter<ControlMotors>) {
    if let Some(h) = r.0.clone() {
        let mut rng = rand::rng();
        ew.send(ControlMotors { handle: h, velocities: (0..6).map(|_| rng.random_range(-5.0..5.0)).collect() });
    }
}

#[derive(Debug, Clone, Copy, Default, Eq, PartialEq, Hash, States)]
enum S {
    #[default]
    L,
    Sim,
}

#[allow(deprecated)]
fn setup(
    mut c: Commands,
    mut m: ResMut<Assets<Mesh>>,
    mut mat: ResMut<Assets<StandardMaterial>>,
    mut ew: EventWriter<LoadRobot>,
) {
    c.insert_resource(AmbientLight { color: WHITE.into(), brightness: 300.0 });
    c.spawn(Camera3dBundle::default());
    c.spawn((
        Mesh3d(m.add(Cuboid::new(180., 0.1, 180.))),
        MeshMaterial3d(mat.add(Color::srgb_u8(124, 144, 255))),
        Collider::cuboid(90., 0.05, 90.),
        Transform::from_xyz(0.0, -5.0, 0.0),
        RigidBody::Fixed,
    ));
    ew.send(LoadRobot {
        urdf_path: "robots/unitree_a1/urdf/a1.urdf".to_string(),
        mesh_dir: "assets/robots/unitree_a1/urdf".to_string(),
        interaction_groups: None,
    });
}