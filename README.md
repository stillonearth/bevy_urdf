# bevy_urdf_rapier

Import robots from URDF files and run simulation with rapier.

## API 

1. *Add Urdf and Stl plugins to bevy app*

```rust
UrdfPlugin,
StlPlugin,
```

2. *Load robot in your startup systems*

```rust

fn setup(mut ew_load_robot: EventWriter<LoadRobot>,) {
...
ew_load_robot.send(LoadRobot {
    urdf_path: "robots/flamingo_edu/urdf/Edu_v4.urdf".to_string(),
    mesh_dir: "assets/robots/flamingo_edu/urdf".to_string(),
});
...
}
```

3. *Subscribe to loaded event and emit spawn event*

```rust
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
```

## Limitations

You may need to hand-inspect urdf files to ensure mesh links are relative to urdf file. `package://` and links and gazebo nodes are not supported.

## Work in Progress

Support for controlling actuators and reading sensors in coming soon. 