# bevy_urdf
<img src="https://github.com/user-attachments/assets/1ec9c5e8-0de1-4f25-a69a-4cfba12cd8ae" width="100">

Import robots from URDF files and run simulation with rapier. Drones are simulated by integrating dynamics model.

## API 

1. *Add Urdf and Stl plugins to bevy app*

```rust
UrdfPlugin,
StlPlugin,
ObjPlugin,
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
            robot_type: RobotType::NotDrone,
        });
        state.set(AppState::Simulation);
    }
}
```

### Events

Read sensors and control motors through events:


```rust

#[derive(Event)]
pub struct SensorsRead {
    pub handle: Handle<UrdfAsset>,
    pub transforms: Vec<Transform>,
    pub joint_angles: Vec<f32>,
}

// for ground robots / cars
#[derive(Event)]
pub struct ControlMotors {
    pub handle: Handle<UrdfAsset>,
    pub velocities: Vec<f32>,
}

// for ground drones
#[derive(Event)]
pub struct ControlThrusts {
    pub handle: Handle<UrdfAsset>,
    pub thrusts: Vec<f32>,
}
```


## Limitations

You may need to hand-inspect urdf files to ensure mesh links are relative paths. `package://` and links and gazebo nodes are not supported.
Drone rotors aren't yet animated.

## Examples

```bash
cargo run --example quadrotor --release # drone
cargo run --example quadruped --release # robot-dog
```