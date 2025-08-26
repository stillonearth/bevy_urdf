# bevy_urdf

[![Crates.io](https://img.shields.io/crates/v/bevy_urdf.svg)](https://crates.io/crates/bevy_urdf)
[![Crates.io](https://img.shields.io/crates/d/bevy_urdf.svg)](https://crates.io/crates/bevy_urdf)

A Bevy plugin for importing robots from URDF files and running physics simulations. Ground vehicles use Rapier physics, while drones are simulated using integrated dynamics models.

<img width="300" alt="image" src="https://github.com/user-attachments/assets/bcd6ea71-00e0-4dd6-9f56-761e1c9efe28" />
<img width="300" alt="image" src="https://github.com/user-attachments/assets/8990a53b-812d-4ca7-8040-af2091294e0c" />
<img width="300" alt="image" src="https://github.com/user-attachments/assets/f597ce45-f3cd-4cd7-8656-0b47747a07ae" />

## Features

- Import URDF robot descriptions into Bevy
- Support for STL and OBJ mesh formats
- Rapier physics integration for ground vehicles
- Custom dynamics simulation for drone vehicles
- Event-driven sensor reading and motor control
- Multiple robot type support (ground vehicles, quadrupeds, drones)

## Installation

Add to your `Cargo.toml`:

```toml
[dependencies]
bevy_urdf = "0.4"
```

## Quick Start

### 1. Add Plugins

Add the necessary plugins to your Bevy app:

```rust
use bevy::prelude::*;
use bevy_urdf::{UrdfPlugin, StlPlugin, ObjPlugin, CameraControlPlugin};

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins((
            UrdfPlugin,
            StlPlugin,
            ObjPlugin,
            CameraControlPlugin,
        ))
        .run();
}
```

### 2. Load Robot

Load a robot in your startup systems:

```rust
use bevy_urdf::{LoadRobot, RobotLoaded};

fn setup(mut load_robot_events: EventWriter<LoadRobot>) {
    load_robot_events.send(LoadRobot {
        urdf_path: "robots/flamingo_edu/urdf/Edu_v4.urdf".to_string(),
        mesh_dir: "assets/robots/flamingo_edu/urdf".to_string(),
    });
}
```

### 3. Spawn Robot

Subscribe to the loaded event and spawn the robot:

```rust
use bevy_urdf::{RobotLoaded, SpawnRobot, RobotType};

fn start_simulation(
    mut robot_loaded_events: EventReader<RobotLoaded>,
    mut spawn_robot_events: EventWriter<SpawnRobot>,
    mut next_state: ResMut<NextState<AppState>>,
) {
    for event in robot_loaded_events.read() {
        spawn_robot_events.send(SpawnRobot {
            handle: event.handle.clone(),
            mesh_dir: event.mesh_dir.clone(),
            robot_type: RobotType::NotDrone,
        });
        next_state.set(AppState::Simulation);
    }
}
```

## API Reference

### Events

The plugin uses an event-driven architecture for robot control and sensor feedback:

#### Sensor Reading

```rust
#[derive(Event)]
pub struct SensorsRead {
    pub handle: Handle<UrdfAsset>,
    pub transforms: Vec<Transform>,
    pub joint_angles: Vec<f32>,
}
```

#### Motor Control (Ground Robots)

For ground-based robots and vehicles:

```rust
#[derive(Event)]
pub struct ControlMotorVelocities {
    pub handle: Handle<UrdfAsset>,
    pub velocities: Vec<f32>,
}
```

#### Motor Control (Manipulators)

```rust
#[derive(Event)]
pub struct ControlMotorPositions{
    pub handle: Handle<UrdfAsset>,
    pub positions: Vec<f32>,
    pub motor_props: Vec<MotorProps>,
}
```

#### Thrust Control (Drones)

For aerial vehicles and drones:

```rust
#[derive(Event)]
pub struct ControlThrusts {
    pub handle: Handle<UrdfAsset>,
    pub thrusts: Vec<f32>,
}
```

#### Thruster Control (UUVs)

For underwater vehicles:

```rust
#[derive(Event)]
pub struct ControlThrusters {
    pub handle: Handle<UrdfAsset>,
    pub thrusts: Vec<f32>,
}
```

#### Camera Rotation

Control the viewing angle of any active camera:

```rust
#[derive(Event)]
pub struct RotateCamera {
    pub delta_yaw: f32,
    pub delta_pitch: f32,
}
```

### Robot Types

- `RobotType::NotDrone` - Ground vehicles, quadrupeds, manipulators
- `RobotType::UAV` - Aerial vehicles with thrust-based control
- `RobotType::UUV` - Underwater vehicles with thruster control
- `RobotType::Manipulator` - Robotic manipulators with fixed root and joint position control

## Examples

Run the included examples to see the plugin in action:

```bash
# Drone simulation
cargo run --example uav --release

# Quadruped robot simulation
cargo run --example quadruped --release

# Underwater vehicle simulation
cargo run --example uuv --release

# Manipulator
cargo run --example manipulator --release
```
## URDF Requirements

Before using URDF files with this plugin:

1. **Mesh Paths**: Ensure all mesh file references use relative paths
2. **Unsupported Elements**: Remove or replace `package://` URIs
3. **Gazebo Elements**: Gazebo-specific nodes are not supported and should be removed

Example of supported mesh reference:
```xml
<!-- Good: relative path -->
<mesh filename="meshes/base_link.stl"/>

<!-- Not supported: package URI -->
<mesh filename="package://robot_description/meshes/base_link.stl"/>
```

## Known Limitations

- Package URIs (`package://`) are not supported
- Gazebo-specific URDF elements are ignored
- Drone rotor animations are not yet implemented
- Limited to STL and OBJ mesh formats

## Contributing

Contributions are welcome! Please feel free to submit issues and pull requests.
