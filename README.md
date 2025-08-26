# bevy_urdf

[![Crates.io](https://img.shields.io/crates/v/bevy_urdf.svg)](https://crates.io/crates/bevy_urdf)
[![Crates.io](https://img.shields.io/crates/d/bevy_urdf.svg)](https://crates.io/crates/bevy_urdf)

A Bevy plugin for importing robots from URDF files and running physics simulations. Ground vehicles use Rapier physics, while drones are simulated using integrated dynamics models.

<img width="300" alt="image" src="https://github.com/user-attachments/assets/bcd6ea71-00e0-4dd6-9f56-761e1c9efe28" />
<img width="300" alt="image" src="https://github.com/user-attachments/assets/8990a53b-812d-4ca7-8040-af2091294e0c" />
<img width="300" alt="image" src="https://github.com/user-attachments/assets/f597ce45-f3cd-4cd7-8656-0b47747a07ae" />

# Bevy URDF Plugin

A Bevy plugin for loading and simulating robots from URDF files.

## Installation

```toml
[dependencies]
bevy_urdf = "0.4"
```

## Setup

```rust
use bevy::prelude::*;
use bevy_urdf::{UrdfPlugin, StlPlugin, ObjPlugin};

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins((UrdfPlugin, StlPlugin, ObjPlugin))
        .run();
}
```

## Loading Robots

```rust
use bevy_urdf::{LoadRobot, RobotLoaded, SpawnRobot, RobotType};

fn setup(mut load_robot_events: EventWriter<LoadRobot>) {
    load_robot_events.send(LoadRobot {
        urdf_path: "robots/robot.urdf".to_string(),
        mesh_dir: "assets/robots".to_string(),
    });
}

fn spawn_robot(
    mut robot_loaded_events: EventReader<RobotLoaded>,
    mut spawn_robot_events: EventWriter<SpawnRobot>,
) {
    for event in robot_loaded_events.read() {
        spawn_robot_events.send(SpawnRobot {
            handle: event.handle.clone(),
            mesh_dir: event.mesh_dir.clone(),
            robot_type: RobotType::NotDrone, // or UAV, UUV, Manipulator
        });
    }
}
```

## Control Events

**Motor Velocities** (ground robots)
```rust
ControlMotorVelocities {
    handle: robot_handle,
    velocities: vec![1.0, -1.0, 0.5],
}
```

**Motor Positions** (manipulators)
```rust
ControlMotorPositions {
    handle: robot_handle,
    positions: vec![0.5, 1.2, -0.3],
    motor_props: motor_properties,
}
```

**Thrusts** (drones)
```rust
ControlThrusts {
    handle: robot_handle,
    thrusts: vec![10.0, 10.0, 10.0, 10.0],
}
```

**Sensor Reading**
```rust
SensorsRead {
    handle: robot_handle,
    transforms: link_transforms,
    joint_angles: current_angles,
}
```

## URDF Requirements

- Use relative mesh paths: `meshes/part.stl`
- Remove `package://` URIs
- Remove Gazebo-specific elements
- Support STL and OBJ mesh formats only

## Examples

```bash
cargo run --example uav --release
cargo run --example quadruped --release
cargo run --example manipulator --release
```
