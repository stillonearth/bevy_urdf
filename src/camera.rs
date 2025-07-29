use bevy::prelude::*;

#[derive(Event, Default, Clone, Copy)]
pub struct RotateCamera {
    pub delta_yaw: f32,
    pub delta_pitch: f32,
}

pub struct CameraControlPlugin;

impl Plugin for CameraControlPlugin {
    fn build(&self, app: &mut App) {
        app.add_event::<RotateCamera>()
            .add_systems(Update, apply_camera_rotation);
    }
}

fn apply_camera_rotation(
    mut query: Query<&mut Transform, With<Camera>>,
    mut events: EventReader<RotateCamera>,
) {
    let mut total_yaw = 0.0;
    let mut total_pitch = 0.0;
    for ev in events.read() {
        total_yaw += ev.delta_yaw;
        total_pitch += ev.delta_pitch;
    }
    if total_yaw != 0.0 || total_pitch != 0.0 {
        for mut transform in query.iter_mut() {
            transform.rotate_y(total_yaw);
            transform.rotate_local_x(total_pitch);
        }
    }
}
