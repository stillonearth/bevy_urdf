use actix_web::{middleware::Logger, web, App, HttpResponse, HttpServer, Result};
use rapier3d::prelude::*;
use serde::{Deserialize, Serialize};
use std::sync::Mutex;
use uav::*;

#[derive(Deserialize)]
struct MotorInput {
    motor_thrusts: [f32; 4],
    steps: Option<u32>,
}

#[derive(Serialize)]
struct DroneState {
    position: [f32; 3],
    velocity: [f32; 3],
    rotation: [f32; 3],
    angular_velocity: [f32; 3],
    timestamp: f64,
}

#[derive(Serialize)]
struct ApiResponse {
    success: bool,
    data: Option<DroneState>,
    error: Option<String>,
}

#[derive(Clone, Copy)]
struct PhysicsWorld {
    pub state: State,
    pub consts: Consts,
    current_time: f64,
}

impl PhysicsWorld {
    fn new() -> Result<Self, Box<dyn std::error::Error>> {
        let initial_state = State {
            position_x: 0.0,
            position_y: 0.0,
            position_z: 0.0,
            velocity_x: 0.0,
            velocity_y: 0.0,
            velocity_z: 0.0,
            roll: 0.0,
            pitch: 0.0,
            yaw: 0.0,
            roll_rate: 0.0,
            pitch_rate: 0.0,
            yaw_rate: 0.0,
        };

        let consts = Consts {
            g: 9.81,
            mass: 0.027,
            ixx: 1.4e-5,
            iyy: 1.4e-5,
            izz: 2.17e-5,
        };

        Ok(PhysicsWorld {
            state: initial_state,
            consts,
            current_time: 0.0,
        })
    }

    fn step(&mut self, motor_thrusts: [f32; 4]) -> DroneState {
        let (forces, torques) = self.drone_dynamics(motor_thrusts);

        match simulate_drone(
            self.state,
            self.consts,
            Forces {
                x: forces[0] as f64,
                y: forces[1] as f64,
                z: forces[2] as f64,
            },
            Torques {
                x: torques[0] as f64,
                y: torques[1] as f64,
                z: torques[2] as f64,
            },
            (self.current_time, self.current_time + 1. / 60.),
            1e-6,
        ) {
            Ok(new_state) => {
                println!("{:?}", new_state);

                self.current_time += 1. / 60.;
                println!("time: {}", self.current_time);
                println!("---");
                self.state = new_state;
                return self.convert_uav_state(self.state);
            }
            Err(_e) => todo!(),
        }
    }

    fn drone_dynamics(&self, thrusts: [f32; 4]) -> ([f32; 3], [f32; 3]) {
        let torque_to_thrust_ratio = 7.94e-12 / 3.16e-10;

        let rotor_1_position = vector![0.028, -0.028, 0.0];
        let rotor_2_position = vector![-0.028, -0.028, 0.0];
        let rotor_3_position = vector![-0.028, 0.028, 0.0];
        let rotor_4_position = vector![0.028, 0.028, 0.0];

        let f = vector![0.0, 0.0, 1.0];
        let f1 = f * thrusts[0];
        let f2 = f * thrusts[1];
        let f3 = f * thrusts[2];
        let f4 = f * thrusts[3];

        let full_force = (f1 + f2 + f3 + f4);

        let t1_thrust = (rotor_1_position).cross(&(f1));
        let t1_torque = torque_to_thrust_ratio * (f1);

        let t2_thrust = (rotor_2_position).cross(&(f2));
        let t2_torque = torque_to_thrust_ratio * (f2);

        let t3_thrust = (rotor_3_position).cross(&(f3));
        let t3_torque = torque_to_thrust_ratio * (f3);

        let t4_thrust = (rotor_4_position).cross(&(f4));
        let t4_torque = torque_to_thrust_ratio * (f4);

        let t_thrust = (t1_thrust + t2_thrust + t3_thrust + t4_thrust);
        let t_torque = ((t1_torque - t4_torque) - (t2_torque - t3_torque));

        let torque = t_thrust - t_torque;

        return (
            [full_force.x, full_force.y, full_force.z],
            [torque.x, torque.y, torque.z],
        );
    }

    fn convert_uav_state(self, uav_state: State) -> DroneState {
        return DroneState {
            position: [
                uav_state.position_x as f32,
                uav_state.position_y as f32,
                uav_state.position_z as f32,
            ],
            velocity: [
                uav_state.velocity_x as f32,
                uav_state.velocity_y as f32,
                uav_state.velocity_z as f32,
            ],
            rotation: [
                uav_state.roll as f32,
                uav_state.pitch as f32,
                uav_state.yaw as f32,
            ],
            angular_velocity: [
                uav_state.roll_rate as f32,
                uav_state.pitch_rate as f32,
                uav_state.yaw_rate as f32,
            ],
            timestamp: self.current_time,
        };
    }

    fn get_drone_state(self) -> DroneState {
        return self.convert_uav_state(self.state.clone());
    }
}

// Global state to store single simulation instance
type SimulationState = Mutex<Option<PhysicsWorld>>;

async fn simulate_step(
    data: web::Json<MotorInput>,
    simulation: web::Data<SimulationState>,
) -> Result<HttpResponse> {
    let steps = data.steps.unwrap_or(1);

    let mut simulation = simulation.lock().unwrap();

    // Create simulation instance if it doesn't exist
    if simulation.is_none() {
        match PhysicsWorld::new() {
            Ok(world) => {
                *simulation = Some(world);
            }
            Err(e) => {
                return Ok(HttpResponse::InternalServerError().json(ApiResponse {
                    success: false,
                    data: None,
                    error: Some(format!("Failed to create physics world: {}", e)),
                }));
            }
        }
    }

    let sim = simulation.as_mut().unwrap();

    // Run simulation for specified number of steps
    let mut final_state = None;
    for _ in 0..steps {
        final_state = Some(sim.step(data.motor_thrusts));
    }

    Ok(HttpResponse::Ok().json(ApiResponse {
        success: true,
        data: final_state,
        error: None,
    }))
}

async fn reset_simulation(simulation: web::Data<SimulationState>) -> Result<HttpResponse> {
    let mut simulation = simulation.lock().unwrap();

    match PhysicsWorld::new() {
        Ok(world) => {
            *simulation = Some(world);
            Ok(HttpResponse::Ok().json(ApiResponse {
                success: true,
                data: None,
                error: None,
            }))
        }
        Err(e) => Ok(HttpResponse::InternalServerError().json(ApiResponse {
            success: false,
            data: None,
            error: Some(format!("Failed to reset simulation: {}", e)),
        })),
    }
}

async fn get_simulation_state(simulation: web::Data<SimulationState>) -> Result<HttpResponse> {
    let simulation = simulation.lock().unwrap();

    if let Some(sim) = simulation.as_ref() {
        let state = sim.get_drone_state();
        Ok(HttpResponse::Ok().json(ApiResponse {
            success: true,
            data: Some(state),
            error: None,
        }))
    } else {
        Ok(HttpResponse::NotFound().json(ApiResponse {
            success: false,
            data: None,
            error: Some("Simulation not initialized".to_string()),
        }))
    }
}

#[actix_web::main]
async fn main() -> std::io::Result<()> {
    env_logger::init();

    let simulation: web::Data<SimulationState> = web::Data::new(Mutex::new(None));

    println!("Starting Drone Physics API server on http://localhost:8080");

    HttpServer::new(move || {
        App::new()
            .app_data(simulation.clone())
            .wrap(Logger::default())
            .route("/simulate", web::post().to(simulate_step))
            .route("/reset", web::post().to(reset_simulation))
            .route("/state", web::get().to(get_simulation_state))
    })
    .bind("127.0.0.1:8080")?
    .run()
    .await
}
