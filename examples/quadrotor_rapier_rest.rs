use actix_web::{middleware::Logger, web, App, HttpResponse, HttpServer, Result};
use rapier3d::prelude::*;
use rapier3d_urdf::{UrdfLoaderOptions, UrdfMultibodyOptions, UrdfRobot};
use serde::{Deserialize, Serialize};
use std::sync::Mutex;

#[derive(Deserialize)]
struct MotorInput {
    motor_thrusts: [f32; 4],
    steps: Option<u32>,
}

#[derive(Serialize)]
struct DroneState {
    position: [f32; 3],
    velocity: [f32; 3],
    rotation: [f32; 4], // quaternion [x, y, z, w]
    angular_velocity: [f32; 3],
    timestamp: f64,
}

#[derive(Serialize)]
struct ApiResponse {
    success: bool,
    data: Option<DroneState>,
    error: Option<String>,
}

struct PhysicsWorld {
    integration_parameters: IntegrationParameters,
    physics_pipeline: PhysicsPipeline,
    island_manager: IslandManager,
    broad_phase: DefaultBroadPhase,
    narrow_phase: NarrowPhase,
    impulse_joints: ImpulseJointSet,
    multibody_joints: MultibodyJointSet,
    ccd_solver: CCDSolver,
    query_pipeline: QueryPipeline,
    colliders: ColliderSet,
    bodies: RigidBodySet,
    drone_handle: RigidBodyHandle,
    gravity: Vector<f32>,
    current_time: f64,
}

impl PhysicsWorld {
    fn new() -> Result<Self, Box<dyn std::error::Error>> {
        let gravity = vector![0.0, -9.81, 0.0];
        let mut integration_parameters = IntegrationParameters::default();
        let physics_pipeline = PhysicsPipeline::new();
        let island_manager = IslandManager::new();
        let broad_phase = DefaultBroadPhase::new();
        let narrow_phase = NarrowPhase::new();
        let impulse_joints = ImpulseJointSet::new();
        let mut multibody_joints = MultibodyJointSet::new();
        let ccd_solver = CCDSolver::new();
        let query_pipeline = QueryPipeline::new();
        let mut colliders = ColliderSet::new();
        let mut bodies = RigidBodySet::new();

        // Set step size
        integration_parameters.dt = 1.0 / 100.0;

        // Load robot
        let options = UrdfLoaderOptions {
            create_colliders_from_visual_shapes: false,
            create_colliders_from_collision_shapes: true,
            make_roots_fixed: false,
            // Z-up to Y-up.
            shift: Isometry::rotation(Vector::x() * std::f32::consts::FRAC_PI_2),
            ..Default::default()
        };

        let (drone, _) =
            UrdfRobot::from_file("assets/quadrotors/crazyflie/cf2x.urdf", options, None)
                .map_err(|e| format!("Failed to load URDF: {}", e))?;

        let robot_handle = drone.insert_using_multibody_joints(
            &mut bodies,
            &mut colliders,
            &mut multibody_joints,
            UrdfMultibodyOptions::DISABLE_SELF_CONTACTS,
        );

        let drone_handle = robot_handle.links[0].body;

        Ok(PhysicsWorld {
            integration_parameters,
            physics_pipeline,
            island_manager,
            broad_phase,
            narrow_phase,
            impulse_joints,
            multibody_joints,
            ccd_solver,
            query_pipeline,
            colliders,
            bodies,
            drone_handle,
            gravity,
            current_time: 0.0,
        })
    }

    fn step(&mut self, motor_thrusts: [f32; 4]) -> DroneState {
        let physics_hooks = ();
        let event_handler = ();

        // Apply drone dynamics
        if let Some(drone_body) = self.bodies.clone().get_mut(self.drone_handle) {
            self.drone_dynamics(drone_body, motor_thrusts);
            self.bodies[self.drone_handle] = drone_body.clone();
        }

        // Step physics simulation
        self.physics_pipeline.step(
            &self.gravity,
            &self.integration_parameters,
            &mut self.island_manager,
            &mut self.broad_phase,
            &mut self.narrow_phase,
            &mut self.bodies,
            &mut self.colliders,
            &mut self.impulse_joints,
            &mut self.multibody_joints,
            &mut self.ccd_solver,
            Some(&mut self.query_pipeline),
            &physics_hooks,
            &event_handler,
        );

        self.current_time += self.integration_parameters.dt as f64;

        // Get current state
        self.get_drone_state()
    }

    fn drone_dynamics(&self, drone_center_body: &mut RigidBody, thrusts: [f32; 4]) {
        let torque_to_thrust_ratio = 7.94e-12 / 3.16e-10;

        drone_center_body.reset_forces(true);
        drone_center_body.reset_torques(true);

        let isometry = Isometry::rotation(Vector::x() * std::f32::consts::FRAC_PI_2);
        let inverse = isometry.rotation.inverse();
        let rotation = inverse * drone_center_body.rotation().clone();

        let rotor_1_position = vector![0.28, 0.0, -0.28];
        let rotor_2_position = vector![-0.28, 0.0, -0.28];
        let rotor_3_position = vector![-0.28, 0.0, 0.28];
        let rotor_4_position = vector![0.28, 0.0, 0.28];

        let f = vector![0.0, 1.0, 0.0];
        let f1 = f * thrusts[0];
        let f2 = f * thrusts[1];
        let f3 = f * thrusts[2];
        let f4 = f * thrusts[3];

        drone_center_body.add_force(rotation * (f1 + f2 + f3 + f4), true);

        let t1_thrust = (rotor_1_position).cross(&(rotation * f1));
        let t1_torque = torque_to_thrust_ratio * (rotation * f) * thrusts[0];
        // drone_center_body.add_torque(t1_thrust - t1_torque, true);

        let t2_thrust = (rotor_2_position).cross(&(rotation * f2));
        let t2_torque = torque_to_thrust_ratio * (rotation * f) * thrusts[1];
        // drone_center_body.add_torque(t2_thrust - t2_torque, true);

        let t3_thrust = (rotor_3_position).cross(&(rotation * f3));
        let t3_torque = torque_to_thrust_ratio * (rotation * f) * thrusts[2];
        // drone_center_body.add_torque(t3_thrust + t3_torque, true);

        let t4_thrust = (rotor_4_position).cross(&(rotation * f4));
        let t4_torque = torque_to_thrust_ratio * (rotation * f) * thrusts[3];
        // drone_center_body.add_torque(t4_thrust + t4_torque, true);

        let t_thrust = inverse * (t1_thrust + t2_thrust + t3_thrust + t4_thrust);
        let t_torque = inverse * ((t1_torque - t4_torque) - (t2_torque - t3_torque));

        println!(
            "~~~thrust {}{}{}{}~~~",
            t1_thrust, t2_thrust, t3_thrust, t4_thrust
        );

        println!(
            "~~~torque {}{}{}{}~~~",
            t1_torque, t2_torque, t3_torque, t4_torque
        );

        println!("t_thrust:{} t_torque: {}", t_thrust, t_torque);

        drone_center_body.add_torque(t_thrust + t_torque, true);
    }

    fn get_drone_state(&self) -> DroneState {
        if let Some(drone_body) = self.bodies.get(self.drone_handle) {
            let isometry = Isometry::rotation(Vector::x() * std::f32::consts::FRAC_PI_2);
            let inverse = isometry.rotation.inverse();
            let position = drone_body.position().translation.vector;
            let velocity = drone_body.linvel();
            let rotation = inverse * drone_body.rotation();
            let angular_velocity = drone_body.angvel();

            DroneState {
                position: [position.x, position.y, position.z],
                velocity: [velocity.x, velocity.y, velocity.z],
                rotation: [rotation.w, rotation.i, rotation.j, rotation.k],
                angular_velocity: [angular_velocity.x, angular_velocity.y, angular_velocity.z],
                timestamp: self.current_time,
            }
        } else {
            // Return default state if drone not found
            DroneState {
                position: [0.0, 0.0, 0.0],
                velocity: [0.0, 0.0, 0.0],
                rotation: [1.0, 0.0, 0.0, 0.0],
                angular_velocity: [0.0, 0.0, 0.0],
                timestamp: self.current_time,
            }
        }
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

async fn health_check() -> Result<HttpResponse> {
    Ok(HttpResponse::Ok().json(serde_json::json!({
        "status": "healthy",
        "service": "drone-physics-api"
    })))
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
            .route("/health", web::get().to(health_check))
            .route("/simulate", web::post().to(simulate_step))
            .route("/reset", web::post().to(reset_simulation))
            .route("/state", web::get().to(get_simulation_state))
    })
    .bind("127.0.0.1:8080")?
    .run()
    .await
}
