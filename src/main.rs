use std::sync::Arc;

mod core;
mod drivers;
mod http;
mod protocol;
mod udp;

use core::registry::SensorRegistry;
use drivers::gps::GpsSensor;
use drivers::motor_controller::MotorController;
use http::routes::build_router;
use udp::server::spawn_sensor_udp;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| "capra_rove_interface=info".into()),
        )
        .init();

    // --- Sensor Registration ---
    // To add a new sensor:
    //   1. Create src/drivers/my_sensor.rs implementing SensorDriver
    //   2. Add `pub mod my_sensor;` to src/drivers/mod.rs
    //   3. Register it here:

    let registry = Arc::new(SensorRegistry::new(5000));

    registry.register(GpsSensor::new());
    registry.register(MotorController::new("motor_fl", "Front-Left Motor"));
    registry.register(MotorController::new("motor_fr", "Front-Right Motor"));
    registry.register(MotorController::new("motor_rl", "Rear-Left Motor"));
    registry.register(MotorController::new("motor_rr", "Rear-Right Motor"));

    // --- Start UDP listeners ---
    for (id, driver) in registry.iter_drivers() {
        let (data_port, cmd_port) = registry.ports(&id).unwrap();
        spawn_sensor_udp(driver, data_port, cmd_port).await?;
    }

    // --- Start HTTP server with Scalar UI ---
    let app = build_router(registry.clone());
    let listener = tokio::net::TcpListener::bind("0.0.0.0:8080").await?;

    tracing::info!("Scalar UI:   http://localhost:8080/docs");
    tracing::info!("OpenAPI:     http://localhost:8080/openapi.json");
    tracing::info!("Discover:    http://localhost:8080/discover");
    tracing::info!("{} sensors registered", registry.len());

    axum::serve(listener, app).await?;

    Ok(())
}
