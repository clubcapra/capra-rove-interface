use std::sync::Arc;
use std::time::Duration;

mod core;
mod drivers;
mod http;
mod protocol;
mod udp;

use core::registry::SensorRegistry;
use drivers::gps::GpsSensor;
use drivers::motor_controller::MotorController;
use drivers::odrive::{discover_nodes, endpoints::SharedEndpointMap, node::WatchdogConfig};
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
    let registry = Arc::new(SensorRegistry::new(5000));

    //registry.register(GpsSensor::new());

    // --- ODrive Discovery ---
    // Scans the CAN bus for heartbeat frames for 2 seconds.
    // Each discovered node gets its own UDP ports and Scalar endpoints.
    let odrive_iface = std::env::var("CAN_IFACE").unwrap_or_else(|_| "can0".to_string());
    let watchdog = WatchdogConfig::default(); // 100ms, setpoint-only keepalive

    let shared_endpoints: SharedEndpointMap = match discover_nodes(&odrive_iface, Duration::from_secs(2), watchdog).await {
        Ok((nodes, ep_map)) => {
            for node in nodes {
                registry.register(node);
            }
            ep_map
        }
        Err(e) => {
            tracing::warn!(error = %e, iface = odrive_iface, "ODrive discovery failed — continuing without ODrives");
            drivers::odrive::endpoints::new_shared()
        }
    };

    // --- Start UDP listeners ---
    for (id, driver) in registry.iter_drivers() {
        let (data_port, cmd_port) = registry.ports(&id).unwrap();
        spawn_sensor_udp(driver, data_port, cmd_port).await?;
    }

    // --- Start HTTP server with Scalar UI ---
    let app = build_router(registry.clone(), shared_endpoints);
    let listener = tokio::net::TcpListener::bind("0.0.0.0:8080").await?;

    tracing::info!("Scalar UI:   http://localhost:8080/docs");
    tracing::info!("OpenAPI:     http://localhost:8080/openapi.json");
    tracing::info!("Discover:    http://localhost:8080/discover");
    tracing::info!("{} sensors registered", registry.len());

    axum::serve(listener, app).await?;

    Ok(())
}
