use std::sync::Arc;
use std::time::Duration;

use serde_json::Value;
use tokio_util::sync::CancellationToken;

use crate::core::driver::SensorDriver;

/// Continuously re-sends a command to a driver at the given interval.
///
/// Used for CAN bus sensors with hardware watchdogs that require
/// periodic command frames to keep the device active.
pub async fn run_stream_loop(
    driver: Arc<dyn SensorDriver>,
    payload: Value,
    interval: Duration,
    cancel: CancellationToken,
) {
    let sensor_id = driver.id();
    tracing::info!(sensor = sensor_id, ?interval, "stream loop started");

    let mut ticker = tokio::time::interval(interval);

    loop {
        tokio::select! {
            _ = ticker.tick() => {
                match driver.execute_command(&payload) {
                    Ok(_) => {}
                    Err(e) => {
                        tracing::warn!(sensor = sensor_id, error = %e, "stream command failed");
                    }
                }
            }
            _ = cancel.cancelled() => {
                tracing::info!(sensor = sensor_id, "stream loop stopped");
                break;
            }
        }
    }
}
