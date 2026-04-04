use std::sync::Mutex;

use serde::{Deserialize, Serialize};
use serde_json::Value;

use crate::core::driver::{CommandMode, FieldDescriptor, SensorDriver};
use crate::core::error::DriverError;

/// Example REST-style sensor: a GNSS receiver.
///
/// Demonstrates the template pattern for simple request/response sensors.
/// Replace the simulated data with real serial/I2C reads for production.
pub struct GpsSensor {
    last_fix: Mutex<GpsFix>,
}

#[derive(Debug, Clone, Serialize, Deserialize, Default)]
struct GpsFix {
    latitude: f64,
    longitude: f64,
    altitude_m: f64,
    speed_mps: f64,
    satellites: u8,
}

#[derive(Debug, Deserialize)]
struct GpsCommand {
    #[serde(default)]
    set_update_rate_hz: Option<u8>,
}

impl GpsSensor {
    pub fn new() -> Self {
        Self {
            last_fix: Mutex::new(GpsFix {
                latitude: 45.5017,
                longitude: -73.5673,
                altitude_m: 35.0,
                speed_mps: 0.0,
                satellites: 12,
            }),
        }
    }
}

impl SensorDriver for GpsSensor {
    fn id(&self) -> &'static str {
        "gps"
    }

    fn display_name(&self) -> &'static str {
        "GNSS Receiver"
    }

    fn data_schema(&self) -> Vec<FieldDescriptor> {
        vec![
            FieldDescriptor::new("latitude", "WGS84 latitude", "f64").with_unit("degrees"),
            FieldDescriptor::new("longitude", "WGS84 longitude", "f64").with_unit("degrees"),
            FieldDescriptor::new("altitude_m", "Altitude above mean sea level", "f64")
                .with_unit("m"),
            FieldDescriptor::new("speed_mps", "Ground speed", "f64").with_unit("m/s"),
            FieldDescriptor::new("satellites", "Number of satellites in fix", "u8"),
        ]
    }

    fn command_schema(&self) -> Vec<FieldDescriptor> {
        vec![FieldDescriptor::new(
            "set_update_rate_hz",
            "Set the GPS update rate",
            "u8",
        )
        .with_unit("Hz")]
    }

    fn command_mode(&self) -> CommandMode {
        CommandMode::Rest
    }

    fn read_data(&self) -> Result<Value, DriverError> {
        let fix = self.last_fix.lock().unwrap();
        Ok(serde_json::to_value(&*fix)?)
    }

    fn execute_command(&self, payload: &Value) -> Result<Value, DriverError> {
        let cmd: GpsCommand = serde_json::from_value(payload.clone())?;

        if let Some(rate) = cmd.set_update_rate_hz {
            tracing::info!(rate_hz = rate, "GPS update rate changed");
            Ok(serde_json::json!({
                "status": "ok",
                "update_rate_hz": rate
            }))
        } else {
            Ok(serde_json::json!({"status": "no-op"}))
        }
    }
}
