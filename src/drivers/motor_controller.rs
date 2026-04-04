use std::sync::Mutex;
use std::time::Duration;

use serde::{Deserialize, Serialize};
use serde_json::Value;

use crate::core::driver::{CommandMode, FieldDescriptor, SensorDriver};
use crate::core::error::DriverError;

/// Example stream-style sensor: a CAN bus motor controller.
///
/// Demonstrates the template pattern for sensors with hardware watchdogs.
/// The framework automatically re-sends commands at the configured interval
/// after a StreamStart is received over UDP.
pub struct MotorController {
    name: &'static str,
    id: &'static str,
    last_telemetry: Mutex<MotorTelemetry>,
}

#[derive(Debug, Clone, Serialize, Deserialize, Default)]
struct MotorTelemetry {
    rpm: f64,
    current_amps: f64,
    temperature_c: f64,
    fault_code: u16,
}

#[derive(Debug, Deserialize)]
struct MotorCommand {
    target_rpm: f64,
    enable: bool,
}

impl MotorController {
    pub fn new(id: &'static str, name: &'static str) -> Self {
        Self {
            name,
            id,
            last_telemetry: Mutex::new(MotorTelemetry {
                rpm: 0.0,
                current_amps: 0.0,
                temperature_c: 25.0,
                fault_code: 0,
            }),
        }
    }
}

impl SensorDriver for MotorController {
    fn id(&self) -> &'static str {
        self.id
    }

    fn display_name(&self) -> &'static str {
        self.name
    }

    fn data_schema(&self) -> Vec<FieldDescriptor> {
        vec![
            FieldDescriptor::new("rpm", "Current motor speed", "f64").with_unit("rpm"),
            FieldDescriptor::new("current_amps", "Motor current draw", "f64").with_unit("A"),
            FieldDescriptor::new("temperature_c", "Motor temperature", "f64").with_unit("C"),
            FieldDescriptor::new("fault_code", "Active fault code (0 = none)", "u16"),
        ]
    }

    fn command_schema(&self) -> Vec<FieldDescriptor> {
        vec![
            FieldDescriptor::new("target_rpm", "Desired motor speed", "f64").with_unit("rpm"),
            FieldDescriptor::new("enable", "Enable/disable motor", "bool"),
        ]
    }

    fn command_mode(&self) -> CommandMode {
        // CAN watchdog requires a command frame every 100ms
        CommandMode::stream(Duration::from_millis(100))
    }

    fn read_data(&self) -> Result<Value, DriverError> {
        let telem = self.last_telemetry.lock().unwrap();
        Ok(serde_json::to_value(&*telem)?)
    }

    fn execute_command(&self, payload: &Value) -> Result<Value, DriverError> {
        let cmd: MotorCommand = serde_json::from_value(payload.clone())?;

        tracing::debug!(
            motor = self.id,
            target_rpm = cmd.target_rpm,
            enable = cmd.enable,
            "sending CAN frame"
        );

        // In production: encode CAN frame and write to bus here.
        // Simulate updating telemetry:
        {
            let mut telem = self.last_telemetry.lock().unwrap();
            if cmd.enable {
                telem.rpm = cmd.target_rpm * 0.95; // simulate lag
            } else {
                telem.rpm = 0.0;
            }
        }

        Ok(serde_json::json!({
            "status": "sent",
            "target_rpm": cmd.target_rpm,
            "enabled": cmd.enable
        }))
    }
}
