use serde_json::Value;
use std::time::Duration;

use super::error::DriverError;

/// Whether commands are one-shot or must be continuously re-sent.
#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize, utoipa::ToSchema)]
#[serde(tag = "type")]
pub enum CommandMode {
    /// Request/response: send once, get one reply.
    Rest,
    /// Streaming: command must be re-sent at `interval_ms` to satisfy
    /// a hardware watchdog (e.g. CAN bus motor controllers).
    Stream {
        /// Re-send interval in milliseconds.
        interval_ms: u64,
    },
}

impl CommandMode {
    pub fn stream(interval: Duration) -> Self {
        CommandMode::Stream {
            interval_ms: interval.as_millis() as u64,
        }
    }

    pub fn interval(&self) -> Option<Duration> {
        match self {
            CommandMode::Rest => None,
            CommandMode::Stream { interval_ms } => Some(Duration::from_millis(*interval_ms)),
        }
    }
}

/// Describes one field in a data or command schema (for OpenAPI/Scalar docs).
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize, utoipa::ToSchema)]
pub struct FieldDescriptor {
    pub name: String,
    pub description: String,
    /// Rust type name: "f64", "u16", "bool", "String", etc.
    pub type_name: String,
    /// Physical unit if applicable: "degrees", "rpm", "m/s".
    #[serde(skip_serializing_if = "Option::is_none")]
    pub unit: Option<String>,
}

impl FieldDescriptor {
    pub fn new(name: &str, description: &str, type_name: &str) -> Self {
        Self {
            name: name.to_string(),
            description: description.to_string(),
            type_name: type_name.to_string(),
            unit: None,
        }
    }

    pub fn with_unit(mut self, unit: &str) -> Self {
        self.unit = Some(unit.to_string());
        self
    }
}

/// The core trait every sensor driver must implement.
///
/// # Adding a new sensor
///
/// 1. Create `src/drivers/my_sensor.rs` and implement this trait.
/// 2. Add `pub mod my_sensor;` to `src/drivers/mod.rs`.
/// 3. Register it in `main.rs`: `registry.register(MySensor::new());`
///
/// That's it. The framework handles UDP sockets, HTTP routes, and documentation.
pub trait SensorDriver: Send + Sync + 'static {
    /// Unique string ID used in UDP port mapping and HTTP routes.
    fn id(&self) -> &'static str;

    /// Human-readable name shown in Scalar UI.
    fn display_name(&self) -> &'static str;

    /// Describes the data fields this sensor produces.
    fn data_schema(&self) -> Vec<FieldDescriptor>;

    /// Describes the command fields this sensor accepts.
    fn command_schema(&self) -> Vec<FieldDescriptor>;

    /// REST or Stream (with watchdog interval).
    fn command_mode(&self) -> CommandMode;

    /// Read current sensor data. Returns a JSON-serializable value.
    fn read_data(&self) -> Result<Value, DriverError>;

    /// Execute a command. For Stream-mode drivers, the framework
    /// calls this repeatedly at the configured interval.
    fn execute_command(&self, payload: &Value) -> Result<Value, DriverError>;
}
