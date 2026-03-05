mod args;
mod control;
mod http;
mod json;
mod manifest;
mod state;
mod telemetry;
mod value;

pub use args::{parse_cli_args, parse_web_args, CliArgs, WebArgs};
pub use control::{ControlClient, ServerStats};
pub use http::{run_http_server, WebServerConfig};
pub use json::render_state_json;
pub use manifest::ManifestItem;
pub use state::DashboardState;
pub use telemetry::TelemetrySubscription;
pub use value::format_signal_value;
