# athena-dashboard

Dashboard/control bridge boundary for Athena 2027.

This module keeps ARCP servers and UI dependencies out of the default robot
runtime. It consumes telemetry and diagnostics snapshots, publishes them over an
optional TCP dashboard transport, and defines explicit control messages that a
separate dashboard workspace can send back to the robot.

## Current Slice

- `DashboardPacket` packages telemetry and diagnostics snapshots.
- `DashboardPublisher` publishes packets to a pluggable sink.
- `DashboardControlRegistry` dispatches named control messages.
- `DashboardWireCodec` defines the dependency-free JSON shape for packet and
  control transport payloads.
- `DashboardTcpServer` publishes newline-delimited JSON packets to connected
  dashboards and accepts JSON control messages from those clients.
- Tests verify packet contents, publishing, socket transport, and control
  dispatch.

## Dependencies

- Production: `athena-runtime`, `athena-telemetry`.
- Test-only: none.
