# athena-telemetry

Telemetry registration and publishing boundary for Athena 2027.

The initial scaffold keeps telemetry isolated from mechanism and hardware
declarations.

## Current Slice

- `TelemetryKey` and `TelemetryValue` provide typed telemetry.
- `TelemetryRegistry` collects suppliers and can either publish to a sink or
  capture a `TelemetrySnapshot` for tests.
- `NetworkTablesTelemetrySink` maps typed values to normalized `/Athena/...`
  paths through a minimal `NetworkTableWriter` adapter boundary.
- `InMemoryNetworkTableWriter` supports deterministic tests and examples until
  the real WPILib NetworkTables writer is added.

## Dependencies

- Production: `athena-runtime`.
- Test-only: none.
