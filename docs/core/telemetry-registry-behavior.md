# Telemetry Registry Behavior Contract

This document defines expected behavior for annotation-driven telemetry routing and publish gating.

## Routing Rules

- `TelemetryDestination.DISK` writes only to disk sink.
- `TelemetryDestination.SHUFFLEBOARD` writes only to network-table/dashboard sink.
- `TelemetryDestination.BOTH` writes to both sinks.

## Publish Gating Rules

- Entries publish only when due for their configured period.
- High-bandwidth array types enforce a minimum publish period.
- `epsilon` suppresses small floating-point changes.
- Disabled registry (`setEnabled(false)`) suppresses all publishing.

## Executable References

- Example API patterns:
  - `athena-examples/src/main/java/ca/frc6390/athena/core/examples/TelemetryRegistryExamples.java`
- Behavior tests:
  - `athena-test/src/test/java/ca/frc6390/athena/logging/TelemetryRegistryContractTest.java`
