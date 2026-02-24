# Diagnostics and Event Log Behavior Contract

This document defines expected behavior for bounded event logging and diagnostics snapshots.

## BoundedEventLog

- Capacity is fixed and must be positive.
- New events get monotonic sequence numbers.
- When capacity is exceeded, oldest events are evicted first.
- `snapshot(limit)` returns newest `limit` events (or all if limit is large).

## DiagnosticsChannel

- Channel names must be non-blank.
- Summary fields are key-value metadata for live health state.
- `info/warn/error` append structured events.
- `snapshot(limit)` includes both summary and recent events.
- `clear()` removes event history but keeps channel identity.

## Executable References

- Example API patterns:
  - `athena-examples/src/main/java/ca/frc6390/athena/core/examples/DiagnosticsExamples.java`
- Behavior tests:
  - `athena-test/src/test/java/ca/frc6390/athena/core/diagnostics/DiagnosticsExamplesTest.java`
