# Mechanism Config Apply/Export Behavior Contract

This document defines expected behavior for the runtime record snapshot/apply path and
mechanism export.

## Snapshot / Apply Contract

- `MechanismConfigIO.snapshot(mechanism)` captures a data-only `MechanismConfigRecord`
  from a live runtime mechanism.
- `MechanismConfigIO.apply(mechanism, record)` pushes a `MechanismConfigRecord` back onto
  a live mechanism.
- Runtime snapshot/apply covers record-backed values such as motors, encoder metadata,
  limits, control values, and simulation-facing data.
- Code-only declarations such as callbacks, custom loop bodies, and annotation methods
  are not serialized into the data record.

## Export Contract

- `MechanismConfigExport` emits data-only fields suitable for JSON/TOML output by reading
  the runtime snapshot record.
- Export remains intentionally data-only. Hooks, lambdas, annotation callbacks, and custom
  control loop code are excluded.

## Executable References

- Runtime classes:
  - `athena-core/src/main/java/ca/frc6390/athena/mechanisms/base/MechanismConfigIO.java`
  - `athena-core/src/main/java/ca/frc6390/athena/mechanisms/config/MechanismConfigExport.java`
- Behavior tests:
  - `athena-test/src/test/java/ca/frc6390/athena/mechanisms/runtime/MechanismRuntimeExportTest.java`
