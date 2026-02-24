# Mechanism Config Apply/Export Behavior Contract

This document defines expected behavior for data-file mechanism config application and export.

## Apply Contract

- `MechanismConfigApplier` maps data-only file fields into fluent `MechanismConfig` sections.
- Named PID/feedforward profiles are registered for control-loop usage.
- Bounds, motion limits, sensors, and simulation hints are applied when present.

## Export Contract

- `MechanismConfigExport` emits data-only fields suitable for JSON/TOML output.
- Code-only constructs (hooks, lambdas, custom loops) are intentionally excluded.

## Executable References

- Runtime classes:
  - `athena-core/src/main/java/ca/frc6390/athena/mechanisms/config/MechanismConfigApplier.java`
  - `athena-core/src/main/java/ca/frc6390/athena/mechanisms/config/MechanismConfigExport.java`
- Behavior tests:
  - `athena-test/src/test/java/ca/frc6390/athena/mechanisms/config/MechanismConfigApplyExportTest.java`
