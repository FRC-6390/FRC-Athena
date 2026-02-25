# State Seed Runtime Behavior Contract

This document defines expected behavior for `StateSeed`, `StateSeedProvider`, and `StateSeedRuntime`.

## Seed Kind Contract

- `AUTO` resolves to `null` setpoint.
- `SETPOINT` resolves to the configured scalar setpoint.
- `DSL` resolves by applying the seed DSL to a `StateBuilder` and reading the builder setpoint.

## Executable References

- Example API patterns:
  - `athena-examples/src/main/java/ca/frc6390/athena/mechanisms/examples/state/StateSeedExamples.java`
  - `athena-examples/src/main/java/ca/frc6390/athena/mechanisms/examples/state/StateDslPluginExamples.java`
- Runtime classes:
  - `athena-core/src/main/java/ca/frc6390/athena/mechanisms/statespec/StateSeed.java`
  - `athena-core/src/main/java/ca/frc6390/athena/mechanisms/statespec/StateSeedRuntime.java`
- Behavior tests:
  - `athena-test/src/test/java/ca/frc6390/athena/mechanisms/statespec/StateSeedRuntimeExamplesTest.java`
  - `athena-test/src/test/java/ca/frc6390/athena/mechanisms/state/StateDslPluginExamplesTest.java`
