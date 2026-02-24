# Sensor Wrapper Behavior Contract

This document defines expected behavior for sensor wrappers (`limitswitch`, `button`, `beambreak`, and basic camera wrappers).

## Limit Switch Rules

- Config-derived limit switches preserve hardstop flags, block direction, and position metadata.
- Block direction multipliers map to `-1`, `0`, and `1`.

## Button and Beam-Break Rules

- Generic button and beam-break wrappers default to inverted trigger semantics.
- Simulation-trigger updates, when supported, map cleanly to trigger state reads.

## Camera Wrapper Rules

- Target-specific accessors (`yaw`, `pitch`, `distance`, `tag id`, observation lists) return empty when `hasValidTarget()` is false.
- Yaw readings are present only when finite targeting values are available.
- Non-finite yaw values are treated as no-target observations.

## Executable References

- Example API patterns:
  - `athena-examples/src/main/java/ca/frc6390/athena/sensors/examples/SensorWrapperExamples.java`
- Behavior tests:
  - `athena-test/src/test/java/ca/frc6390/athena/sensors/SensorWrapperExamplesTest.java`
