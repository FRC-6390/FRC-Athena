# Commands: Vision Behavior Contract

This document defines expected behavior for vision-assisted command routines.

## Rotate-To-Tag

- `RotateToTagCommand` reads yaw from the selected camera each execute cycle.
- If no yaw is available, it clears RobotSpeeds source `feedback`.
- Heading correction comes from a PID controller in degrees.
- Angular output is converted to radians/second and clamped by configured max omega.
- Command completes when PID is at setpoint or optional timeout is reached.
- `end()` clears `feedback`.

## Align-And-Drive-To-Tag

- Camera table inputs are trimmed/deduplicated and must contain at least one valid key.
- Optional role filters are sanitized to an internal defensive copy.
- Observation selection prefers:
  1. higher confidence
  2. lower absolute distance
  3. lower translation magnitude
- Tolerance values are normalized to finite absolute values.

## Executable References

- Example API patterns:
  - `athena-examples/src/main/java/ca/frc6390/athena/commands/examples/VisionCommandExamples.java`
- Behavior tests:
  - `athena-test/src/test/java/ca/frc6390/athena/commands/vision/VisionCommandExamplesTest.java`
  - `athena-test/src/test/java/ca/frc6390/athena/commands/vision/AlignAndDriveToTagCommandContractTest.java`
