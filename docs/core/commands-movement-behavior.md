# Commands: Movement Behavior Contract

This document defines expected behavior for closed-loop movement helpers.

## Rotate Commands

- `RotateToAngle` drives RobotSpeeds source `feedback` using a profiled PID controller.
- Heading control uses continuous input across `[-pi, pi]`.
- PID tolerance normalization is radians-based (`2.5 deg` equivalent).
- Rotation output is clamped to drivetrain `maxAngularVelocity`.
- `setSupplierRadians(...)` and `setSupplierDegrees(...)` are both supported for target heading sources.
- `RotateToPoint` derives a heading target from robot pose to `(x, y)` and delegates to `RotateToAngle`.
- `end()` clears `feedback`.

## Translate Command

- `TranslateToPoint` drives RobotSpeeds source `feedback` with XY PID outputs toward the requested translation target.
- `setPID()` applies the supplied gains/tolerance template to both X and Y controllers.
- Commanded XY outputs are clamped by drivetrain `maxVelocity`.
- `isFinished()` returns true only when both translation controllers are at setpoint.
- `end()` clears `feedback`.

## Executable References

- Example API patterns:
  - `athena-examples/src/main/java/ca/frc6390/athena/commands/examples/MovementCommandExamples.java`
- Behavior tests:
  - `athena-test/src/test/java/ca/frc6390/athena/commands/movement/MovementCommandExamplesTest.java`
