# Commands: Control Behavior Contract

This document defines expected behavior for manual drivetrain control commands.

## Tank Drive

- `TankDriveCommand` reads `xInput` and `thetaInput` each execute cycle.
- Inputs are scaled by drivetrain max velocity.
- Commanded speeds are written to RobotSpeeds source `drive`.
- `end()` clears the `drive` source.
- Constructor sets drivetrain neutral mode to brake.

## Swerve Drive

- `SwerveDriveCommand` reads `xInput`, `yInput`, and `thetaInput` each cycle.
- Translation scales with max linear velocity; rotation scales with max angular velocity.
- When `fieldRelative=true`, it converts through the drivetrain driver virtual axis.
- Commanded speeds are written to RobotSpeeds source `drive`.
- `end()` clears the `drive` source.

## Executable References

- Example API patterns:
  - `athena-examples/src/main/java/ca/frc6390/athena/commands/examples/TankDriveCommandExamples.java`
- Behavior tests:
  - `athena-test/src/test/java/ca/frc6390/athena/commands/control/TankDriveCommandExamplesTest.java`
