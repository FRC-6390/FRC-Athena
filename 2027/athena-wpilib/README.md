# athena-wpilib

WPILib adapter artifact for Athena 2027.

This module keeps WPILib-facing concepts out of the default Athena artifacts.
It depends on WPILib only inside the optional `athena-wpilib` tier, so teams do
not pull WPILib classes into the default Athena runtime artifacts.

## Current Slice

- Real WPILib `Command` adapter from `CommandSpec`, including named subsystem
  requirement mapping.
- Scheduler facade for adapting and scheduling `CommandSpec` instances.
- Trigger binding helpers for `onTrue`, `whileTrue`, and `toggleOnTrue`.
- Differential-drive runtime adapter from `RobotSpeeds` to WPILib
  `DifferentialDrive`.
- Swerve-drive runtime adapter from `RobotSpeeds` to WPILib
  `SwerveModuleState` outputs.
- Pose-estimator vision measurement bridge from Athena localization weights to
  WPILib estimator standard-deviation vectors.
- Controller axis and button binding helpers over WPILib-shaped suppliers,
  raw `GenericHID` devices, `XboxController`, and `Joystick`.
- Named driver-station profiles for driver/operator controller ports and
  shared input shaping defaults.
- `TimedRobot` lifecycle adapter for init and periodic phases.
- NetworkTables writer bridge backed by `NetworkTableInstance`, with cached
  entries per telemetry path.
- Tests proving adapter behavior against WPILib command classes, subsystem
  requirements, and drive output consumers.

## Dependencies

- Production: `athena-commands`, `athena-drivetrain`, `athena-localization`,
  `athena-telemetry`, `athena-runtime`, WPILib Java artifacts from FRC Maven, and Jackson
  annotations/core, EJML, and Quickbuf for WPILib math and drive/sendable
  runtime paths.
- Test-only: none.
