# Swerve Drivetrain Behavior Contract

This document defines expected behavior for swerve module config DSL and inversion resolution semantics.

## Module Config Contract

- PID gains are configurable per module through `pid(...)`.
- Drive feedforward is configurable and independently gateable.
- Explicit drive/steer/encoder inversion flags are tracked.

## Inversion Resolution Contract

- Signed IDs can encode inversion defaults.
- Drivetrain-level explicit inversion overrides take precedence over inherited defaults.

## Executable References

- Example API patterns:
  - `athena-examples/src/main/java/ca/frc6390/athena/drivetrains/swerve/examples/SwerveDrivetrainExamples.java`
- Behavior tests:
  - `athena-test/src/test/java/ca/frc6390/athena/drivetrains/swerve/SwerveDrivetrainExamplesTest.java`
  - `athena-test/src/test/java/ca/frc6390/athena/drivetrains/swerve/SwerveModuleConfigDslTest.java`
  - `athena-test/src/test/java/ca/frc6390/athena/drivetrains/swerve/SwerveDrivetrainConfigInversionTest.java`
