# Differential Drivetrain Behavior Contract

This document defines expected behavior for differential drivetrain control and simulation helpers.

## Control Rules

- Arcade command helpers route inputs into drivetrain source `drive`.
- Translation/turn values are scaled by drivetrain max velocity constraints.
- Control reset clears active drivetrain speed sources.

## Feedforward Rules

- Feedforward config is opt-in and has explicit enable/disable state.
- Runtime toggles must preserve deterministic enabled state for updates.

## Simulation Rules

- When simulation is enabled, simulation pose setters update the published sim pose.
- When simulation is not enabled, simulation section remains disabled and pose mutation is a no-op.

## Executable References

- Example API patterns:
  - `athena-examples/src/main/java/ca/frc6390/athena/drivetrains/examples/DifferentialDrivetrainExamples.java`
- Behavior tests:
  - `athena-test/src/test/java/ca/frc6390/athena/drivetrains/differential/DifferentialDrivetrainExamplesTest.java`
