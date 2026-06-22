# Athena V3 Example Project

This module is a student-facing example project for the V3 API.

The full walkthrough lives in [../docs/examples.md](../docs/examples.md).

## Covered Examples

- `IntakeExample`: simple percent-output mechanism.
- `ShooterExample`: velocity mechanism with integrated encoder requirements.
- mechanism inputs: digital beam-break and runtime shooter target examples.
- mechanism encoders: named shooter velocity source.
- mechanism states: intake and shooter setpoint examples.
- control gains: shooter PID and feedforward examples.
- `MechanismV2Example`: fluent simple roller, arm, elevator, flywheel, and
  turret declarations replacing legacy v2 annotation examples.
- `DriveExample`: differential drivetrain declaration.
- `SwerveExample`: four-module swerve drivetrain declaration.
- `SuperstructureExample`: named mechanism coordination.
- `CompositeSuperstructureExample`: nested superstructure state propagation.
- `TurretSuperstructureExample`: turret, hood, and shooter assembly
  coordination.
- `AutoExample`: autonomous chooser, source-backed routine, and scoped input
  handoff examples.
- `AutoVendorAdapterExample`: PathPlanner command and Choreo trajectory source
  examples.
- `RobotHardware`: robot-wide hardware aliases.
- `RobotCommands`: command descriptor example independent of WPILib adapters.
- `DriveCommandExample`: tank, field-relative, distance, and target-alignment
  command examples backed by `RobotSpeeds`.
- `ControlUtilityExample`: controller axis shaping, named driver-station
  profiles, WPILib Xbox axis binding, boolean gating, and filter pipelines.
- `MotionLimitsExample`: conservative motion-limit aggregation and timed
  runner utilities.
- `RobotSpeedsExample`: source blending, heading assist override, and
  field-relative speed conversion.
- `RobotTelemetry`: typed telemetry key, snapshot, and NetworkTables-shaped
  publishing examples.
- `DiagnosticsExample`: bounded event log and health summary example.
- `DashboardBridgeExample`: dashboard packet publishing and control-message
  dispatch examples.
- `SensorExample`: limit switch, beam-break, and camera target sensor wrappers.
- `VisionExample`: generic camera declaration and target observation examples.
- `VisionVendorAdapterExample`: PhotonVision and Limelight adapter examples.
- `LocalizationExample`: pose-estimation weighting, slip detection, field
  bounds, and autonomous pose aliases.
- `SimulationExample`: stateful motor, IMU, and vision simulation world example.
- `VendorOptionsExample`: typed CTRE and REV motor option escape hatches.
- `StudicaImuAdapterExample`: Studica/NavX and CTRE Pigeon2 IMU adapter
  examples.
- `WpilibBoundaryExample`: command, lifecycle, and NetworkTables adapter
  boundary examples.

The examples use `AthenaMotor.SIM` so they validate without CTRE, REV, or any
other vendor dependency. Vendor-specific examples separately demonstrate the
syntax teams get after installing matching Athena adapter artifacts.

## Dependencies

- Production: all Athena V3 modules used by the examples.
- Test-only: none.
