# athena-simulation

Simulation helpers and fake backends for examples and tests.

This module proves that optional backends can be added without changing the
student-facing API.

## Current Slice

- Simulation motor backend and in-memory motor device.
- Simulation IMU backend and zeroed in-memory IMU device.
- Stateful `SimWorld` for motor position, IMU yaw, and simulated vision frames.
- `SimMechanism` binding that applies mechanism states to simulated motor
  position, velocity, or percent-output state.
- Differential and swerve drivetrain bindings that apply generic drivetrain
  targets into simulated motor state.
- ServiceLoader entries for optional backend discovery.

## Dependencies

- Production: `athena-runtime`, `athena-hardware`, `athena-mechanisms`, `athena-drivetrain`, `athena-vision`.
- Test-only: none.
