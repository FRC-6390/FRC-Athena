# athena-drivetrain

Drivetrain declaration boundary for Athena 2027.

This module models drivetrain declarations using generic motor and encoder specs
so vendor backends remain optional and selected elsewhere.

## Current Slice

- Differential drivetrain declarations with side motor groups and track width.
- Swerve drivetrain declarations with four-module hardware, geometry, inversion
  defaults, steer PID gains, and drive feedforward gains.
- Validation for required module hardware and installed motor backend
  capabilities.

## Dependencies

- Production: `athena-hardware`, `athena-commands`.
- Test-only: `athena-simulation`.
