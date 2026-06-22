# athena-commands

Command integration boundary for Athena 2027.

The initial scaffold keeps this module separate so WPILib command integration
does not become part of the core API bucket.

## Current Slice

- `CommandSpec` describes command lifecycle actions without depending directly
  on WPILib.
- `CommandSpec` can carry named subsystem/resource requirements while staying
  dependency-free.
- `CommandGroups` composes specs into sequence and parallel groups.
- `CommandRunner` executes those specs for tests and simple non-WPILib use.
- `RobotDriveCommands` provides tank, arcade, field-relative, distance,
  waypoint-following, and vision-assist command factories backed by
  `RobotSpeeds`.
- The optional WPILib adapter translates descriptors and requirement names to
  real WPILib commands and subsystems.

## Dependencies

- Production: `athena-runtime`.
- Test-only: none.
