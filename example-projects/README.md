# Athena Example Projects

- `tank-drive` - tank drive robot
- `swerve-drive` - swerve drive robot
- `auto-following` - autonomous setup with PathPlanner, Choreo-style paths, markers, and a custom timed provider
- `localization-setups` - odometry, multi-camera vision, weighted, latest-valid, and Kalman localization pipelines
- `mechanism-examples` - shooter, 2DOF arm, intake, elevator, simulations, and a composed superstructure
- `rule-examples` - guarded Actions using clamps, limit switches, conditional branches, timeouts, hooks, and sensor gates
- `custom-controls` - software control loops and arbitrary feedforward layered onto closed-loop controls

Each folder is a standalone WPILib robot project.

The base vendordeps are:

- `FRC6390-Athena.json`
- `WPILibNewCommands.json`

The examples declare real motor, encoder, camera, path, and mechanism APIs. For real hardware or real vendor path tools, install the normal vendor vendordeps for the hardware and tools on the robot.
