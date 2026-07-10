# Athena Example Projects

- `tank-drive` - tank drive robot with processed driver axes in a dedicated `Controls.java`
- `swerve-drive` - swerve drive robot with processed translation and rotation axes in a dedicated `Controls.java`
- `auto-following` - autonomous setup with PathPlanner, Choreo-style paths, markers, and a custom timed provider
- `localization-setups` - odometry, multi-camera vision, weighted, latest-valid, and Kalman localization pipelines
- `mechanism-examples` - button-bound open-loop, velocity, and split-wheel shooters; manual, single-joint, and 2DOF arms; roller and indexed intakes; single and follower elevators; simulations; and a composed superstructure
- `rule-examples` - button-bound guarded Actions using clamps, limit switches, conditional branches, timeouts, hooks, and sensor gates
- `custom-controls` - button-bound software control loops and arbitrary feedforward layered onto closed-loop controls
- `controller-bindings` - Xbox axes and button bindings contained in a dedicated `Controls.java` mechanism

Each folder is a standalone WPILib robot project.

The base vendordeps are:

- `FRC6390-Athena.json`
- `WPILibNewCommands.json`

The examples declare real motor, encoder, camera, path, and mechanism APIs. For real hardware or real vendor path tools, install the normal vendor vendordeps for the hardware and tools on the robot.
