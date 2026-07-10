# Athena Example Projects

- `tank-drive` - tank drive robot with processed driver axes in a dedicated `Controls.java`
- `swerve-drive` - swerve drive robot with selectable field-oriented and robot-oriented control using a declared Pigeon 2 IMU
- `auto-following` - graduated one-auto-per-file PathPlanner and Choreo examples covering markers,
  mechanism states, multipath routines, Choreo splits, runtime conditions, generated paths, a custom
  marker provider, and autonomous-style teleop assists
- `localization-setups` - odometry, multi-camera vision, weighted, latest-valid, and Kalman localization pipelines
- `mechanism-examples` - button-bound open-loop, velocity, and split-wheel shooters; manual, single-joint, and constrained 2DOF arms; a bounded field-relative turret; roller and indexed intakes; single and follower elevators; simulations; and a composed superstructure
- `rule-examples` - reusable control constraints using ranges and Boolean-supplier boundaries, plus conditional branches, timeouts, hooks, and sensor gates
- `custom-controls` - button-bound software control loops, arbitrary feedforward, modular/CRT position decoding,
  absolute-relative fusion, filtered velocity, and redundant feedback
- `controller-bindings` - Xbox axes and button bindings contained in a dedicated `Controls.java` mechanism

Each folder is a standalone WPILib robot project.

The base vendordeps are:

- `FRC6390-Athena.json`
- `WPILibNewCommands.json`

The examples declare real motor, encoder, camera, path, and mechanism APIs. For real hardware or real vendor path tools, install the normal vendor vendordeps for the hardware and tools on the robot.
