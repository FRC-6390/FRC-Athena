# Athena Example Projects

- `tank-drive` - supplier-backed arcade drive, followers, device configuration, automatic discovery, and simulation
- `swerve-drive` - absolute module offsets, slot-configured SDS modules, field-oriented control, heading reset, optimization, and simulation
- `auto-following` - graduated one-auto-per-file PathPlanner and Choreo examples covering markers,
  mechanism actions, multipath routines, Choreo splits, runtime conditions, generated paths, a custom
  marker provider, WPILib command adaptation, and autonomous-style teleop assists
- `localization-setups` - swerve odometry, three camera sources, inspectable filtering/fusion stages, Kalman estimation, and pose resets
- `mechanism-examples` - open/closed-loop mechanisms, composition, constraints, profiles, neutral release, simulation, and a non-swerve mechanism template with configured slots
- `rule-examples` - ranges, Boolean boundaries, conditional actions, timeouts, hooks, unified DIO declarations, and simulated limits
- `custom-controls` - button-bound software control loops, arbitrary feedforward, modular/CRT position decoding,
  absolute-relative fusion, filtered velocity, redundant feedback, neutral output, and simulation
- `controller-bindings` - composed controller signals, sign-preserving axis processing, toggle state, teleop gating, and live supplier actions

Each folder is a standalone WPILib robot project.

The base vendordeps are:

- `FRC6390-Athena.json`
- `WPILibNewCommands.json`

The examples declare real motor, encoder, camera, path, and mechanism APIs. Hardware declarations use real device kinds in simulation as well as on the robot; there are no separate simulated motor kinds or parallel robot implementations.

Athena PID and feedforward gains are voltage-based. The mechanism and custom-control projects show both Athena-hosted loops and vendor-native voltage loops.
