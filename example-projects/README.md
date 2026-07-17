# Athena Example Projects

- `tank-drive` - supplier-backed arcade drive, followers, device configuration, automatic discovery, and simulation
- `swerve-drive` - absolute module offsets, slot-configured SDS modules, field-oriented control, heading reset, optimization, and simulation
- `auto-following` - Choreo markers/splits/branches, PathPlanner `AutoBuilder` integration, a generated
  vendor-neutral path provider with preview geometry, chooser composition, and WPILib command adaptation
- `localization-setups` - swerve odometry, three camera sources, inspectable filtering/fusion stages,
  Kalman estimation, pose resets, camera target reads, target-driven aiming, and a custom camera adapter
- `mechanism-examples` - open/closed-loop mechanisms, composition, constraints, profiles, interpolation,
  stall handling, SysId, telemetry/live tuning, hardware/vendor declarations, runtime workers, explicit
  simulation harnesses, and a non-swerve mechanism template with configured slots
- `rule-examples` - ranges, Boolean boundaries, conditional actions, timeouts, hooks, unified DIO declarations, and simulated limits
- `custom-controls` - button-bound software control loops, arbitrary feedforward, modular/CRT position decoding,
  absolute-relative fusion, filtered velocity, redundant feedback, neutral output, and simulation
- `controller-bindings` - composed controller signals, sign-preserving axis processing, toggles, click counts,
  holds/repeats, debounce, hysteresis, chords, ordered sequences, command arbitration, and teleop gating

Each folder is a standalone WPILib robot project.

See [`COVERAGE.md`](COVERAGE.md) for the maintained feature-to-example matrix and deliberately
non-example internal surfaces.

The base vendordeps are:

- `FRC6390-Athena.json`
- `WPILibNewCommands.json`

The examples declare real motor, encoder, camera, path, and mechanism APIs. Hardware declarations use real device kinds in simulation as well as on the robot; there are no separate simulated motor kinds or parallel robot implementations.

Athena PID and feedforward gains are voltage-based. The mechanism and custom-control projects show both Athena-hosted loops and vendor-native voltage loops.
