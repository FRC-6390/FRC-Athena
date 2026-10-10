# Athena Example Projects

- `tank-drive` - supplier-backed arcade drive, followers, device configuration, automatic discovery, simulation, and a seeded
  `SimExplorer` test that expects no findings
- `swerve-drive` - absolute module offsets, slot-configured SDS modules, field-oriented control, heading reset, optimization, and simulation
- `auto-following` - Choreo markers/splits/branches, PathPlanner `AutoBuilder` integration, a generated
  vendor-neutral path provider with preview geometry, chooser composition, `AllianceTransform` path mirroring, and WPILib command adaptation
- `localization-setups` - swerve odometry, three camera sources, inspectable filtering/fusion stages,
  Kalman estimation, pose resets, `AllianceTransform` field geometry and alliance-relative heading, camera target reads,
  target-driven aiming, and a custom camera adapter
- `mechanism-examples` - typed-unit controls, characterization and identified simulation plants, checks, motion policies
  (retarget, release, interruptible, attempt chains), power profiles, thermal derating, staging, persisted tuning with lock
  groups, interpolation recording, open/closed-loop mechanisms, composition, constraints, profiles, stall handling, SysId,
  telemetry/live tuning, hardware/vendor declarations, runtime workers, explicit simulation harnesses, the robot-level
  `ReplayPolicy`, `ArcpPolicy`, `PowerPolicy`, and `tuningLock(...)` declarations, and a `ReplaySimulation` record-and-replay test
- `rule-examples` - ranges, Boolean boundaries, conditional actions, timeouts, hooks, unified DIO declarations, and simulated limits
- `custom-controls` - button-bound software control loops, arbitrary feedforward, modular/CRT position decoding,
  absolute-relative fusion, filtered velocity, redundant feedback, neutral output, and simulation
- `controller-bindings` - composed controller signals, sign-preserving axis processing, toggles, click counts,
  holds/repeats, debounce, hysteresis, chords, ordered sequences, binding sets with `in(condition, ...)`, command
  arbitration, and teleop gating

Each folder is a standalone WPILib robot project.

See [`COVERAGE.md`](COVERAGE.md) for the maintained feature-to-example matrix and deliberately
non-example internal surfaces. The off-season tour that links each feature document is
[`../docs/2027-offseason-overview.md`](../docs/2027-offseason-overview.md).

The base vendordeps are:

- `FRC6390-Athena.json`
- `WPILibNewCommands.json`

The examples declare real motor, encoder, camera, path, and mechanism APIs. Hardware declarations use real device kinds in simulation as well as on the robot; there are no separate simulated motor kinds or parallel robot implementations.

Athena PID and feedforward gains are voltage-based. The mechanism and custom-control projects show both Athena-hosted loops and vendor-native voltage loops.

## Building against a local Athena snapshot

The examples resolve Athena from Maven Local. `mechanism-examples` and `tank-drive` also run JUnit tests with `./gradlew test`.

- `athena-arcp` (a transitive dependency of `athena-wpilib`) is published to Maven Local only with `-ParcpPublish=true`.
  Pass it to any publish that the examples will consume, or they cannot resolve `athena-wpilib`.
- ARCP's native library is built with cargo. Use `-ParcpSkipNative=true` on a machine without Rust; ARCP is then
  disabled at runtime with a warning.
- To keep a snapshot out of your real `~/.m2`, point Gradle at a separate repository:
  `./gradlew publishToMavenLocal -ParcpPublish=true -Dmaven.repo.local=<dir>` in the repository root, then
  `./gradlew build -Dmaven.repo.local=<dir>` in each example.
