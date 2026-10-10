# 2027 Off-season Overview

This is the user-facing tour of the off-season features. Each section links to the feature guide with the details.
The design and the section IDs (`A1`, `B4`, ...) are in [`2027-offseason-design.md`](2027-offseason-design.md). Runnable
code for each feature is in [`example-projects`](../example-projects/README.md), and the feature-to-example matrix is in
[`COVERAGE.md`](../example-projects/COVERAGE.md).

## Tour

### Tuning and controls

- **Typed units** ([typed-units.md](typed-units.md)). Encoders declare what they measure (`measures(Degrees)`,
  `reduction(10)`), and bindings take measures (`Controls.position(motor, Degrees)`). Targets, tolerances, and
  constraints are checked against the encoder when the mechanism is declared. Untyped code keeps working.
  Example: `mechanism-examples/SingleJointArm`.
- **Gamepad binding sets** ([gamepad-controls.md](gamepad-controls.md#binding-sets)). `pad.in(condition, ...)` gates a
  group of bindings on any condition. Released bindings end their actions, and conflicting sets get a warning.
  Example: `controller-bindings/AdvancedControls`.
- **Persisted telemetry and lock groups** ([telemetry-and-simulation.md](telemetry-and-simulation.md#persisted-values-and-lock-groups)).
  `@Telemetry(persist = true)` keeps a value across reboots. A lock group decides when dashboard writes are accepted:
  `tuningLock(Calibration.class, DriverStation::isEnabled)` in the robot constructor. Enforcement is in the robot.
  Example: `mechanism-examples/VelocityShooter`, `StagedArm`, `Robot`.
- **Recording interpolation points** (telemetry-and-simulation.md, [Recording points](telemetry-and-simulation.md#recording-points)).
  `recordShot` adds the current input and output to a persisted table, with a replay marker.
- **Why is it not moving?** (telemetry-and-simulation.md, [Why is this mechanism not moving?](telemetry-and-simulation.md#why-is-this-mechanism-not-moving)).
  Each mechanism publishes `State/Reason`.

### Control and motion

- **Characterization and models** ([characterization-and-models.md](characterization-and-models.md)). A
  `Characterization` field runs motion tests from the dashboard, fits a `MechanismModel`, watches for drift, and
  feeds feedforward and the simulation plant. Checks (`blocked`, `load`, `feedbackStalled`, `slipCheck`) come from the
  same binding. Example: `mechanism-examples/SingleJointArm`.
- **Motion policies** ([motion-policies.md](motion-policies.md)). `retarget` (SMOOTH or IMMEDIATE), `release`
  (NEUTRAL, HOLD, DECELERATE), `interruptible` (with bounded commitments), `Actions.attempt(...)` fallback chains, and
  `Actions.fault(...)`. The rules are in [action-runtime-contract.md](action-runtime-contract.md#motion-policies).
- **Power, thermal, and staging** ([power-thermal-staging.md](power-thermal-staging.md)). `PowerProfile` shares a battery budget
  between motors, `Thermal.derate` scales current limits as the motor heats, and `Staging` moves a mechanism through
  team-defined health stages. Example: `mechanism-examples/StagedArm`, `VelocityShooter`.
- **Alliance transforms** (`AllianceTransform`, see [localization-setups](../example-projects/localization-setups/README.md#alliance-geometry)
  and [auto-following](../example-projects/auto-following/README.md)). Field geometry and path mirroring resolve for the
  current alliance at call time.

### Replay, simulation, and runtime

- **Replay** ([replay.md](replay.md)). Every cycle's inputs go into a ring buffer, and files are written on match end,
  disable, and brownout recovery. `ReplaySimulation` feeds a recording back into new robot code and reports the command
  differences. Declare a `ReplayPolicy` field on the robot. Example: `mechanism-examples/ReplayRecordingTest`.
- **Simulation exploration** ([sim-exploration.md](sim-exploration.md)). `SimExplorer` drives random legal modes and input
  for a fixed seed, reports starvation, timeouts, faults, and exceptions, shrinks each finding to a minimal input, and can
  save it as a replay. Example: `tank-drive/ExplorationTest`.
- **ARCP** ([arcp.md](arcp.md)). Realtime telemetry, writes, Action requests, file service, tuning diff and pull, and replay
  streaming, over TCP 5808 (control) and UDP 5809 (realtime). Ports are listed in [`athena-arcp/README.md`](../athena-arcp/README.md#ports). The protocol is in Rust; Java is a thin JNI binding. Declare an `ArcpPolicy`
  field, or `ArcpPolicy.disabled()` to turn it off. Build notes are in [`athena-arcp/README.md`](../athena-arcp/README.md).
- **Dashboard.** `athena-arcp/host-dashboard` is the desktop dashboard that connects to ARCP. It is an application and is
  not part of the robot API.

## Build and native notes

- The ARCP native library is built by cargo. Install Rust, or pass `-ParcpSkipNative=true` to build without it.
- The roboRIO library is cross-compiled with `./gradlew :athena-arcp:buildArcpNativeRoboRio` and needs the WPILib 2026
  roboRIO toolchain (or `-ParcpRoborioLinker=<path>`).
- `athena-arcp` is published to Maven Local only with `-ParcpPublish=true`. `athena-wpilib` depends on it, so any
  consumer of a Maven Local publish needs that flag.

## Migration notes for existing robot code

Source compatibility is kept: existing code compiles unless a section says otherwise. These behaviour changes can
still change what a robot does on the next deploy.

1. **Dashboard writes to Config, Test, and Setup default to the `InMatch` lock.** Writes are rejected while FMS is attached,
   and the rejection is reported as `rejected: locked (InMatch)`. To allow those writes during matches, define the
   group's condition in the robot constructor, for example `tuningLock(Locks.InMatch.class, () -> false)`.
2. **`MotorDevice.stall()` is model-based for known motors.** Motors with built-in constants detect a stall from the
   expected speed at the free-run current, with no fixed voltage threshold. Calling `atCurrentLimit`, `velocityBelow`,
   or `commandedVoltageAbove` selects the legacy fixed-threshold detector, so mechanisms that set thresholds
   explicitly (such as `IndexedIntake`) keep their behaviour. Check stall events on any other mechanism after deploying.
3. **REV current limits default when none is declared.** An undeclared SPARK smart current limit takes a per-motor-kind
   default (for example 40 A for a NEO) instead of leaving the device value. Keep the device value with
   `RevMotorOptions.preserveDeviceCurrentLimit(true)`, or declare `smartCurrentLimit(...)`.
4. **CANcoder stored configuration is only reported.** A stored magnet configuration (offset, direction, discontinuity)
   that differs from Athena's baseline logs a warning and is not written. Use `EncoderDevice.applyBaselineConfig()` to
   write the baseline; a write failure then fails activation. Check the warning once after the first deploy.
5. **ARCP listens on TCP port 5808 and sends realtime frames from UDP 5809 by default** (on the roboRIO and in simulation).
   Set `ArcpPolicy.controlPort(0)` and `realtimePort(0)` for ephemeral ports, or `ArcpPolicy.disabled()` to turn it off.
6. **Characterization sample alignment changed.** Each row now pairs the applied voltage of the previous cycle with the
   measured motion (I5). Fits from runs before this change can differ slightly. Re-run `FitModel` on existing data before
   comparing with an older model.
7. **`Actions.fault` no longer neutralizes hardware.** It only reports the message and completes. Stop the hardware with
   `neutral()`, `cancel(...)`, or `Actions.abort(...)`.
