# Mechanism telemetry and simulation

Runnable coverage lives in `example-projects/mechanism-examples`: `VelocityShooter` and `FieldTelemetry`
cover annotations, live tuning, custom values, geometry, and trace profiles; `SimulationHarness` covers the
explicit session API in addition to the declarative `SimModel` fields used throughout that project.

## Mechanism-scoped telemetry

The mechanism tree is also the telemetry tree. `AthenaRobot` publishes everything declared by a
mechanism beneath `/Athena/Mechanisms/<mechanism path>` using the standard groups `State`,
`Values`, `Actions`, `Devices`, `Controls`, `Hooks`, and `Trace`. A child mechanism stays nested
under its declaring field, including mechanisms stored in lists, arrays, and maps.

For example, fields in an `Arm` mechanism appear together:

```text
/Athena/Mechanisms/robot/arm/State
/Athena/Mechanisms/robot/arm/Values/target
/Athena/Mechanisms/robot/arm/Actions/home
/Athena/Mechanisms/robot/arm/Devices/motor
/Athena/Mechanisms/robot/arm/Devices/encoder
/Athena/Mechanisms/robot/arm/Controls/positionControl
/Athena/Mechanisms/robot/arm/Hooks/homeOnLimit
/Athena/Mechanisms/robot/arm/Trace
```

There is no separate global action or device registry and no legacy flat telemetry map. Ownership
follows the mechanism field that declares the value. The dashboard-independent representation is
available from `RobotRuntime.mechanismTelemetrySchema()`.

### Runnable actions

Public `Action` fields are published as real WPILib command sendables beneath the declaring
mechanism's `Actions` group. Shuffleboard, Glass, and compatible dashboards can therefore render
Run/Cancel controls without a command wrapper in robot code. Non-public actions remain internal
unless explicitly annotated with `@Telemetry`:

```java
public final Action score = arm.position(75.0);

@Telemetry("calibration/zero")
private final Action zeroForTesting = encoder.setPosition(0.0);
```

Each action also publishes its Athena type, running state, and completion state. Dashboard commands
do not run while the robot is disabled.

A public `ControlSysId` field expands into all four characterization actions automatically; robot
code does not create WPILib command wrappers or four duplicate action fields:

```java
private final ControlBinding driveControl = Controls.velocity(driveMotor)
        .feedback(driveEncoder);
public final ControlSysId sysId = driveControl.sysId().name("drive");
```

This publishes `Actions/sysId/QuasistaticForward`, `QuasistaticReverse`, `DynamicForward`, and
`DynamicReverse`. As with ordinary actions, a non-public SysId field requires `@Telemetry` to opt in.

### Devices and controls

Device state and setup live with the field that owns the device:

- Motors publish command, position, velocity, voltage, and currents under `State`. Temporary
  `Disabled`, `NeutralMode`, `Inverted`, supply-current, and stator-current overrides live under
  `Config`, along with support, status, and restore controls.
- Encoders publish position, absolute position, and velocity under `State`; `Setup` provides a
  requested position plus Set, Zero, support, and status controls.
- IMUs publish orientation, rate, and acceleration under `State`; `Setup` provides requested yaw,
  Set, Zero, and status controls.
- Digital inputs publish raw and active state and expose a latched-edge clear operation.
- Controls publish mode and feedback under `State`, with disabled, slot, PID, and feedforward values
  under `Config`. `Config/Constraints` exposes temporary minimum/maximum position, maximum velocity,
  maximum acceleration, and maximum output (0–1) overrides. `Test/Percent` supplies an output for
  the adjacent `RunPercent` dashboard action. The output remains under normal action ownership,
  constraint, saturation, enable-state, and Run/Cancel handling; editing the number alone does not
  move a motor. `Config/Restore` returns the constraints and test percent to their code defaults.

Declare those defaults on the binding; dashboard edits are temporary overrides of this baseline:

```java
private final ControlBinding armControl = Controls.position(armMotor)
        .feedback(armEncoder)
        .constraints(
                Constraints.range(Range.of(0.0, 105.0)),
                Constraints.motion(60.0, 120.0),
                Constraints.clamp(8.4));
```

`motionLimits` is valid for both control modes. A position control uses the values for its
trapezoidal reference profile. A velocity control clamps its requested velocity and slews the
reference by at most `maximumAcceleration * dt` each cycle:

```java
private final ControlBinding shooterControl = Controls.velocity(shooterMotor)
        .feedback(shooterEncoder)
        .constraints(
                Constraints.motion(100.0, 250.0),
                Constraints.clamp(10.8));
```

Runtime motor configuration is deliberately temporary. It changes the already-created controller
without mutating the immutable `MotorDevice`, is not persisted across a robot-program restart, and
can be returned to the code declaration with `Restore`. Unsupported vendor operations report
`Unsupported` instead of silently changing the declaration.

### Trace capture

Athena also builds one immutable `MechanismTraceSnapshot` after each mechanism cycle. Nested
mechanisms receive filtered trace views; reading or exporting a trace never refreshes hardware or
evaluates a dynamic action supplier.

The available profiles are:

- `OFF`: no mechanism `Trace` topics.
- `SUMMARY`: packed scheduler state plus action labels.
- `CAPTURE`: state plus packed arrays for candidates, controls, motors, and hooks at 50 Hz.

These profiles also control core trace materialization, not only NetworkTables publication. `OFF`
creates no per-cycle trace records, `SUMMARY` skips candidate/control/motor/hook detail construction,
and `CAPTURE` pays the full diagnostic cost. Code that uses `RobotRuntime` without `AthenaRobot` can
select the same behavior with `runtime.mechanismTraceLevel(MechanismTraceLevel.SUMMARY)`.

Simulation automatically promotes the default `SUMMARY` profile to `CAPTURE`. A robot can opt in
to capture on real hardware in its constructor:

```java
public Robot() {
    traceTelemetry(MechanismTracePublisher.Profile.CAPTURE);
}
```

To change pressure without redeploying, publish `OFF`, `SUMMARY`, or `CAPTURE` to the single
`/Athena/Telemetry/Profile` string topic from AdvantageScope. An empty or invalid value falls back
to the profile selected in robot code.

Capture uses five packed NT4 struct topics instead of one scalar topic per value. Names and route
labels are represented by integer IDs in the high-rate frames and published only when their
metadata changes. There is no on-robot data log in this path; AdvantageScope can record the live
NT4 stream on the driver-station computer.

The control frame includes requested and transformed targets, constrained goal, motion-profile
reference, measured position and velocity, error, PID voltage components, feedforward components,
final applied output, route, and constrained/blocked/saturated flags. The motor frame contains
every declared motor, including while disabled, with its actual post-control-loop command and
cached feedback/current values. Candidate and hook frames make
action ownership, leases, sequence progress, and event transitions inspectable.

As a rough upper bound, ten candidates, ten controls, ten motors, and ten hooks are about 120 KB/s
of packed payload at 50 Hz before NT4 framing. `SUMMARY` is appropriate when capture-level detail
is not needed.

### Custom values and live tuning

Mechanisms can publish fields and zero-argument methods with `@Telemetry` without calling
`SmartDashboard.put*` every loop:

```java
@Telemetry
private double armPosition() { return encoder.position(); }

@Telemetry
private boolean homed() { return homeSwitch.active(); }

@Telemetry(writable = true, min = 0.0, max = 30.0)
private double target = 20.0;

private final ControlBinding control = Controls.position(armMotor)
        .feedback(encoder)
        .pid(2.0, 0.0, 0.0)
        .ff(0.1, 0.4, 0.0);
public final Action move = control.position(() -> target);
```

Annotated methods are read-only. Annotated fields are read-only unless they explicitly request
`writable = true`. Writable numeric fields may specify `min` and `max`; booleans, strings, and enums
are also writable. A relative path such as `@Telemetry("status/ready")` remains inside the owning
mechanism's `Values` group. Athena inspects annotations once when the mechanism is registered and
then uses cached readers and writers.

`TelemetryValue` fields remain supported for custom reader/writer behavior and dynamic declarations:

```java
public final TelemetryValue custom =
        TelemetryValue.writableNumber(this::readCustom, this::writeCustom);
```

Custom values appear at `/Athena/Mechanisms/<mechanism path>/Values/<field path>` and are sampled at
10 Hz. PID/feedforward values embedded in a control stay beneath that control's `Config` group.
Application-specific setpoints should be writable `@Telemetry` fields, while generated device setup
operations stay beneath the corresponding device.

## Autonomous preview

Athena inspects the selected autonomous `Action` without running it. Path providers contribute the
geometry they already load for execution, so Choreo paths, splits, marker Actions, sequences,
parallel groups, and both sides of conditional branches are visible before enable.

- `/Athena/Auto/Selected`: selected routine name.
- `/Athena/Auto/Running`: frozen action currently owned by autonomous, empty outside autonomous.
- `/Athena/Auto/Plan`: readable ordered Action-tree description.
- `/Athena/Auto/Path`: combined `Pose2d[]` that can be added directly to an AdvantageScope field.
- `/Athena/Auto/Paths/*`: one `Pose2d[]` per path or conditional branch for independent coloring.
- `/Athena/Auto/Events`: marker poses, paired by index with `/Athena/Auto/EventLabels`.

Declare an `AutoChooser` as a Robot field. Athena publishes it and updates the preview automatically.
Changing the chooser is inert: the selected Action is frozen and activated only when autonomous
begins, and selection changes cannot replace the running Action.

## Mechanism simulation

Declare simulation beside the same hardware used by a mechanism. Athena discovers these models
when the mechanism graph is registered:

```java
private final SimModel armSimulation = SimModel.motor(arm)
        .encoder(armEncoder)
        .range(rawRange)
        .limit(homeSwitch, rawHomePosition, 0.1);
```

Use `SimModel.flywheel`, `arm`, `elevator`, or `motor` as appropriate. Compose multiple axes with
`SimModel.compose`. Swerve kinematics already supplies its drive/steer and field-pose model.

An auto test should use the normal `RobotRuntime`, alternate one robot cycle with one simulation
physics step, and assert both physical movement and scheduler completion:

```java
SimulationSession simulation = SimulationSession.create();
RobotRuntime runtime = RobotRuntime.simulated(simulation).register(mechanism);
runtime.request(autoAction);

for (int cycle = 0; cycle < 250; cycle++) {
    double now = cycle * 0.02;
    runtime.robotPeriodic(now, 0.02);
    runtime.simulationPeriodic(now + 0.02, 0.02);
}
```

This exercises the same action scheduler, arbitration, constraints, control loops, encoder paths,
hooks, and simulation declarations used by the robot rather than replacing the mechanism with a
test-only implementation.
