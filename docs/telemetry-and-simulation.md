# Mechanism telemetry and simulation

Athena builds one immutable `MechanismTraceSnapshot` after each mechanism cycle. The snapshot is
made from values already sampled or calculated by the runtime; exporting or reading it does not
refresh a motor, encoder, digital input, or dynamic action supplier.

## Live telemetry

`AthenaRobot` publishes the latest snapshots under:

`/Athena/Mechanisms/<root>/Trace`

Nested mechanisms receive filtered views at paths such as
`/Athena/Mechanisms/robot/intake/Trace`. Each child view contains only the candidates, controls,
motors, and hooks owned by that mechanism subtree.

The available profiles are:

- `OFF`: no Athena mechanism topics.
- `SUMMARY`: packed scheduler state plus action labels.
- `CAPTURE`: state plus packed arrays for candidates, controls, motors, and hooks at 50 Hz.

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

## Custom telemetry and live tuning

Mechanisms can declare their own sparse values without calling `SmartDashboard.put*` every loop:

```java
public final TelemetryValue armPosition = TelemetryValue.number(encoder::position);
public final TelemetryValue homed = TelemetryValue.bool(homeSwitch::active);
public final TelemetryValue target = TelemetryValue.number(20.0);

private final ControlBinding control = Controls.position(armMotor)
        .feedback(encoder)
        .pid(2.0, 0.0, 0.0)
        .ff(0.1, 0.4, 0.0);
public final Action move = control.position(target::number);
```

Values appear directly under `/Athena/Mechanisms/<mechanism path>/<field path>` and are sampled at
10 Hz. Athena automatically adds a writable `disabled` value to each declared motor and control,
and exposes PID/feedforward components beneath the control that owns them. There is no separate
tuning declaration or enable gate. Runtime changes take effect on the next control cycle; hardware
closed-loop configuration is only resent when its effective gain values actually change.

CAN identity, bus selection, inversion, follower topology, and controller current limits remain
deployment-time hardware configuration. Application-specific setpoints have no existing device or
control declaration to discover, so those remain explicit writable `TelemetryValue.number(...)`
fields as shown above.

## Autonomous preview

Athena inspects the selected autonomous `Action` without running it. Path providers contribute the
geometry they already load for execution, so Choreo paths, splits, marker Actions, sequences,
parallel groups, and both sides of conditional branches are visible before enable.

- `/Athena/Auto/Selected`: selected routine name.
- `/Athena/Auto/Plan`: readable ordered Action-tree description.
- `/Athena/Auto/Path`: combined `Pose2d[]` that can be added directly to an AdvantageScope field.
- `/Athena/Auto/Paths/*`: one `Pose2d[]` per path or conditional branch for independent coloring.
- `/Athena/Auto/Events`: marker poses, paired by index with `/Athena/Auto/EventLabels`.

Call `AutoRuntime.select(...)` when the chooser changes—not only at autonomous init—so the preview
tracks the disabled-period selection. The selected Action is prepared once, and that same instance
is activated when autonomous begins.

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
