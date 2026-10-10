# Athena mechanism examples

This project shows declarative devices, reusable controls, action composition, mechanism templates, and simulation. `Robot` is the mechanism root: Athena discovers its child mechanisms, actions, controls, hooks, devices, and simulation models automatically. There is no registration or `configure()` method.

`Controls` receives that root and binds directly to `robot.<mechanism>.<action>`. Controller bindings are declarations owned by the discovered `Gamepad`; `onTrue` makes one request, `whileTrue` maintains a request lease, and `onFalse` requests the release action.

## Feedback-completing actions

Position and velocity targets can complete when their configured feedback enters an explicit tolerance:

```java
public final Action high = lift.position(1.25)
        .untilWithin(0.03);

public final Action ready = wheel.velocity(78.0)
        .untilWithin(2.0);
```

The tolerance uses the configured feedback units and controls action completion. Tolerance is intentionally not part
of `PidGains`; use `untilWithin(...)`, `control.at(...)`, or `control.isAt(...)` where completion or coordination needs
a tolerance.

Travel limits and motion behavior belong to the reusable control binding, so percent, voltage, and closed-loop actions share the same safety policy:

```java
private final ControlBinding lift = Controls.position(motor)
        .feedback(height)
        .pid(6.0, 0.0, 0.0)
        .constraint(Constraints.range(travel))
        .profile(MotionProfiles.trapezoid(0.8, 1.8));
```

Athena PID gains produce volts: `kP`, `kI`, and `kD` are volts per feedback-error unit, integrated-error unit, and error-rate unit respectively. Feedforward is also volts, so PID and feedforward add directly before the final output is limited to the available voltage.

`PidGains.of(p, i, d).iZone(error)` enables integral accumulation only while the absolute error is inside that zone.
The zone is expressed in the control's configured feedback units. A zero zone means the integral is always enabled.
Athena applies anti-windup at the 12 V boundary without clearing accumulated integral state.

Feedforward supports `kS`, `kV`, `kA`, and constant `kG`:

```java
control.ff(kS, kV, kA, kG);
```

The three-argument overload remains `ff(kS, kV, kG)` and defaults `kA` to zero. Profiled controls provide velocity
and acceleration references. For an unprofiled position target, `kS` follows position-error direction. Angle-based
arm gravity compensation is state-dependent and should remain a custom loop rather than using constant `kG`.

`TwoJointArm` demonstrates constraints that inspect another Athena feedback signal. `Turret` demonstrates field-heading correction before bounded-angle planning, range enforcement, profiling, PID, output saturation, and predictive stopping.

## Templates and device slots

`TemplateRoller` is a compact non-swerve `MechanismTemplate`. Its motor slot applies mechanism-owned configuration to whichever real device fills it:

```java
public final MotorSlot<TemplateRoller> motor = Slots.motor(this, "motor", this::configureIfReady)
        .coast()
        .currentLimit(20);
```

The robot supplies the physical device while the template owns its requirements:

```java
public final TemplateRoller templateRoller = new TemplateRoller().motor.fill(
        Constants.RIO.motor(MotorControllerKinds.SPARK_FLEX, MotorKinds.NEO, 15));
```

This also shows the explicit controller-plus-physical-motor overload for a non-default pairing. Filling the slot creates the template actions and simulation model before Athena discovers the robot graph. The left-stick Y threshold in `Controls` runs this roller and releases it when the threshold becomes inactive.

## Simulation

`PositionElevator`, `VelocityShooter`, `TwoJointArm`, `LimitedManualArm`, and `TemplateRoller` declare `SimModel` fields beside the devices they represent. Athena finds those models automatically in simulation and runs the same actions, constraints, profiles, and control bindings used on the robot. Limits declared with `SimModel.limit(...)` drive the same `DigitalInputDevice` declarations used by real code.

`examples/SimulationHarness` shows the lower-level `SimulationSession` API used by tests and non-WPILib hosts:
explicit model registration, simulated handles, custom physics stepping, pose reset, and a vision field layout.

## Interpolation, stalls, and telemetry

`VelocityShooter.distanceShot` linearly interpolates wheel speed from the live-tunable
`shotDistanceMeters` field. The field and measured velocity use `@Telemetry`, while `FieldTelemetry` shows
custom `TelemetryValue` declarations and circle, rectangle, and polygon field overlays. `Robot` selects the
capture trace profile so arbitration, control, motor, and hook channels are available during diagnostics.

`IndexedIntake.stopOnStall` derives a timed/rearming stall event from the motor's configured current limit,
commanded voltage, and measured velocity. The rising edge requests the ordinary neutral action.

## Hardware and runtime extension references

`examples/HardwareConnections` compile-checks named CAN buses, PWM and quadrature encoders, analog, SPI, I2C,
serial, USB, NavX ports, and CTRE/REV-specific options. `examples/RuntimeWorkerExamples` shows optional inline
runtime workers and failure reporting. These are reference declarations rather than additional mechanisms, so
copy only the connections and workers used by a real robot.

## Stop versus release

The shooter and roller `stop` actions use `neutral()`. That stops Athena control output and lets the motor use its configured coast mode. Arms and elevators configure brake mode so their neutral behavior resists motion. Use `percent(0.0)` only when commanding zero output is intentionally part of an active control action; it is not the same as releasing control.

## Repeating between feedback targets

`Actions.cycle().run(...)` advances whenever its current child action completes and wraps after its final step:

```java
public final Action exercise = Actions.cycle()
        .run(position.position(20.0).untilWithin(2.0))
        .run(position.position(85.0).untilWithin(2.0));
```

Request the cycle once with an edge binding. A later action request for the same mechanism root replaces it:

```java
operator.start().onTrue(arm.exercise);
operator.back().onTrue(arm.stow);
```

Avoid `whileTrue(exercise)` for a persistent cycle because that requests and resets the cycle every active tick.

## Coordinating several mechanisms

Parallel feedback-completing actions let a superstructure wait for every participating mechanism before advancing:

```java
scoreHigh = Actions.sequence()
        .run(Actions.parallel(shooter.podium, arm.trapScore, elevator.high))
        .timeout(2.0)
        .then(Actions.parallel(shooter.podium, arm.trapScore, intake.eject, elevator.high));
```

The timeout provides a fallback so a failed or obstructed mechanism cannot prevent the sequence from advancing forever.

## Typed units and characterization

`SingleJointArm` declares its encoder with `measures(Degrees)` and its binding with `Controls.position(motor, Degrees)`.
Targets, tolerances (`untilWithin(Degrees.of(2))`), and constraints are measures of the same unit. A binding built on
a mismatched encoder fails when the mechanism is declared.

The same arm declares a `Characterization`. Athena exposes its tests (`QuasistaticForward`, `DynamicForward`,
`FitModel`, and the others) as dashboard Actions. `SimModel.arm(motor).model(model)` uses the active model as the
simulated plant. The arm starts from a code default model (`defaults(...)`) until a fit runs on the robot, and the
simulation reads the model when the robot is built. Checks come from the same binding: `blocked` and `load`. The
`stowWhenBlocked` hook uses `blocked` to back off to stow.

## Motion policies

`SingleJointArm` sets `retarget(Retarget.SMOOTH)` and `release(Release.HOLD)` on its binding. `stow` is bounded with
`atMost(...)` and uses `interruptible(Interrupt.AFTER_COMPLETE)`, so a newer request waits for it. `pickupOrFault`
is an `Actions.attempt` chain: a tight tolerance first, then a looser one, then `Actions.fault(...)`.

## Power, thermal, and staging

`VelocityShooter.SHOOT` is a shared `PowerProfile` used by both shooter motors. `endgame` is an `update(...)` Action
that lowers the profile's share while the match ends, and `normalPower` restores it.

`StagedArm` derates its motor with `Thermal.derate(...)`. Its `ArmHealth` enum drives a `Staging`: `STRAINED` caps
effort, `HOT` also caps speed and acceleration, and `SAFE_SHUTDOWN` requests stow. `lift.staged(health)` applies those
caps only while the lift runs. The worst true stage wins, and a stage is left only after its condition has been false
for two seconds.

## Persisted tuning, lock groups, and recording

`VelocityShooter.shotDistanceMeters` is `persist = true`, so the value survives reboots and deploys. The shooter's PID
edits persist through `persistTuning(Locks.InMatch.class)`. `StagedArm.calibrated` and its tuning use the custom
`Calibration` group. `Robot` defines that group's condition in its constructor with `tuningLock(...)`. Writes in a locked
group are rejected by the robot.

`VelocityShooter.distanceShot` is a persisted interpolation. Its `recordShot` Action inserts the current distance and
wheel speed as a new table point, which tunes the table from the field.

## Robot policies and replay tests

`Robot` declares three typed policy fields, which `AthenaRobot` discovers like `AutoChooser`:

- `ReplayPolicy`: a 48 MiB buffer, match and disable flushes, and retention limits.
- `ArcpPolicy`: the ARCP control port (TCP 5808) and realtime UDP port (5809) and a bandwidth budget. Use `ArcpPolicy.disabled()` to turn ARCP off.
- `PowerPolicy`: a 7.5 V floor and staggered starts.

`src/test/java/frc/robot/ReplayRecordingTest.java` records a simulated teleop run of the arm, then replays the
recording against the same code with `ReplaySimulation`. The replay must report zero command differences.

Run the tests with `./gradlew test` from this project directory.
