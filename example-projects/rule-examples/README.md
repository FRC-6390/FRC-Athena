# Athena rule and constraint examples

This project keeps safety policy on the mechanism control that owns the actuator. Athena discovers `Robot`, its child mechanisms, devices, actions, hooks, and simulation models automatically; no registration or `configure()` method is required.

## Guarded motion

`GuardedArm` combines four protections on one position binding:

```java
private final ControlBinding position = Controls.position(motor)
        .feedback(motor.encoder())
        .pid(0.72, 0.0, 0.0)
        .constraints(
                Constraints.range(travel),
                Constraints.lower(home),
                Constraints.upper(hardStop))
        .profile(MotionProfiles.trapezoid(70.0, 180.0));
```

Controls also expose their configured feedback for cross-mechanism coordination. `GuardedArm` publishes a live
condition, and `Conveyor` refuses to feed until the arm is actually at the scoring position:

```java
public final BooleanSupplier atScore = position.at(90.0, 2.0);

public final Action feedWhenLoaded = Actions.when(
        () -> armAtScore.getAsBoolean() && beamBreak.active())
        .then(motor.percent(0.7))
        .otherwise(stop);
```

`position.position()`, `position.velocity()`, `position.measurement()`, `position.error(target)`, and
`position.isAt(target, tolerance)` provide immediate reads. `at(target, tolerance)` returns a reusable
`BooleanSupplier` for events, action conditions, constraints, and other mechanisms. These APIs require an explicit
`.feedback(...)` binding so their units cannot silently fall back to raw motor rotations.

The numeric range corrects out-of-range targets. The lower and upper constraints reject movement farther into an active end stop while still allowing movement away from it. Both accept `BooleanSupplier`, so the source can be a DIO input, computed condition, or another mechanism signal.

PID output is volts, and the profile limits requested velocity and acceleration before PID is evaluated. Manual percent actions use the same binding, so they do not bypass its constraints. The homing event resets the encoder through an Athena action when the lower input rises.

## Conditional actions and events

`Conveyor.feedWhenLoaded` chooses an action from the sampled beam break and a test-mode override. Lifecycle hooks enable the override on test init and clear it on test exit. Controller `whileTrue` bindings maintain an action only while held; `onFalse` requests `neutral()` so the motor is released to its configured coast behavior.

All DIO devices use the unified bus API:

```java
private final DigitalInputDevice beamBreak = Constants.RIO.dio(3)
        .digitalInput()
        .inverted();
```

## Simulation

The arm simulation declares its range and both limit inputs beside the real devices. The conveyor declares a basic motor model. Athena discovers these models only when running in simulation, while the mechanism actions, constraints, and event bindings remain the same code used on the robot.
