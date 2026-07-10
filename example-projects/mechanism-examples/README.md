# Athena mechanism examples

This project shows declarative motor, sensor, control-loop, action-composition, and simulation patterns.

## Feedback-completing actions

Position and velocity targets can complete when their configured feedback enters an explicit tolerance:

```java
public final Action high = lift.position(1.25)
        .untilWithin(0.03);

public final Action ready = wheel.velocity(78.0)
        .untilWithin(2.0);
```

The tolerance uses the configured feedback units. It controls action completion independently from any tolerance configured on the PID controller.

Travel limits and motion behavior belong to the reusable control binding, so percent, voltage, and closed-loop actions share the same safety policy:

```java
private final ControlBinding lift = Controls.position(motor)
        .feedback(height)
        .pid(0.5, 0.0, 0.0)
        .constraint(Constraints.range(travel))
        .profile(MotionProfiles.trapezoid(0.8, 1.8));
```

`TwoJointArm` demonstrates constraints that inspect another Athena feedback signal. `Turret` demonstrates field-heading correction before bounded-angle planning, range enforcement, profiling, PID, output saturation, and predictive stopping.

## Repeating between feedback targets

`Actions.cycle().run(...)` advances whenever its current child action completes and wraps after its final step:

```java
public final Action exercise = Actions.cycle()
        .run(position.position(20.0).untilWithin(2.0))
        .run(position.position(85.0).untilWithin(2.0));
```

Request the cycle once with an edge binding. A later action request for the same mechanism root replaces it:

```java
operator.start().onActive(arm.exercise);
operator.back().onActive(arm.stow);
```

Avoid `whileActive(exercise)` for a persistent cycle because that requests and resets the cycle every active tick.

## Coordinating several mechanisms

Parallel feedback-completing actions let a superstructure wait for every participating mechanism before advancing:

```java
scoreHigh = Actions.sequence()
        .run(Actions.parallel(shooter.podium, arm.trapScore, elevator.high))
        .timeout(2.0)
        .then(Actions.parallel(shooter.podium, arm.trapScore, intake.eject, elevator.high));
```

The timeout provides a fallback so a failed or obstructed mechanism cannot prevent the sequence from advancing forever.
