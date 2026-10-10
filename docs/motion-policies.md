# Motion Policies

Motion policies decide what a control does when targets change, when a running Action may be
replaced, what the motor does once nothing drives it, and how fallbacks and faults are handled.
The rules are in [`action-runtime-contract.md`](action-runtime-contract.md#motion-policies). This page
shows how to use them.

## Setting defaults on a control binding

```java
public final class Arm implements Mechanism {
    private final ControlBinding<AngleUnit> pivot = Controls.position(PIVOT_MOTOR, Degrees)
            .feedback(PIVOT_ENCODER)
            .pid(PidGains.of(4.0, 0.0, 0.2))
            .profile(new MotionProfile(180.0, 360.0))   // deg/s, deg/s^2
            .retarget(Retarget.SMOOTH)                  // default; shown for clarity
            .release(Release.HOLD);                     // keep the stow angle when the lease ends

    public final Action stow = pivot.position(Degrees.of(0));
    public final Action score = pivot.position(Degrees.of(75));
}
```

`HOLD` means that when no Action drives the pivot, it keeps closed-loop control of the last
reference, so gravity does not drop the arm. The default release is `NEUTRAL`, which stops the motor.

## Retargeting while moving

A new target that arrives while a profile runs re-plans from where the profile currently is, so the
motion stays smooth:

```java
// SMOOTH (default): the arm keeps its current speed while it turns toward the new target.
Actions.sequence().run(pivot.position(Degrees.of(75))).forTime(0.5, pivot.position(Degrees.of(20)));

// IMMEDIATE on one request: re-plan from the measured angle. Use it when the reference is known
// to be wrong, for example after a slip or a jam.
pivot.position(Degrees.of(20)).retarget(Retarget.IMMEDIATE).request();
```

A takeover by a different Action on the same binding inherits the reference while SMOOTH is active. An
IMMEDIATE takeover starts from measured feedback.

## Holding a running Action (interrupt policies)

Use `interruptible(...)` on the root Action you request. A newer request for overlapping resources
then waits until the running Action allows it:

```java
// Finish the shot before the feeder can retarget the shooter. Values are in the binding's units.
Action shot = shooter.velocity(3000.0)
        .atMost(Duration.ofSeconds(1.5))
        .interruptible(Interrupt.AFTER_COMPLETE);

// Let a newer request replace this one as soon as the robot-defined condition is true.
Action pass = shooter.velocity(1500.0)
        .atMost(Duration.ofSeconds(3))
        .interruptible(Interrupt.after(() -> ball.isClearedOfShooter()));   // robot-defined condition

shot.request();
pass.request();   // held until the shot completes; disjoint mechanisms are unaffected
```

Rules that apply in every case:

- `cancel(action)` always releases the running Action. A held request then starts.
- Disable overrides the commitment. Held requests start while the robot is disabled.
- `Actions.abort(mechanism)` cancels the running Actions on that mechanism, ignoring their policies.
- An unbounded Action is rejected for `AFTER_COMPLETE` and `after(...)`. Add a timeout:

```java
pivot.position(Degrees.of(75))
        .interruptible(Interrupt.AFTER_COMPLETE);   // IllegalArgumentException: requires a bounded Action

pivot.position(Degrees.of(75)).atMost(Duration.ofSeconds(2))
        .interruptible(Interrupt.AFTER_COMPLETE);   // accepted
```

## Stopping on purpose

```java
// Emergency stop for one mechanism. Commitments do not protect the running Actions.
Action stopShooter = Actions.abort(shooterMechanism);
stopShooter.request();
```

## Release examples

```java
// Stow: brake along the profile, then hold where it stops.
pivot.release(Release.DECELERATE);

// Intake roller: no closed-loop target, so release has no effect beyond NEUTRAL.
// Stop commanding and let the motor's declared mode decide.
intakeRoller.release(Release.NEUTRAL);

// Brake the motors after a lease ends, until the next command restores the declared mode.
// Supported by backends with runtime configuration.
pivot.release(Release.neutral(MotorNeutralMode.BRAKE));
```

`Release.HOLD` and `Release.DECELERATE` apply only to position and velocity controls. For percent
and voltage controls both behave as `NEUTRAL`. `Release.neutral(mode)` takes effect only on
backends that support runtime configuration. Others keep the declared mode.

## Fallback chains

Try a precise approach first, then a coarser one, and finally report a fault:

```java
// vision.alignTo and odometry.driveTo are robot-defined Actions.
Action acquire = Actions.attempt(vision.alignTo(target).atMost(Duration.ofSeconds(1.0)))
        .within(1.0)                                  // attempt 1: must finish within 1 s
        .orTry(odometry.driveTo(target))
        .within(2.0)                                  // attempt 2: must finish within 2 s
        .orElse(Actions.fault("could not reach the target"));
```

An attempt succeeds when it completes before its time limit. A failed or timed-out attempt moves to
the next one. `orElse` runs only when every attempt has failed.

## Faults

`Actions.fault(message)` completes immediately and reports the message through the runtime failure
reporter. Use it as a terminal step, not as a way to stop hardware. Stopping is done with
`neutral()`, `cancel(...)`, or `Actions.abort(...)`.

## Gain changes are bumpless

A gain change applied while a control is running (for example a dashboard write to `kP`) does not
cause a step in the output. The integral is re-seeded so the next output equals the previous one:

```java
PidGains gains = PidGains.of(2.0, 0.5, 0.0);
ControlBinding<?> arm = Controls.position(ARM).feedback(ENCODER).pid(gains);
// While arm.position(3.0) is running, gains.telemetry().get("p").set(6.0) changes kP
// without a voltage step at that cycle.
```

Feedforward-only controls have no integrator and are not bumpless. Their output steps by the
feedforward change.
