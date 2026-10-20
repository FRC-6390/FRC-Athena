# Typed units

Athena control bindings can be declared with WPILib units (`edu.wpi.first.units`) instead of
plain doubles. The unit is part of the declaration: targets, feedback, constraints, tolerances and
interpolation points are converted to base-unit doubles once, when the robot declares the
mechanism. The control loop still works on primitive doubles and never allocates measures.

Every control binding declares its unit. There are no unit-less `Controls.position(motor)` or
`Controls.velocity(motor)` factories: a dimensionless binding says so with `Units.Value`. Encoder
declarations use the typed builders below.

## Declaring a unit-typed encoder

```java
// Output-shaft absolute encoder that reports degrees.
EncoderDevice turretEncoder = EncoderDevice.of(EncoderKinds.CANCODER, 4)
        .measures(Units.Degrees)
        .reduction(10.0);            // 10 sensor rotations per mechanism rotation
```

Every encoder declares the unit it measures. `EncoderDevice.of(...)` starts as rotations (one sensor
revolution is one rotation); `measures(...)` changes it.

- `measures(AngleUnit)` declares the unit `position()` and `velocity()` are reported in. It sets one full
  revolution in that unit (360 for degrees).
- `measures(DistanceUnit)` declares a linear encoder. Set the travel per revolution with
  `travelPerRevolution(Distance)`, for example `travelPerRevolution(Inches.of(4 * Math.PI))`.
- `reduction(double)` reads as the reduction on the robot (10 means 10:1). `gearing(GearRatio)` takes a
  `GearRatio` object instead (for example one shared constant).
- `rotationsFromPosition(Measure)` converts a typed position into raw sensor rotations. The measure must be in
  the encoder's dimension; it is converted to the declared unit first. `measure(double)` wraps a value in the
  declared unit when a stored double is all there is.

## Declaring a unit-typed control

```java
public final ControlBinding<AngleUnit> turret =
        Controls.position(turretMotor, Units.Degrees)
                .feedback(turretEncoder)                       // must measure Degrees
                .pid(4.0, 0.0, 0.1)
                .constraint(Constraints.range(Units.Degrees.of(-170), Units.Degrees.of(170)))
                .constraint(Constraints.motion(Units.DegreesPerSecond.of(360),
                        Units.DegreesPerSecondPerSecond.of(720)));

public final Action face = turret.position(Units.Degrees.of(90)).untilWithin(Units.Degrees.of(1.5));
public final Action sweep = turret.position(() -> sweepDegrees, Units.Degrees); // primitive supplier
```

- `Controls.position(motor, AngleUnit)` and `Controls.position(motor, DistanceUnit)` return
  `ControlBinding<AngleUnit>` or `ControlBinding<DistanceUnit>`. `Controls.velocity(motor, unit)` does
  the same for velocity controls, where targets are per-second rates of the unit.
- `position(Measure)` and `velocity(Measure)` accept any measure of the binding's dimension, so
  `Degrees.of(90)` works on a `Rotations` binding and `RPM.of(120)` on a `Rotations` velocity binding.
  A measure of the wrong dimension is rejected when declared.
- `position(DoubleSupplier, Unit unit)`, `velocity(DoubleSupplier, Unit unit)` and `set(DoubleSupplier, Unit unit)`
  take primitive suppliers with the unit stated. The conversion factor is computed once, when the action is
  declared, and a unit of the wrong dimension is rejected then. Suppliers are primitive on purpose: a
  `Supplier<Measure>` would allocate a measure every control cycle. The unit is required because a bare
  `DoubleSupplier` does not say what its numbers mean, and a `velocity.velocity(() -> rps)` with no unit is
  exactly the mistake the typed system removes.
- `untilWithin(Measure)` on an action uses the control's unit for the tolerance, or the rate unit for
  velocity actions.
- `continuous(Measure, Measure)` and `interpolate(...)` have typed forms (see below).
- `error(Measure)` returns the signed error as a measure in the binding's unit (a rate for velocity bindings).
- `isAt(Measure, Measure)` and `at(Measure, Measure)` (a `BooleanSupplier`) take the target and tolerance as measures.
- `armFf(s, v, g, Angle offset)` takes the phase offset as an angle; any angle unit converts to radians once.
- `voltage(Voltage)` and `voltage(DoubleSupplier, VoltageUnit)` set the voltage output.

Declared constraints are converted to the binding's unit when the binding is declared. A typed
constraint can be attached to any binding of the same dimension.

Output voltage limits are output-stage constraints, not setpoint limits:

```java
.constraint(Constraints.maxVoltage(Volts.of(12.0)))            // drive voltage within +/- 12 V
.constraint(Constraints.clamp(Volts.of(-10.0), Volts.of(8.0))) // asymmetric drive voltage limits
```

`Constraints.range(...)` limits the target (the setpoint), so it cannot replace an output clamp.

Velocity bindings express typed constraints in their own rate units, because their target is a rate:

- `Constraints.range(RotationsPerSecond.of(-200), RotationsPerSecond.of(200))` bounds the velocity target.
- `Constraints.motion(acceleration, jerk)` takes an acceleration and its rate, for example
  `RotationsPerSecondPerSecond.of(200)` and `RotationsPerSecondPerSecond.per(Seconds).of(200)`. A position
  binding reads the same call as velocity and acceleration, so a jerk-sized second limit is rejected there.
  The velocity loop records these limits; it does not enforce the acceleration limit yet.

```java
private final ControlBinding<?> shooter = Controls.velocity(leader, Units.Rotations)
        .constraints(
                Constraints.range(Units.RotationsPerSecond.of(-200), Units.RotationsPerSecond.of(200)),
                Constraints.motion(Units.RotationsPerSecondPerSecond.of(200),
                        Units.RotationsPerSecondPerSecond.per(Units.Seconds).of(200)),
                Constraints.maxVoltage(Units.Volts.of(12)));
```

## Interpolation

```java
public final Action distanceShot = wheel.interpolate(InterpolationKinds.LINEAR, () -> shotMeters, Units.Meters)
        .at(Units.Meters.of(2.0), Units.RPM.of(3000))
        .at(Units.Meters.of(4.0), Units.RPM.of(4200));
```

The third argument is the unit of the input supplier. Points are measures of that unit, and values
are measures of the binding's unit (or its per-second rate for velocity bindings).

The key can be sampled at runtime, with each value stated in its own unit:

```java
wheel.interpolate(InterpolationKinds.LINEAR, () -> distanceInches, Units.Inches)
        .at(Units.Inches.of(40.0), () -> rpsCommand, Units.RotationsPerSecond)
```

`at(Measure, DoubleSupplier, Unit)` takes a fixed point with a runtime value;
`at(DoubleSupplier, Unit, DoubleSupplier, Unit)` takes both at runtime.

## Mismatch errors

Binding a unit-typed control to an encoder whose unit differs fails when the mechanism is declared:

```
Control unit Degrees does not match encoder ctre:cancoder_... which measures Rotations (position feedback).
```

A binding always has a unit, so there are no raw bindings. Encoder feedback must measure the binding's
unit, and an encoder that measures another unit is rejected when declared. A dimensionless binding
(`Controls.position(motor, Units.Value)`) rejects angle and distance measures.

## Removed encoder API

The encoder declaration is typed-only. These members are gone:

| Removed | Use instead |
| --- | --- |
| `EncoderDevice.conversion(double)` | `measures(AngleUnit)` (one revolution in that unit) or `measures(DistanceUnit).travelPerRevolution(Distance)` |
| `EncoderDevice.units(EncoderUnit)` | `measures(AngleUnit)` or `measures(DistanceUnit)` |
| `EncoderDevice.units()` and the `EncoderUnit` enum | `measuresUnit()` (a WPILib `Unit`) |
| `EncoderDevice(source, inverted, gear, conversion, offset, EncoderUnit[, failurePolicy[, baseline]])` | `new EncoderDevice(source, inverted, gear, conversion, offset, failurePolicy, baseline, measuresUnit)` with a WPILib unit |
| `EncoderDevice.rotationsFromPosition(double)` | `rotationsFromPosition(Measure)`, for example `Degrees.of(90)`; or `rotationsFromPosition(encoder.measure(value))` for a stored double |
| `EncoderUnit.RAW` | the default declaration (rotations, one revolution per raw turn) |
| `EncoderUnit.METERS` | `measures(Units.Meters)` |

## Removed timing API

Durations are WPILib `Time` measures. The `double` seconds overloads are gone.

| Removed | Use instead |
| --- | --- |
| `Actions.waitSeconds(double)` | `Actions.waitSeconds(Time)`, for example `Actions.waitSeconds(Seconds.of(0.2))` |
| `Action.timeout(double)`, `Actions.timeout(Action, double)`, `Actions.timeout(DeviceAction, double)` | `timeout(Time)`, `Actions.timeout(action, Time)` |
| `DeviceAction.timeout(Number)` | `timeout(Time)` |
| `Action.settled(double)`, `Actions.settled(Action, double)` | `settled(Time)`, `Actions.settled(action, Time)` (`java.time.Duration` overloads remain) |
| `Action.untilSettled(double, double)` | `untilSettled(Measure, Time)` |
| `Attempt.within(double)` | `within(Time)` |
| `Sequence.forTime(double, Action)`, `Sequence.waitSeconds(double)`, `Sequence.timeout(double)` | `Time` overloads |
| `Cycle.forTime(double, Action)` | `forTime(Time, Action)` |
| `Actions.WaitSeconds.seconds()`, `Actions.Timeout.seconds()` | `duration()`, which returns a `Time` |

`untilSettled(Measure, Time)` is the only form; see the removed control API table below.

## Removed control API

Every control binding is typed. These unit-less members are gone (migrated in the typed-only control pass):

| Removed | Use instead |
| --- | --- |
| `Controls.position(motor)`, `Controls.velocity(motor)` | `Controls.position(motor, unit)`, `Controls.velocity(motor, unit)`; `Units.Rotations` for a motor-rotation control, `Units.Value` for a dimensionless one |
| `Controls.position(motor, more...)`, `Controls.velocity(motor, more...)` (several motors) | `Controls.position(motor, unit).followers(more...)` |
| `Controls.position(sink)`, `Controls.velocity(sink)` | `Controls.position(sink, unit)`, `Controls.velocity(sink, unit)`; `Units.Value` for a plain scalar sink |
| `ControlBinding.position(double)`, `velocity(double)`, `set(double)` | `position(Measure)`, `velocity(Measure)`, `set(Measure)`, for example `position(Units.Degrees.of(90))` |
| `ControlBinding.position(DoubleSupplier)`, `velocity(DoubleSupplier)`, `set(DoubleSupplier)` | `position(DoubleSupplier, Unit)`, `velocity(DoubleSupplier, Unit)`, `set(DoubleSupplier, Unit)`, for example `velocity(() -> rps, Units.Rotations)` |
| `ControlBinding.voltage(double)` | `voltage(Voltage)`, for example `voltage(Volts.of(4.0))` |
| `ControlBinding.voltage(DoubleSupplier)` | `voltage(DoubleSupplier, VoltageUnit)`, for example `voltage(() -> v, Units.Volts)` |
| `ControlBinding.error(double)` | `error(Measure)`, which returns a measure in the binding's unit |
| `ControlBinding.isAt(double, double)`, `at(double, double)`, `at(DoubleSupplier, double)` | `isAt(Measure, Measure)`, `at(Measure, Measure)`, `at(DoubleSupplier, Unit, Measure)` |
| `ControlBinding.continuous(double, double)` | `continuous(Measure, Measure)` |
| `ControlBinding.slipCheck(encoder, ratio, double)` | `slipCheck(encoder, ratio, Measure)` |
| `ControlBinding.interpolate(model, DoubleSupplier)`, `interpolate(CurveMapping, DoubleSupplier)` | `interpolate(model, DoubleSupplier, Unit)`, `interpolate(CurveMapping, DoubleSupplier, Unit)` |
| `ControlBinding.armFf(s, v, g, double radians)` | `armFf(s, v, g, Angle)`, for example `armFf(s, v, g, Units.Degrees.of(-30))` |
| `FeedforwardGains.arm(s, v, g, double radians)` | `FeedforwardGains.arm(s, v, g, Angle)` |
| `FeedforwardGains.gravityOffsetRadians()` | `gravityOffset()`, which returns an `Angle`; use `.in(Units.Radians)` for the radian value |
| `InterpolatedControlAction.at(double, double)`, `at(double, DoubleSupplier)`, `at(DoubleSupplier, double)`, `at(DoubleSupplier, DoubleSupplier)` | `at(Measure, Measure)`, `at(Measure, DoubleSupplier, Unit)`, `at(DoubleSupplier, Unit, DoubleSupplier, Unit)` |
| `Action.untilWithin(double)`, `Actions.untilWithin(action, double)` | `untilWithin(Measure)` |
| `Action.maxVelocity(double)`, `Actions.maxVelocity(action, double)` | `maxVelocity(Measure)`, a rate of the binding unit per second |
| `Action.untilSettled(double, Time)` | `untilSettled(Measure, Time)` |
| `Constraints.clamp(double, double)` | `Constraints.clamp(Voltage, Voltage)`, an output-stage voltage limit |
| `Constraints.clamp(double)` | `Constraints.maxVoltage(Voltage)`, a symmetric output-stage voltage limit |
| `Constraints.motion(double, double)` | `Constraints.motion(Measure, Measure)`, for example `DegreesPerSecond.of(60)` and `DegreesPerSecondPerSecond.of(120)` (on a velocity binding: acceleration and its rate) |

## Motion kind

`ControlBinding.motionKind()` returns `MotionKind.ROTARY` for angle units, `MotionKind.LINEAR` for
distance units, and `MotionKind.UNSPECIFIED` for raw bindings. Model and effort features use it to
pick the right formulas.

## Summary

| Need | Use |
| --- | --- |
| Declare encoder units | `EncoderDevice.measures(AngleUnit | DistanceUnit)`, `travelPerRevolution(Distance)` |
| Raw encoder position to rotations | `EncoderDevice.rotationsFromPosition(Measure)` |
| Typed binding | `Controls.position(motor, unit)`, `Controls.velocity(motor, unit)` |
| Typed target | `position(Measure)`, `velocity(Measure)` |
| Primitive supplier with unit | `position(DoubleSupplier, unit)`, `velocity(DoubleSupplier, unit)` |
| Typed tolerance | `Action.untilWithin(Measure)` |
| Settled tolerance | `untilWithin(t).settled(Time)`, `untilWithin(t).maxVelocity(Measure)`, `untilSettled(t, Time)` |
| Typed limits | `Constraints.range(Measure, Measure)`, `Constraints.motion(Measure, Measure)` |
| Typed interpolation | `interpolate(model, input, inputUnit)` and `at(Measure, Measure)` |
| Rotary or linear | `ControlBinding.motionKind()` |
