# Typed units

Athena control bindings can be declared with WPILib units (`edu.wpi.first.units`) instead of
plain doubles. The unit is part of the declaration: targets, feedback, constraints, tolerances and
interpolation points are converted to base-unit doubles once, when the robot declares the
mechanism. The control loop still works on primitive doubles and never allocates measures.

Plain-double controls such as `Controls.position(motor)` keep working. Encoder declarations use the typed
builders below.

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

## Reading an encoder

Encoder reads are typed to match the declaration. Use the accessor for the dimension you declared:

```java
Angle turretAngle = turretEncoder.positionAngle();               // measures(Degrees): an angle
AngularVelocity turretRate = turretEncoder.velocityAngular();
Angle turretAbsolute = turretEncoder.absolutePositionAngle();

Distance travel = elevatorEncoder.positionDistance();            // measures(Meters): a distance
LinearVelocity speed = elevatorEncoder.velocityLinear();
```

- `positionAngle()`, `absolutePositionAngle()` and `velocityAngular()` require an angle declaration.
- `positionDistance()`, `absolutePositionDistance()` and `velocityLinear()` require a distance declaration.
- A read in the other dimension throws `IllegalStateException`, naming the accessor to use. The check runs
  on every read, so a mistake fails at the first call instead of returning a number in the wrong unit.
- The returned measure has the same magnitude as the value in the declared unit, so `positionAngle().in(Units.Degrees)`
  is the configured position in degrees with no extra rounding beyond the unit conversion.

To feed a control, pass the encoder itself (`ControlBinding.feedback(encoder)`). `positionSignal()`,
`velocitySignal()` and `absoluteSignal()` return the feedback signal types for the mixed case, such as a
position from one encoder and a velocity from another.

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
- `position(DoubleSupplier, U unit)` and `velocity(DoubleSupplier, U unit)` take primitive suppliers
  with an explicit unit. The conversion factor is computed once.
- `untilWithin(Measure)` on an action uses the control's unit for the tolerance, or the rate unit for
  velocity actions.
- `continuous(Measure, Measure)` and `interpolate(...)` have typed forms (see below).

Declared constraints are converted to the binding's unit when the binding is declared. A typed
constraint can be attached to any binding of the same dimension.

## Interpolation

```java
public final Action distanceShot = wheel.interpolate(InterpolationKinds.LINEAR, () -> shotMeters, Units.Meters)
        .at(Units.Meters.of(2.0), Units.RPM.of(3000))
        .at(Units.Meters.of(4.0), Units.RPM.of(4200));
```

The third argument is the unit of the input supplier. Points are measures of that unit, and values
are measures of the binding's unit (or its per-second rate for velocity bindings).

## Mismatch errors

Binding a unit-typed control to an encoder whose unit differs fails when the mechanism is declared:

```
Control unit Degrees does not match encoder ctre:cancoder_... which measures Rotations (position feedback).
```

Raw bindings (`Controls.position(motor)`) have no unit, so typed methods on them throw
`IllegalStateException`, and any encoder can be used as feedback. Arm feedforward on a raw binding needs an
angle unit from the encoder's `measures(...)`.

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
| `EncoderDevice.position()`, `absolutePosition()`, `velocity()` (`double`) | `positionAngle()` / `positionDistance()`, `absolutePositionAngle()` / `absolutePositionDistance()`, `velocityAngular()` / `velocityLinear()` |
| `EncoderDevice.positionFromRotations(double)`, `velocityFromRotationsPerSecond(double)` (`double`) | `encoder.raw().positionFromRotations(...)` for Athena internals; robot code reads the typed accessors |
| `EncoderDevice` as a `PositionSignal` or `VelocitySignal` | `encoder.positionSignal()`, `encoder.velocitySignal()`, `encoder.absoluteSignal()`; `feedback(encoder)` still works |

## Removed timing API

Durations are WPILib `Time` measures. The `double` seconds overloads are gone.

| Removed | Use instead |
| --- | --- |
| `Actions.waitSeconds(double)` | `Actions.waitSeconds(Time)`, for example `Actions.waitSeconds(Seconds.of(0.2))` |
| `Action.timeout(double)`, `Actions.timeout(Action, double)`, `Actions.timeout(DeviceAction, double)` | `timeout(Time)`, `Actions.timeout(action, Time)` |
| `DeviceAction.timeout(Number)` | `timeout(Time)` |
| `Action.settled(double)`, `Actions.settled(Action, double)` | `settled(Time)`, `Actions.settled(action, Time)` (`java.time.Duration` overloads remain) |
| `Action.untilSettled(double, double)` | `untilSettled(double, Time)` (tolerance is still a double in control units; see the note below) |
| `Attempt.within(double)` | `within(Time)` |
| `Sequence.forTime(double, Action)`, `Sequence.waitSeconds(double)`, `Sequence.timeout(double)` | `Time` overloads |
| `Cycle.forTime(double, Action)` | `forTime(Time, Action)` |
| `Actions.WaitSeconds.seconds()`, `Actions.Timeout.seconds()` | `duration()`, which returns a `Time` |

`Action.untilSettled(double, Time)` keeps a `double` tolerance for now, so the tolerance is still
in control units. Its typed form, `untilSettled(Measure, Time)`, is the one to use with unit-typed bindings.

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
