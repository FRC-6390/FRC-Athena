# Typed units

Athena control bindings can be declared with WPILib units (`edu.wpi.first.units`) instead of
plain doubles. The unit is part of the declaration: targets, feedback, constraints, tolerances and
interpolation points are converted to base-unit doubles once, when the robot declares the
mechanism. The control loop still works on primitive doubles and never allocates measures.

Untyped code keeps working. `Controls.position(motor)` and `EncoderDevice.conversion(...)` are unchanged.

## Declaring a unit-typed encoder

```java
// Output-shaft absolute encoder that reports degrees.
EncoderDevice turretEncoder = EncoderDevice.of(EncoderKinds.CANCODER, 4)
        .measures(Units.Degrees)
        .reduction(10.0);            // 10 sensor rotations per mechanism rotation
```

- `measures(AngleUnit)` declares the unit `position()` and `velocity()` are reported in. It sets the
  conversion to one full revolution in that unit (360 for degrees). Call `conversion(...)` afterwards
  to override it.
- `measures(DistanceUnit)` declares a linear encoder. Set the travel per revolution with
  `travelPerRevolution(Distance)`, for example `travelPerRevolution(Inches.of(4 * Math.PI))`.
- `reduction(double)` reads as the reduction on the robot (10 means 10:1). `gearing(GearRatio)` is the
  typed alternative to `gearRatio(GearRatio)`. The old `gearRatio(...)` and `conversion(...)` methods
  still exist.

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

An encoder that declares no `measures(...)` is also rejected for a typed binding.
Raw bindings (`Controls.position(motor)`) have no unit, so typed methods on them throw
`IllegalStateException`, and any encoder can be used as feedback.

## Motion kind

`ControlBinding.motionKind()` returns `MotionKind.ROTARY` for angle units, `MotionKind.LINEAR` for
distance units, and `MotionKind.UNSPECIFIED` for raw bindings. Model and effort features use it to
pick the right formulas.

## Summary

| Need | Use |
| --- | --- |
| Declare encoder units | `EncoderDevice.measures(AngleUnit | DistanceUnit)`, `travelPerRevolution(Distance)` |
| Typed binding | `Controls.position(motor, unit)`, `Controls.velocity(motor, unit)` |
| Typed target | `position(Measure)`, `velocity(Measure)` |
| Primitive supplier with unit | `position(DoubleSupplier, unit)`, `velocity(DoubleSupplier, unit)` |
| Typed tolerance | `Action.untilWithin(Measure)` |
| Typed limits | `Constraints.range(Measure, Measure)`, `Constraints.motion(Measure, Measure)` |
| Typed interpolation | `interpolate(model, input, inputUnit)` and `at(Measure, Measure)` |
| Rotary or linear | `ControlBinding.motionKind()` |
