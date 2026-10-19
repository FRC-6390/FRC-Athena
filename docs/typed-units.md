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

## Motion kind

`ControlBinding.motionKind()` returns `MotionKind.ROTARY` for angle units, `MotionKind.LINEAR` for
distance units, and `MotionKind.UNSPECIFIED` for raw bindings. Model and effort features use it to
pick the right formulas.

## Removed runtime API (typed-runtime branch)

The runtime, drivetrain, localization, WPILib and robot modules now take WPILib units
(`LinearVelocity`, `AngularVelocity`, `Distance`, `Angle`, `Time`, `Frequency`, `Rotation2d`, `Translation2d`,
`Pose2d`) on team-facing methods. The old primitive overloads are removed rather than deprecated. Primitive
hot-path access stays available through `RobotVelocity.raw()`, `MeasurementStdDevs.raw()` and the
`Raw` nested classes. Each row is a removed or renamed public member; unchanged signatures are not listed.

| Removed | Use instead |
| --- | --- |
| `new RobotVelocity(double, double, double)` | `RobotVelocity.robot(LinearVelocity, LinearVelocity, AngularVelocity)` (same ROBOT frame; the record constructor is private) |
| `RobotVelocity.robot(double, double, double)` / `field(double, double, double)` | `robot(...)` / `field(...)` with `MetersPerSecond.of(...)`, `RadiansPerSecond.of(...)` |
| `RobotVelocity.linear(double, double[, VelocityFrame])` | `linear(LinearVelocity, LinearVelocity[, VelocityFrame])` |
| `RobotVelocity.angular(double[, VelocityFrame])` | `angular(AngularVelocity[, VelocityFrame])` |
| `RobotVelocity.withLinear(double, double)` / `withAngular(double)` | `withLinear(LinearVelocity, LinearVelocity)` / `withAngular(AngularVelocity)` |
| `RobotVelocity.linearMagnitude()` returning `double` | returns `LinearVelocity`; `.in(MetersPerSecond)` for a double |
| `RobotVelocity.fieldToRobot(double)`, `robotToField(double)`, `inFrame(frame, double)` | same names, take `Rotation2d` |
| `RobotVelocity.clamp(double, double)` | `clamp(LinearVelocity, AngularVelocity)` |
| `RobotVelocity.xMetersPerSecond()`, `yMetersPerSecond()`, `angularRadiansPerSecond()` | `x()`, `y()`, `angular()` (typed); primitives via `raw().xMetersPerSecond()` and so on |
| `RobotVelocityPool.robotRelative(double)`, `fieldRelative(double)`, `combined(frame, double)` | take `Rotation2d` |
| `RobotVelocityPool.radiansPerSecond()` | `angularVelocity()` (returns `AngularVelocity`) |
| `SwerveKinematics.drive(RobotVelocityPool, DoubleSupplier)` | `drive(RobotVelocityPool, Supplier<Rotation2d>)` |
| `SwerveOdometry.pose()` returning `PoseSnapshot` | `pose()` returns `Pose2d`; the primitive view is `poseSnapshot()` |
| `Point2d` record | `Translation2d` (`new Translation2d(x, y)`, `getX()`, `getY()`) |
| `Point2d.from(PoseSnapshot)` | `new Translation2d(pose.xMeters(), pose.yMeters())` |
| `Point2d.distance(Point2d)` | `Translation2d.getDistance(Translation2d)` |
| `Geometry2d.contains(Point2d)`, `contains(PoseSnapshot)`, `expanded(double)` | `contains(Translation2d)`, `contains(Pose2d)`, `expanded(Distance)` |
| `Circle2d(Point2d, double)` record, `Circle2d.of(double, double, double)` | `Circle2d.of(Translation2d, Distance)` or `Circle2d.of(Distance, Distance, Distance)`; `center()` is a `Translation2d`, `radius()` a `Distance` |
| `Circle2d` / `Polygon2d` / `Rectangle2d` `signedDistance(Point2d)` returning `double` | `signedDistance(Translation2d)` returns `Distance`; `signedDistancePrimitive(Translation2d)` returns `double` |
| `Rectangle2d(double x4)` record, `Rectangle2d.of(double x4)`, `Rectangle2d.field(double, double)` | `Rectangle2d.of(Distance x4)`, `Rectangle2d.field(Distance, Distance)`; `minimumX()` and the others return `Distance` |
| `Polygon2d.of(Point2d...)`, `vertices()` returning `List<Point2d>` | `of(Translation2d...)`, `List<Translation2d>` |
| `runtime.geometry.AllianceTransform` | renamed `runtime.geometry.AllianceMapping` (internal, double meters); `translation(...)` takes and returns `Translation2d` |
| `wpilib.geometry.AllianceTransform.rotate180(double, double)`, `mirrorX(double)`, `mirrorY(double)` | take `Distance`, for example `rotate180(Meters.of(16.54), Meters.of(8.21))` |
| `MeasurementSignal.stdDevs(double, double)`, `stdDevs(double, double, double)` | `stdDevs(Distance, Angle)`, `stdDevs(Distance, Distance, Angle)` |
| `latencyLimit(double)`, `maxAge(double now, double maxAge)` on `MeasurementSignal` and `ConfiguredMeasurementSignal` | `latencyLimit(Time)`, `maxAge(Time now, Time maxAge)` |
| `MeasurementStdDevs(double, double, double)` record and `MeasurementStdDevs.of(double, double, double)` | `MeasurementStdDevs.of(Distance, Distance, Angle)`; primitive input is `MeasurementStdDevs.Raw.of(double, double, double)` (radians) |
| `MeasurementStdDevs.xMeters()`, `yMeters()`, `headingRadians()` | `xStdDev()`, `yStdDev()` (`Distance`), `headingStdDev()` (`Angle`); primitives via `raw()` |
| `MeasurementStdDevs.translationVariance()` | `raw().translationVarianceMetersSquared()` |
| `Measurements.poses(Supplier<PoseSnapshot>)`, `poseAndSpeeds(Supplier<PoseSnapshot>, ...)` | take `Supplier<Pose2d>`; the single-sample `Measurements.pose(PoseSnapshot)` stays, and `pose(Pose2d)` is added |
| `PoseMeasurement.timing(double, double)`, `PoseMeasurementSample.timing(double, double)` | `timing(Time, Time)` |
| `PoseSignal.value()` returning `Optional<PoseSnapshot>` | returns `Optional<Pose2d>` |
| `PoseSignal.pose()` returning `PoseSnapshot` | returns `Pose2d`; the primitive view is `poseSnapshot()` |
| `PoseSignal.singleTagStdDevs(double x3)`, `multiTagStdDevs(double x3)` | `(Distance, Distance, Angle)` |
| `PoseSignal.distanceStdDevScaling(double, double)` | `distanceStdDevScaling(Distance, double)` |
| `TelemetryValue.points(Supplier<Point2d[]>)` | `Supplier<Translation2d[]>` |
| `MechanismTelemetryPublisher.readOnlyPeriodSeconds(double)` | `readOnlyPeriod(Time)` |
| `RobotRuntime.fastSignalSampling(double[, executor])` | `fastSignalSampling(Time[, executor])` |
| `RobotRuntime.localizationMaxAge(double)` | `localizationMaxAge(Time)` |
| `RobotRuntime.mechanismTracePeriodSeconds(double)` | `mechanismTracePeriod(Time)` |
| `RobotRuntime.recordDisable(double)` | `recordDisable(Time)` |
| `RuntimeWorker(String, double, Runnable)`, `RuntimeWorker.every(String, double, Runnable)` | take a `Time` period |
| `SimExplorer.duration(double)`, `starvationSeconds(double)` | `duration(Time)`, `starvation(Time)` |
| `InputModel.buttonPressesPerSecond(double)`, `povChangesPerSecond(double)` | `buttonPresses(Frequency)`, `povChanges(Frequency)` |
| `InputModel.meanHoldSeconds(double)` | `meanHold(Time)` |
| `InputModel.axisWalk(double, double)` | `axisWalk(Frequency, double)` |
| `ModeSchedule.dwell(mode, double, double)`, `ModeSchedule.Segment(mode, double seconds)` | `dwell(mode, Time, Time)`, `Segment(mode, Time duration)` |

Examples of the conversion:

```java
// Before
RobotVelocity.robot(1.0, 0.0, 0.5);
RobotVelocity.field(1.0, 0.0, Math.toRadians(90));
runtime.fastSignalSampling(0.005, scheduler);
driveTrain.drive(pool, () -> pose.getRotation().getRadians());

// After
RobotVelocity.robot(MetersPerSecond.of(1.0), MetersPerSecond.of(0.0), RadiansPerSecond.of(0.5));
RobotVelocity.field(MetersPerSecond.of(1.0), MetersPerSecond.of(0.0), DegreesPerSecond.of(90));
runtime.fastSignalSampling(Milliseconds.of(5), scheduler);
driveTrain.drive(pool, () -> pose.getRotation());
```

## Summary

| Need | Use |
| --- | --- |
| Declare encoder units | `EncoderDevice.measures(AngleUnit | DistanceUnit)`, `travelPerRevolution(Distance)` |
| Raw encoder position to rotations | `EncoderDevice.rotationsFromPosition(Measure)` |
| Typed binding | `Controls.position(motor, unit)`, `Controls.velocity(motor, unit)` |
| Typed target | `position(Measure)`, `velocity(Measure)` |
| Primitive supplier with unit | `position(DoubleSupplier, unit)`, `velocity(DoubleSupplier, unit)` |
| Typed tolerance | `Action.untilWithin(Measure)` |
| Settled tolerance | `untilWithin(t).settled(seconds)`, `untilWithin(t).maxVelocity(Measure)`, `untilSettled(t, seconds)` |
| Typed limits | `Constraints.range(Measure, Measure)`, `Constraints.motion(Measure, Measure)` |
| Typed interpolation | `interpolate(model, input, inputUnit)` and `at(Measure, Measure)` |
| Rotary or linear | `ControlBinding.motionKind()` |
