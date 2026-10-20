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
- `Constraints.motion(RotationsPerSecond.of(200), RotationsPerSecondPerSecond.of(200))` keeps the position meaning:
  the first measure is the largest velocity (the target is clamped to it) and the second is the largest
  acceleration (the reference slews by at most that per second). The velocity loop enforces both.

```java
private final ControlBinding<?> shooter = Controls.velocity(leader, Units.Rotations)
        .constraints(
                Constraints.range(Units.RotationsPerSecond.of(-200), Units.RotationsPerSecond.of(200)),
                Constraints.motion(Units.RotationsPerSecond.of(200), Units.RotationsPerSecondPerSecond.of(200)),
                Constraints.maxVoltage(Units.Volts.of(12)));
```

A position binding rejects the per-second range, since its target is a position.

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
        .at(Units.Inches.of(40.0), () -> rpsCommand, Units.Rotations)
```

`at(Measure, DoubleSupplier, Unit)` takes a fixed point with a runtime value. The value unit is the binding's unit
(`Units.Rotations` for a velocity binding, which then reads rotations per second);
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
| `EncoderDevice.position()`, `absolutePosition()`, `velocity()` (`double`) | `positionAngle()` / `positionDistance()`, `absolutePositionAngle()` / `absolutePositionDistance()`, `velocityAngular()` / `velocityLinear()` |
| `EncoderDevice.positionFromRotations(double)`, `velocityFromRotationsPerSecond(double)` (`double`) | `encoder.raw().positionFromRotations(...)` for Athena internals; robot code reads the typed accessors |
| `ImuSource.yawDegrees()` and the other primitive IMU readings | typed accessors (`yaw()`, `yawRate()`, ...); internals use `raw()`; implementers override `readYawDegrees()` and friends |
| `EncoderDevice` as a `PositionSignal` or `VelocitySignal` | `encoder.positionSignal()`, `encoder.velocitySignal()`, `encoder.absoluteSignal()`; `feedback(encoder)` still works |

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

## Removed API in the small-mechanism migration

These untyped forms are gone. The typed form is the only public form, except where the last table says otherwise.

| Removed | Use instead |
| --- | --- |
| `TorqueVelocityGains.staticAmps(double)`, `staticAmps()` | `staticCurrent(Current)`, `staticCurrent()` |
| `TorqueVelocityGains.loadEstimate(double)`, `loadTimeConstant()` returning `double` | `loadEstimate(Time)`, `loadTimeConstant()` returning `Time` |
| `TorqueVelocityGains(double, double, double, double, double, double)` | `(double p, double i, double d, Current staticCurrent, double inertia, Time loadTimeConstant)` |
| `Staging.Change.seconds()` (`double`) | `time()` (`Time`) |
| `BindingCheck.forSeconds(double)`, `BindingCheck(..., double seconds, ...)` | `forTime(Time)`, `BindingCheck(..., Time duration, ...)` |
| `EffortRequest(double currentLimitAmps)`, `currentLimitAmps()` | `EffortRequest(Current currentLimit)`, `currentLimit()`; `limitValue()` for the per-cycle double |
| `EffortLimit.ceilingAmpsMax()` | `ceilingMax()` (`Current`) |
| `MechanismModel.withPayloadDelta(double)` | `withPayloadDelta(Voltage)`. The delta is a change of kG, which is a volts amplitude, so it is not a mass. |
| `MechanismModel.inertia()` returning `double` | `inertia()` returning `MomentOfInertia`; the measure is NaN when unknown, as before |
| `MechanismDeviceActions.motorVoltage(MotorDevice, double)`, `DeviceActionFactory.motorVoltage(...)` | `motorVoltage(MotorDevice, Voltage)` |
| `MechanismDeviceActions.imuSetYaw(ImuSource, double)`, `DeviceActionFactory.imuSetYaw(...)` | `imuSetYaw(ImuSource, Angle)` |
| `MotionPlanners.boundedAngular(double period)` | `boundedAngular(Angle period, AngleUnit unit)`; the period is converted to `unit` once, so pass the binding's unit |
| `RecordMetadata.at(double)` | `at(Time)`. Per-cycle recorder code uses the package-private `atSeconds(double)`. |
| `ControlOutput.voltage(double)` (team form) | `ControlOutput.voltage(Voltage)`. See the last table for the primitive form that remains. |

## Internals left primitive

These stay `double` because they run every control cycle, or because no WPILib unit fits. Converting them would allocate a
measure per cycle.

| Member | Why it stays primitive |
| --- | --- |
| `MechanismScheduler`, `MechanismRuntime` `nowSeconds` / `dt` | per-cycle clock (`EventContext(nowSeconds, dtSeconds, ...)` too) |
| `Staging.evaluate(double nowSeconds, ...)`, `Staging` transition `nowSeconds` | per-cycle clock; the transition builds the `Time` for `Change` |
| `PidController.calculate(measurement, target, dtSeconds)`, `PersistentStore.flushIfDue(nowSeconds)`, `TelemetryPersistence.periodic(nowSeconds, ...)` | per-cycle |
| `PathRuntime` `context.timeInStateSeconds()`, `hasElapsed` | per-cycle clock |
| `MotionReference(position, velocity, acceleration)`, `MotionProfileRuntime.step(..., dtSeconds)` | per-cycle reference; the units come from the binding |
| `MotionProfile(double, double)`, `MotionProfiles.trapezoid(double, double)`, `ControlBinding.profile(MotionProfile)` | a raw binding has no unit, so the profile is in feedback units. Typed bindings use `Constraints.motion(Measure, Measure)`. |
| `MechanismModel.gravityVolts(double)`, `feedforwardVolts(double, ...)` | per-cycle feedforward |
| `MechanismModel.payloadDelta()`, `MotorVoltage(motor, double)`, `ImuSetYaw(imu, double)`, `ControlVoltage(...)`, `RecordMetadata(double, ...)` record components | record carriers for the per-cycle `Output` and `ImuYawMutation` interfaces; the typed factories are the API |
| `DeviceAction` `motorVoltage(MotorDevice, DoubleSupplier)`, `imuSetYaw(ImuSource, DoubleSupplier)` | per-cycle suppliers. A `Supplier<Voltage>` would allocate a measure every cycle. |
| `EffortLimit.threshold()` (N m) | WPILib has no torque unit |
| `BindingCheck` clock (`DoubleSupplier`) | per-cycle monotonic clock |

Still `double` in the public API, owned by another change: `ControlOutput.voltage(double)` (kept because `FeedforwardGains` and `PidGains`
use it), `Actions.voltage(ControlBinding, double)` (internal), and `MechanismDeviceActions.encoderSetPosition(EncoderDevice, double)`.
The control API itself is typed-only: `ControlBinding.voltage`, `Constraints.motion`, and the unit-less `Controls.position(motor)`
factories are removed (see the removed control API table).

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

## Raw views and implementer hooks

Athena's per-cycle code (control loops, odometry, simulation, telemetry) reads primitive doubles so it does not
allocate a measure every cycle. Those primitives are internal. Each type with primitive readings exposes them
through one nested `Raw` view, reached with `raw()`, and its javadoc says "Internal, not for robot code". Within the
same package the primitive methods are package-private. Robot code uses the typed accessor in the table.

| Type | `raw()` members | Typed team API |
| --- | --- | --- |
| `MotorDevice` | `appliedVoltageVolts()`, `supplyCurrentAmps()`, `statorCurrentAmps()`, `velocityRotationsPerSecond()`, `fusedTemperatureCelsius()` | `appliedVoltage()`, `supplyCurrent()`, `statorCurrent()`, `velocity()`, `temperature()` |
| `MotorConstants` | `torqueConstantNmPerAmp()`, `speedConstantRadPerSecPerVolt()`, `expectedUnloadedSpeedRadPerSec(double)` | `expectedUnloadedSpeed(Voltage)` |
| `MotorStallSignal` | `durationSeconds()`, `rearmSeconds()`, `currentThresholdAmps()`, `currentAmps()` | `duration()`, `rearmAfter()`, `currentThreshold()`, `current()` |
| `PowerProfile` | `sustainedAmps()`, `burstAmps()`, `burstSeconds()`, `rampAmpsPerSecond()` | `sustainedCurrent()`, `burstCurrent()`, `burstDuration()`, `rampStep()` |
| `PowerPolicy` | `floorVolts()`, `staggerSeconds()`, `defaultBatteryOhms()` | `floor()`, `staggerStarts()`, `defaultBatteryResistance()` |
| `Thermal` | `derateFractionCelsius(double)`, `loadCelsius(double)` | `derateFraction(Temperature)`, `load(Temperature)` |
| `Range` | `minimumInUnit()`, `maximumInUnit()` | `minimum()`, `maximum()` |
| `ImuSource` / `ImuDevice` | `yawDegrees()`, `pitchDegrees()`, `rollDegrees()`, `angleDegrees()`, `*RateDegreesPerSecond()`, `linearAccelerationXG()` / `YG()` / `ZG()`, `lastUpdateSeconds()`, `applyYawDegrees(ctx, double)` | `yaw()`, `pitch()`, `roll()`, `angle()`, `yawRate()`, `pitchRate()`, `rollRate()`, `linearAccelerationX()` / `Y()` / `Z()`, `applyYaw(ctx, Angle)` |
| `EncoderDevice` | `position()`, `absolutePosition()`, `velocity()`, `positionFromRotations(double)`, `velocityFromRotationsPerSecond(double)` | `positionAngle()`, `positionDistance()`, `absolutePositionAngle()`, `absolutePositionDistance()`, `velocityAngular()`, `velocityLinear()` |

For example, a robot reads a motor's supply current with `motor.supplyCurrent()`. Athena's control code reads
`motor.raw().supplyCurrentAmps()` for the same value without allocating a measure.

### Implementer SPI

Some interfaces must keep public primitive methods because vendors and simulation implement them. Those methods are
the implementer SPI. They are documented as not for robot code, and the names say so.

- `ImuSource` hooks are renamed so they do not look like the team API: `readYawDegrees()`,
  `readPitchDegrees()`, `readRollDegrees()`, `readAngleDegrees()`, `readYawRateDegreesPerSecond()`,
  `readPitchRateDegreesPerSecond()`, `readRollRateDegreesPerSecond()`, `readLinearAccelerationXG()` / `YG()` / `ZG()`,
  `readLastUpdateSeconds()`, and `writeYawDegrees(ActionContext, double)`. Robot code calls the typed accessors or
  `raw()`, which delegates to these hooks.
- `MotorHandle`, `EncoderHandle` and `ImuHandle` (in `ca.frc6390.athena.hardware.backend`) keep their primitive
  methods (`appliedVoltage()`, `positionRotations()`, `yawDegrees()`, ...). Teams never receive these handles, and
  the interfaces are documented as backend SPI.

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
