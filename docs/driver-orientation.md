# Driver orientation

Field-oriented teleop needs one heading: the direction the driver calls forward. Field geometry (Choreo paths,
localization, `AllianceTransform`) always has +X pointing away from the **blue** driver station. A driver's forward
points away from their **own** station, which is field +X on blue and 180 degrees away on red. `DriverOrientation`
(`ca.frc6390.athena.drivetrain.DriverOrientation`) owns that offset, so robot code does not keep a second yaw and
re-align it after auto.

## The math

```
forward       = allianceForward + driverOffset       blue: 0, red: pi, driverOffset: 0 unless the driver reset
driverHeading = normalize(fieldHeading - forward)     0 when the robot faces the driver's forward
```

Feed `driverHeading` to field-oriented drive. Stick input stays in the driver frame (a `RobotVelocity.field(...)`
channel is the driver frame when the heading is the driver heading), so forward moves the robot away from its own
driver station on both alliances.

## API

```java
DriverOrientation orientation = DriverOrientation.of(
        localization::pose,                 // Supplier<PoseSnapshot>: localized field pose
        () -> DriverStation.getAlliance()   // Supplier<Optional<DriverOrientation.Alliance>>, empty while unknown
                .map(a -> a == Alliance.Red ? DriverOrientation.Alliance.RED : DriverOrientation.Alliance.BLUE),
        () -> autoRan || visionSeeded,      // BooleanSupplier: true once auto or vision has set the pose
        localization::reset);               // Function<PoseSnapshot, ActionBinding>: resets the localized pose

DoubleSupplier heading = orientation.headingRadians();
Action drive = kinematics.drive(velocityPool, orientation);   // same as drive(pool, heading)
Action driverResetHeading = orientation.resetHeading();        // Start button
Action restoreForward = orientation.restoreAllianceForward();  // undo a driver reset
Action teleopStart = orientation.assumeDriverForward();        // only if the pose is not seeded
```

Other methods: `forwardRadians()`, `driverHeadingRadians()`, `allianceKnown()`.

## Behaviour

| Case | Behaviour |
| --- | --- |
| Auto to teleop | Nothing to re-align. Auto ends with the localized pose, which carries the field heading. Teleop reads the same heading and the alliance offset. |
| Alliance unknown at boot | Forward is blue until the Driver Station reports an alliance. The value changes on the next cycle when it arrives. In competition the alliance is known before enable. `allianceKnown()` reports it. |
| No auto run, pose seeded by vision | `assumeDriverForward()` does nothing. The vision pose heading is used. |
| No auto run, no seed | `assumeDriverForward()` resets the pose heading to the alliance forward, so the robot is assumed to face away from its own wall. The translation is kept. Place the robot facing away from its own wall, or press reset. |
| Driver reset in teleop | `resetHeading()` stores an offset so the robot's current facing is forward. The field pose is not changed, so field-oriented drive and field-pose consumers (aiming, paths) stay consistent. |
| Mid-auto pose reset | The heading is read from the pose on every call. A reset to a correct field pose changes the field heading, and driver forward does not move. |

## Notes

- The pose heading must be correct, because driving uses it directly. Vision seeding and `assumeDriverForward()` are
  how the boot heading becomes correct. A wrong seed is a localization error and needs fixing there.
- A driver reset is relative to the alliance default. Calling `restoreAllianceForward()` (for example on
  `Events.autonomousInit()`) clears it.
- Tests: `athena-drivetrain/src/test/java/ca/frc6390/athena/drivetrain/DriverOrientationTest.java`.
