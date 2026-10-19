# Driver orientation

Field-oriented teleop needs one heading: the direction the driver calls forward. Field geometry (Choreo paths,
localization, `AllianceTransform`) always has +X pointing away from the **blue** driver station. A driver's forward
points away from their **own** station: field +X on blue, 180 degrees away on red. `DriverOrientation`
(`ca.frc6390.athena.drivetrain.DriverOrientation`) owns that offset, so robot code does not keep a second yaw and
does not re-align it after auto.

## The math

```
forward       = allianceForward + driverOffset       blue: 0, red: pi, driverOffset: 0 unless the driver reset
driverHeading = normalize(fieldHeading - forward)     0 when the robot faces the driver's forward
```

The drive heading is `driverHeading`. Stick input stays in the driver frame: a `RobotVelocity.field(...)` channel
is the driver frame when the heading is the driver heading, so forward moves the robot away from its own driver
station on both alliances.

## Robot code

In the robot, the orientation is one declaration in the drivetrain mechanism:

```java
private final DriverOrientation orientation = DriverOrientations.forDriverStation(() -> localization);
public final Action resetDriverHeading = orientation.resetHeading();
public final Action pooledDrive = kinematics.drive(velocity, orientation);
```

`localization` is the `Localization` built after the drivetrain (`private Localization localization;`, assigned in
`paths(Localization)`). The supplier is read on each use, so the orientation can be declared first. If the
localization already exists, `DriverOrientations.forDriverStation(localization)` works too.

`DriverOrientations` is in `athena-wpilib` (`ca.frc6390.athena.wpilib.drivetrain`). It is the only place that
touches the Driver Station (`DriverStation.getAlliance()`). Drivetrain and localization have no WPILib dependency.
It wires:

| Input | Source |
| --- | --- |
| alliance | `DriverStation.getAlliance()`, empty while unknown |
| field pose | `Localization.pose()` |
| pose seeded | `Localization.seeded()` |
| pose reset | `Localization.reset(PoseSnapshot)` |

## API

| Member | Purpose |
| --- | --- |
| `DriverOrientations.forDriverStation(Localization \| Supplier<Localization>)` | Builds the orientation from the Driver Station and the localization. |
| `DriverOrientation.headingRadians()` | Heading supplier for drive. |
| `kinematics.drive(pool, orientation)` | Drive overload, same as `drive(pool, orientation.headingRadians())`. |
| `resetHeading()` | Driver reset (Start). Current facing becomes forward. Stores an offset only. |
| `restoreAllianceForward()` | Clears the driver reset. |
| `assumeDriverForward()` | Sets the pose heading to the alliance forward if the pose is unseeded. Runs automatically at teleop start, see below. |
| `hooks()` | The teleop-start hook, `assumeForwardAtTeleop`. |
| `forwardRadians()`, `driverHeadingRadians()`, `allianceKnown()` | Status for telemetry. |

### Teleop-start hook (automatic)

`DriverOrientation` implements `HookGroup`. A mechanism field whose value is a `HookGroup` contributes its hooks
with the mechanism, so the teleop-start hook is discovered as `<field>.assumeForwardAtTeleop` (for example
`drivetrain.orientation.assumeForwardAtTeleop`). The robot does not bind it. The hook runs
`assumeDriverForward()` on every teleop enable. It is a no-op while the pose is seeded.

The orientation must be a field of a registered mechanism for the hook to run. A plain local variable is not
discovered.

### Localization seeding

`Localization.seeded()` is false at boot and becomes true when:

- `reset(PoseSnapshot)` is applied (for example by the path follower at the start of auto), or
- a vision frame passes the Kalman node's filters (`seedFromVisionWhileDisabled` seeds the same way).

It never returns to false. A robot that resets the localization at boot (for example to the origin) is seeded
immediately, and the boot assumption is skipped. Do not reset at boot unless the pose is known.

## Behaviour

| Case | Behaviour |
| --- | --- |
| Auto to teleop | Nothing to re-align. Auto ends with the localized pose, which carries the field heading. Teleop reads the same heading and the alliance offset. |
| Alliance unknown at boot | Forward is blue until the Driver Station reports an alliance. The value changes on the next cycle when it arrives. In competition the alliance is known before enable. `allianceKnown()` reports it. |
| No auto run, pose seeded by vision | The teleop-start hook does nothing. The vision pose heading is used. |
| No auto run, no seed | The teleop-start hook resets the pose heading to the alliance forward, so the robot is assumed to face away from its own wall. The translation is kept. Place the robot facing away from its own wall, or press reset. |
| Driver reset in teleop | `resetHeading()` stores an offset so the robot's current facing is forward. The field pose is not changed, so field-oriented drive and field-pose consumers (aiming, paths) stay consistent. |
| Mid-auto pose reset | The heading is read from the pose on every call. A reset to a correct field pose changes the field heading, and driver forward does not move. |

## Notes

- Driving uses the localized heading directly. A wrong seed is a localization error and needs fixing there.
- A driver reset is relative to the alliance default. `restoreAllianceForward()` clears it (for example from
  `Events.autonomousInit()`).
- Tests: `athena-drivetrain/.../DriverOrientationTest.java` (math and drive direction),
  `athena-localization/.../WpilibLocalizationParityTest.java` (`seededTracksResetAndAcceptedVisionOnly`),
  `athena-wpilib/.../drivetrain/DriverOrientationsTest.java` (facade and discovered hook),
  `athena-robot/.../DriverOrientationRuntimeTest.java` (actions and the drive overload through `RobotRuntime`).
