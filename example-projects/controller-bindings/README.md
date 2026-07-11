# Controller bindings

This project shows the current Athena controller flow without WPILib command-based triggers.
`Robot` owns the mechanisms, and `Controls` receives the robot so bindings can refer directly to
`robot.driveTrain` and `robot.intake`.

## Bindings shown

| Input | Behavior |
| --- | --- |
| Left Y / right X | Continuously supplies squared arcade-drive inputs while teleop is enabled |
| Y | Toggles Athena's latched `ToggleSignal` between full and 40% speed |
| A | Sends a one-cycle stop request on the press edge |
| Right bumper | Holds the intake collect action |
| Left bumper | Holds the intake eject action |

The bumper signals are composed so pressing both requests neither direction. Held actions own their
target devices while the signal is active and release that ownership when the signal becomes false,
so a separate release-time stop binding is not required.

Axes are converted to `DoubleSupplier`s after deadband, sign-preserving squaring, and inversion. The
drivetrain action reads those suppliers every runtime cycle; controls do not stage mutable values or
call `Action.request()` from a periodic callback.

## Build

From this project directory:

```powershell
./gradlew.bat compileJava
```
