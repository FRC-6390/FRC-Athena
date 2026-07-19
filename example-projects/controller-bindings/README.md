# Controller bindings

This project shows the current Athena controller flow without WPILib command-based triggers.
`Robot` owns the mechanisms, and `Controls` receives the robot so bindings can refer directly to
`robot.driveTrain` and `robot.intake`.

## Bindings shown

| Input | Behavior |
| --- | --- |
| Left Y / right X | Continuously supplies shaped and slew-limited arcade-drive inputs while teleop is enabled |
| Y | Toggles Athena's latched `ToggleSignal` between full and 40% speed |
| A | Sends a one-cycle stop request on the press edge |
| Right bumper | Holds the intake collect action |
| Left bumper | Holds the intake eject action |

`AdvancedControls` uses a second gamepad to keep the gesture examples independent from the driver bindings.
It demonstrates shared single/double/triple/ranged click recognition, debounced buttons, long and short holds,
key-repeat behavior, axis hysteresis, explicitly controlled toggle state, `allOf`/`anyOf`/`noneOf`, timed chords,
ordered input sequences, `CommandAction` requirements, and WPILib command wrapping.

The bumper signals are composed so pressing both requests neither direction. Held actions own their
target devices while the signal is active and release that ownership when the signal becomes false,
so a separate release-time stop binding is not required.

The forward axis uses live-tunable RC rate, expo, super rate, and slew rate. The turn axis shows a
custom sign-preserving curve with separate acceleration and deceleration slew rates. Axes are
converted to `DoubleSupplier`s after their declared processing pipeline. The
drivetrain action reads those suppliers every runtime cycle; controls do not stage mutable values or
call `Action.request()` from a periodic callback.

Athena publishes each supplier axis under the owning controller in `/Athena/Mechanisms`. Add
`State/Raw`, `State/Mapped`, and `State/Output` to an AdvantageScope line graph while driving.
`Curve/Input` and `Curve/Output` contain the sampled transfer curve, and writable `Config`, `Curve/Config`,
and `Slew` values update the running robot and curve data immediately.
Add the `Visualization/Curve`, `Raw`, `Mapped`, and `Output` point arrays to an AdvantageScope Points
tab with centered 2.2-by-2.2 dimensions to inspect the transfer curve and each live processing stage.

## Build

From this project directory:

```powershell
./gradlew.bat compileJava
```
