# Gamepad controls

Athena controller inputs are composable `ControlSignal` values. A signal can bind actions directly,
be combined with conditions, or be transformed into a toggle or gesture.

## Controller types

Create Xbox and PlayStation controllers through the same factory:

```java
Gamepad driver = Controllers.xbox(0);
PlayStationGamepad operator = Controllers.playstation(1);

operator.cross().onPress(arm.score);
operator.r2Axis().above(0.55, 0.45).whileTrue(shooter.shoot);
operator.touchpad().onPress(diagnostics.capture);
```

PlayStation controllers expose `cross`, `circle`, `square`, `triangle`, `l1`/`r1`, analog and
digital `l2`/`r2`, `share`, `options`, `l3`/`r3`, `ps`, and `touchpad`. The inherited Xbox-style
names remain available as position-based aliases, so shared bindings can accept either controller:
cross maps to A, circle to B, square to X, and triangle to Y. `Controllers.ps4(port)` is an alias
for `Controllers.playstation(port)`.

Xbox gamepads also expose the PlayStation-style names for the equivalent controls. This lets code
using `cross`, `circle`, `square`, `triangle`, `l1`/`r1`, `share`, `options`, `l3`/`r3`, and
`l2Axis`/`r2Axis` move to Xbox without rewriting bindings. On Xbox, digital `l2()` and `r2()`
activate when the corresponding analog trigger passes 50%. `ps()` and `touchpad()` remain
PlayStation-only because Xbox controllers have no equivalent buttons.

For custom HID hardware, use raw WPILib IDs through `GenericController`:

```java
GenericController custom = Controllers.generic(2);

DoubleSupplier throttle = custom.axis(3).deadband(0.08).inverted().toSupplier();
custom.button(7).onPress(arm.score);
custom.pov(135).whileHeld(arm.nudge);
```

Axis IDs are zero-based and button IDs are one-based, matching WPILib. Repeated calls with the
same ID return the same signal, and raw inputs retain the normal controller transforms, gestures,
bindings, telemetry, and disconnect-safe toggle behavior.

## Button lifecycle

```java
operator.rightBumper()
        .whileHeld(intake.collect)
        .onRelease(intake.stop);

operator.a().onPress(arm.score);
```

The existing `onActive`, `whileActive`, `onDeactive`, and `whileDeactive` names remain available for
source compatibility. New code should prefer `onPress`, `whileHeld`, and `onRelease`, or the generic
`onTrue`, `whileTrue`, `onFalse`, and `whileFalse` signal methods.

## Toggle state

`toggle()` flips only on a rising edge, so holding a button never toggles repeatedly. Toggle state
resets while disabled or disconnected by default.

```java
ToggleSignal intakeEnabled = operator.a().toggle();

intakeEnabled
        .whileTrue(intake.collect)
        .onFalse(intake.stop);
```

An application can also control the state explicitly:

```java
ToggleSignal shooterEnabled = operator.y().toggle()
        .setWhen(operator.x().pressed())
        .clearWhen(operator.b().pressed())
        .resetWhen(operator.back().pressed());
```

Use `resetOnDisable(false)` or `resetOnDisconnect(false)` only when preserving the state is safe.

## Click counts

For a single binding, use the convenience operations:

```java
operator.a().click().onTrue(intake.collect);
operator.a().doubleClick().onTrue(intake.eject);
operator.a().clicks(3).onTrue(intake.clearJam);
```

When one button assigns several meanings, create one shared recognizer. Exact counts emit after the
window expires, preventing a double-click from also running the single-click action.

```java
ClickSequence clicks = operator.a().clicks(Duration.ofMillis(300));

clicks.exactly(1).onTrue(intake.collect);
clicks.exactly(2).onTrue(intake.eject);
clicks.exactly(3).onTrue(intake.clearJam);
clicks.between(4, 6).onTrue(diagnostics.capture);
clicks.atLeast(7).onTrue(diagnostics.reset);
```

A click completes on release. Presses longer than 500 ms are excluded from click sequences.

## Holds and repeats

```java
operator.start()
        .holdStarted(Duration.ofSeconds(1))
        .onTrue(climber.unlock);

operator.y()
        .heldFor(Duration.ofMillis(500))
        .whileTrue(climber.raise)
        .onFalse(climber.stop);

operator.x()
        .shortPress(Duration.ofMillis(250))
        .onTrue(arm.nudge);

operator.povUp()
        .repeated(Duration.ofMillis(400), Duration.ofMillis(100))
        .onTrue(() -> trim.increment(0.01));
```

`heldFor` is a level that stays active until release. `holdStarted` and `shortPress` are one-tick
pulses. `repeated` pulses immediately, waits for its initial delay, and then pulses at its interval.

## Conditions and boolean composition

```java
operator.a()
        .pressed()
        .onlyIf(shooter::atSpeed)
        .unless(climber::isExtended)
        .onTrue(feeder.feed);

ControlSignal mayShoot = operator.rightTrigger()
        .above(0.55, 0.45)
        .and(shooter::atSpeed)
        .and(indexer::hasPiece);
```

Signals support `and`, `or`, `xor`, `negate`, `onlyIf`, and `unless`. `ControlSignals.allOf`,
`anyOf`, and `noneOf` combine larger groups.

## Debounce and axis thresholds

```java
operator.a()
        .debounce(Duration.ofMillis(20))
        .doubleClick()
        .onTrue(arm.calibrate);

driver.rightTrigger()
        .above(0.55, 0.45)
        .whileTrue(shooter.shoot)
        .onFalse(shooter.stop);
```

The two-threshold axis form uses the first value to engage and the second to release. This hysteresis
prevents noise near the boundary from rapidly changing the signal. Axes also provide `below`,
`inside`, and `outside` signals. The original `rightTrigger(0.5)` and `leftTrigger(0.5)` forms remain
available.

Transforms execute from left to right, so debounce a physical input before applying click or hold
recognition.

## Axis curves and slew rates

Axis pipelines support standard, FPV-style, and arbitrary mappings. Processing order is deadband,
optional sign-preserving squaring, curve, inversion, then slew limiting. Output remains normalized
to `[-1, 1]`.

```java
DoubleSupplier forward = driver.leftY()
        .named("Forward")
        .deadband(0.07)
        .curve(AxisCurves.superRate()
                .rcRate(1.0)
                .expo(0.25)
                .superRate(0.65))
        .slew(3.0)
        .inverted()
        .toSupplier();

DoubleSupplier turn = driver.rightX()
        .named("Turn")
        .curve(AxisCurves.custom(value -> Math.copySign(value * value, value)))
        .slew(5.0, 7.0)
        .toSupplier();
```

`AxisCurves.linear()`, `expo(value)`, `power(exponent)`, `superRate(...)`, and `custom(mapping)` are
available. A custom reusable curve can implement `AxisCurve`; it may expose writable parameters by
returning `TelemetryValue`s from `telemetry()`.

Calling `toSupplier()` registers that final pipeline with its `Gamepad`. Athena then publishes the
controller connection state and each named axis's raw, mapped, and slew-limited output under
`/Athena/Mechanisms/<owner>/.../Axes`. Curve input/output sample arrays and writable deadband, curve,
and slew settings are included, allowing the response trace and live controller values to be graphed
and tuned in AdvantageScope without redeploying.

Each axis also publishes native `Translation2d[]` data under `Visualization`. Open an AdvantageScope
**Points** tab and add `Curve`, `Raw`, `Mapped`, and `Output` as sources. Set both display dimensions
to about `2.2`, place the origin in the center, and assign distinct symbols or colors. X is raw stick
input and Y is mapped output. `Raw` sits on the zero line, `Mapped` tracks the requested point on the
curve, and `Output` shows the actual slew-limited value. This format is intended for the Points tab;
the 2D Field tab uses full-size FRC field coordinates and will make a normalized curve difficult to see.

## Chords and ordered sequences

```java
ControlSignals.chord(operator.back(), operator.start())
        .within(Duration.ofMillis(200))
        .onTrue(system.resetFaults);

ControlSignals.sequence(
        operator.povUp().pressed(),
        operator.povUp().released(),
        operator.povDown().pressed())
    .within(Duration.ofSeconds(2))
    .onTrue(diagnostics.enable);
```

All timing uses the runtime's monotonic event timestamp, and each `ControlSignal` evaluator is
sampled at most once for a runtime tick.
