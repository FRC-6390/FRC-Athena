# Gamepad controls

Athena controller inputs are composable `ControlSignal` values. A signal can bind actions directly,
be combined with conditions, or be transformed into a toggle or gesture.

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
