# Characterization and mechanism models

Status: implemented for offseason stream B1, B2 and B3 (`docs/2027-offseason-design.md`).

A control binding can identify its own mechanism. The identified parameters are a `MechanismModel`
value. The model drives feedforward, simulation inertia, external-load estimates and drift alerts.

## Declaring characterization

```java
public final ControlBinding<?> turret = Controls.position(turretMotor).feedback(turretEncoder);

public final Characterization characterization = turret.characterization()
        .maxVoltage(4.0)            // no test ever applies more than this
        .rampRate(0.5)              // quasistatic ramp, volts per second
        .learnContinuously(true)    // recursive least squares on normal operation
        .driftTolerance(0.25)       // 25% band against the baseline
        .gearRatio(10.0);           // motor turns per mechanism turn

public final MechanismModel model = characterization.model();
public final ControlBinding<?> turretControl = turret.model(characterization);
```

`characterization()` is a declaration. Athena discovers it on the mechanism the same way it discovers
`ControlSysId`. The dashboard gets these Actions under the field name:

| Action | What it does |
| --- | --- |
| `QuasistaticForward`, `QuasistaticReverse` | Held voltage ramp, from rest, up to `maxVoltage`. |
| `DynamicForward`, `DynamicReverse` | Step to `maxVoltage`. The step response also gives latency. |
| `FrictionSweep` | Forward then reverse ramp. Friction-versus-position and breakaway come from it. |
| `Backlash` | Forward then reverse ramp. Lost motion is measured at the reversal. |
| `Latency` | The dynamic step. Latency is the time from onset to first motion. |
| `FitModel` | Fits all parameters from the collected data. The result becomes the active model and the baseline. |
| `AdoptRunningEstimate` | Makes the continuous-learning estimate the active model. |
| `Rebaseline` | Makes the active model the drift baseline. |

All tests run through the normal scheduler. They use the binding's constraints, ownership and
arbitration, and they never exceed `maxVoltage`. The collected samples are bounded (5000 entries, oldest
dropped first).

## Fitting

Positions are in rotations (rotary) or meters (linear). A typed binding converts its own unit to
these base units first. Raw bindings use their values as they are.

Run data pairs each interval's acceleration with the voltage that produced it. The mechanism's state is
measured after the previous cycle's command has acted, so a row takes the previous command, the interval's
mean position and velocity, and the acceleration from the two samples (I5). The first sample of each run has
no interval and is not used.

Feedforward is fitted by ordinary least squares on samples whose speed exceeds 0.02 units per second:

```
rotary:  V = kS * sign(v) + kV * v + kA * a + gc * cos(2 pi p) + gs * sin(2 pi p)
linear:  V = kS * sign(v) + kV * v + kA * a + kG
```

Gravity is recovered as `kG = hypot(gc, gs)` and `offset = atan2(-gs, gc)`, so the model term is
`kG * cos(2 pi p + offset)`.

- Coefficients solve the normal equations on column-equilibrated regressors. Standard errors are
  `sqrt(s^2 * [(X'X)^-1]_jj)`, with `s^2 = RSS / (n - p)`.
- R^2 is `1 - RSS / TSS`. The condition number is the 1-norm condition of the equilibrated Gram matrix.
- Static friction is the breakaway voltage in a quasistatic ramp that starts from rest, minus the
  gravity term at that position.
- Friction table: each position bin holds the mean kinetic residual
  `sign(v) * (V - kV v - kA a - gravity) - kS`. The model interpolates it linearly.
- Backlash: distance between the last moving forward sample and the first moving reverse sample.
- Latency: time from the first sample of a dynamic step to the first sample with speed at or above
  0.05 units per second.
- Inertia (rotary): `J = kA / (2 pi) * motors * gear * Kt / R`, using the motor's constants.

Fitting needs at least 30 samples and more samples than parameters. Otherwise it throws with a message
that says what to run.

## Continuous learning

When `learnContinuously(true)` is set, each cycle of a model-driven loop reports the applied voltage
and the measured motion to the characterization. The recursive least-squares estimator (forgetting
factor 0.995) updates only when the excitation gate accepts the sample. The estimate is kept apart
from the active model. It only becomes active through `AdoptRunningEstimate`.

Excitation criteria (`ExcitationGate.ExcitationCriteria`, defaults shown). The gate looks at the last 100
samples and needs at least 200 samples since the last reset:

- velocity standard deviation of at least 0.05 units per second;
- acceleration standard deviation of at least 0.05 units per second squared;
- 1-norm condition number of the window's regressors at most 1e4. A constant-velocity window fails this.

Caveats. The applied voltage includes any PID effort, so a binding with active feedback gains biases
the estimate. The applied voltage is read one cycle before the measured motion, so acceleration lags by
one cycle. Use the test Actions for precise fits, and treat continuous learning as a monitor.

## History and drift

`recordDisable(enabledSeconds)` snapshots the running estimate when the robot was enabled for more than
30 s. `recordMatchEnd()` snapshots it at the end of a match. Both keep at most 32 snapshots.

`drift()` compares the running estimate with the baseline. Parameter `p` drifts when
`|running - baseline| / max(|baseline|, 1e-3)` exceeds the tolerance. The result is a `DriftReport` with
a boolean and a message. The characterization is also a `TelemetrySource`, so it publishes `driftAlert`,
`driftMessage`, `runningSamples`, `excitation` and `testSamples`.

Storage is an injected `ModelStorage` (`InMemoryModelStorage` by default). A persistent store can
implement that interface without changing the API.

## Payload

```java
public final Characterization characterization = turret.characterization()
        .payloadWhen(() -> intake.hasGamePiece());
```

While the supplier is true and the binding is holding (speed below the deadband), each holding sample is
kept. The payload is the change of the gravity amplitude. It is confident when there are at least 100
holds, the rotary spread of `cos(2 pi p)` is at least 0.1, and the delta exceeds twice its standard
error. Until then the payload counts as external load.

## Using the model

- `binding.model(MechanismModel)` uses a fixed model.
- `binding.model(characterization)` follows the characterization's active model.
- `ff(...)` after `model(...)` replaces the model feedforward, and `model(...)` after `ff(...)` replaces
  the `ff` gains.
- `MechanismModel.feedforwardVolts(position, velocity, acceleration, directionHint)` predicts the volts.
  Static friction is used at rest, with the sign of `directionHint`. Kinetic friction is used while
  moving.
- `SimModel.arm(motor).model(model)` applies the identified plant (I5). The WPILib physics engine then
  simulates the mechanism the model describes: static friction holds the mechanism at rest until the drive
  exceeds it, kinetic friction and kV act while it moves, kA sets the inertia, and gravity is
  `kG cos(2 pi p + offset)` for rotary and constant `kG` for linear mechanisms (including any payload delta).
  Volts become mechanism torque (rotary) or force (linear) through the model's motor constants, so the
  inertia is `kA * n * G * Kt / (2 pi R)`, the relation the fit uses. The simulation runs in fixed substeps of
  at most 1 ms. Limits:
  - the friction-versus-position table and backlash are not simulated;
  - the declared `momentOfInertia` and `gravity(...)` are not used once a plant is applied;
  - linear plants convert force with a 0.025 m drum radius (the elevator default).

## Checks (B3)

Model-equipped bindings expose checks. A check is a `BindingCheck`, a `BooleanSupplier`. It has
`threshold(value)` and `forSeconds(seconds)` overrides and reports true once its condition has held for
the debounce time.

| Method | Needs a model | Condition (default threshold, debounce) |
| --- | --- | --- |
| `feedbackStalled()` | no | Output draws more than 2 A while the feedback is still and the output is driven (2 A, 0.1 s). |
| `blocked()` | yes | Command voltage above 1 V while the feedback is still (1 V, 0.2 s). |
| `backdriven()` | yes | Motion opposite to the drive direction above 0.05 units per second (0.05, 0.1 s). |
| `load()` | yes | `DoubleSupplier`: applied minus predicted volts minus `kA * a`. In N m when motor constants are known, otherwise in volts. |
| `slipCheck(second, ratio, tolerance)` | no | `SlipCheck.slipping()`: the primary feedback and `second * ratio` differ by more than the tolerance. |

Checks that need a model throw `IllegalStateException` on a binding without one. Their debounce uses the
monotonic clock.

## Model-based stall detection (B3)

`MotorDevice.stall()` uses model-based detection when the motor model has built-in constants. These are
the stall torque, stall current, free current and free speed of the published characteristics. There is
no fixed voltage threshold. A stall is reported when:

- the motor draws at least twice its free-run current;
- the expected unloaded speed `(V - I_free * R) * Kv` is at least 10% of free speed;
- the measured speed is below 20% of the expected speed;
- and this holds for 0.1 s.

The expected speed uses the free-run current rather than the measured current. At a true stall the
back-EMF is zero, so a drop computed from the measured current would predict zero speed and hide the stall.

Calling `atCurrentLimit`, `velocityBelow` or `commandedVoltageAbove` selects the legacy fixed-threshold
detector. Motors without constants keep the legacy detector too. Built-in constants are datasheet-level
approximations (`MotorConstants`). Teams can supply their own with `MotorConstants.of(...)`.

## Persistence (I3)

`PersistentModelStorage` implements `ModelStorage` on the `PersistentStore`. The mechanism tree path plus the
declaration name is the key (for example `robot/turret/angle`), so the entries are
`<key>/model`, `<key>/running` and `<key>/history` (plus small companion keys for motor kinds and motion kind).

- Attached when the robot registers its mechanisms (`MechanismScheduler`), before the first schema build. Each
  characterization restores the stored model (it becomes the active model and drift baseline), the running estimate and
  the history.
- Written through on fit, adopt-running-estimate, rebaseline, and history snapshots. Snapshots come from
  `recordDisable` (enabled more than 30 s) and `recordMatchEnd`. `AthenaRobot` calls these: `recordDisable` on disable,
  with the length of the enabled period, and `recordMatchEnd` at teleop exit when the FMS is attached.
- Not stored: standard errors (a restored model reports none), and the baseline separately (a restored baseline equals
  the restored active model).
- Boot-time cleanup keeps these keys (`TelemetryPersistence.keep`).

Battery internal resistance (I3c): the rolling estimate is written to `power/battery/internalResistanceOhms`
while it has observed excitation, and restored as the prior at boot (`BatteryResistanceEstimator.restorePrior`). It
is used until new excitation is observed. Persistence is rolling, not per battery.

## Gear ratio (I6)

The default gear ratio is read from the feedback encoder of the binding: `1 / (gearRatio * conversion / unitsPerRevolution)`,
so an encoder with `reduction(10)` gives 10 motor turns per mechanism turn. A binding without an encoder, or whose
encoder measures a distance, keeps 1. `gearRatio(...)` overrides either way.

## Not yet implemented

- Thermal characterization and thermal parameters.
- Friction-versus-position and backlash in the simulated plant.
