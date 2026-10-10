# Power, thermal and staging

Sections B7 (power profiles), B8 (thermal) and B9 (staging) of `docs/2027-offseason-design.md`.
Each runs inside the mechanism scheduler's cycle. Nothing here is configured by calling a runtime
method each loop: robot code declares values, and Athena applies them.

## Power profiles (B7)

```java
public static final PowerProfile SHOOT = PowerProfile.of()
        .sustained(Amps.of(60))
        .burst(Amps.of(90), Seconds.of(0.5))
        .rampLimit(Amps.of(10), Seconds.of(0.1))   // at most +10 A per 0.1 s
        .share(3.0)
        .onSag(Sag.shed(0.5).below(Volts.of(9)));  // remove half the cap while the bus is below 9 V

MotorDevice shooter = MotorDevice.of(MotorKinds.KRAKEN_X60, 43).currentLimit(120).power(SHOOT);
public final Action endgame = SHOOT.update(p -> p.share(0.5));   // applied from the next cycle
public final Action normal  = SHOOT.restore();                   // back to the declared values
```

- A profile is a shared, mutable value. Several motors may reference one profile.
- `sustained` caps continuous current. `burst` allows a higher cap while a time budget lasts; the
  budget refills at half speed while the motor draws within sustained.
- `rampLimit` limits increases of the cap. Decreases are immediate.
- `share` is the relative weight when the battery budget is short (see below).
- `onSag` reduces the cap while the measured bus voltage is below the threshold.

### Robot policy

Declare one `PowerPolicy` field on a mechanism. It is discovered like `AutoChooser`. Without it,
`PowerPolicy.defaults()` applies.

```java
public final PowerPolicy power = PowerPolicy.defaults()
        .floor(Volts.of(7.5))              // default 7.0 V
        .staggerStarts(Seconds.of(0.2))    // default 0 (off)
        .estimateBatteryResistance(true);  // default true
```

### Per-cycle pass

The pass (`PowerRuntime`) runs in `MechanismScheduler.periodicBound` after every controller has
produced its outputs and before any motor command is applied:

1. **Demand.** Each motor's demand is `|V - omega/Kv| / R` from its last applied voltage and speed,
   using the nominal motor constants (see below). Without those readings the measured supply current
   is used.
2. **Battery model.** Bus voltage is modeled as `V_bus = Voc - R * I_total`. `R` and `Voc` are
   estimated online by exponentially weighted least squares on bus voltage and total supply current.
   `R` only updates when the current varies (standard deviation above about 0.5 A). Before that it
   stays at the default `0.02 ohm`.
3. **Budget.** Predicted bus voltage `Voc - R * sum(demand)`. Below the floor (with 0.25 V
   hysteresis), the total budget `(Voc - floor) / R` is water-filled across motors. Each motor gets
   `min(demand, L * share)`, with `L` chosen so the total equals the budget. The lowest-share motors
   are reduced first.
4. **Profile.** Sustained/burst cap, then the sag response.
5. **Stagger.** A motor whose demand crosses its large-current threshold (20 A, or half its sustained
   cap) is admitted only if `staggerStarts` has passed since the last admitted start. Otherwise it is
   held at the threshold for that cycle.
6. **Ramp.** Increases are limited to the ramp rate.
7. **Write.** The effective supply limit is the declared (derated) limit, capped by the planner. It
   is written through `MotorHandle.applyRuntimeConfiguration`, the same path as the dashboard
   overrides.

### Why configuration writes (and their rate limit)

The cap is a controller-side supply-current limit, so it protects the battery even when the RoboRIO
loop is slow. The alternative, scaling commanded voltage from a model, would not bound the current
the controller draws. Writes are rate limited:

- at most one write per motor every 0.25 s (`PowerRuntime.MIN_WRITE_INTERVAL_SECONDS`);
- only when the change is at least 2 A (`WRITE_DEADBAND_AMPS`), or the limit switches on or off;
- a user change to the configuration is written at once, because it bypasses the throttle.

A cap is never written as `0`: controllers read zero as "no limit", so an active cap is at least 1 A.

Handles that do not support runtime configuration are never written. The motor's status reads
`Unsupported` in telemetry.

## Thermal (B8)

```java
MotorDevice arm = MotorDevice.of(MotorKinds.KRAKEN_X60, 44).currentLimit(80)
        .thermal(Thermal.derate(Celsius.of(100), Celsius.of(130), 0.4));
```

- `Thermal.derate(start, end, minFraction)`: the limit is full below `start`, falls linearly to
  `minFraction` at `end`, and stays there above. Thermal only scales current limits. It never brakes,
  holds or locks out a motor. A stronger response belongs to a `Staging` (B9).
- Without a declared curve, the motor kind's default applies: full limit until 30 C below the rated
  winding temperature, then 50 percent at that temperature.
- Derating only has an effect when the motor has a current limit to scale. Declare `currentLimit(...)`
  (or `supplyCurrentLimit`/`statorCurrentLimit`).

### Temperature estimate

`motor.temperature()` (typed) and `motor.temperatureCelsius()` return the fused estimate:

- **Model.** `dT/dt = I^2 R / C - (T - T_ambient) / tau`, with `I` the stator current. Constants come
  from `MotorThermalSpec`.
- **Vendor.** `MotorHandle.temperatureCelsius()` returns an `OptionalDouble`. The default is empty
  (unsupported). When a reading is present it is returned, and the model is pulled toward it with a
  2 s time constant.

`motor.thermalLoad()` is `(T - start) / (end - start)` clamped to `[0, 1]`.

Derate fraction = `1 - (1 - minFraction) * thermalLoad`. The power pass applies it to the declared
limit before the budget, so the two combine through `min`.

> **Caveat.** The constants come from one nominal table, `MotorSpec` (I4), which feeds both
> `MotorConstants` (electrical: R, Kt, Kv, free speed, stall and free current) and `MotorThermalSpec`
> (thermal: winding resistance and speed constant from the same electrical values, plus heat capacity,
> cooling time constant and rated winding temperature). The values are not calibrated. Replace them with
> characterized values (thermal run) before relying on temperature estimates. Motor kinds without a row,
> such as `MINION`, use `MotorThermalSpec.GENERIC`. Those kinds previously had thermal-only values; they now
> need a `MotorSpec` row to get any.

### Vendor temperature (I8)

- CTRE TalonFX and TalonFXS: `DeviceTemp` is part of the existing `refreshAll` batch. An invalid status or a
  non-finite value reads as empty, never as zero.
- REV SPARK: `getMotorTemperature()` is read with the other inputs. NaN reads as empty.

## Staging (B9)

```java
enum ArmHealth { NORMAL, STRAINED, HOT, SAFE_SHUTDOWN }

public final Staging<ArmHealth> health = Staging.of(ArmHealth.NORMAL)
        .stage(ArmHealth.STRAINED, strained, angle.limitEffort(0.6))
        .stage(ArmHealth.HOT, hot, angle.limitSpeed(0.5), angle.limitAccel(0.5))
        .stage(ArmHealth.SAFE_SHUTDOWN, shutdown, stow)
        .exitAfter(Seconds.of(2))
        .override(manualRecovery);

public final Action lift = angle.position(Degrees.of(80)).staged(health);   // only while lift runs
```

- A stage either requests an `Action` (entered: requested through the normal scheduler; left:
  cancelled), or caps bindings with modifiers: `limitEffort(f)` (voltage and percent output),
  `limitSpeed(f)` (motion-profile maximum velocity and velocity targets), `limitAccel(f)` (maximum
  acceleration). A stage with several bindings lists several modifiers.
- The worst true stage (highest ordinal) is entered immediately. Leaving a stage requires its
  condition to stay false for `exitAfter`. The exit timer restarts if the condition flickers true.
- `override(condition)` forces the initial stage while true.
- `current()` returns a `Supplier` of the current stage.
- `action.staged(staging)` returns the same action. The staging is evaluated and its caps apply only
  while that action runs. When the action stops, the staging resets to its initial stage.
- Stage transitions are recorded and returned by `RobotRuntime.drainStageChanges()` (and the
  scheduler's `drainStageChanges()`). Condition exceptions are reported as runtime faults.

Caps reach the output path in `OutputApplier`: effort scales voltage and percent output after the
control loop, speed scales the profile's maximum velocity and velocity targets, and acceleration
scales the profile's maximum acceleration. Position targets are limited by the profile, not scaled
directly.

## Telemetry

`PowerTelemetryPublisher` (wired by `AthenaRobot`) publishes under `/Athena/Power`:

- `Bus/Volts`, `Bus/OpenCircuitVolts`, `Bus/BatteryOhms`, `Bus/FloorVolts`, `Bus/PredictedVolts`,
  `Bus/BudgetAmps`, `Bus/DemandAmps`, `Bus/Shedding`.
- `Motors/<motor>/Demand`, `CapAmps`, `Derate`, `TemperatureCelsius`, `Share`, `Status`.

`Status` is one of `Declared`, `Power capped`, `Power shed`, `Thermal derate`, `Unsupported`, or
`Unavailable` / `Error: ...`.

## Current-limit writes per vendor (I9)

- **CTRE**: current limits are a `CurrentLimitsConfigs` apply on the TalonFX, the same path as before.
- **REV**: each cap is a blocking `SparkBase.configure` call (one per motor per write, with reset/persist modes).
  A cost was not measured on a roboRIO in this change, so the switch to model-based voltage scaling (no CAN configure
  writes) has **not** been made. Decision pending a measured write time. Until then the write rate limit above
  (0.25 s per motor, 2 A deadband) is the mitigation.

## Limits and not done

- The battery resistance is estimated and persisted as a rolling prior (I3c); it is not stored per battery.
- Thermal constants are nominal, not characterized (see the caveat above).
- Closed-loop outputs that the device computes on the controller (offloaded control) do not receive
  the staging effort cap. Use `limitSpeed`/`limitAccel` for those.
- Stage transitions are not yet replay markers (C1) or A5 reasons. They are available through
  `drainStageChanges()`.
