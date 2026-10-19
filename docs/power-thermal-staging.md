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

MotorDevice shooter = MotorDevice.of(MotorKinds.KRAKEN_X60, 43).currentLimit(Amps.of(120)).power(SHOOT);
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
7. **Cap.** The effective cap is the planner's cap: the declared limit (supply and stator, the tighter of
   the two, scaled by the derate, never below 1 A) combined with the budget, profile and ramp limits. It is
   published through `PowerRuntime.capAmps(motor)`. Nothing is written to a controller.
8. **Enforce.** The output path reads each motor's cap while it applies the command and folds it into the
   effort voltage clamp (see `docs/effort-and-contact.md`, "Enforcement"). The cap is passed to the clamp only
   when it is below the motor's declared limit (the supply/stator limit before the derate). Otherwise the
   motor gets no software clamp: its command and the controller's hardware limit are unchanged from before.
   A cap that binds is applied to control bindings (software loops), open-loop percent and voltage motor
   outputs, and software followers. Device-offloaded closed loops are the exception (see below).

### Why the cap is a voltage clamp, not a configuration write

The cap is enforced by bounding the commanded voltage so the predicted motor current stays within it:
`|V - omega/Kv| / R <= cap`. This is the same model the effort ceiling uses (see `docs/effort-and-contact.md`).

Runtime CAN configuration writes are not used for caps. Both vendors block the robot thread on them:

- REV: `SparkBase.configure` is a blocking call with reset and persist modes.
- CTRE: the configurator apply waits for a timeout.

A cap that moves every few cycles would therefore stall the scheduler. Declared static limits
(`MotorDevice.currentLimit` and the supply/stator variants) are still written once at configuration, as before.
Athena does not write them again while the robot runs.

Consequences:

- A cap that binds takes effect in the same cycle, with no rate limit and no deadband.
- Only a cap below the declared limit is enforced in software. A declared limit alone is not re-enforced in software:
  the controller enforces it in hardware, and a model error (for example a hot winding's resistance) would
  otherwise under-drive a healthy mechanism. Thermal derate and power shed/profile caps are the cases that enter.
- Motors without a `MotorSpec` row use a proportional fallback (see below).

### Motors without known constants

A motor kind without a `MotorSpec` row (for example `MINION`) has no resistance or speed constant, so the
voltage clamp cannot invert its current. For these motors:

- Demand is the measured supply current, not the model estimate.
- The output path scales the command by `cap / measured supply current` when the measured current is above the
  cap. The scale is an estimate, not a bound. It settles over several cycles, so brief overshoot is possible.
- Nothing is written to the controller.

Add a `MotorSpec` row to get a model-based bound.

### Device-offloaded closed loops

A position or velocity binding that the device runs in its own firmware loop is not clamped and is not moved to
the software loop. A dynamic cap therefore does not reach its current. When such a binding has an active cap that
is not applied, State/Reason reports `Power cap NN A not applied (device loop)` (NN is the cap in whole amps).
Use a software loop (for example, declare an effort ceiling on the binding) if the cap must bound it.

### Open-loop and follower outputs

- Open-loop motor outputs (percent or voltage Actions without a control binding) are clamped with the motor's cap
  and its software followers' caps, the same way a binding's command is.
- Software followers (`SoftwareFollowerMotorHandle`) receive their leader's command, so the command is clamped
  against the intersection of the leader's and the followers' limits. A follower's cap therefore bounds the leader
  too, and the leader's command is scaled as a whole.
- Hardware followers (the device mirrors the leader in its own firmware) receive the command from the device.
  Athena cannot clamp it. Their caps are not enforced separately. The leader's command is clamped by the leader's
  cap, so the follower sees a bounded command. Use a software follower when a follower needs its own cap.

## Thermal (B8)

```java
MotorDevice arm = MotorDevice.of(MotorKinds.KRAKEN_X60, 44).currentLimit(Amps.of(80))
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

`motor.temperature()` returns the fused estimate as a `Temperature`:

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

`Status` is one of `Declared`, `Power capped`, `Power shed`, `Thermal derate`, or `Unavailable`.

`State/Reason` (the mechanism's explanation of an applied output) shows `Power cap NN A` when the cap binds on
the motor of the selected output, `Power cap NN A not applied (device loop)` for a device-offloaded binding (see
above), and `Thermal derate NN%` when the thermal derate applies. These values are read
from the same cycle's power pass, so they describe the cap that the output path enforced.

## Current-limit writes per vendor (I9): decided

Decision: caps are enforced by the effort voltage clamp. Runtime current-limit configuration writes are not used
for power or thermal caps, on either vendor. The write cost was a blocking call on the robot thread
(`SparkBase.configure` on REV, a configurator apply with timeout on CTRE), so the earlier rate limit (0.25 s per
motor, 2 A deadband) is removed with the writes. The `Unsupported` status is removed too: every handle is capped
by the clamp, so no handle reads as unsupported.

Declared static limits are still written once at configuration, as before.

## Limits and not done

- The battery resistance is estimated and persisted as a rolling prior (I3c); it is not stored per battery.
- Thermal constants are nominal, not characterized (see the caveat above).
- Closed-loop outputs that the device computes on the controller (offloaded control) do not receive
  the staging effort cap or the power cap. Use `limitSpeed`/`limitAccel` for those. The clamp can only bound a
  voltage Athena commands.
- Hardware followers are not clamped separately (see "Open-loop and follower outputs").
- Motors without a `MotorSpec` row are capped by a proportional estimate, not a model-based bound.
- Caps are compared with the stator current the demand model uses. A supply-current cap is therefore conservative
  for the battery, not an exact supply limit.
- Stage transitions are not yet replay markers (C1). They are available through `drainStageChanges()`. An applied
  output that a stage caps is explained by `State/Reason` as `Stage <CONSTANT>` (A5).
