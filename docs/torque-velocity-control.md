# Torque-based velocity control

Status: implemented for design item B6 of `docs/2027-offseason-design.md` (model-based current control), as a
velocity loop. It runs on any controller that accepts a voltage, with no Pro license or device-side torque mode.
It builds on the motor model and the effort path in `docs/effort-and-contact.md`.

Use it for a velocity mechanism that must hold its speed under load, such as a shooter flywheel: a game piece
that loads the wheel should cost as little speed as possible, and the wheel should recover quickly.

## Declaring it

```java
public final ControlBinding<?> shooter = Controls.velocity(leader, follower)   // follower is commanded too
        .feedback(position, velocity)                                          // velocity in rotations per second
        .torqueVelocity(TorqueVelocityGains.of(kP, kI, 0.0)                     // amps of total motor current
                .staticAmps(kS)
                .inertia(kA)
                .loadEstimate(0.01))                                           // optional disturbance observer
        .effort(Effort.max(Units.Amps.of(60.0)));                              // optional ceiling, per motor
```

`torqueVelocity(...)` is declared on a velocity binding (`Controls.velocity`). It replaces the PID and the feedforward
already declared on the binding, and it cannot be combined with `pid(...)`, `ff(...)`, `feedforward(...)` or
`model(...)` afterwards: each of those adds a voltage that the torque loop already computes. The declaration fails
with a clear message when:

- the binding is a position control, or has no motor output;
- a motor kind has no built-in constants (for example `MotorKinds.MINION`). Use `pid` and `ff` for those.

Declaring it makes the binding's loops run in software, as an effort ceiling does. Motor-side device loops are not used.

## What the loop computes

Each cycle, for a velocity error `e = target - measured` in the binding's velocity units:

```
I   = kP e + kI integral(e) - kD d(measured)/dt + kS sign(target) + kA a_ref + d_hat     (amps, all motors together)
V_i = (I / N) R_i + omega_i / Kv_i                                                      (per motor, averaged)
V   = clamp(mean(V_i), -12 V, 12 V)
```

- `N` is the number of motors in the binding (leader and followers). Each motor carries `I / N`.
- `omega_i` is the measured motor speed from the controller. `R_i` and `Kv_i` come from the motor's `MotorSpec` row.
- The back-EMF term uses the measured speed, so the current follows the command on any controller. There is no
  velocity feedforward gain to tune: `Kv` and `R` do that job.
- `a_ref` is the acceleration of the reference the control follows (the same reference the voltage feedforward uses).
- `d_hat` is the load estimate, or zero when the observer is off (below).

The voltage then passes through the same path as every other command: the binding's effort ceiling, the Action
effort and the motor's power and thermal cap all bound it (see `docs/effort-and-contact.md`).

**Anti-windup.** When the voltage saturates, or an effort or power cap clamps it, the integral is re-seeded
from the voltage that was actually applied, so it cannot wind up. After a saturated step, a smaller target is
reached without a large undershoot: the test keeps the speed above 90% of the new target (tested).

**Gain changes are bumpless.** Changing a gain at run time, through telemetry or `persistTuning`, does not step the
output. The integral absorbs the difference, as it does for PID.

**Gains are total current.** `kP`, `kI`, `kD`, `kS`, `kA` are in amps of total current across all motors, not amps per
motor. The per-motor share is `I / N`. Because the torque is `Kt * I_total` at the mechanism, the same gains work
for one motor or two. A test checks the split: with two motors, each draws half the current and the voltage is halved.

## Load estimate (disturbance observer, opt-in)

`loadEstimate(tau)` enables a disturbance observer that uses only the motor's measured current and speed:

```
raw = I_measured - kA * a_measured - kS sign(v)       (I_measured: stator current summed over the motors)
d_hat += (dt / (tau + dt)) (raw - d_hat)              (low-pass, time constant tau)
```

When a ball loads the wheel, the current that the measured acceleration does not explain is the load, and the
command adds it at once instead of waiting for the speed error to build up.

Set `tau` to about one control period, **0.01 to 0.02 s** at the 20 ms loop. In the simulation below, `tau = 0.01 s`
gave the smallest dip. Larger values smooth the estimate, which helps with noisy current readings, but slow the
compensation roughly in proportion. `tau = 0` turns the observer off.

Requirements and limits:

- The estimate needs a **signed** current. TalonFX stator current is signed. A SPARK output current is a magnitude,
  so the estimate is wrong while the mechanism is pushed in the reverse direction. Use the observer only on
  controllers with signed stator current, or on mechanisms that run in one direction.
- Speed-proportional losses (damping) show up in the estimate too, so the estimate is the total load at the motor.
- The observer adds current after the load goes away until its time constant has passed. With `tau` near one
  control period, the release overshoot was about as large as the dip in the simulation (below), so it is worth
  checking on the real mechanism.

## Tuning

Work from the mechanism's physical values. Let `J_m` be the mechanism's inertia at the mechanism (kg m²),
`g` the motor turns per mechanism turn, `Kt` the motor torque constant, and `lambda` the closed-loop bandwidth you
want (rad/s; start with 10 to 20).

1. **`kS` from the free current.** `kS` is the friction current that must be overcome to keep the wheel turning.
   A reasonable first guess is the sum of the motors' free-run currents, `N * I_free`, from the datasheet. This is
   a starting point, not a measurement. Confirm it with a characterization run, or by holding a constant speed and
   reading the current.
2. **`kA` from the inertia.** The current to accelerate the mechanism at 1 rps/s is `2 pi J_m / (g Kt)`. Use that
   value for `kA`.
3. **`kP` from the bandwidth.** `kP = 2 pi J_m lambda / (g Kt)`, the gain that gives a closed-loop rate of
   `lambda`. For a motor on the shaft (`g = 1`), this is the gain in amps per rps of error.
4. **`kI` from the zero.** `kI = kP * lambda / 4`, which puts the integrator's zero at a quarter of the bandwidth.
   Raise it only if the steady-state error under a constant load does not close.
5. **`kD` stays at 0.** The loop is a PI on the speed, and the derivative of a measurement is noisy.
6. **Observer.** Turn `loadEstimate` on with `tau` of one control period, then check the telemetry
   `LoadEstimateAmps` against a known load.

Check the steady state first, then a step in target, then a step load. Watch the telemetry under the binding's
`Config/TorqueVelocity/` group: `Target`, `Measured`, `CurrentCommandAmps`, `VoltageCommand` and `LoadEstimateAmps`.
Gains are live-tunable there (`p`, `i`, `d`, `static`, `inertia`, `loadTimeConstant`, `disabled`), and `persistTuning`
saves them like PID gains.

## How it differs from voltage PID plus feedforward

| | Voltage PID + FF (`pid`, `ff`) | Torque velocity (`torqueVelocity`) |
| --- | --- | --- |
| Output of the velocity error | volts | amps of motor current, then volts through `R` and measured `Kv` |
| Back-EMF | feedforward gain `kV` on the reference | cancelled exactly with the measured speed |
| Velocity feedforward to tune | `kV`, `kA`, `kS` in volts | none for `kV`; `kA`, `kS` in amps |
| Effort ceilings and power caps | bound the voltage through the current model | bound the voltage through the same current model |
| Load compensation | none (the PID reacts to the speed error) | optional observer on measured current |
| Runs on | device or software | software only |

Measured in the simulation below, at equivalent tuning (same bandwidth), the torque loop without the observer
behaves like the voltage PID+FF, because a voltage loop with the same bandwidth produces the same current. The
gain comes from the observer. Do not expect the torque loop alone to be a faster or stiffer controller.

## Simulation check

The test `TorqueVelocityTest` runs a Kraken X60 flywheel (`J = 0.01 kg m^2`, viscous damping `0.0002 N m s`) at
50 rps, both loops tuned for 20 rad/s. A 0.6 N m load (about 31 A) is applied from 1.0 s to 1.5 s. Measured:

| Variant | Speed dip | Time more than 0.25 rps from the target | Speed above the target after the load ends |
| --- | --- | --- | --- |
| Voltage PID+FF | 0.354 rps | 0.30 s | 0.33 rps |
| Torque loop, no observer | 0.366 rps | 0.32 s | 0.35 rps |
| Torque loop, observer `tau = 0.01 s` | 0.210 rps | 0.00 s | 0.22 rps |

The dip floor is set by the loop's latency: one 20 ms cycle at this load is about 0.19 rps, so the observer
gets close to the floor. The numbers are for this plant; re-measure on the real flywheel.

## Limits

- The load estimate averages the stator current that each motor reports and scales it to all motors. A motor that
  does not report current is left out of the average.
- The observer needs a signed stator current (see above).
- A motor without built-in constants cannot use the loop. Declare a kind with constants, or use the voltage loop.
- The loop does not model battery sag. Bus voltage above 12 V is not used; the voltage is clamped at 12 V.
- The power cap uses the same path as an effort ceiling, and the test covers it through the output applier.
  It is not verified on a robot.

## Files

- `athena-mechanisms/src/main/java/ca/frc6390/athena/mechanism/control/TorqueVelocityGains.java`: the gains,
  the loop and the observer.
- `athena-mechanisms/src/main/java/ca/frc6390/athena/mechanism/core/ControlBinding.java`: `torqueVelocity(...)` and
  the guards against stacking voltage loops.
- `athena-mechanisms/src/test/java/ca/frc6390/athena/mechanism/core/TorqueVelocityTest.java`: the tests above.
