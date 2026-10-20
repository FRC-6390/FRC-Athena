# Effort, load responses and contact

Status: implemented for offseason stream B5 (effort) and B6 (model-based current control) of
`docs/2027-offseason-design.md`. Depends on the model and checks in `docs/characterization-and-models.md` (B3).

For a velocity control that holds its speed under load with a motor current command, see
`docs/torque-velocity-control.md` (`torqueVelocity`). It uses the same effort path described here.

Every control Action can carry an effort limit: the largest torque, force or current its motors may draw.
The limit is enforced on the software output path as a voltage clamp, so it works with any controller.

## Declaring a binding ceiling

```java
public final ControlBinding<?> angle = Controls.position(motor, Units.Degrees)
        .feedback(encoder)                  // encoder measures Degrees
        .pid(4.0, 0.0, 0.1)
        .model(characterization)            // only needed for load responses and blocked()
        .effort(Effort.max(Units.NewtonMeters.of(12))
                .gearing(25.0)              // motor turns per mechanism turn
                .onLoad(LoadResponse.YIELD.recoverAfter(Units.Seconds.of(0.5))));
```

| Binding kind | Accepted effort measures |
| --- | --- |
| Rotary (`Units.Degrees`, `Units.Rotations`) | `Torque` (N m) or `Current` (A) |
| Linear (`Units.Meters`, `Units.Inches`) | `Force` (N) or `Current` (A) |
| Dimensionless (`Units.Value`) | `Current` only |

A wrong kind is an `IllegalArgumentException` when the binding is declared, with a message that names the
accepted kinds. Torque and force need known gearing (`Effort.gearing(motorTurnsPerMechanismTurn)`). Linear
force also needs the drum radius (`Effort.radius(Distance)`). Without them the declaration fails with
"requires known gearing — use Amps". Torque and force also need the motor kind's constants. Motor kinds without
built-in constants (for example `MotorKinds.MINION`) must use amps.

Declare the ceiling before creating the Actions that use the binding. Torque and force on an Action convert
through the binding's gearing.

## Per-Action effort

```java
public final Action press = angle.position(stow).effort(Units.Amps.of(8));
public final Action gentle = angle.position(stow).effort(Units.Amps.of(1));
```

`Action.effort(Measure)` works on every control Action (position, velocity, percent, voltage, and the dynamic
forms). Composite Actions (`until`, `then`, `timeout`, `untilWithin`) pass the limit to their control Actions.
The limit is converted to a per-motor current when the Action is declared. While the Action runs, the
binding's ceiling and the Action's limit both apply, and the smaller one wins.

## Conversion to motor current

For a binding with `N` motors (leader and followers), gearing `g` (motor turns per mechanism turn) and motor
torque constant `Kt_i`:

```
torque  T      ->  I_i = T / (N * g * Kt_i)          (N m)
force   F      ->  T = F * r, then as above           (r = drum radius)
current I      ->  I_i = I                            (per motor)
```

Each motor has its own `Kt_i` (from `MotorConstants.forKind`). Hardware followers mirror the leader's
command, so the same per-motor limit applies to each of them. Amps are per motor, the same as
`MotorDevice.currentLimit`.

A multi-motor Action (`Action.effort`) uses the smallest per-motor limit, so no motor in the binding exceeds
its share.

## Enforcement (B6)

Each applied cycle, after saturation and before the loops see the applied output, the output path does one
thing: it bounds the commanded voltage so the predicted current never exceeds the limit.

The per-motor limit is the smallest of three values:

- the binding ceiling (`Effort.max`, with the YIELD, pushThrough or slow responses applied);
- the running Action's effort (`Action.effort`);
- the motor's power or thermal cap from the power pass (`PowerRuntime.capAmps`, see `docs/power-thermal-staging.md`).

The power cap is read once per cycle, after the power pass and before the motor commands are applied. Nothing is
written to the controller, so a cap that changes every cycle costs nothing on the robot thread.

Every motor command is clamped, not only control bindings:

| Command path | Clamp |
| --- | --- |
| Control binding (position, velocity, percent, voltage) | binding ceiling, Action effort and the power cap of each binding motor (the cap only when below the declared limit) |
| Open-loop motor output (percent or voltage, no binding) | Action effort and the power cap of the motor (only when below the declared limit) |
| Software follower (receives its leader's command) | the follower's own power cap joins the intersection, so it bounds the shared command (only when below its declared limit) |
| Device-offloaded position or velocity loop | not clamped; an active cap is reported as `Power cap NN A not applied (device loop)` |
| Hardware follower (mirrors the leader in firmware) | not clamped separately. The leader's command is clamped by the leader's limits |

Percent commands are clamped at the nominal 12 V and sent as voltage when the clamp changes them.

For motor `i` with resistance `R_i`, speed constant `Kv_i` (rad/s per volt) and measured speed `omega_i`:

```
I_i(V) = (V - omega_i / Kv_i) / R_i
clamp V to [ omega_i/Kv_i - I_i,lim R_i ,  omega_i/Kv_i + I_i,lim R_i ]   (intersected over motors)
```

The clamp uses the speed measured at the start of the cycle. A change of speed inside the cycle (for example
an impact with a hard stop) can push the current above the limit for one cycle. Motors without known constants
are handled by a proportional estimate (see below). When no voltage satisfies every motor, the midpoint of
the empty interval is used.

Motors without known constants (no `MotorSpec` row) have no model to invert. For these the command is scaled by
`limit / measured supply current` when the measured current is above the limit. This is an estimate: it settles
over several cycles and can overshoot briefly.

Controls that run device-side closed loops switch to the software loop while a ceiling is declared, because the
device cannot be clamped. A power cap alone does not switch a binding's loops to software, so a device-side
closed loop with only a power cap is not clamped.

`EffortCurrentMode` is an extension point. A motor handle that supports a hardware current or torque mode can
implement it later. No built-in handle does yet, so the voltage clamp is always used.

## Load responses

`Effort.onLoad(...)` says what the binding does while the external load reaches its ceiling. A load response
needs a model on the binding (`model(...)` before `effort(...)`). Without a model the responses are ignored and
only the ceiling applies.

The load uses the B3 formula, with the controller's requested voltage and before the clamp:

```
load = g * sum_i Kt_i (V_requested - predicted - kA * accel) / R_i     (N m)
```

`predicted` is the model's feedforward voltage at the current position and velocity. The requested voltage is
used because the clamp caps the applied current at the ceiling, so an estimate from the applied voltage could
never exceed the ceiling. "Loaded" means `|load| >= ceiling torque`.

| Response | Meaning |
| --- | --- |
| `LoadResponse.YIELD.recoverAfter(t)` | While loaded, the limit is zero (compliant), so the mechanism follows the push. It returns to the ceiling once the applied load has been clear for `t`, ramping in over the same time. Detection is suspended during the ramp so the controller can move back toward its target. |
| `LoadResponse.HOLD` | Keep the position command and the ceiling. This is the default behaviour. |
| `LoadResponse.pushThrough(burst, duration)` | While loaded, allow `burst` (a torque, force or current) from the moment the load appears for `duration`, then fall back to the ceiling. |
| `LoadResponse.slow(fraction)` | While loaded, scale the motion profile speed and velocity targets by `fraction`. The change applies from the next cycle. |

Each kind may appear at most once. `YIELD` and `pushThrough` both apply to the same loaded period, and `YIELD`
takes precedence.

Known limitation. While yielded, the controller's position error keeps the requested demand high. Recovery
depends on the applied load (near zero at zero current), so a sustained push beyond the ceiling produces periodic
re-engagement. The arm follows the push on average and returns once the push ends. It does not hold position
under a sustained push.

## Contact by composition

No new primitive is needed. Position past a hard stop with an effort cap holds against the stop:

```java
public final Action stow = angle.position(stowDegrees).effort(Units.Amps.of(8))
        .until(angle.blocked().threshold(0.1).forTime(Units.Seconds.of(0.0)))
        .then(angle.position(stowDegrees).effort(Units.Amps.of(1)));
```

`blocked()` is the B3 check. Under an effort ceiling the stall voltage is `I R`, which is small (0.26 V at 8 A on a
Kraken), so set the blocked threshold below it, for example with `.threshold(...)`. The default 1 V only fires when
the motor is driving hard.

## Telemetry

The binding's effort loop is a telemetry source, published under the binding's `Config` group:

| Name | Unit | Meaning |
| --- | --- | --- |
| `CeilingUnit`, `Ceiling` | declared unit | ceiling in the unit it was declared with |
| `CeilingAmps` | A | largest per-motor ceiling at declaration |
| `ActiveAmps` | A | smallest per-motor limit applied in the last cycle |
| `ActiveEffort` | declared unit | the same limit in mechanism torque or force (amps for current) |
| `Load`, `LoadThreshold` | N m | load estimate and its threshold |
| `Yielded`, `SpeedScale` | - | load response state |

## Not done

- Hardware torque or current modes (`EffortCurrentMode` is only the hook).
- Per-motor constants supplied by the user (`MotorConstants.of` is not yet wired into the effort conversions).
- Replay markers for yield and recovery (C1). Use the telemetry above.
- Hardware followers are not clamped separately (see the table above).
- The effort clamp runs after staging's effort cap. The power cap joins the same intersection.
