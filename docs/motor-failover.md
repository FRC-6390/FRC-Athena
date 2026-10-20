# Motor failover

A mechanism whose motors are a leader plus `.follow(leader)` followers can declare what happens
when one of those motors dies. Without a declaration nothing changes: a dead motor applies its
`FailurePolicy`, which by default disables the whole mechanism.

```java
private final MotorDevice leader = Constants.CAN.motor(MotorKinds.FALCON_500, 31);
private final MotorDevice follower = Constants.CAN.motor(MotorKinds.FALCON_500, 32).follow(leader);
private final MotorDevice follower1 = Constants.RIO.motor(MotorKinds.FALCON_500, 33).follow(leader).inverted();

private final MotorFailover failover = MotorFailover.of(leader)
        .promoteLeader()                 // the leader dies: the first alive follower leads
        .dropFailedFollowers()           // a follower dies: keep running without it
        .derateOutput(0.8)               // while degraded, scale percent and voltage commands
        .onFailure(event -> DataLogManager.log("Shooter " + event.kind() + ": " + event.motor().defaultName()));
```

`MotorFailover` is a mechanism field, discovered like `Staging` or `PowerPolicy`. The group is the
declared leader and every motor declared with `.follow(leader)`. Controls, feedback and robot code
keep naming `leader`; Athena redirects to whichever motor leads the group.

## Policy

| Method | Default | Effect |
| --- | --- | --- |
| `promoteLeader()` | off | The leader dies: promote the first alive follower, by ascending CAN id. |
| `promoteLeader(a, b, ...)` | | Promote in this order; unlisted followers come after, by CAN id. |
| `dropFailedFollowers()` | off | A follower dies: stop commanding it and keep running. |
| `disableMechanism()` | | Back to strict: any member death applies the motors' `FailurePolicy`. |
| `minimumAlive(n)` | 1 | Fewer than `n` alive motors: apply the failure policy anyway. |
| `derateOutput(f)` | 1.0 | While any member is dead, scale percent and voltage commands by `f`. |
| `derateCurrentLimit(f)` | 1.0 | While any member is dead, scale the survivors' declared current limits by `f`. |
| `rejoinOnRecovery(b)` | true | Let a recovered motor rejoin the group. |
| `recoverAfter(n)` | 5 | Healthy hardware refreshes a dead motor needs before it rejoins. |
| `onFailure(callback)` | | Runs on the robot loop for `LEADER_PROMOTED`, `FOLLOWER_DROPPED` and `GROUP_FAILED`. |
| `onRecovery(callback)` | | Runs on the robot loop for `MOTOR_RECOVERED`. |

When the group cannot continue (no promotion allowed, no alive candidate, a follower died without
`dropFailedFollowers()`, or below `minimumAlive`), it reports `GROUP_FAILED` and the dead motor's
`FailurePolicy` applies as before: by default the mechanism is disabled, and it is re-enabled once
the motor is healthy again. While the group handles a death itself, the failure is still reported,
as a warning, and the mechanism keeps running.

## Detecting a dead motor

Each hardware refresh judges every member from signals the hardware layer already has:

- **Lost CAN or stale signals.** The vendor handle's input refresh throws. CTRE checks
  `isConnected()`, which goes false when status signals stop arriving; REV checks the controller's
  firmware version. A failed refresh is retried with the graph's usual backoff (100 ms, doubling to 2 s).
- **Faults.** `MotorHandle.faulted()`, read from the inputs cached by the same refresh, so it adds no
  bus traffic. Only faults that stop the controller from driving count:
  - REV SPARK: gate driver, motor type mismatch, ESC EEPROM and firmware faults. The handle reads
    `hasActiveFault()` from the cached status frame and decodes the fault bits only when it is set.
  - CTRE TalonFX: `Fault_Hardware`. TalonFXS also counts `Fault_BridgeShort`,
    `Fault_DriveDisabledHallSensor` and `Fault_MotorArrangementNotSelected`. These signals ride in
    the handle's existing `refreshAll` batch.
  - Simulation: whatever the test sets with `SimMotorHandle.faulted(true)`.

  Transient faults and warnings (current, supply or temperature limits, brownout, CAN, sensor and
  limit-switch faults) do not mark a motor dead. Sticky faults are ignored; once the active fault
  clears, the motor can rejoin through recovery.
- **Commands that throw.** A command that throws for a member marks it dead in the next refresh.

A dead motor is stopped, so a stale request cannot resume if it reconnects, and is no longer
commanded.

## Promotion

When the leader dies and promotion is allowed:

1. The first alive follower leaves hardware follower mode (`MotorHandle.stopFollowing()`) and runs
   uninverted; Athena applies its direction in software, so every shaft keeps turning the way it did.
2. Commands, onboard closed-loop targets and the declared leader's integrated encoder now go to that
   motor. Integrated position stays continuous: an offset makes `leader.encoder()` read where the dead
   leader last was, and closed-loop position targets are shifted by the same offset.
3. The other hardware followers are re-pointed to the new leader, with their inversion relative to it.
   A follower that cannot hardware-follow it (another bus or vendor) is mirrored in software instead.
   Software followers keep mirroring the group command and need no change.

Feedback from an external sensor (CANcoder, through-bore) is unaffected. Feedback from the leader's
attached absolute encoder (`leader.absoluteEncoder()`) is lost with the leader, so the group does not
promote and fails instead.

Directions follow the existing declarations: a hardware follower's `inverted()` means it opposes the
leader; a software follower's `inverted()` flips the command Athena mirrors to it.

## Recovery

A dead motor that has `recoverAfter(n)` consecutive healthy refreshes rejoins as a follower of the
motor leading at that time, in hardware when possible. Leadership never flips back on its own: a
recovered original leader follows the promoted motor, so a recovered motor never causes a transient.
The derate lifts once every member is alive again.

## Telemetry

Under the field's name in the mechanism's `Values`:

| Key | Value |
| --- | --- |
| `Leader` | Name of the motor leading the group. |
| `Alive` | Number of alive motors. |
| `AliveMotors`, `DeadMotors` | Comma-separated motor names. |
| `Degraded` | Some member is dead and the group keeps running. |
| `Failed` | The group could not continue; the failure policy applies. |

From code, `failover.leader()`, `failover.alive(motor)` and `failover.group()` read the same state.
The per-motor `Connected` and `Failure` fields of the runtime inspection still report each
controller itself, including the dead leader.

## Cost

Each refresh reads a few flags per member and does not allocate. Promotion, re-pointing and derating
run once per change. The leader handle adds one field read per command.

## Limits

- Groups come from `.follow(leader)`. Bindings that command several motors with
  `Controls.x(...).followers(...)` are not grouped.
- Failover needs a hardware graph, so it runs under `RobotRuntime` (real or `RobotRuntime.simulated`).
  A bare `MechanismScheduler` with hand-registered handles ignores the declaration.
- A motor that fails to bind at startup is not promoted around; the startup failure policy applies.
- `derateOutput` does not scale onboard closed-loop targets; use `derateCurrentLimit` for those.
- Only the REV, CTRE and simulation handles implement `faulted()`. Other vendor handles report
  `false` and rely on the disconnect check above.
- Fault reporting follows the controller's fault status frame. Phoenix 6 fault signals update at
  their default rate (4 Hz) unless a robot raises it, so a CTRE fault can take up to about 250 ms to
  reach `faulted()`. A motor that keeps receiving status but cannot drive for a reason the controller
  does not flag (a cut phase wire, a seized gearbox) is not detected.

## Simulation and tests

`SimMotorHandle.connected(false)` drops a simulated controller off the bus: its refreshes throw and
its commands are dropped. `faulted(true)` raises a fault. See
`athena-robot/src/test/java/ca/frc6390/athena/robot/MotorFailoverIntegrationTest.java`.

```java
SimulationSession simulation = SimulationSession.create();
RobotRuntime runtime = RobotRuntime.simulated(simulation).register(shooter);
runtime.request(shooter.spin);
runtime.teleopPeriodic(0.00, 0.02);

simulation.motor(shooter.leader).connected(false);
runtime.teleopPeriodic(0.02, 0.02);   // the follower now leads; the mechanism keeps running
```
