# Action Runtime Contract

This document defines the behavior Athena guarantees for actions. The executable
contract lives in `ActionLifecycleContractTest` and must change with this document.

## Ownership

- A requested action must either be declared under a registered mechanism or target
  hardware or a control binding declared under one.
- Actions may be constructed outside the mechanism or after registration. Their target
  declaration determines ownership.
- Composite actions may span multiple registered mechanisms. Athena partitions them by
  owner while preserving one request lifecycle.
- A request with no inferable owner fails immediately before a periodic cycle runs.

## Request And Acquisition

- `action.request()` and `MechanismScheduler.request(action)` create a lease for the
  complete action tree.
- A lease reserves the resources it is using or can use soon: the active node subtree, the
  next sequence/`then` stage (so consecutive stages hand a resource over without a gap), both
  branches of a live conditional, every step of a cycle, and a running path's ownership plus
  its pending and running markers. Stages two or more steps ahead, completed paths, and
  completed markers are not reserved, so older leases (default drive, homing) keep running
  while a newer lease does not need their resources. Opaque provider paths keep their full
  compiled reservation.
- Disjoint leases run together.
- For a conflicting resource, the most recently requested lease supplies the output.
- A preempted lease remains active (its timers keep running). It resumes automatically when
  all newer conflicting leases release. Requesting an existing action again restarts it and
  makes it newest, so request an action once (for example from `onTrue`/`whileTrue`), not
  every loop; a per-loop `request()` restarts it each cycle and its waits never finish.
- An action that requests itself from inside its own tree restarts on the next cycle.

## Evaluation

- Active actions evaluate once per runtime cycle while the robot is enabled.
- Dynamic values and conditional branches are sampled each cycle.
- Athena compiles ownership, mechanism partitions, and reserved resources when a declared
  action graph is registered. Method-created actions are compiled once on first request.
- `Actions.compute(resolver, ownership...)` requires the static superset of devices, controls,
  mechanisms, or actions the resolver may emit. The resolver runs only during periodic evaluation.
  Emitting hardware outside that ownership fails immediately.
- Provider paths advertise their output ownership. Providers without ownership retain a
  conservative owner-wide fallback for compatibility.
- Changing a branch does not require another request. Resources no longer driven by the
  selected branch are neutralized unless another active lease drives them.
- A composite can replace only the conflicting portion of another composite; unrelated
  resources continue running.

## Composition

The composition rules below are checked against an independent timing model by
`ActionFuzzTest` (random trees of every operator, plus lease restart, cancel, preemption and
disable) and by `ActionRegressionTest`.

- `a.then(b)` and sequence stages run back to back; the next stage starts on the same cycle
  the previous one completes. `a.until(c).then(b).then(d)` keeps every stage.
- Composing never mutates its inputs. `intake.then(stop)` or `intake.timeout(2)` on a declared
  sequence returns a new action; `intake` is unchanged.
- `timeout(s)` / `forTime(s)` end their whole subtree after `s` seconds, measured from when
  that node starts. On a sequence it limits everything built so far; stages added after it
  (including `then`) run afterwards.
- `parallel` completes when every child has completed, `race` when any child completes, and
  `deadline` when its first child completes. Children of a finished group are stopped.
- `until(c)` completes when `c` is true or the wrapped action finishes on its own. Once it
  completes it never falls back into the wrapped action.
- `Actions.when(c).run(a).otherwise(b)` is a live rule: `c` is checked every cycle and the
  running branch switches when it changes. Without `otherwise`, a false condition outputs
  neutral and never completes. For a one-time decision use `Actions.either(c, a, b)` or
  `Actions.onlyIf(c, a)`, which check `c` once when they start and skip when false.
- Durations must be numbers; `NaN` is rejected when the action is built.

## Stall Detection In Actions

- `Actions.untilStalled(signal, action)`, `deviceAction.untilStalled(signal)`, and the
  `Sequence`/`Cycle` `untilStalled(signal, action)` steps run an action until the motor's
  stall detector trips. The debounce window (`signal.forSeconds(...)`) restarts every time
  the action starts, so one detector can be shared by several actions.
- Typical homing: `Actions.sequence().untilStalled(motor.stall(), arm.voltage(-2.0))
  .doOnce(motor.encoder().setPosition(0.0)).then(arm.neutral())`.
- `Events.when(signal).rising()` remains available for robot-wide stall reactions.

## Completion And Cancellation

- A finite action completes when its action tree reports completion. Athena then releases
  its lease and records completion for `isComplete(action)`.
- Requesting a completed action clears the old completion and starts it from the beginning.
- `cancel(action)` releases the action, clears its completion record, and cancels leases the
  action requested while it was running. Actions that only appear inside its tree but were
  requested independently (for example by a button) keep running.
- A released resource is neutralized exactly once unless another active lease drives it
  in that cycle. Holding position or retaining output requires an active action; stale
  output is never a completion behavior.

## Faults

- `RobotRuntime` isolates runtime exceptions per mechanism and lease: the faulting lease
  contributes no output that cycle (its motors go neutral), other leases and mechanisms keep
  running, and the fault is reported (rate limited) through the runtime failure reporter and
  the mechanism `State/Fault` telemetry value. `PANIC` failure policies still propagate.
  A bare `MechanismScheduler` throws unless `isolateFaults(true)` is set.
- Non-finite dynamic targets hold the last valid value (or neutral when there is none); non-finite
  open-loop efforts are neutral. Both report a rate-limited warning instead of throwing.

## Control Bindings

- One runtime loop set is bound to each `ControlBinding` identity and reused for the life
  of its mechanism runtime.
- Switching position or velocity actions on the same binding changes the target without
  recreating or resetting PID, feedforward, or profile runtime state.
- Neutral, cancellation, loss of arbitration, and disable reset state exactly once. A
  later request reuses the bound loops and restarts from current measured feedback.

## Robot Modes

- Disabled mode suspends action evaluation and neutralizes driven motors. It does not
  complete or cancel directly requested actions; they resume when enabled. Action timers are
  paused while disabled, so a wait or timeout resumes with the time it had left.
- Direct requests are not teleop-only and may run in autonomous or teleop.
- Lifecycle hooks are mode-scoped. For example, `Events.teleopPeriodic().whileActive(...)`
  releases its lease outside teleop and reacquires it when teleop periodic resumes.

## Related Contracts

- Control state reset, constraints, and repeated closed-loop requests are covered by
  `MechanismRuntimeTest`.
- Multi-mechanism and `collect -> shoot -> collect` arbitration are covered by
  `ActionArbitrationTest`.
- Cross-bus hardware following and transient backend recovery are covered by
  `HardwareGraphTest` and `RobotRuntimeTest`.
