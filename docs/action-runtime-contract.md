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
- A preempted lease remains active. It resumes automatically when all newer conflicting
  leases release. Requesting an existing action again restarts it and makes it newest.
- A newer request that overlaps a running Action whose interrupt policy does not allow
  interruption is held pending: it does not preempt, reserve, or evaluate until the blocker allows
  it (the blocker completes, its `Interrupt.after(...)` condition becomes true, or the robot is
  disabled). Held requests start in request order. Disjoint requests run immediately.
  `cancel(action)` removes a held request. See "Motion Policies".

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

## Completion And Cancellation

- A finite action completes when its action tree reports completion. Athena then releases
  its lease and records completion for `isComplete(action)`.
- Requesting a completed action clears the old completion and starts it from the beginning.
- `cancel(action)` releases the action, clears its completion record, and cancels leases the
  action requested while it was running. Actions that only appear inside its tree but were
  requested independently (for example by a button) keep running. Cancellation overrides interrupt
  policies.
- `Actions.abort(mechanism)` cancels every running lease that touches the mechanism, ignoring
  interrupt policies. Held newer requests are not cancelled and start in the same cycle. The abort
  is a request-only Action and cannot be nested in another Action.
- A released resource is neutralized exactly once unless another active lease drives it
  in that cycle. Holding position or retaining output requires an active action; stale
  output is never a completion behavior, except when a control binding's release policy is
  HOLD or DECELERATE (see "Motion Policies").

### Tolerance Completion

`untilWithin(tolerance)` completes a position or velocity control on the first cycle its measured
feedback is inside the window. That is the default and is unchanged. A mechanism that swings through
the target at full speed therefore completes mid-swing, releases its control, and coasts past.
Add an explicit settle condition when the mechanism must actually stop there:

- `untilWithin(1.5).settled(0.1)` (or `.settled(Duration)`) requires the measurement to stay inside the
  window continuously for 0.1 s. The timer restarts whenever the measurement leaves the window.
- `untilWithin(1.5).maxVelocity(20.0)` (or `.maxVelocity(Measure)`, a rate of the control unit) also
  requires `|measured velocity| <= 20` in control units per second. Position controls only: a velocity
  control's target already is a velocity, so `maxVelocity` on one is rejected when declared.
- `untilSettled(1.5, 0.1)` is shorthand for `untilWithin(1.5).settled(0.1)`.

`settled` and `maxVelocity` apply only to an `untilWithin` action; calling them on anything else
throws. Both options survive `effort(...)` limits.

## Faults

- `RobotRuntime` isolates runtime exceptions per mechanism and lease: the faulting lease
  contributes no output that cycle (its motors go neutral), other leases and mechanisms keep
  running, and the fault is reported (rate limited) through the runtime failure reporter and
  the mechanism `State/Fault` telemetry value. `PANIC` failure policies still propagate.
  A bare `MechanismScheduler` throws unless `isolateFaults(true)` is set.
- Non-finite dynamic targets hold the last valid value (or neutral when there is none); non-finite
  open-loop efforts are neutral. Both report a rate-limited warning instead of throwing.
- `Actions.fault(message)` completes immediately and queues the message on the runtime fault channel.
  `RobotRuntime` reports it through the runtime failure reporter. It does not neutralize hardware.

## Control Bindings

- One runtime loop set is bound to each `ControlBinding` identity and reused for the life
  of its mechanism runtime.
- Switching position or velocity actions on the same binding changes the target without
  recreating or resetting PID, feedforward, or profile runtime state.
- Neutral, cancellation, loss of arbitration, and disable reset state exactly once. A
  later request reuses the bound loops and restarts from current measured feedback, except as
  described below.
- Takeover by a different request kind (position to velocity or back) and a new target on a running
  profile follow the binding's retarget policy (see "Motion Policies"). SMOOTH, the default, inherits
  the reference position and velocity. IMMEDIATE re-plans from measured feedback.
- Under SMOOTH, an idle gap keeps the reference. A cycle in which the binding is neither driven nor
  held, and no other output drives its motors, releases the loops but keeps the reference, so a later
  request resumes without restarting from zero velocity. A cycle in which another output (for example
  an open-loop percent action) drives the motors is a takeover by that output, and the reference
  restarts from measured feedback. Under IMMEDIATE, every idle gap restarts from measured feedback.
- Gain changes are bumpless. When PID or feedforward gains of an active control change at runtime
  (dashboard writes or any other gain API), the next output equals the previous applied output. The
  integral term is re-seeded and the derivative history is reset. Feedforward-only controls have no
  integrator, so their output steps by the feedforward change.
- After a `Release.HOLD` or `Release.DECELERATE` release, the binding keeps its closed-loop target
  without a lease, and its motors stay driven. Disable, neutral, and runtime-override disable clear
  the hold.

## Motion Policies

Policies are typed values, set as defaults on a control binding or overridden per Action.

- **Retarget** (`ControlBinding.retarget(...)`, `action.retarget(...)`). `SMOOTH` (default): a new
  target re-plans the profile from the current reference position and velocity. It never steps and
  never restarts from rest. `IMMEDIATE`: each new target re-plans from the measured state and may
  brake at the full profile deceleration.
- **Interrupt** (`action.interruptible(...)`, root Actions only). `Interrupt.ANYTIME` (default),
  `Interrupt.AFTER_COMPLETE`, or `Interrupt.after(BooleanSupplier)`. The policy belongs to the
  running Action and is checked against each newer request for overlapping resources.
- **Release** (`ControlBinding.release(...)`, `action.release(...)`). Applies when the last Action
  driving the binding ends and nothing replaces it. `Release.NEUTRAL` (default) keeps today's
  behavior. `Release.HOLD` keeps the last reference position, or zero velocity for velocity controls,
  and stops the profile immediately. `Release.DECELERATE` brakes along the profile at its maximum
  deceleration to the braking distance, then holds. Without a profile, and for velocity controls,
  DECELERATE holds at once, or ramps to zero when a velocity acceleration limit is configured.
  `Release.neutral(mode)` applies the neutral mode override to the motors until the next Action
  commands them. Backends without runtime configuration keep their declared mode. HOLD and
  DECELERATE act on position and velocity controls only; for percent and voltage controls they
  behave as NEUTRAL.
- **Bounded commitment.** `AFTER_COMPLETE` and `after(...)` are accepted only for Actions that
  `Actions.isBounded(action)` proves finish on their own. Every continuous leaf (a position, velocity,
  percent, voltage, wait-until, or computed Action) must sit inside `timeout(...)`, `forTime(...)`, or
  `atMost(Duration)`. Unknown custom Actions are unbounded. The check runs when the policy is applied,
  so violations fail at declaration with an `IllegalArgumentException`.
- **Fallback chains.** `Actions.attempt(a).within(t).orTry(b).within(t).orElse(c)`. An attempt
  succeeds when it completes before its time limit. A failed or timed-out attempt moves to the next,
  and `orElse` runs only when every attempt fails.
- **Abort.** `Actions.abort(mechanism)` cancels committed Actions on the mechanism, overriding
  interrupt policies. Disable also overrides them.

Policies are only valid on a single position or velocity control Action, or on a root Action for
interrupt. Nesting a root-only policy in another Action fails when the tree is compiled.

## Robot Modes

- Disabled mode suspends action evaluation and neutralizes driven motors. It does not
  complete or cancel directly requested actions; they resume when enabled.
- Direct requests are not teleop-only and may run in autonomous or teleop.
- Lifecycle hooks are mode-scoped. For example, `Events.teleopPeriodic().whileActive(...)`
  releases its lease outside teleop and reacquires it when teleop periodic resumes.

## Structure

- Every Action describes its shape once. `Action.children()` returns the labelled structural
  children (`step`, `branch`, `body`, `first`, `then`, `primary`, `with`, `true`, `false`, `drive`,
  or `marker:<name>` for a path marker). `Action.describe()` returns a one-line label such as
  `TIMEOUT 2.0s`; pure sequencing glue (`then`) returns an empty label.
- Read-only walkers (`AutoPlan`, `PathIntrospector`, the scheduler's declaration and opaque-graph
  scans) traverse through `children()`. Evaluation stays type-specific because it depends on runtime
  state.
- `Actions.Then` and `Actions.Conditional` are the only sequencing and until types. Fluent
  (`action.then(...)`, `action.until(...)`) and static (`Actions.then(...)`, `Actions.until(...)`) forms
  build the same types, and a chained continuation after an existing one sequences after it.
- Hardware declarations (`MotorDevice.percent(...)`, encoder and IMU actions, power profile actions)
  create their Actions through `DeviceActionFactories`, which athena-mechanisms supplies through
  `ServiceLoader`. athena-hardware has no reflective reference to the mechanism types.

## Related Contracts

- Control state reset, constraints, and repeated closed-loop requests are covered by
  `MechanismRuntimeTest`.
- Multi-mechanism and `collect -> shoot -> collect` arbitration are covered by
  `ActionArbitrationTest`.
- Retarget, release, interrupt holds, abort, bounded commitment, attempt chains, faults, and
  bumpless gain changes are covered by `MotionPolicyTest`. Examples live in `docs/motion-policies.md`.
- Cross-bus hardware following and transient backend recovery are covered by
  `HardwareGraphTest` and `RobotRuntimeTest`.
