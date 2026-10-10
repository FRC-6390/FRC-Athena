# Athena 2027 Off-season Design

Status: approved for implementation on branch `2027-offseason`.

This document is the single source of truth for the off-season work. Each section has an ID
(`A1`, `B4`, `E2`, ...). Implementation branches reference these IDs in commit messages and tests.

## 0. Ground rules (apply to every section)

1. **Declarations, not calls.** Robot code declares typed fields on a `Mechanism` (devices, signals,
   bindings, Actions, policies). The runtime discovers them by introspection, exactly as it already
   does for `Action`, `ControlSysId`, `SimModel`, `AutoChooser` and `@Telemetry` fields.
2. **No string keys.** Names and storage keys come from the mechanism tree / field path. Groups are
   identified by classes or team-defined enums, never strings.
3. **Typed policies.** Behaviour choices are typed values on the declaration (like
   `FailurePolicy`). Robot-wide policy is a typed field on the robot (auto-discovered) or a
   constructor call in the style of `systemTuning(...)`.
4. **Constraints stay the home of limits.** Absolute limits live in `Constraints`.
5. **Source compatibility.** Existing robot code (`Rebuilt-2026`, `Rebuilt-Off-Season-Bot`,
   `example-projects`) must keep compiling unless a section explicitly says an API is replaced.
   Prefer additive overloads.
6. **Hot path.** No per-cycle allocation in control/output paths. Convert typed units to primitive
   base units at declaration time.
7. **Every feature ships with tests** (unit tests in the owning module; simulation tests using
   `RobotRuntime.simulated(SimulationSession.create())` where behaviour spans modules).
8. **Docs.** Each public API gets Javadoc. User-facing docs go in `docs/` next to existing topics.

---

## A. Tuning and controls

### A1. Persistent telemetry
`@Telemetry` gains:

```java
boolean persist() default false;   // value survives reboot/deploy
boolean publish() default true;    // false = stored only, never sent over the network
Class<?> lock() default Locks.InMatch.class;   // see A2; only meaningful when writable
```

`TelemetryValue` gains `.persisted()` with the same meaning.

- Storage key = mechanism tree path + field path (the same path telemetry uses).
- Store: a robot-side key/value store under `/home/lvuser/athena/store/` (sim: build dir/temp dir
  configurable). Writes are batched in memory and flushed at most once per second and on disable,
  using write-temp-then-atomic-rename. Reads at registration override the code default.
- Supported persisted types: `double`, `int`, `long`, `boolean`, `String`, enums, `PidGains`,
  `FeedforwardGains`, and arrays of numbers.
- Unknown keys found at boot (field renamed/removed) are deleted. No migration API.
- `ControlBinding` gains `.persistTuning(Class<?> lockGroup)` so dashboard `Config` edits (gains,
  constraint overrides) persist with the same store and lock rules.
- Each persisted write records `{old, new, timestampSeconds, lifecycleMode, matchTime}` into the
  replay markers (C1) when replay is enabled.

### A2. Lock groups
- A lock group is any class. Built-ins: `Locks.InMatch` (default), `Locks.Never`, `Locks.Always`.
- Conditions are defined once on the robot: `tuningLock(Class<?> group, BooleanSupplier locked)`
  in `AthenaRobot` (callable in the robot constructor).
- A group without a defined condition behaves as `Locks.InMatch` (locked while FMS attached).
- **Enforcement is server-side only** (robot runtime). A locked write is rejected and reported
  (`rejected: locked (<group simple name>)`). Clients never decide.
- The current lock state of each writable value is exposed in the telemetry schema.

### A3. Binding sets
`Gamepad` (and every controller type) gains:

```java
Gamepad in(BooleanSupplier condition, Consumer<Gamepad> bindings);
```

- Inside the consumer, every binding created on the passed gamepad view is gated by `condition`
  (the binding only activates/fires while the condition is true; edge bindings treat the condition
  turning false as a release).
- Bindings made directly on the original gamepad are unaffected.
- Sets can be nested (`pad.in(...)` inside a set ANDs conditions).
- At registration, if two sets whose conditions can be simultaneously true bind the same physical
  input with the same edge type, report a warning listing both (best effort: report when both
  bindings are active in the same cycle, rate limited).
- No mode-select type. Conditions are any `BooleanSupplier`/`ControlSignal`/toggle.

### A4. `record()` on interpolation Actions
- `InterpolatedControlAction.record()` returns an `Action` that, when run, inserts a point
  `(current input, current output target)` into that interpolation (persisted per A1 if the
  interpolation's points are persisted; otherwise kept until reboot).
- Recording also forces a **full-detail replay capture** of the current cycle (all signals, not
  rate limited) and plants a replay marker (C1) of type `RECORD` with the interpolation path,
  input, output, robot mode, match time, and event/match identifiers.
- Points carry metadata `{timestampSeconds, replayId, frameIndex, matchType, matchNumber,
  eventName}` exposed in telemetry so the dashboard can link to replay.

### A5. Explain why a mechanism is not moving
- Each mechanism publishes `State/Reason` (string, published only on change) summarising the
  existing `RuntimeInspection` decision for its outputs, e.g.
  `Blocked: require(...) not satisfied`, `Starved: newer lease <action> reserves <motor>`,
  `Guard: at max edge`, `Stage: <enum constant>` (B9), `Thermal derate 60%` (B8).

---

## B. Control system

Layers, evaluated inside every `ControlBinding` each cycle:

```
measure -> model predicts expected motion -> load estimate (actual - predicted)
        -> motion policy (B4) -> effort policy (B5) -> electrical limits (B7/B8) -> output
staging (B9) can cap/override at the motion and effort layers.
```

### B0. Typed units
- Use WPILib units (`edu.wpi.first.units`).
- `EncoderDevice` gains `.measures(AngleUnit | DistanceUnit)` and `.reduction(double)` /
  `.gearing(GearRatio)` typed alternatives to `.conversion(...)`/`.gearRatio(...)` (old methods stay).
- `Controls.position(motor, Unit)` / `Controls.velocity(motor, Unit)` overloads return
  `ControlBinding<U>` where `U` is the unit type (`AngleUnit`, `DistanceUnit`). Existing
  `Controls.position(motor)` keeps returning a binding usable with raw doubles.
- Typed methods on bindings: `position(Measure<U>)`, `velocity(Measure<velocity of U>)`,
  `untilWithin(Measure<U>)`, `Constraints.range(Measure<U> min, Measure<U> max)`,
  `Constraints.motion(...)` typed overloads, interpolation with typed input/output.
- Dynamic suppliers use primitive overloads with an explicit unit:
  `position(DoubleSupplier value, U unit)`.
- Mismatch between the binding unit and the encoder `measures(...)` is a startup error with a clear
  message.
- Rotary vs linear is derived from the unit (angle = rotary, distance = linear). B1/B5 use it.
- No per-cycle `Measure` allocation: convert to base-unit doubles at declaration.

### B1. Characterization with history
```java
public final Characterization characterization = angle.characterization()
        .maxVoltage(Volts.of(4))
        .learnContinuously(true);
public final MechanismModel model = characterization.model();
```
- `Characterization` is a declaration discovered like `ControlSysId`; it expands into dashboard
  Actions: `Quasistatic{Forward,Reverse}`, `Dynamic{Forward,Reverse}`, `FrictionSweep`,
  `Backlash`, `Latency`, optional `Thermal`. All run through the normal scheduler with the
  binding's constraints active, and never exceed `maxVoltage`/`rampRate`.
- `MechanismModel` contains: kS static, kS kinetic, kV, kA, kG and gravity angle offset (rotary)
  or constant kG (linear), friction vs position table, backlash, latency, per-parameter confidence,
  model kind (rotary/linear from B0), motor count/kinds/ratios, optional payload delta (B2),
  optional thermal resistance/time constant. Raw run data is stored (A1 store, bounded size) so the
  model can be re-fit.
- Fitting is done on the robot (ordinary least squares; document the equations).
- **Continuous learning**: when enabled, a recursive least-squares estimator updates a *running*
  estimate from normal operation only when excitation is sufficient (documented criteria). It never
  replaces the active model automatically.
- **History**: after each match (FMS) or on disable after enabled time > 30 s, a snapshot of the
  running estimate is appended to the history (persisted, bounded count). Drift outside a
  configurable tolerance band relative to the baseline publishes an alert.
- `characterization.adoptRunningEstimate()` Action and `rebaseline()` Action.
- `MechanismModel` is a value usable by `SimModel` (`SimModel.arm/elevator/flywheel(...).model(m)`).
- Replaces fitting role of `ControlSysId` (keep `ControlSysId` working; it may delegate).

### B2. Passive payload learning
- `characterization.payloadWhen(BooleanSupplier)` collects loaded samples only while true and
  excitation is sufficient; produces a payload delta with confidence. Until confident, payload is
  treated as an external load (B3).

### B3. Checks
Model-equipped bindings expose:
- `load()` – `NumberSignal`/`DoubleSupplier` estimated external torque/force (B0 units).
- `blocked()`, `backdriven()`, `feedbackStalled()` – `BooleanSignal`s with `.threshold(...)`,
  `.forSeconds(...)` overrides.
- `.slipCheck(EncoderDevice second, GearRatio ratio, Measure<U> tolerance)` – only exists when
  configured; exposes `slipping()`.
- Checks only exist when their data exists (no model => only `feedbackStalled` from current/output).
- `MotorDevice.stall()` keeps all overrides; default uses model-based detection when a motor kind
  has known constants (expected speed from voltage/current vs measured), with no fixed voltage
  threshold.

### B4. Motion policies and bounded commitments
Binding defaults, overridable per Action:
- `.retarget(Retarget.SMOOTH | Retarget.IMMEDIATE)` – policy of the **incoming** Action. SMOOTH
  re-plans the profile from the current reference position *and velocity* (never steps, never
  restarts from rest). Lease takeover inherits the current reference state.
- `.interruptible(Interrupt.ANYTIME | Interrupt.AFTER_COMPLETE | Interrupt.after(BooleanSupplier))`
  – policy of the **running** Action. A newer request for the same resources is held until the
  running Action allows interruption.
- `.release(Release.DECELERATE | Release.HOLD | Release.NEUTRAL | Release.neutral(MotorNeutralMode))`
  – what happens when this was the last Action and nothing replaces it. `NEUTRAL` keeps the motor's
  declared neutral mode. `neutral(mode)` overrides it only until the next Action uses the motor.
- **Bounded commitment rule**: `AFTER_COMPLETE`/`after(...)` is rejected at registration unless
  the Action is guaranteed to finish (every step has a timeout, or `.atMost(Duration)` is given).
- `Actions.attempt(a).within(t).orTry(b).within(t).orElse(c)` fallback chains.
- `Actions.fault(String message)` reports a fault (DriverStation + telemetry + replay marker) and
  completes.
- `Mechanism`-level `abort()` Action and robot disable always override commitments.

### B5. Effort
- Any control Action gains `.effort(Measure)`: rotary bindings accept `Torque` or `Current`,
  linear accept `Force` or `Current` (compile-time where generics allow, startup check otherwise).
  Torque/Force require known gearing; otherwise startup error suggesting `Amps`.
- Binding ceiling: `.effort(Effort.max(Measure).onLoad(LoadResponse...))` with
  `LoadResponse.YIELD.recoverAfter(t)`, `HOLD`, `pushThrough(burst, t)`, `slow(fraction)`.
- Contact behaviour is composition, no new primitives: `angle.position(x).effort(e)
  .until(angle.blocked()).then(angle.position(x).effort(smaller))`. A position target past a hard
  stop with an effort cap is how "hold against stop" is expressed.
- Multi-motor: torque/force <-> current conversion uses each motor's kind (Kt), ratio and count;
  hardware followers share the leader command (leader target scaled by follower count).
- Telemetry shows effort in both mechanism units and amps.

### B6. Model-based torque/current control
- Default implementation: compute voltage from target current using motor resistance, Kv and
  measured speed (`V = I*R + omega/Kv`), so it works on any controller (NEO/SPARK included).
- Vendor hardware torque modes may be used when present; never required.

### B7. Power profiles
```java
public static final PowerProfile SHOOT = PowerProfile.of()
        .sustained(Amps.of(60)).burst(Amps.of(90), Seconds.of(0.5))
        .rampLimit(60.0 /* amps per second? use typed overload */)
        .share(3.0)
        .onSag(Sag.shed(0.5).below(Volts.of(9)));
MotorDevice shooter = CAN.motor(MotorKinds.KRAKEN_X60, 43).power(SHOOT);
public final Action endgame = SHOOT.update(p -> p.share(0.5));
```
- Profiles are shared mutable values referenced by motors; `update(...)` and `restore()` return
  Actions; changes apply next cycle.
- `PowerPolicy` robot field (auto-discovered by `AthenaRobot`, defaults apply when absent):
  `floor(Voltage)`, `staggerStarts(Time)`, battery internal-resistance estimation (persisted per
  battery when identifiable, otherwise rolling).
- Each cycle: predict bus voltage from requested currents; if below floor, scale current limits by
  share weights; apply ramp limits; stagger simultaneous large rises.
- Telemetry under `/Athena/Power`.

### B8. Thermal
- `MotorDevice.thermal(Thermal.derate(Temperature start, Temperature end, double minFraction))`.
  Default when absent: derate only, documented default curve per motor kind.
- `MotorDevice.temperature()` signal (model estimate fused with vendor-reported temperature when
  available), `thermalLoad()` fraction.
- Thermal never brakes/holds/locks out by itself. Any stronger behaviour is B9.

### B9. Staging
```java
enum ArmHealth { NORMAL, STRAINED, HOT, SAFE_SHUTDOWN }
public final Staging<ArmHealth> health = Staging.of(ArmHealth.NORMAL)
        .stage(ArmHealth.STRAINED, condition, angle.limitEffort(0.6))
        .stage(ArmHealth.HOT, condition, angle.limitSpeed(0.5))
        .stage(ArmHealth.SAFE_SHUTDOWN, condition, someAction)
        .exitAfter(Seconds.of(2))
        .override(BooleanSupplier);
```
- Team-defined enum; worst active stage (by ordinal) applies; exit requires the condition to stay
  false for `exitAfter`.
- A stage carries either an Action (requested through the normal scheduler) or modifiers
  (`limitEffort`, `limitSpeed`, `limitAccel`) applied as caps to whatever Action runs.
- `current()` signal; stage changes are replay markers and A5 reasons.
- Staging can be attached to an Action as well (`action.staged(...)`).

### B10. Bumpless gain changes
- When PID/FF gains change at runtime (telemetry write or A1 persisted load while active), the loop
  re-seeds integrator/derivative state so the output is continuous at the change instant.

---

## C. Replay and simulation testing

### C1. Replay
```java
public final ReplayPolicy replay = ReplayPolicy.of()
        .stream(true)
        .buffer(Megabytes.of(48))
        .flushOn(Flush.MATCH_END, Flush.DISABLE_AFTER_MATCH, Flush.BROWNOUT_RECOVERY)
        .retention(Retention.maxTotal(Megabytes.of(400)).maxFiles(60)
                .keepMatchesOverPractice().minimumFreeDisk(Megabytes.of(150)));
```
- Frame = `RuntimeCycleSnapshot` (inputs only), delta-encoded against the previous frame with
  integer IDs for names; compressed in blocks. **The frame format is the ARCP v2 frame format
  (E1)** so stream, buffer, and files share bytes.
- Stream and buffer are independent and can both be enabled. Frames carry sequence numbers; a
  client that detects a gap requests backfill from the buffer.
- Markers: boot, FMS match identified (new file named `<event>-<type><number>`), auto start,
  teleop start, match end, disable, faults (`Actions.fault`), B9 stage changes, check transitions,
  overrides, A1 persisted writes, A4 records.
- Files written only on flush events (or by stream on the client). Retention enforced on the robot.
- API: list/delete/export replays and other stored categories (A1 values, B1 models/history, health)
  through the E6 file service.
- `AthenaSim.replay(ReplayFile).runAgainst(Supplier<Robot>).report()` feeds recorded inputs into a
  new robot instance in simulation and reports per-frame output differences.

### C2. Simulation exploration
- `AthenaSim.explore(Supplier<Robot>)` drives random but legal controller inputs and mode
  transitions for a configurable duration/seed.
- Reports: Actions requested but never given outputs (starvation), commitments that timed out,
  auto markers never fired, effort ceiling hits, constraint violations, faults, stage entries,
  uncaught exceptions.
- Failing runs are minimised (shrink input sequence) and saved as replay files.

---

## D. Alliance transforms
```java
static final AllianceTransform ROTATED = AllianceTransform.rotate180(fieldLength, fieldWidth);
static final AllianceTransform MIRRORED = AllianceTransform.mirrorX(fieldLength);
AllianceTransform.identity(); AllianceTransform.of(UnaryOperator<Pose2d>);
ROTATED.pose(Pose2d); ROTATED.translation(Translation2d); ROTATED.rotation(Rotation2d);
ROTATED.region(Rectangle2d); ROTATED.poseSupplier(Pose2d);
```
Resolved for the current alliance at call time (blue = identity). Path providers accept one.

---

## E. ARCP v2

Current state (v1): one UDP datagram per value per subscriber, no timestamps, subscribe-to-all,
primitive types only, text TCP control, u16 ids capped at 8192, not part of the vendordep.

### E1. Frames
One frame per robot cycle: `{protocolVersion, cycleTimestampMicros (FPGA), sequence, entries[]}`
where entries are `{signalId (varint u32), value}` for changed values only. Split at MTU with a
fragment index. Same encoding used for replay files (C1).

### E2. Subscriptions
`subscribe(pattern, rate)` where pattern is a mechanism-tree glob (`robot/turret/**`) and rate is
Hz or `ON_CHANGE`. Server sends only subscribed signals at the highest requested rate per signal.
Latest values for newly subscribed signals are sent once on subscribe. A server bandwidth budget
lowers low-priority subscription rates first and reports it.

### E3. Schema
Schema announced on connect/change; values reference schema ids. Struct types (Pose2d, Pose3d,
Translation2d, Rotation2d, ChassisSpeeds, SwerveModuleState[], enums) using WPILib struct layout
for lossless NT4/WPILog conversion. Per-signal metadata: unit, range, writable, lock group and
current lock state, owning mechanism path, role (state/value/control/device/action/marker).

### E4. Reliability and time
Realtime telemetry over UDP with sequence numbers (gaps -> backfill). Reliable typed binary control
channel over TCP with request ids and structured replies (writes, Action run/cancel, schema, files).
NTP-style time sync messages.

### E5. Writes
No client roles. Every write is validated by the robot (A2 locks, ranges); rejections are returned
with a reason. Clients must never pre-block writes.

### E6. File service
List/download/delete for replays, persisted values, models/history, health records; tuning diff and
"pull tuning" (robot values vs deployed `deploy/athena/tuning` file vs code defaults).

### E7. Packaging
ARCP becomes an optional transport behind the runtime's sink extension point and is re-added to the
build and vendordep. Rust core via JNI where available; pure-Java implementation of the same wire
format as fallback (required for sim/tests). Optional NT4 bridge mirroring selected subtrees.

---

## F. Dashboard (host-dashboard)

- F1. Generated mechanism pages from the telemetry schema (state, reason, stage, actions run/cancel,
  control plots target/reference/measured, gains with lock state, constraints, devices with
  current/temperature/derate, telemetry values, characterization runner + history).
- F2. Workspaces: Driver, Pit, Tuning, Analysis.
- F3. Analysis: replay timeline with markers, synchronized WebGL plots with LTTB decimation, 2D/3D
  field, replay comparison including re-sim diff, CSV/WPILog export, WPILog/AdvantageKit import.
- F4. Architecture: Rust data core in Tauri backend (decode, record, index, backfill, files); small
  typed widget modules (each declares accepted types); layouts per user, optionally saved into the
  robot project; alerts; plugin API on the same typed contract. Break up the 3k-line
  `WidgetCanvas.svelte`, `WidgetConfigEditor.svelte`, `widget-config.ts`.

---

## Workstreams

| Stream | Sections | Modules |
| --- | --- | --- |
| units | B0 | athena-hardware, athena-mechanisms, athena-runtime |
| controls | A3, D | athena-wpilib (controls), athena-runtime (geometry) |
| arcp | E1-E7 | athena-arcp, build, vendordep |
| dashboard | F1-F4 | athena-arcp/host-dashboard |
| replay | C1 | athena-robot, athena-wpilib, athena-simulation |
| telemetry | A1, A2, A4, A5 | athena-mechanisms, athena-wpilib, athena-robot |
| model | B1-B3 | athena-mechanisms (sysid/model), athena-hardware, athena-simulation |
| motion | B4, B10 | athena-mechanisms core |
| effort | B5, B6 | athena-mechanisms, athena-hardware |
| power | B7-B9 | athena-hardware, athena-mechanisms, athena-wpilib |
| sim-testing | C2, examples, docs | athena-simulation, example-projects, docs |
