# Simulation exploration

Section C2 of `docs/2027-offseason-design.md`. `SimExplorer` drives a robot with random but legal driver station
modes and controller input, for a fixed seed, and reports what went wrong: requested Actions that never get output,
commitments that hit their timeout, unfired path markers, effort ceilings, constraint and guard limits, faults,
stage entries, and uncaught exceptions. The first finding of each problem category is shrunk to a minimal input
sequence and can be saved as a replay.

## Running it

```java
ExplorationReport report = SimExplorer.of(session -> RobotRuntime.simulated(session).register(new Robot()))
        .seed(7)                                  // fixes the mode sequence and the input script
        .duration(150)                            // seconds
        .controllers(0)                           // joystick ports, 0 to 5
        .modes(ModeSchedule.random())             // or ModeSchedule.matchPlay()
        .inputs(InputModel.standard())            // button, axis and POV rates
        .starvationSeconds(2.0)                   // default
        .save(Path.of("build/explore"))           // optional: write minimal replays here
        .run();

if (!report.clean()) System.out.println(report.describe());
```

The factory receives the `SimulationSession` and must build the runtime on it, as `ReplaySimulation` does.

### Modes

- `ModeSchedule.random()`: a random sequence of legal transitions. Disabled is followed by autonomous, teleop or
  test. Every enabled mode is followed by disabled. Dwell times default to disabled 1 to 4 s, autonomous 3 to 15 s,
  teleop 10 to 60 s, test 3 to 10 s. `dwell(mode, min, max)` changes them.
- `ModeSchedule.matchPlay()`: disabled 1 s, autonomous 15 s, disabled 3 s, teleop 135 s, then disabled.
- `ModeSchedule.of(segments...)`: a fixed sequence.

The robot is disabled after the last segment until the duration ends.

### Inputs

`InputModel.standard()` gives eight buttons pressed about 0.6 times a second, held 1.2 s on average, four axes
random-walking at 4 Hz, and POVs changing about 0.1 times a second. The model is adjusted with `buttons`,
`buttonPressesPerSecond`, `meanHoldSeconds`, `axes`, `axisWalk` and `povChangesPerSecond`. Each controller port has
its own generator, so adding a port does not change the events of the others.

Inputs are fixed events, not live decisions. Each event is `InputEvent(time, port, kind, index, value)`, and the
controller state at any time is the result of applying every earlier event. Removing events is therefore well
defined, which is what shrinking needs.

## How input reaches the robot

Each cycle the explorer does what a replay does:

1. It builds a `RuntimeInputSnapshot` for the cycle (joysticks, driver station state with the current mode).
2. It calls `RuntimeInputs.inject(snapshot)`, so controls read the recorded buttons, axes and POVs through the
   same `InjectedJoysticks` path as replay. Bindings fire as they would from a real controller.
3. It calls `runtime.inputs(...)` with the same snapshot, so a recording captures it.
4. It runs `periodic(...)` with the mode and phase, releases the injection, then calls `simulationPeriodic(...)`.

Lifecycle: the first cycle of a new mode uses the `INIT` phase, and the rest use `PERIODIC`. Driver station match
time counts from the start of the current segment.

## What is detected

| Category | Source | Meaning |
| --- | --- | --- |
| `STARVATION` | Trace candidates (`MechanismTraceSnapshot.candidates`) | A lease's output request is rejected with "resource reserved by newer action" for at least `starvationSeconds` without output. |
| `COMMITMENT_TIMEOUT` | `RuntimeEvents.TIMEOUT` | An `atMost` or `Actions.timeout` bound expired before its Action completed. |
| `AUTO_MARKER_PENDING` | `RuntimeEvents.PATH_MARKERS_PENDING` | A path finished while some of its markers had not triggered. |
| `EFFORT_CEILING` | `RuntimeEvents.EFFORT_CEILING` | An effort ceiling clamped a command (see below). |
| `CONSTRAINT_VIOLATION` | Trace controls, `constrained` | A control's requested target was limited by its declared constraints. |
| `GUARD_BLOCK` | Trace controls, `blocked` | A control's output was held back by a guard. |
| `FAULT` | `RuntimeFailureReporter.error` | Fault isolation contained a runtime fault, or `Actions.fault` ran. |
| `STAGE_ENTRY` | `RobotRuntime.drainStageChanges()` | Informational. Not shrunk. Not counted as a problem. |
| `EXCEPTION` | Exception from `periodic` or `simulationPeriodic` | The run stops at that cycle. |

A starvation finding is not raised for a request held back by an interruptible commitment, because a held request
has no candidate yet. Commitment holds are visible only through their timeouts.

Warnings are not findings. They are rate limited on the wall clock, which would make runs differ.

Mechanism trace materialization is raised to `CAPTURE` with a per-cycle period for the duration of a run, because
candidates and controls exist only at that level. This changes what is materialized, not what the robot does.

Findings are grouped by category and subject. Each group has a first timestamp and an occurrence count. The count
for starvation counts threshold crossings, so a request that is rejected, granted and rejected again counts twice.

### Effort hook (B5, not merged yet)

`RuntimeEvents.EFFORT_CEILING` is reserved. The effort work should call

```java
RuntimeEvents.emit(RuntimeEvents.EFFORT_CEILING, mechanismPath + "/" + controlName + " clamped to " + ceiling);
```

at the point where the ceiling is applied. The explorer reports those events with no further change.

## Shrinking

For each problem category, the first finding is shrunk:

1. The script is cut at the finding time plus 0.1 s, and the run is shortened to match.
2. Delta debugging (`Shrinker.minimize`, ddmin) removes chunks of events while the same category still reproduces,
   with the same seed-derived modes and physics. The result is 1-minimal: removing any remaining event loses the
   finding.
3. The minimal script is run again. With `save(...)` set, that run is recorded and the category's markers are planted
   in the file.

Other findings in the same category keep their first timestamp, history and occurrence count, but are not shrunk.
With `shrink(false)` nothing is shrunk and no replays are written.

The report lists the minimal script with timestamps, the replay path, and the history: up to 40 input and mode lines
before the finding.

## Saved replays

With `save(directory)` a shrunk finding writes `explore-<category>-seed<seed>.athrp`. The file is a normal `.athrp`
recording of the minimal run:

- Inputs are recorded through the same snapshot path as the robot, so `ReplaySimulation.replay(file).runAgainst(...)`
  reproduces the commands with no differences when the code has not changed.
- Markers are planted at detection time: `STARVATION` as user kind 101, `COMMITMENT_TIMEOUT` 102,
  `AUTO_MARKER_PENDING` 103, `EFFORT_CEILING` 104 (`ReplayMarkers.USER_BASE` + 1..4), `CHECK` for constraint and
  guard findings, `FAULT` for faults and exceptions, `STAGE_CHANGE` for stage entries.

## Determinism

The mode sequence and the input script come from the seed. The simulation and robot code have no wall-clock input,
so the same seed, robot code and options give an equal `ExplorationReport`. The report contains no wall-clock
values. The test `sameSeedGivesSameReport` checks this.

## Limits

- Action completion when expected (optional item 9) is not implemented.
- `AUTO_MARKER_PENDING` is raised only when a path reaches its normal end. A path interrupted by a mode change is not
  checked.
- Shrinking runs one reproduction per category. Multi-cause findings in the same category may shrink to a sequence
  that reproduces a different subject.
- `DriverStation` statics are not redirected, as in replay. Robot code must read controls through the controls
  adapters or `RuntimeInputs.injected()`.
- Events and staging changes are consumed by the explorer. Use it on a runtime that owns its stage observers.
- Joysticks beyond port 5 and non-joystick inputs are not generated.

## Tests

`athena-robot/src/test/java/ca/frc6390/athena/robot/explore/SimExplorerTest.java`:

- A robot whose button 2 shot and button 1 hook-held roller share one motor. The explorer finds starvation, shrinks it
  to two inputs, saves a replay that replays with no differences, and finds the marker in it.
- A clean robot with disjoint resources, random modes and random input, over 120 seconds, has no findings.
- The same seed gives the same report.
- The mode generator produces only legal transitions, and the shrinker returns the minimal pair from a list.
