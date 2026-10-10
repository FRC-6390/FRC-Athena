# Replay recording and replay in simulation

Section C1 of `docs/2027-offseason-design.md`. Every runtime cycle is recorded as encoded frames. The frames
are kept in RAM, written to `.athrp` files on match and disable events, and can be replayed against robot code
in simulation.

## Turning it on

Declare a typed `ReplayPolicy` field on the robot class. `AthenaRobot` finds it by type, with any visibility.
Without the field the defaults apply.

```java
public final class Robot extends AthenaRobot {
    public final ReplayPolicy replay = ReplayPolicy.of()
            .stream(false)                                   // needs a ReplayStreamSink (ARCP, later)
            .bufferMegabytes(48)
            .flushOn(ReplayPolicy.Flush.MATCH_END,
                     ReplayPolicy.Flush.DISABLE_AFTER_MATCH,
                     ReplayPolicy.Flush.BROWNOUT_RECOVERY)
            .retention(Retention.maxTotalMegabytes(400)
                    .maxFiles(60)
                    .keepMatchesOverPractice()
                    .minimumFreeDiskMegabytes(150));
}
```

Defaults when the field is absent: buffer 48 MiB, streaming off, flushes on all three events, retention of
400 MiB, 60 files, matches kept over practice, 150 MiB free. The roboRIO directory is
`/home/lvuser/athena/replays`. In simulation it is `java.io.tmpdir/athena/replays`. Set
`ReplayPolicy.directory(Path)` to override.

Without `AthenaRobot`, call `RobotRuntime.replay(policy[, directory])` and
`RobotRuntime.replayRecorder()`.

## Frames

A frame is one runtime cycle with these inputs and outputs:

- lifecycle: `lifecycle/mode`, `phase`, `enabled`, `autonomous`, `simulation`, `mechanismsRan`,
  `timestampSeconds`, `dtSeconds`
- hardware inputs the mechanisms used that cycle: `hardware/motor/<name>/...`, `hardware/encoder/<name>/...`,
  `hardware/imu/<name>/...` (yaw, pitch, roll, angle, rates, linear acceleration, connected, calibrating, last update)
- digital inputs: `digital/<name>/raw`, `digital/<name>/active`
- driver station input, read at the start of the cycle before mechanisms run:
  - `input/driverStation/<field>`: `enabled`, `autonomous`, `test`, `fmsAttached`, `dsAttached`, `alliance`
    (`red`, `blue`, or empty), `matchTime`, `eventName`, `matchType`, `matchNumber`, `gameMessage`
  - `input/joystick/<port>/<field>` for ports 0 to 5: `connected`, `name`, `type`, `buttons` (bitmask, bit n is
    WPILib button n+1), `axis0` to `axis5`, `pov0` to `pov3`
- localization summaries: `localization/<index>/<field>` with `type`, `count`, `kind` (class of the newest
  measurement), `timestampSeconds`, `latencySeconds`, and `xMeters`, `yMeters`, `headingRadians` from the newest
  pose measurement (NaN when there is none)
- outputs: `command/<motor>/mode` and `command/<motor>/value`, the resolved motor commands. They are written
  only for cycles that ran mechanisms.

The input and localization signals are declared when the recorder is created, so the dictionary has the same
entries in every file. Input is captured by `RobotRuntime.inputs(Supplier<RuntimeInputSnapshot>)`. `AthenaRobot`
sets this to a WPILib driver station sampler whenever replay is enabled.

Hardware inputs are captured at the refresh that mechanisms read, before physics steps. The recording
therefore matches what the robot saw.

Signal ids are stable integers assigned on first sight. A `DICTIONARY` record announces each new id. A frame
carries only the values that changed. A keyframe carries every known value. Keyframes occur on the first frame,
at least once per second, and on `ReplayMarkers.keyframe()` or `ReplayRecorder.keyframe()`.

## File format (`.athrp`, appendix E1-W)

```
"ATHRP\0"  u16 formatVersion (=1, little-endian)
METADATA record           (event, matchType, matchNumber, reason, first/last sequence and time, createdAt...)
DICTIONARY record         (all entries known at flush)
COMPRESSED_BLOCK records  (varint uncompressedLength + raw DEFLATE, about 64 KiB uncompressed each)
```

Each block holds self-contained segments. A segment starts with the full dictionary and a keyframe, so any block
decodes without earlier data. Seek with `ReplayReader.Cursor.seekBlock(i)`. The byte format is the one implemented in
`ca.frc6390.athena.runtime.wire`. Frames, dictionary and marker records use that codec unchanged.

Marker kinds are the appendix E1-W kinds: BOOT, MATCH_IDENTIFIED, AUTO_START, TELEOP_START, MATCH_END,
DISABLE, FAULT, STAGE_CHANGE, CHECK, OVERRIDE, PERSISTED_WRITE, RECORD, TEST_START, and BROWNOUT (14,
`ReplayMarkers.BROWNOUT` = `Wire.MARKER_BROWNOUT`), which marks battery recovery from a brownout.

### Seeking

`ReplayReader.Cursor.seekBlock(i)` jumps to a block. `Cursor.seekTime(micros)` jumps to the first frame or marker
at or after a time. It binary-searches block start times, which are read on demand and cached, so it decodes at
most one block per probe and then starts at that block's keyframe. The next `next()` call returns the event the
seek stopped on. `ReplayReader.blockStartMicros(i)` returns a block's first timestamp.

```java
try (ReplayReader reader = ReplayReader.open(path)) {
    ReplayReader.Cursor cursor = reader.cursor();
    cursor.seekTime(Math.round(37.5 * 1_000_000));
    while (cursor.next()) {
        // frames and markers from 37.5 s on
    }
}
```

Reading a file:

```java
try (ReplayReader reader = ReplayReader.open(path)) {
    ReplayReader.Cursor cursor = reader.cursor();
    while (cursor.next()) {
        if (cursor.isFrame()) {
            double volts = cursor.f64("hardware/motor/intake/appliedVoltage");
        } else {
            int kind = cursor.markerKind();
        }
    }
}
```

## Markers and flush events

Lifecycle markers are added automatically: BOOT after the first robot-init cycle, AUTO_START, TELEOP_START,
TEST_START, DISABLE, and MATCH_END on teleop exit. Other modules add markers through
`ReplayMarkers.mark(kind, message)` and force keyframes with `ReplayMarkers.keyframe()`. These calls go to the
active recorder and are safe from any thread.

| Flush | When |
| --- | --- |
| `MATCH_END` | Teleop exit. The MATCH_END marker is written first. |
| `DISABLE_AFTER_MATCH` | Disable after an auto or teleop period that has not been flushed. |
| `BROWNOUT_RECOVERY` | Battery voltage recovers. A BROWNOUT marker is written first. |
| match identified | The driver station or FMS reports a new match. Buffered pre-match data is flushed as a practice file first, and then MATCH_IDENTIFIED is written. |

### Flushes run on a writer thread

A flush does not compress or touch the disk on the robot thread. `flush()` takes the buffered segments and the
dictionary, names the file, and hands a job to a single background thread (`athena-replay-writer`, daemon, minimum
priority). The thread compresses, writes the `.athrp` file atomically, and then applies retention.

- The hand-off queue holds `ReplayWriter.QUEUE_CAPACITY` (2) waiting jobs. When it is full, the oldest waiting job
  is dropped and counted. A job already being written is never dropped. Memory is therefore bounded by three
  buffers in the worst case.
- Drops are reported from the writer thread, once, as a warning. `ReplayRecorder.droppedFlushes()` returns the
  total.
- `flush()` returns the destination path. The file exists only after the writer finishes, and the writer may pick
  a later name if one with the same name appeared on disk. `lastFile()` returns the file actually written.
- `awaitFlushed(Duration)` blocks until every accepted flush is written or dropped. Use it in tests and before
  reading a file. Do not call it from the robot loop. `pendingFlushes()` reports the backlog.
- `close()` lets queued writes finish, then stops the thread. A recorder that is replaced or closed by
  `RobotRuntime.close()` releases its thread this way.

## File names

- With match information: `<event>-<type><number>-<yyyyMMdd-HHmmss>.athrp`, for example
  `2026mrcmp-q12-20260307-143005.athrp`. The type is `p`, `q`, `e`, or `m`.
- Without: `practice-<yyyyMMdd-HHmmss>.athrp`.

## Retention

After each flush, retention deletes files until every limit holds: total bytes, file count, and minimum
usable bytes on the replay volume. Practice files go first, then matches, oldest first. Set
`deleteOldestFirst()` on `Retention` to ignore the distinction. The file just written is never deleted. A zero
limit means no limit.

## Management API

`ReplayFiles.at(directory)` lists `FileInfo` records (name, size, time, event, match type and number, practice
flag), deletes by name, and exports a copy:

```java
Path copy = ReplayFiles.at(directory).export("2026mrcmp-q12-20260307-143005.athrp", Path.of("/media/usb"));
```

`export(name, target)` copies to `target`, or into `target` when it is an existing directory, through a temporary
file that is moved into place. Names with separators or `..` are rejected. `StoredFiles.of(directory, suffix)` provides the same listing, deletion, and
name validation for other categories. Names with separators or `..` are rejected. The ARCP file service will
use these later.

## Streaming and backfill

`ReplayStreamSink` has `publishDictionary`, `publishFrame(sequence, bytes)`, and `publishMarker`. The sink
receives a `ReplayBackfill` through `attach`. `records(from, to)` returns the buffered segments that cover a
sequence range, each decodable on its own. Streaming runs only when the policy enables it and a sink is
installed. Sinks run on the robot thread and must not block.

## Replay in simulation

```java
ReplaySimulation.ReplayReport report = ReplaySimulation.replay(file)
        .runAgainst(session -> RobotRuntime.simulated(session).register(new Intake()))
        .report();
report.identical();          // true when every recorded motor command was reproduced
report.differences();        // first 1000 differences: sequence, motor, recorded and replayed commands
```

`runAgainst` takes a function that builds a `RobotRuntime` on the `SimulationSession` it is given. Replay
writes each recorded mechanism cycle's hardware inputs into that session with `SimulationInjector` (motor
position, velocity, voltage and currents; encoders; digital inputs; IMU yaw, pitch, roll, rates, and linear
acceleration). It also makes the recorded joystick and driver station state visible to controllers through
`RuntimeInputs.inject`, for the duration of that cycle. The robot runs for the same timestamp, mode, and phase, and
its motor commands are compared with the recorded ones. Physics is not stepped.

Controller bindings read joysticks through the controls `Input` adapters (`Gamepad`, `PlayStationGamepad`,
`GenericController`). These check `RuntimeInputs.injected()` first, so a recorded button press drives the new code's
bindings exactly. The same code gives zero differences; code that binds the button to another action shows them.

Driver station state is recorded and injected too, and is available to robot code through
`RuntimeInputs.injected()`. Code that calls `DriverStation` statics directly, such as
`DriverStation.isTeleopEnabled()`, reads the live driver station unless a sink drives it. To make those reads see the
recorded state, pass the WPILib sink:

```java
ReplaySimulation.replay(file)
        .driverStation(ReplayDriverStationSim.sink())   // athena-wpilib
        .runAgainst(session -> RobotRuntime.simulated(session).register(new Robot()))
        .report();
ReplayDriverStationSim.reset();                          // when finished, before other code reads DS
```

The sink runs before the robot for every recorded frame. It sets the `DriverStationSim` control word (enabled,
autonomous, test), FMS and DS attachment, alliance station, match time, event, match type and number, game message,
and each joystick port's axes, POVs, buttons and name, then calls `notifyNewData()`. The HAL carries 32 buttons per
stick, so the recorded mask is 32 bits wide. The sink is opt-in: without it only `RuntimeInputs` is injected.

Recording a simulated run uses the same path as `AthenaRobot`:

```java
runtime.replay(ReplayPolicy.of(), dir);
runtime.periodic(new MechanismContext(now, 0.0, dt, true, false, true),
        new EventContext(now, dt, LifecycleMode.TELEOP, LifecyclePhase.PERIODIC, true, true));
runtime.simulationPeriodic(now, dt);
runtime.replayRecorder().orElseThrow().flush();
runtime.replayRecorder().orElseThrow().awaitFlushed(Duration.ofSeconds(10));
Path file = runtime.replayRecorder().orElseThrow().lastFile().orElseThrow();
```

Limits:

- Motor connection and failure flags are recorded but not injected. IMU `connected` and `calibrating` flags are
  recorded but not injected. Localization measurements are recorded as summaries and are not injected. The
  localization pipeline runs on the injected sensors, so its output is reproduced only when its inputs are.
- Code that reads `DriverStation` statics directly is redirected only when the `driverStation(...)` sink is set (see
  above). Joysticks are read through the controls adapters, and through the sink when it is set.
- Replay compares commands only. It does not compare telemetry, traces, or localization output.
- Files written before the input signals existed have none. Replay of those files injects empty input.
