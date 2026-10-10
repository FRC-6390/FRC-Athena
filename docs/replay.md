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
  `hardware/imu/<name>/...`
- digital inputs: `digital/<name>/raw`, `digital/<name>/active`
- outputs: `command/<motor>/mode` and `command/<motor>/value`, the resolved motor commands. They are written
  only for cycles that ran mechanisms.

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
DISABLE, FAULT, STAGE_CHANGE, CHECK, OVERRIDE, PERSISTED_WRITE, RECORD, TEST_START. Athena also uses kind 100
(`ReplayMarkers.BROWNOUT`) in the user range.

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

Flushes are synchronous and run on the robot thread.

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
flag), and deletes by name. `StoredFiles.of(directory, suffix)` provides the same listing, deletion, and
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
position, velocity, voltage and currents; encoders; digital inputs). It runs the robot for the same timestamp,
mode, and phase, and compares its motor commands with the recorded ones. Physics is not stepped.

Recording a simulated run uses the same path as `AthenaRobot`:

```java
runtime.replay(ReplayPolicy.of(), dir);
runtime.periodic(new MechanismContext(now, 0.0, dt, true, false, true),
        new EventContext(now, dt, LifecycleMode.TELEOP, LifecyclePhase.PERIODIC, true, true));
runtime.simulationPeriodic(now, dt);
Path file = runtime.replayRecorder().orElseThrow().flush().orElseThrow();
```

Limits:

- IMU readings and motor connection and failure flags are recorded but not injected. Localization
  measurements are not recorded at all. A replay that depends on any of them will diverge.
- Controller and joystick inputs are not part of `RuntimeCycleSnapshot`, so they are not recorded yet.
- Replay compares commands only. It does not compare telemetry or traces.
