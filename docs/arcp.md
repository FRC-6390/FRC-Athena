# ARCP on the robot

ARCP (Athena Realtime Control Protocol) publishes the robot's mechanism telemetry to dashboards over UDP, accepts
writes and Action requests over TCP, serves the robot's files, and streams replay records. The protocol is
implemented in Rust (`athena-arcp/rust`). The robot code only declares a policy and the bridge in
`ca.frc6390.athena.wpilib.arcp` marshals the data across JNI. See `athena-arcp/README.md` for the protocol
and the native build, and `docs/2027-offseason-design.md` section E for the design.

ARCP is **on by default** on the roboRIO and in simulation. The robot does not need it to run.

## Configuring

Declare one `ArcpPolicy` field on the robot class. `AthenaRobot` discovers it, like `ReplayPolicy`. Without the
field the defaults apply.

```java
public final ArcpPolicy arcp = ArcpPolicy.of()
        .controlPort(5808)                          // TCP control port; 5808 is the default (0 picks a free port)
        .realtimePort(5809)                         // UDP port realtime frames are sent from; 5809 is the default
        .bandwidthBudgetBytesPerSecond(400_000)     // 0 (default) = no budget
        .mtu(1200)                                  // largest UDP datagram
        .keyframeInterval(Duration.ofSeconds(1))
        .storeDirectory(Path.of("/home/lvuser/athena/store"))   // optional; persistence follows it
        .modelDirectory(Path.of("/home/lvuser/athena/models")); // optional; characterization storage

// Turn ARCP off completely:
public final ArcpPolicy arcp = ArcpPolicy.disabled();
```

- Ports: the robot listens on TCP 5808 (control) and sends realtime frames from UDP 5809. Dashboards receive on UDP
  5807 by default, falling back to an ephemeral port when 5807 is taken (`udpFallback` in the client status). All of
  these sit inside the FRC team-use range 5800-5810. NT4 uses TCP 5810, so ARCP must never use it. See the port
  table in [`athena-arcp/README.md`](../athena-arcp/README.md#ports).
- Port conflicts: on the roboRIO a port already in use stops ARCP and the DriverStation shows an error naming the
  ports. In simulation ARCP falls back to ephemeral ports with a DriverStation warning. Either way the bound ports are
  printed at startup as `ARCP listening on TCP <port> / UDP <port>`.
- Replay files follow the replay policy's directory (`ReplayPolicy.directory`). The `replays/` file category
  always exposes that directory.
- When the native library cannot start, the bridge reports a warning and the robot runs without ARCP.

## What clients see

### Dictionary

The dictionary is built once from the mechanism telemetry schema when the robot starts.

| Signal | Path | Role | Notes |
| --- | --- | --- | --- |
| Published `@Telemetry` values and `TelemetryValue`s | Telemetry path, for example `turret/Values/kP` | `state` for values under `State`, `control` for writable values, `device`, `hook`, otherwise `value` | Types: number (f64), boolean, string, number array. Writable: flag bit0. Persisted: flag bit1. `lockGroup` is the fully qualified group class name, for example `ca.frc6390.athena.mechanism.persist.Locks$InMatch`. |
| Actions (`TelemetryAction`) | Telemetry path under `Actions`, for example `turret/Actions/aim` | `action` (5) | Boolean running state. Run and cancel are requests (see below). |
| Lock state | `athena/locks/<fully qualified group class name>` | `lock` (7) | Boolean, true = locked. One per lock group in use. |
| Robot system signals | `athena/robot/...`, `athena/power/...` (see below) | `system` (8) | Read-only. Per-motor power channels are added later (see Growth). |

The `State` rule applies wherever the `State` group sits, so `Controls/<c>/State/...` and `Devices/<m>/State/...` are
role `state`. The first group in the path decides for everything else.

Units come from the path for the values listed below. Everything else is empty: `TelemetryValue` does not declare
units yet. Points and geometry telemetry are not sent over ARCP; use NT4 for those.

#### Control, motor, power and stage paths

These come from the mechanism schema, so NT4 publishes the same names under `/Athena/Mechanisms`. `<mech>` is the
mechanism path, `<c>` a control, `<m>` a motor declared on the mechanism. A control's motors sit at
`Controls/<c>/Devices/<index>` instead of `Devices/<m>`.

| Path | Role | Unit | Meaning |
| --- | --- | --- | --- |
| `<mech>/Controls/<c>/State/Target` | state | control units | The requested target, as the caller asked for it. |
| `<mech>/Controls/<c>/State/Reference` | state | control units | Motion-profile reference position (the goal without a profile). |
| `<mech>/Controls/<c>/State/ReferenceVelocity` | state | control units | Motion-profile reference velocity. |
| `<mech>/Controls/<c>/State/Error` | state | control units | Goal minus measurement, the same error as the trace. |
| `<mech>/Controls/<c>/State/Output` | state | `V` | Applied output in volts. NaN when the controller runs the loop on the device (no software volts) or the control is not driven. |
| `<mech>/Devices/<m>/State/TemperatureCelsius` | state | `C` | Winding temperature estimate (the thermal model, fused with the vendor reading). |
| `<mech>/Devices/<m>/State/Derate` | state | none | Thermal derate fraction, 0 to 1; 1 means no derate. |
| `<mech>/Devices/<m>/State/PowerCapAmps` | state | `A` | Effective current cap of the latest power cycle. 0 when the motor is not capped. |
| `<mech>/State/Stage` | state | none | Current stage of the mechanism's first `Staging`, by enum constant name. Empty when it declares none. |

The control values are the ones the tuning plot needs: `Target` against `Reference` against the measured
`FeedbackPosition` or `FeedbackVelocity`. They are NaN until the control is first driven, and go back to NaN when a cycle
does not drive it.

#### Power channels and growth

| Path | Role | Unit | Meaning |
| --- | --- | --- | --- |
| `athena/power/<motor>/current` | system | `A` | The motor's estimated current draw from the power pass (its demand). |
| `athena/power/<motor>/capAmps` | system | `A` | The planner's cap on the motor. 0 when it is not capped. |

`<motor>` is the motor's default name, as the power pass reports it. These paths do not exist until the power pass has
cycled and reported the motor. The bridge checks once per second; when a motor is new it adds the two entries and calls
`publishDictionary` again. The server pushes the new schema to every session. Existing signal ids never change, and
entries are only added, so a client keeps its subscriptions and can look up the new paths when the schema arrives.

#### Vision channels

| Path | Role | Unit | Meaning |
| --- | --- | --- | --- |
| `athena/vision/<camera>/State` | system | none | `OK`, `REJECTED`, `RECOVERING`, `UNSYNCED`, `STALE` or `NO_TARGETS`. |
| `athena/vision/<camera>/Reason` | system | none | Short text naming the rejection, for example `ambiguity 0.31 > 0.20`. Empty when OK. |
| `athena/vision/<camera>/AcceptedPerSecond` | system | `Hz` | Frames accepted in the last second. |
| `athena/vision/<camera>/RejectedPerSecond` | system | `Hz` | Frames rejected in the last second, by any stage. |
| `athena/vision/<camera>/FrameAgeMs` | system | `ms` | Robot time minus the capture time of the newest frame. |
| `athena/vision/<camera>/TagCount` | system | none | Tags in the newest frame (i64). |
| `athena/vision/<camera>/SecondsSinceAccepted` | system | `s` | Seconds since the last accepted frame. |

`<camera>` is the camera's device name. The paths appear the first time a Kalman localization judges a frame from that
camera, and they are added by the same growth step as the power channels. The values come from every Kalman
localization in the process, refreshed each robot cycle. When two localizations report one camera name, one of the two
is shown. NetworkTables publishes the same paths under `athena/vision/...` for localizations that call
`publishNetworkTables()`. A `TimeSync` flag is planned and is not published yet.

### Realtime values

Each robot cycle the bridge samples the values and publishes only those that changed (`ArcpCycle`, one JNI call).
Clients subscribe with mechanism-tree globs (`robot/turret/**`) and rates (`ON_CHANGE` or Hz). The server sends one
frame per subscriber per cycle and a keyframe at least once per second.

### Writes

A write is applied with `TelemetryValue.tryWrite`, the same path the NT4 dashboard uses. The robot therefore
enforces every rule:

- read-only values reject with `rejected: read-only`;
- values in a locked lock group reject with `rejected: locked (<group simple name>)`. The lock is defined with
  `tuningLock(Group.class, condition)` on the robot. Undefined groups behave as `Locks.InMatch`, which is locked
  while FMS is attached;
- a value of the wrong type rejects with `rejected: type mismatch (...)`. Integers are accepted for numbers.

The reply is `{accepted, reason, actualValue}`. `actualValue` is the value the robot holds after deciding, so a
client can restore the displayed value. The server never blocks a write itself.

### Actions

`ACTION_RUN` calls `TelemetryAction.request()`, and `ACTION_CANCEL` calls `cancel()`. Unknown signals reject with
`rejected: unknown action`.

### Markers

Replay markers (boot, match identified, auto and teleop start and end, disable, faults, stage changes, checks,
overrides, persisted writes, records, brownout) are sent live to every control session with the realtime marker
record. Marker kinds equal `ArcpMarker` and `ReplayMarkers`. The `signalId` is 0: replay signal ids are not ARCP
signal ids (see below).

### Replay stream

Replay streaming needs both `ReplayPolicy.stream(true)` and the ARCP bridge. The records are the replay encoder's
bytes (dictionary, frame and marker records in the shared ARCP v2 record format), forwarded unchanged.

- A client opts in with `REPLAY_SUBSCRIBE`. Only opted-in sessions receive replay records, so ordinary dashboards do
  not pay for them.
- Frames carry sequence numbers. When a client sees a gap, it sends `BACKFILL(from, to)`. The reply is a sequence of
  `varint length, group` pairs. Each group is a self-contained run of records that starts with a dictionary and a
  keyframe, so it decodes on its own. The robot answers from its replay buffer. A reply is capped at 1 MiB; ask
  again for the rest. A range that is no longer buffered returns no groups.
- The dictionary is repeated at the start of every 64 KiB segment, so a client that joins late can resynchronise.
- Replay subscribers also receive the realtime markers. Deduplicate markers by timestamp and kind if you consume both.

**Why the replay and telemetry dictionaries are separate.** The replay encoder assigns integer ids to the signals it
records (hardware, inputs, commands, lifecycle). The telemetry dictionary is the mechanism schema. The paths differ and
the id spaces are independent. Sharing one id space would need a single signal registry for both, which is not
done yet. Clients join the two by time and by marker.

### Files

The file service exposes these categories. Names are `category/file`:

| Category | Directory | Contents |
| --- | --- | --- |
| `replays` | replay directory | `.athrp` replay files. Retention is enforced on the robot. |
| `store` | persistent store | `athena-store.txt`, the persisted values. Deleting it resets them on the next boot. |
| `models` | `ArcpPolicy.modelDirectory` | Characterization and model files. Empty until models are written there. |
| `tuning` | `<store>-exports/tuning` | Generated: `diff.txt` and `pull.txt` (below). |
| `status` | `<store>-exports/status` | Generated: `status.txt`, `key=value` lines (ARCP sessions, replay buffer, retention, free disk). |

`FILE_LIST`, `FILE_DOWNLOAD` and `FILE_DELETE` work on these. Names must be plain: separators, `..` and
unknown categories are rejected. Generated files are rewritten about once a second when their contents change.

## Tuning layers

Three layers, highest first:

1. **Robot store**: values changed on the robot or from a dashboard. Written to `athena-store.txt`.
2. **Deployed file**: `src/main/deploy/athena/tuning`, which WPILib deploys to `/home/lvuser/deploy/athena/tuning`.
   Override it with `ArcpPolicy.deployedTuningFile(...)`. It is installed as a read-only fallback of the store, so a
   deployed value is never copied into the robot store.
3. **Code default**: the value the declaration sets.

The deployed file uses the store file format, one entry per line, tab-separated:

```
# athena persistent store v1
turret/Values/kP	D	2.5
turret/Values/enabled	B	true
turret/Values/label	S	hot
```

Type tags: `D` double, `L` long, `B` boolean, `S` string (escapes `\t`, `\n`, `\r`, `\\`), `A` comma-separated doubles.
Malformed lines are skipped.

`tuning/diff.txt` has one row per persisted value, tab-separated: `key robot deployed codeDefault source`. `source` is
`deployed` (the robot matches the deployed file), `code` (the robot matches the code default), `robot-changed`
(the robot differs from both), or `not-on-robot`. A `-` means the layer has no value.

`tuning/pull.txt` holds the robot's persisted values in the same format. To keep the current tuning, download it and
save it as `src/main/deploy/athena/tuning`, then deploy.

Only persisted, writable numbers, booleans and strings take part. Persisted number arrays are not exported yet.

## Limits

- Writes and Action requests wait at most 250 ms for the robot. Call `NativeArcpServer.serviceRequests()` every
  cycle; the bridge does it in `AthenaRobot.robotPeriodic`. At most 256 requests queue. A timeout rejects with
  `robot did not answer`.
- Struct, point and geometry signals are not sent. `TelemetryValue` has no unit yet, so dictionary units are empty
  except for the paths in the table above.
- Number-array values allocate when they change. Numbers, booleans and integers do not.
- The Rust side owns the sockets and frame encoding. Java never encodes ARCP frames.

## Connecting to a simulated robot

The simulator runs the robot code on this machine and serves ARCP on its control port. Three pieces connect the
dashboard to it: the simulator, the browser bridge (`arcp-ws-bridge`, for a browser dashboard), and the dashboard
itself (`bun run dev` in a browser, or the desktop app).

**Ports.** The defaults need no flags: the robot serves TCP 5808 and UDP 5809, and the dashboard receives on UDP
5807. WPILib's NetworkTables server keeps TCP 5810 (NT4). See [Ports](../athena-arcp/README.md#ports).

1. Publish the Athena artifacts to an isolated Maven repository (never `~/.m2` or `~/wpilib` for this):

   ```bash
   REPO=/path/to/m2-athena
   JAVA_HOME=~/wpilib/2026/jdk ./gradlew publishToMavenLocal -ParcpRoborioNative=false -Dmaven.repo.local=$REPO
   ```

2. Start the simulator headless (no GUI extension) from the example:

   ```bash
   cd example-projects/mechanism-examples
   JAVA_HOME=~/wpilib/2026/jdk sh ./gradlew simulateJava -PathenaHeadless -Dmaven.repo.local=$REPO
   ```

   Drop `-PathenaHeadless` to start with the WPILib simulator GUI. The log line `Athena ARCP` with no warning
   prints `ARCP listening on TCP 5808 / UDP 5809`. Check it with the probe below.

3. Check the robot with the probe (prints the dictionary size, server status, live values and one write):

   ```bash
   cd athena-arcp/rust
   cargo run -p arcp-client --bin arcp-probe -- 127.0.0.1 --seconds 3
   ```

4. Start the browser bridge. It listens on `ws://127.0.0.1:5811` and accepts only loopback origins (plus any
   `--allow-origin`). The dashboard connects to a robot with a `connect` command, so the bridge needs no config:

   ```bash
   cd athena-arcp/rust
   cargo run -p arcp-client --bin arcp-ws-bridge -- --listen 127.0.0.1:5811
   ```

5. Start the dashboard and pick the source:

   - Browser: `cd athena-arcp/host-dashboard && bun install && bun run dev`, then open
     `http://localhost:1420/?bridge=ws://127.0.0.1:5811`. Without the parameter, source selection probes the default
     bridge on 5811 and falls back to the mock robot when it does not answer. `?source=mock` always selects the mock.
   - Desktop app: `bun run tauri:dev` in `athena-arcp/host-dashboard`. Quick connect to **Simulator** (127.0.0.1).

   The data layer for the browser bridge is `src/lib/data/ws-bridge-source.ts`. Source choice is
   `src/lib/data/source-select.ts`, and quick-connect targets are `src/lib/data/connection-targets.ts`.

The bridge protocol is documented at the top of `athena-arcp/rust/crates/arcp-client/src/ws_bridge.rs`. JSON
commands carry an `id`; realtime records are binary messages batched about every 16 ms.

## Build

The native library is built by the normal Gradle build (`:athena-arcp:buildArcpNativeHost`, which runs cargo).
`-ParcpSkipNative=true` builds without it; ARCP is then disabled at runtime with a warning. The roboRIO library
is cross-built with `:athena-arcp:buildArcpNativeRoboRio`. See `athena-arcp/README.md`.

## Tests

- `athena-arcp`: `ArcpPolicyTest`, `ArcpMarshalTest`, and `NativeArcpServerTest` (real sockets through JNI).
- `athena-wpilib`: `ArcpRobotRuntimeTest` starts a simulated robot runtime with the bridge on ephemeral ports. It
  checks the dictionary, cycle values, lock signals, writes including FMS-locked writes (simulated driver station),
  actions, the file service, the tuning layers and exports, replay streaming, markers and backfill.
- `athena-mechanisms`: `PersistentStoreLayerTest` covers the layer precedence and the tuning file format.
- Rust: `arcp-core` codec, `arcp-server` `tests/v2_server.rs` (replay opt-in and backfill, file categories), and
  `arcp-jni` unit tests.
- Rust `arcp-client`: `tests/e2e.rs` (client against an in-process server) and `tests/ws_bridge.rs` (a WebSocket
  client drives the browser bridge, which drives an in-process server: commands, rejected writes, records,
  origin checks). The `arcp-probe` and `arcp-ws-bridge` binaries build with the same crate.
- Dashboard data layer: `bun test src` covers `ws-bridge-source.test.ts` (commands, records, reconnect with a fake
  socket) and `source-select.test.ts` (source choice, bridge probe, quick-connect targets).
