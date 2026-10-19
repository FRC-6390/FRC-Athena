# ARCP (Athena Realtime Control Protocol)

ARCP is an optional transport for robot telemetry, subscriptions, writes and files. The protocol is
implemented **only in Rust**. Java never encodes or serves ARCP: the Java module is a thin JNI binding
to the Rust server (design spec E7). The robot does not need ARCP to run; the runtime reaches it
through `ArcpSink`.

## Layout

| Path | What it is |
| --- | --- |
| `rust/crates/arcp-core` | Wire codec: v2 records, frames, control messages, patterns (appendix E1-W). |
| `rust/crates/arcp-server` | Server. `v2::ArcpServerV2` is the robot server (UDP frames, TCP control, file service). |
| `rust/crates/arcp-jni` | JNI entry points for `ca.frc6390.athena.arcp.ArcpNative`, plus the request queue between client threads and the robot thread. |
| `java` | Gradle module `:athena-arcp` (package `ca.frc6390.athena.arcp`). Public API plus JNI declarations. |
| `../docs/wire-vectors` | Golden vectors checked by the Rust tests. |

## Ports

| Port | Protocol | Who | Purpose |
| --- | --- | --- | --- |
| 5808 | TCP | robot (listens) | ARCP control: hello, dictionary, subscriptions, writes, actions, files |
| 5809 | UDP | robot (sends from) | Realtime frames to each subscriber's UDP port |
| 5807 | UDP | dashboard (receives) | Realtime frames. Falls back to an ephemeral port when taken (`udpFallback`) |
| 5810 | TCP | robot, simulator | WPILib NT4. ARCP never uses it |

All ARCP ports are inside the FRC team-use range 5800-5810, so they are legal on the field network. The robot
side (`ArcpPolicy`, `ServerConfig`) and the client side (`ClientConfig`, `arcp-probe`, `arcp-ws-bridge`, the
host dashboard's live bridge) default to these numbers, so a simulated robot and a dashboard on one machine need
no flags. Simulation falls back to ephemeral robot ports with a DriverStation warning when 5808 or 5809 is taken.
On the roboRIO a taken port fails ARCP loudly, naming the ports.

## Java API

- `NativeArcpServer(Config, WriteHandler, ActionHandler[, Nt4Bridge])` starts the Rust server.
  `Config(controlPort, realtimePort, fileRoot, budgetBytesPerSecond, mtu, keyframeIntervalMicros)`;
  `Config.defaults()` uses ephemeral ports, no file service, no budget, 1200-byte datagrams.
  `ArcpPolicy` (robot field) defaults to TCP 5808 and UDP 5809 (`controlPort`, `realtimePort`).
  `fileRoot` is the directory the file service lists, downloads and deletes from. Null disables it.
- `ArcpSink` is what the runtime publishes through:
  - `publishDictionary(entries, schemas)` replaces the dictionary and pushes it to clients.
  - `publishCycle(timestampMicros, ArcpCycle)` takes the cycle's current values. The server sends only
    the values that changed and only to subscribers that match.
  - `marker(timestampMicros, kind, message, signalId)` goes to every subscriber (`ArcpMarker` kinds).
  - `serviceRequests()` answers writes and actions queued by clients. Call it once per robot cycle.
  - `status()` returns sessions, subscriptions, realtime bytes in the last window and throttled
    subscriptions.
  - `ArcpSink.NONE` discards everything when ARCP is off.
- `WriteHandler.write(signalId, value)` and `ActionHandler.action(signalId, run)` return a `WriteResult`
  `{accepted, reason, actualValue}`. The server never blocks a write itself (E5). The reason is the
  robot's own text, for example `rejected: locked (InMatch)`. `actualValue` is the value the robot holds,
  so the client can restore the displayed value.
- Dictionary roles follow the off-season spec: 0 unspecified, 1 state, 2 value, 3 control, 4 device,
  5 action, 6 hook, 7 lock (boolean `athena/locks/<group>`, true = locked), 8 system. Clients display
  lock state and never have writes blocked by it.
- `Nt4Bridge` is the NT4 mirror hook (E7). The default `Nt4Bridge.DISABLED` does nothing. A bridge
  receives every value passed to `publishCycle` and does its own change detection.

## JNI boundary

- The server is an opaque `long` handle. `NativeArcpServer.close()` destroys it once.
- **Per cycle, one call.** `publishCycle` writes the cycle into a reusable direct `ByteBuffer` and makes
  one JNI call. Rust reads the buffer in place and does the ARCP encoding. There are no per-value calls.
- **No callbacks into Java.** Rust never calls the JVM and never attaches threads. Client writes and
  actions are queued in Rust. The robot thread calls `serviceRequests()`, which makes one `poll` call
  (returns the requests or the buffer size needed) and one `reply` call (returns the answers). The
  robot loop decides and answers on its own thread.
- **Timeouts.** A client waits at most 250 ms for an answer. Beyond that, the request is rejected with
  `robot did not answer` and `actualValue` is the last value the robot published. At most 256 requests
  queue. Beyond that, new requests are rejected with `robot busy`.
- **Marshalling format.** The buffers are little-endian. Value kinds reuse the ARCP typeTag numbers.
  The layout is documented in `rust/crates/arcp-jni/src/marshal.rs` and `ArcpMarshal.java`. It is
  not the ARCP wire format, and the wire is never produced in Java.
- **Errors.** Invalid arguments and marshalling errors throw `IllegalStateException` from Rust. Treat them
  as integration bugs. Config values are validated in Java first.
- **Loading.** `ArcpNative` loads, in order: the `athena.arcp.nativePath` system property (absolute path
  to the library), the bundled `/native/<os>-<arch>/<library>` resource, then `System.loadLibrary("arcp_jni")`.

## Build and packaging

The module is `:athena-arcp`. The Rust library is built by Gradle with cargo and packaged into the jar as
`/native/<os>-<arch>/<library>`. The names are the same in `build.gradle` and in `ArcpNative`:

| Platform | Resource directory | Library |
| --- | --- | --- |
| Linux x86_64 / aarch64 | `linux-x86_64` / `linux-aarch64` | `libarcp_jni.so` |
| macOS x86_64 / aarch64 | `macos-x86_64` / `macos-aarch64` | `libarcp_jni.dylib` |
| Windows x86_64 | `windows-x86_64` | `arcp_jni.dll` |
| roboRIO (armv7) | `linux-arm32` | `libarcp_jni.so` |

### What each build produces

- `./gradlew build` builds the **host** library only (debug, or release with `-ParcpCargoRelease=true`).
- Publishing (`publishToMavenLocal`, `publishToLocalWpilib`, `buildAndPublish`, any task name containing
  `publish`) builds the host library in release **and** the roboRIO library, because robots need it. The
  publish fails if the roboRIO library cannot be built. Opt out with `-ParcpRoborioNative=false`.
- `./gradlew :athena-arcp:buildArcpNativeWindowsCross` is optional. It cross-compiles
  `x86_64-pc-windows-gnu` from Linux and is skipped with a message unless the Rust target and
  `x86_64-w64-mingw32-gcc` are installed. It is not part of any other task.
- Windows and macOS libraries are built on those systems. macOS is not cross-compiled; CI covers it.

### Incremental builds

`buildArcpNativeHost` and `buildArcpNativeRoboRio` declare as inputs every `.rs` file and `Cargo.toml` of
`arcp-core`, `arcp-server` and `arcp-jni`, the workspace `Cargo.toml`, `Cargo.lock`, `.cargo/config.toml`, the
profile, the target directory and the roboRIO linker. The library is the output. With no change, Gradle reports
`UP-TO-DATE` and cargo is not started. `stageArcpNative` copies the library files, so the jar is not rebuilt
when they are unchanged.

### Properties

- `-ParcpSkipNative=true`: no cargo. Only libraries given by `-ParcpNativeDir` are packaged. Native tests are
  excluded unless a host library is available. `ArcpMarshalTest` still runs.
- `-ParcpNativeDir=<absolute dir>`: a directory containing `native/<os>-<arch>/<library>`, as produced by the
  CI workflow `.github/workflows/arcp-natives.yml`. Every library found there is copied into the jar. A library
  present in the directory is not built locally; a missing one is built as usual. This is how a release jar
  contains all platforms without one machine building them all.
- `-ParcpCargoRelease=true`: release profile for the host library (default: debug, release when publishing).
- `-ParcpCargo=<path>`: cargo executable. Default: `PATH`, then `$CARGO_HOME/bin`, then `~/.cargo/bin`
  (`%USERPROFILE%\.cargo\bin` on Windows, where `cargo.exe` is used).
- `-ParcpCargoOffline=true`: pass `--offline` to cargo.
- `-ParcpRoborioNative=<bool>`: build the roboRIO library in the jar step (default: on when publishing).
- `-ParcpRoborioLinker=<path>`: roboRIO C linker. Default: `~/wpilib/2026/roborio/bin/arm-frc2025-linux-gnueabi-gcc`
  on macOS and Linux, `%PUBLIC%\wpilib\2026\roborio\bin\arm-frc2025-linux-gnueabi-gcc.exe` on Windows.
  The tuple is `arm-frc2025` because the WPILib 2026 toolchain ships it.
- `-ParcpPublish=false`: keep this module out of Maven Local (published by default; `athena-wpilib` depends on it).

### Target directory

`CARGO_TARGET_DIR` (absolute, or relative to `athena-arcp/rust`) moves the cargo output. Gradle passes the same
value to cargo and reads the libraries from it. Use it when `/tmp` is a small tmpfs, for example:

```bash
export CARGO_TARGET_DIR=$HOME/.cache/athena-arcp-target
```

The default is `athena-arcp/rust/target`, which is git-ignored.

### roboRIO

`buildArcpNativeRoboRio` cross-compiles `armv7-unknown-linux-gnueabi` in release. Its requirements:

1. The Rust target: `rustup target add armv7-unknown-linux-gnueabi`. The task checks for it and fails with that
   command when it is missing.
2. The WPILib 2026 roboRIO toolchain (see the linker default above), or `-ParcpRoborioLinker=<path>`. The task
   sets the cargo linker through `CARGO_TARGET_ARMV7_UNKNOWN_LINUX_GNUEABI_LINKER`.
3. Then `./gradlew :athena-arcp:buildArcpNativeRoboRio` (or any publish task).

`rust/docker/Dockerfile.cross` (armel/arm64 toolchains and a mingw target) is kept for a containerized
cross build. It is not wired into Gradle.

### CI

`.github/workflows/arcp-natives.yml` builds and tests the host library on `ubuntu-latest` (linux-x86_64),
`ubuntu-24.04-arm` (linux-aarch64), `windows-latest` (windows-x86_64), `macos-14` (macos-aarch64) and
`macos-13` (macos-x86_64). The roboRIO job installs the roboRIO GCC from the WPILib opensdk release named in
the workflow's `ROBORIO_TOOLCHAIN_*` variables. The assemble job runs `./gradlew build -ParcpNativeDir=...` on
the downloaded libraries. The publish job is a placeholder that runs only on a manual run with `publish` set;
no credentials are configured.

## Tests

```
cd athena-arcp/rust && cargo test --offline -p arcp-core -p arcp-server -p arcp-jni
JAVA_HOME=<jdk17> ./gradlew :athena-arcp:test
```

- `arcp-core` golden vectors, `arcp-server` `tests/v2_server.rs` (real TCP and UDP on localhost), and the
  `arcp-jni` unit tests for the marshalling format and the request queue.
- `NativeArcpServerTest` starts the Rust server through JNI and talks to it over TCP and UDP. It checks:
  frame delivery, a write rejected by the robot (reason and `actualValue` reach the client),
  an action rejection (status Rejected), an accepted write, and the timeout rejection. Its test client
  (`ArcpWireClient`) decodes replies and frames with athena-runtime's reference reader. This is test code
  only.
- `ArcpMarshalTest` checks the marshalling buffers and `Config` validation without the native library.

## Wire behaviour

- Realtime: UDP datagrams of one or more records. A cycle becomes one frame per subscriber. Frames
  split into fragments at the MTU. Sequence numbers are per subscriber. A keyframe is sent at least once
  per second, and the snapshot on subscribe counts as the baseline.
- Control: TCP framing is `u32 LE length` then `u8 kind, u32 requestId, payload`. Replies echo the
  request id. Pushes use request id 0. The first request must be `HELLO` (with the UDP port) before
  `SUBSCRIBE`, `WRITE` or action requests.
- Subscriptions: `SUBSCRIBE(pattern, rate)` with mechanism-tree globs (`*` and `?` inside one segment,
  `**` for any depth) and rates `ON_CHANGE` or `Hz`.
- Time sync: `TIME_PING` returns the server clock in microseconds.
- Budget: when `bandwidthBudgetBytesPerSecond` is set, the newest subscription's rate is halved each
  second it exceeds the budget. It is restored when traffic falls below half the budget. Throttled
  subscriptions are reported in `status()`.
- File service: `FILE_LIST`, `FILE_DOWNLOAD` and `FILE_DELETE` act on plain file names in `fileRoot` and on
  `category/file` names in the named categories (`NativeArcpServer(config, categories, ...)`, one absolute
  directory per category). The robot bridge registers `replays`, `store`, `models`, `tuning` and `status`.
- Replay stream: `REPLAY_SUBSCRIBE` (kind 0x12) opts a session in. `ArcpSink.publishRecords` sends complete
  replay record envelopes to opted-in sessions unchanged. `BACKFILL(from, to)` (kind 0x60) asks the robot
  through `ArcpSink.backfill(...)`; the reply holds `varint length, group` pairs, capped at 1 MiB.
- The robot-side usage is in `docs/arcp.md`.
