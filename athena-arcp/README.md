# ARCP (Athena Realtime Control Protocol)

ARCP is an optional transport for robot telemetry, subscriptions, writes and files. The protocol is
implemented **only in Rust**. Java never encodes or serves ARCP: the Java module is a thin JNI binding
to the Rust server (design spec E7). The robot does not need ARCP to run; the runtime reaches it
through `ArcpSink`.

## Layout

| Path | What it is |
| --- | --- |
| `rust/crates/arcp-core` | Wire codec: v2 records, frames, control messages, patterns (appendix E1-W). Also the v1 model used by `arcp-dashboard`. |
| `rust/crates/arcp-server` | Server. `v2::ArcpServerV2` is the robot server (UDP frames, TCP control, file service). The v1 server stays for `arcp-dashboard`. |
| `rust/crates/arcp-jni` | JNI entry points for `ca.frc6390.athena.arcp.ArcpNative`, plus the request queue between client threads and the robot thread. |
| `rust/crates/arcp-dashboard` | Dashboard CLI and web tools. Used by `host-dashboard`; not built by the athena-arcp Gradle module. |
| `java` | Gradle module `:athena-arcp` (package `ca.frc6390.athena.arcp`). Public API plus JNI declarations. |
| `../docs/wire-vectors` | Golden vectors checked by the Rust tests. |

## Java API

- `NativeArcpServer(Config, WriteHandler, ActionHandler[, Nt4Bridge])` starts the Rust server.
  `Config(controlPort, fileRoot, budgetBytesPerSecond, mtu, keyframeIntervalMicros)`;
  `Config.defaults()` uses an ephemeral port, no file service, no budget, 1200-byte datagrams.
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

The module is `:athena-arcp` and is part of `./gradlew build`. The native library is built with cargo
during the normal build:

- `buildArcpNativeHost` runs `cargo build -p arcp-jni` in `rust/` (debug profile; `-ParcpCargoRelease=true`
  for release). `stageArcpNative` copies the result into the jar as `/native/<os>-<arch>/`.
- If cargo is missing, the build fails with a message that says how to install Rust or opt out.
- `-ParcpSkipNative=true` skips cargo and the native library entirely (for CI without Rust). The native
  tests are excluded and `ArcpMarshalTest` still runs.
- Other properties: `-ParcpCargo=<path>`, `-ParcpCargoOffline=true`, `-ParcpRoborioNative=true` (cross-build
  the roboRIO library before staging), `-ParcpRoborioLinker=<path>`.
- Publishing: ARCP is not published to Maven Local unless `-ParcpPublish=true`. The Athena vendordep does
  not list it.

### roboRIO

`buildArcpNativeRoboRio` cross-compiles `armv7-unknown-linux-gnueabi` in release. Its requirements:

1. The Rust target: `rustup target add armv7-unknown-linux-gnueabi`. The task checks for it and fails
   with that command when it is missing.
2. The WPILib 2026 roboRIO toolchain: `~/wpilib/2026/roborio/bin/arm-frc2025-linux-gnueabi-gcc`
   (`%PUBLIC%\wpilib\2026\roborio\bin` on Windows), or `-ParcpRoborioLinker=<path>`. The task sets the
   cargo linker through `CARGO_TARGET_ARMV7_UNKNOWN_LINUX_GNUEABI_LINKER`.
3. Then `./gradlew :athena-arcp:buildArcpNativeRoboRio :athena-arcp:build` (or add `-ParcpRoborioNative=true`).
   The library is packaged as `/native/linux-arm32/libarcp_jni.so`.

`rust/docker/Dockerfile.cross` (armel/arm64 toolchains and a mingw target) is kept for a containerized
cross build. It is not wired into Gradle.

Status on the 2027 development machine: the WPILib linker is present, the `armv7-unknown-linux-gnueabi`
target is **not** installed (only `armv7-unknown-linux-gnueabihf` is), so the roboRIO library has not
been built or tested on the robot yet.

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

Note: `cargo test --offline --workspace` also builds `arcp-dashboard`, whose dependencies are not in the
offline cache on this machine. Use the `-p` list above.

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
- File service: `FILE_LIST`, `FILE_DOWNLOAD` and `FILE_DELETE` act on plain file names in `fileRoot`.
