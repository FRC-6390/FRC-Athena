# ARCP (Athena Realtime Control Protocol)

ARCP is an optional transport for robot telemetry, subscriptions, writes and files. It sits behind
the runtime's `ArcpSink` extension point (E7). The robot does not need it to run.

Components:

| Path | What it is |
| --- | --- |
| `rust/crates/arcp-core` | Wire codec (v2, appendix E1-W) and the v1 model used by the JNI bridge. |
| `rust/crates/arcp-server` | `v2` server: UDP batched frames, subscriptions, TCP control, file service. Also the v1 server. |
| `rust/crates/arcp-jni` | v1 JNI runtime used by `athena-arcp/java` (`ArcpRuntime`). Not built by default. |
| `java` | Gradle module `:athena-arcp`. Pure-Java v2 server and codec (`ca.frc6390.athena.arcp.v2`), plus the v1 JNI facade (`ca.frc6390.athena.arcp.ArcpRuntime`). |
| `../docs/wire-vectors` | Golden vectors checked by the Java and Rust tests. |

## Java: how it works

- `ArcpServer(Config, WriteHandler)` binds a TCP control port and a UDP sender. `Config.defaults()` uses
  an ephemeral port, no file service, no budget and 1200-byte datagrams.
- The robot publishes through `ArcpSink` (`ArcpServer` implements it):
  - `publishDictionary(entries, schemas)` replaces the signal dictionary and pushes it to clients.
  - `publishCycle(timestampMicros, ArcpCycle)` takes the cycle's current values. Unchanged values are
    dropped. Each subscriber gets at most one batched frame per cycle, with only the signals its
    subscriptions match that are due.
  - `marker(timestampMicros, kind, message, signalId)` goes to every subscriber.
  - `ArcpSink.NONE` discards everything when ARCP is off.
- `WriteHandler` decides writes and actions. The server never blocks a write itself. A write is
  answered with `{accepted, reason, actualValue}` (message kind `0x83`): the reason is the robot's own
  text, for example `rejected: locked (InMatch)`, and `actualValue` is the value the robot holds, so a
  client can restore the displayed value. Actions reply with the generic status reply.
- Dictionary roles follow the off-season spec: 0 unspecified, 1 state, 2 value, 3 control, 4 device,
  5 action, 6 hook, 7 lock (boolean `athena/locks/<group>`, true = locked), 8 system. Lock state is
  displayed by clients and never blocks writes.
- `Nt4Bridge` is the NT4 hook (E7). The default `Nt4Bridge.DISABLED` does nothing.
- `ArcpFileService(root)` lists, downloads and deletes plain file names in one directory (E6).
- `ArcpCodec` holds the byte-level encoders and decoders. It uses `athena-runtime`'s `WireWriter` and
  `WireReader`, so the wire format matches the reference.

Clients: connect over TCP, expect the schema push, send `HELLO` with the UDP port to receive frames,
then `SUBSCRIBE(pattern, rate)`. Patterns use mechanism-tree globs: `*` and `?` inside one segment,
`**` for any depth. Rates are `ON_CHANGE` or `Hz`.

Run the tests: `./gradlew :athena-arcp:test`. Regenerate the golden vectors with
`ATHENA_REGEN_VECTORS=1 ./gradlew :athena-arcp:test`.

## Build and packaging

- The module is in `settings.gradle` and is part of `./gradlew build`. The native JNI build is off by
  default (`-ParcpNativeMode=none`). Use `-ParcpNativeMode=host` to build the host JNI library.
- Publishing is off by default. The ARCP publications are skipped during `buildAndPublish` unless
  `-ParcpPublish=true` is given.
- The Athena vendordep does not list `athena-arcp`. WPILib vendordep JSON has no optional-artifact
  concept, so the entry is added only once publishing is enabled.

## Rust: how it works and how to test it

```
cd athena-arcp/rust
cargo test -p arcp-core -p arcp-server
```

- `arcp-core::v2` implements the record envelope, frames, markers, metadata, struct schemas, all type
  tags, `COMPRESSED_BLOCK` (raw DEFLATE through `flate2`) and replay files. Control messages are
  defined in `v2::control` and mechanism-path globs in `v2::pattern`.
- `COMPRESSED_BLOCK` decoding is implemented. Only the inflated content is compared byte for byte, because
  DEFLATE output depends on the encoder.
- `arcp-server::v2::ArcpServerV2` runs the same design as the Java server. It is covered by integration
  tests in `crates/arcp-server/tests/v2_server.rs`.

## Wire behaviour

- Realtime: UDP datagrams of one or more records. A cycle becomes one frame per subscriber. Frames
  split into fragments at the MTU. Sequence numbers are per subscriber. A keyframe is sent at least once
  per second, and the snapshot on subscribe counts as the baseline.
- Control: TCP framing is `u32 LE length` then `u8 kind, u32 requestId, payload`. Replies echo the
  request id. Pushes use request id 0.
- Time sync: `TIME_PING` returns the server clock in microseconds.
- Budget: when `bandwidthBudgetBytesPerSecond` is set, the newest subscription's rate is halved each
  second it exceeds the budget. It is restored when traffic falls below half the budget. Throttled
  subscriptions are reported in `status()`.
