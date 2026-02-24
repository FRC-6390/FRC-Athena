# Athena Realtime Communication Protocol (ARCP) Draft v0.3

## 1. Scope

This is a draft spec for a NetworkTables replacement focused on roboRIO constraints:

- low CPU overhead on robot
- bounded memory usage
- robust real-time behavior under bad links
- first-class UI metadata (buttons, dropdowns, ranges, units, hints)
- compatibility with Athena's current telemetry value set
- Rust-native backend with a thin Java interface

`ARCP` is designed as a **robot-first realtime protocol**, not a browser-first protocol.

### 1.1 Repository Layout

Canonical implementation layout:

- Rust backend: `athena-arcp/rust`
- Java thin interface: `athena-arcp/java`

## 2. Requirements

### 2.1 Functional

The protocol SHALL support:

- scalar values: `boolean`, `int64`, `float64`, `string`
- array values: `boolean[]`, `int64[]`, `float64[]`, `string[]`
- metadata for UI rendering and control behavior
- commands/actions from clients to robot with acknowledgements
- tunable parameters with validation and clamping
- retained defaults and reconnect snapshots
- Java-facing API that does not own network/protocol state

### 2.2 Resource Targets (roboRIO)

Reference target is roboRIO-class ARM CPU and memory budget.

- zero heap allocation on hot publish/receive paths after initialization
- fixed-size memory pools for topics, subscriptions, and queues
- bounded per-client bandwidth and queue memory
- deterministic worst-case reconnect/snapshot cost

Suggested implementation budgets:

- protocol heap budget: <= 2 MB total
- protocol CPU budget at steady state: <= 5% single core for typical match telemetry
- no unbounded data structures keyed by arbitrary topic strings

### 2.3 Practical Review: What Makes Sense vs What Does Not (Current)

After implementation + dashboard validation, the following is clear:

What makes sense and should stay:

- split reliable control and realtime telemetry planes
- manifest-first numeric IDs on wire
- Rust-native backend with thin Java interface
- host-side dashboard process (no robot-side UI runtime)
- bounded queueing + backpressure policy

What does not make sense in current draft and needs tightening:

- over-broad semantic categories can blur transport/control semantics and UI intent
- UI defaults are under-specified (default widget size, layout affinity, ordering)
- system telemetry contract is not strict enough (CPU/RSS/drop/update stats should be required)
- container-aware placement expectations are UI-visible but not represented in metadata/hints

## 3. NT4 Baseline: What It Gets Right

NT4 should be credited for:

- pub/sub model with selective subscriptions
- typed topics and timestamped updates
- topic properties (`persistent`, `retained`, `cached`)
- metadata/control channel concepts and diagnostics via meta-topics
- browser accessibility using WebSockets + JSON + MessagePack

Reference:
- NT4.1 spec (WPILib): https://raw.githubusercontent.com/wpilibsuite/allwpilib/main/ntcore/doc/networktables4.adoc

## 4. NT4 Gaps for Realtime on roboRIO

Based on the NT4.1 protocol and field behavior:

1. Head-of-line blocking risk
- NT4 value transport is stream-based (WebSockets over TCP).
- Large transfers can delay unrelated small urgent messages.
- The NT4 spec itself calls out TCP-only latency spike concerns.

2. Control/data overhead on constrained CPU
- JSON control messages + topic-string-centric flows add parse/alloc pressure.
- Topic/property maps are flexible but expensive in hot paths.

3. Subscription storm risk
- Wide subscriptions (`""`, `"$"`, broad prefixes) can cause large backfill bursts.
- NT4.1 adds mitigations/recommendations, but behavior is still implementation-sensitive.

4. Subscriber-driven publishing is optional
- NT4 recommends using `$sub$<topic>`, but publishers may ignore it.
- This can waste bandwidth/CPU on values no client currently needs.

5. Metadata shape is weakly typed
- Arbitrary JSON properties are flexible, but no strict control schema for UI widgets.
- Teams end up inventing conventions per dashboard.

## 5. ARCP Architecture

ARCP splits concerns into two planes:

1. Control Plane (reliable)
- Transport: TCP (single connection).
- Purpose: session handshake, manifest sync, subscriptions, tunable writes, action commands, acks/errors.
- Encoding: compact binary TLV (no JSON in hot path).

2. Realtime Data Plane (unreliable latest-value)
- Transport: UDP datagrams.
- Purpose: high-rate telemetry updates.
- Behavior: newest-value-wins; stale packets dropped by sequence.

Why this split:
- avoids TCP head-of-line impact on realtime telemetry
- keeps command/tuning reliable and simple
- reduces parse and framing overhead on robot

### 5.1 Rust-First Ownership

The ARCP backend SHALL be implemented in Rust and own:

- all network sockets and packet IO
- all protocol serialization/deserialization
- session/subscription/topic state
- flow control, backpressure, and queueing
- metadata manifest storage and validation

Java SHALL be a thin integration layer only.

## 6. Data Model

### 6.1 Signal IDs (not string keys on wire)

Each published item is a **Signal** with a `uint16 signal_id`.

- Human-readable paths exist only in manifest metadata.
- Runtime packets carry `signal_id`, not full topic strings.
- Signal IDs are stable for a robot build/deploy.

### 6.2 Signal Types

Core value types:

- `BOOL`
- `I64`
- `F64`
- `STR`
- `BOOL_ARR`
- `I64_ARR`
- `F64_ARR`
- `STR_ARR`

This directly covers Athena's current telemetry set.

### 6.3 Signal Contract (Kind + Access + Policy + Durability)

Canonical descriptors SHALL split concerns:

- `kind`: `TELEMETRY` | `COMMAND`
- `access`: `OBSERVE` | `WRITE` | `INVOKE`
- `policy`: `HIGH_RATE` | `ON_CHANGE` | `SAMPLED` | `SNAPSHOT_ONLY`
- `durability`: `VOLATILE` | `RETAINED` | `PERSISTENT`

Interpretation rules:

- `kind` describes meaning for operators and tooling.
- `access` describes allowed client behavior (`WRITE` for value updates, `INVOKE` for command execution).
- `policy` describes default publish/subscription behavior and scheduler priority.
- `durability` defines value retention and persistence semantics.
- `kind=TELEMETRY` covers metrics, state, and tunables (distinguished by `access/policy/durability`).
- `kind=COMMAND` covers invoke-style control points.

UI behavior SHALL be derived from `kind/access/policy/durability` and metadata fields.

## 7. Metadata Schema (First-Class UI Support)

Every signal descriptor MAY include:

- `path` (e.g. `/Athena/Drive/Heading`)
- `kind` (`TELEMETRY`, `COMMAND`)
- `access` (`OBSERVE`, `WRITE`, `INVOKE`)
- `policy` (`HIGH_RATE`, `ON_CHANGE`, `SAMPLED`, `SNAPSHOT_ONLY`)
- `durability` (`VOLATILE`, `RETAINED`, `PERSISTENT`)
- `display_name`
- `unit` (e.g. `deg`, `m/s`, `%`)
- `min`, `max`, `step` (numeric controls)
- `enum_options` (for dropdowns)
- `widget_hint` (`slider`, `dropdown`, `button`, `graph`, `text`, etc.)
- `precision`
- `readonly`
- `priority` (`SAFETY`, `CONTROL`, `TELEMETRY`, `BULK`)
- `default_value`
- `description`
- `ui_group` (logical grouping path for explorer and inspector)
- `ui_order` (stable ordering key within group)
- `ui_default_size` (default panel size, e.g. `2x1`)
- `ui_layout_affinity` (`free`, `list`, `grid`, `section`)
- `ui_search_tags` (token list for discovery/filtering)
- `action_confirm` (requires confirmation before execute)
- `action_cooldown_ms` (minimum interval between action triggers)
- `display_format` (numeric/text formatting hint)
- `metadata_version` (monotonic `u32` per signal descriptor)
- `metadata_hash` (stable `u64` hash of canonical metadata encoding)
- `persistence_scope` (`none`, `robot_flash`, `host_workspace`)
- `persistence_key` (storage key/path when durability is `PERSISTENT`)

Metadata is delivered via manifest sync, not per-value messages.

Metadata schema SHALL be strongly typed and versioned; arbitrary map-shaped properties are NOT part of the hot-path contract.

## 8. Session and Manifest

### 8.1 Handshake

On control connect:

1. client sends `HELLO` (protocol version, client name, capabilities)
2. robot replies `WELCOME` (session id, limits, heartbeat interval)
3. client sends `MANIFEST_QUERY` with known manifest hash
4. robot sends either:
- `MANIFEST_MATCH` (no resend), or
- `MANIFEST_FULL` / `MANIFEST_DELTA`

### 8.2 Manifest Constraints

- fixed max signals (example default: 1024)
- max path length and string lengths declared
- bounded enum list sizes
- manifest ID list is immutable for a session
- metadata fields MAY change at runtime via metadata patches (no ID/type churn)

No dynamic topic creation on fast path unless explicitly enabled.

### 8.3 Metadata Patch Sync (Required)

Metadata changes SHALL be incremental and hash-validated:

1. Robot updates descriptor metadata for `signal_id`.
2. Robot computes canonical `metadata_hash` and increments `metadata_version`.
3. Robot emits `META_PATCH` control message:
   - `signal_id`
   - `base_metadata_version`
   - patch operations (`set`, `remove`, bounded key/value payload)
   - resulting `metadata_version`
   - resulting `metadata_hash`
4. Client applies patch and recomputes hash.
5. If hash mismatch, client sends `META_RESYNC signal_id`.
6. Robot responds with `META_FULL signal_id ...` (full metadata descriptor).

This avoids full manifest re-send while preventing long-lived client/server metadata desync.

## 9. Subscriptions and Flow Control

### 9.1 Subscription Message

Client subscribes by `signal_id` with options:

- `mode`: `LATEST` | `SAMPLED` | `SNAPSHOT_ONLY`
- `min_period_ms`
- `deadband` (numeric only)
- `max_rate_hz`

### 9.2 Enforced Subscriber-Driven Publishing

Robot SHALL only enqueue telemetry for signals that have active subscribers, except:

- local logging sinks
- explicitly flagged always-on safety/status signals

### 9.3 Backpressure

Per-client fixed queue with priority scheduler:

- Priority 0: safety/state
- Priority 1: tunable echoes + command acks
- Priority 2: normal telemetry
- Priority 3: bulk/diagnostic

On overflow, drop oldest low-priority realtime packets first; never block robot control loop.

### 9.4 Subscription Profiles (Optional Convenience)

Implementations MAY expose profile aliases:

- `FAST` -> low period, no deadband
- `NORMAL` -> moderate period + small deadband
- `SLOW` -> low frequency
- `EVENT` -> on-change only

Profiles are compile-time constants that map to explicit subscription fields; wire protocol remains explicit.

## 10. Wire Format (Draft)

### 10.1 Realtime UDP Packet

```
struct RealtimeHeader {
  u8  version;
  u8  flags;          // bit0=compressed, bit1=snapshot, etc.
  u16 session_id;
  u16 stream_seq;     // per-client monotonic
  u32 server_time_us_low32;
  u8  message_count;
}
```

Each packet then carries `message_count` value records:

```
struct ValueRecord {
  u16 signal_id;
  u8  type;
  u8  value_flags;    // e.g. null/default marker
  varint value_len;
  bytes value;
}
```

Design notes:
- packet size capped to MTU-safe payload (e.g. <= 1200 bytes)
- no cross-packet message fragmentation
- stale `stream_seq` packets dropped

### 10.2 Control TLV Frame

Control messages use:

```
u8  msg_type
u8  flags
u16 correlation_id
u32 payload_len
bytes payload
```

`correlation_id` pairs request/reply (`WRITE` -> `WRITE_ACK`, `INVOKE` -> `INVOKE_ACK`, `META_PATCH` -> `META_ACK`).

## 11. Reliability Semantics

- telemetry metrics: best-effort latest-value (UDP)
- tunable writes/actions: reliable (control plane with explicit ack/nack)
- snapshots: chunked and rate-limited on control plane to prevent reconnect storms

## 12. Liveness and Time

### 12.1 Liveness

- control heartbeat interval negotiated (example: 250 ms)
- realtime heartbeat datagram every 100 ms when idle
- timeout policy: disconnect after configurable missed heartbeats

### 12.2 Time Sync

- dedicated lightweight ping/pong control message
- offset uses min-RTT window estimate
- telemetry timestamps always in server time domain

### 12.3 Required System Introspection Signals

Every ARCP server SHALL publish core diagnostic signals under a reserved namespace:

- `/ARCP/System/server_cpu_percent`
- `/ARCP/System/server_rss_bytes`
- `/ARCP/System/realtime_tx_hz`
- `/ARCP/System/realtime_drop_count`
- `/ARCP/System/control_queue_depth`
- `/ARCP/System/client_count`

Host dashboards SHOULD publish local telemetry similarly:

- `/ARCP/Host/ui_cpu_percent`
- `/ARCP/Host/ui_rss_bytes`
- `/ARCP/Host/render_fps` (if available)

## 13. Reconnect Behavior

1. reconnect control plane
2. resume session if token valid; else full handshake
3. manifest hash check
4. reapply subscriptions
5. request bounded snapshots only for subscribed signals
6. resume realtime UDP stream

Snapshot replay SHALL be chunked and byte-budgeted per cycle to avoid burst lockups.

## 14. Rust Backend Layer

### 14.1 Crate Layout

- `arcp-core`: protocol types, wire codecs, manifest schema, validation
- `arcp-server`: control/UDP services, subscriptions, scheduling, snapshots
- `arcp-jni`: C ABI + JNI bindings for Java integration
- `arcp-client-rs` (optional): native diagnostic/dashboard client

### 14.2 Runtime Model

- one control-plane thread (TCP)
- one realtime TX thread (UDP)
- one realtime RX thread (UDP commands/acks if used)
- one bounded worker pool for encode/decode/snapshot chunking

No network parsing may occur on Java threads.

### 14.3 Memory Model

- preallocated pools for signals, subscriptions, client sessions, and packet buffers
- fixed-size ring buffers between network tasks and JNI boundary
- no per-update heap allocation in steady-state publish path
- bounded string storage for path/metadata data

### 14.4 Safety / Security

For FRC LAN defaults:

- optional write authorization level per signal (`readonly`, `operator`, `dev`)
- optional shared-key HMAC on control commands
- robot-side allowlist for clients allowed to send `ACTION` and `TUNABLE` writes

Safety rule: command parsing/execution must be isolated from robot control loop timing.

## 15. Java Thin Interface Layer

### 15.1 Responsibilities

Java layer is responsible for:

- registering signal descriptors during startup
- pushing metric values to native via signal IDs
- polling tunable/action updates from native
- exposing ergonomic Athena-facing APIs/annotations

### 15.2 Non-Responsibilities

Java layer MUST NOT:

- parse or emit protocol frames
- manage client sessions/subscriptions
- store duplicate topic metadata state
- run network retry or heartbeat logic

### 15.3 JNI Boundary Rules

- use numeric `signal_id` APIs; no string-key calls in steady-state loop
- provide batched publish entry points (`publish_batch_*`)
- use direct buffers for bulk reads/writes to avoid copying
- callbacks from native to Java should be pull-based (poll queue), not arbitrary native thread upcalls

## 16. Comparison: NT4 vs ARCP

| Area | NT4 | ARCP Draft | Why ARCP helps roboRIO |
|---|---|---|---|
| Transport | WebSockets/TCP | split TCP control + UDP realtime | avoids TCP HOL for high-rate telemetry |
| Control encoding | JSON objects | binary TLV | lower parse/allocation overhead |
| Value addressing | string topic model + IDs | manifest-first `signal_id` | fewer string ops on hot path |
| Subscriber-driven behavior | recommended | enforced by server | reduced wasted publish work |
| Metadata | arbitrary JSON properties | typed descriptor schema | consistent UI rendering (buttons/dropdowns/sliders) |
| Reconnect backfill | cached topic replay | bounded chunked snapshot policy | avoids reconnect burst storms |
| Flow control | option-based | strict per-client budget + priority queues | prevents telemetry from starving control |
| Liveness | improved in NT4.1 via ping/pong guidance | required heartbeat both planes | faster fault detection and isolation |
| Runtime ownership | mostly Java/WPILib stack | Rust-native backend + thin Java shim | minimizes JVM overhead and GC pressure |

## 16.1 Comparison: Elastic vs NT4 vs ARCP

| Area | Elastic Dashboard | NT4 | ARCP Draft v0.3 |
|---|---|---|---|
| Primary focus | operator UX/layout tooling | transport + topic sync | realtime robot transport + typed control contract |
| Data contract | widget/topic conventions | typed topics + flexible properties | versioned manifest with strict typed metadata |
| Realtime behavior | depends on backing transport | TCP/WebSocket stream (HOL risk) | UDP latest-value for telemetry + reliable control plane |
| Control semantics | mostly dashboard/workflow-driven | topic writes/conventions | explicit `WRITE` / `INVOKE` + ack/nack |
| Scale ergonomics (100s signals) | good UX baseline but config-heavy | possible with discipline, topic-centric | ID-based wire + required grouping/order/size hints |
| Resource pressure on roboRIO | N/A (host app) | can rise with JSON/string and stream stalls | bounded pools, ID-first packets, subscriber-enforced publishing |
| Layout/container semantics | strong UX model | not a protocol concern | UI hints in manifest + deterministic placement policy |

## 17. Why This Is Better Than NT4 on roboRIO

1. Lower CPU overhead
- no JSON parsing in steady-state realtime path
- no per-update topic-string handling
- fixed binary decode path with known signal schemas
- protocol work executes in native Rust instead of Java runtime hot paths

2. Lower and bounded memory
- preallocated pools for subscriptions, descriptors, and queues
- no unbounded meta-topic/property map growth
- bounded snapshot/replay buffers
- thin Java interface reduces heap churn and GC interference

3. Better realtime behavior under stress
- UDP latest-value path prevents TCP retransmit stalls from freezing telemetry freshness
- priority/drop policy preserves critical traffic
- reconnect/snapshot limits prevent disconnect loops

4. Better UI/control semantics
- widget-ready metadata schema is explicit
- actions/tunables are acknowledged commands, not inferred boolean flips
- consistent validation (`min/max/step`, enum checks) at robot boundary
- UI defaults (`ui_group`, `ui_order`, `ui_default_size`, `ui_layout_affinity`) reduce operator setup burden
- metadata changes stream as patches with hash validation (lower bandwidth + safer sync)

## 18. Rust Layer Implementation Plan

1. `arcp-core` crate
- define wire types, manifest schema, encode/decode
- include fuzz/property tests for frame parsing

2. `arcp-server` crate
- implement handshake/session/subscription state machine
- implement realtime UDP scheduler with bounded queues

3. `arcp-jni` crate + Java facade
- implement stable JNI API surface
- add Java wrapper (`ArcpRuntime`) with explicit lifecycle (`start`, `publish`, `poll`, `stop`)

4. Integration validation
- sim harness (x86_64 Linux) plus roboRIO target build
- load tests for reconnect storms, queue overflow, packet loss

## 19. Open Decisions

- compression policy (none vs per-packet LZ4 for large snapshots)
- dynamic signal creation allowance in-season vs disabled for determinism
- optional QUIC control plane instead of TCP (complexity tradeoff)
- persistence location/format defaults for `durability=PERSISTENT` (`robot_flash` vs `host_workspace`)
- whether UI metadata should be in protocol manifest only or mergeable with host-side workspace overlays

---

This draft is intentionally strict on bounded resources and realtime behavior. The core design choice is separating reliable control from realtime telemetry while making metadata strongly typed and manifest-driven.
