# ARCP Rust Workspace

Rust-first runtime for Athena Realtime Communication Protocol (ARCP).

Crates:
- `arcp-core`: protocol model/types and validation.
- `arcp-server`: runtime state and server lifecycle.
- `arcp-jni`: C ABI and JNI-facing bindings for Java integration.
- `arcp-dashboard`: host-side CLI/web dashboards for simulation/testing.

Workspace default build set:
- default runtime build/test (`cargo build` / `cargo test`) includes only `arcp-core`, `arcp-server`, `arcp-jni`
- dashboard tooling is intentionally host-run and excluded from default robot/runtime build targets

Quick simulation checks:
- `cargo test -p arcp-server loopback_publish_reaches_udp_subscriber -- --nocapture`
- `cargo run -p arcp-server --example sim_loopback`
- `cargo run -p arcp-server --example sim_dashboard` (live feed for CLI/web dashboards)
- `cargo run -p arcp-server --example sim_hardware` (hardware-first signal tree + dedicated motor/encoder/imu widgets)
- `cargo run -p arcp-server --release --example sim_stress -- --signals 1200 --publish-hz 100 --updates-per-tick 600`
  (high-load stress simulator with per-second CPU/RSS/update throughput output)

Implemented control protocol (TCP):
- `PING` -> `PONG`
- `SUB <udp_port>` -> `OK SUB`
- `UNSUB <udp_port>` -> `OK UNSUB`
- `MANIFEST` -> `MANIFEST_BEGIN <count>` / `ITEM ...` / `MANIFEST_END`
- `SET <signal_id> <value>` -> `OK SET` (for `access=write` signals; emits event)
- `ACTION <signal_id>` -> `OK ACTION` (for `kind=command` / `access=invoke`; emits event)
- `STATS` -> `STATS cpu=<percent|na> rss=<bytes|na>`

Implemented telemetry transport (UDP):
- updates are binary frames encoded by `arcp-core::encode_update`
- all current Athena value categories are supported:
  `bool`, `i64`, `f64`, `string`, `bool[]`, `i64[]`, `f64[]`, `string[]`

Implemented JNI surface:
- runtime lifecycle (`create`, `start`, `stop`, `destroy`)
- signal registration (`registerSignal`)
- publish methods for all supported value types
- event draining (`pollEvents`) into a direct `ByteBuffer`

Dashboard quick starts:
- CLI dashboard:
  `cargo run -p arcp-dashboard --bin arcp-dashboard-cli --release -- --host 127.0.0.1 --control-port 5805`
- Web dashboard:
  `cargo run -p arcp-dashboard --bin arcp-dashboard-web --release -- --host 127.0.0.1 --control-port 5805 --http-bind 127.0.0.1 --http-port 5810`

Performance note:
- Run dashboards on the client host machine (not on the roboRIO).
- Use `--release` for realistic CPU/memory numbers. Debug builds intentionally trade runtime performance for diagnostics.
