# ARCP Dashboard

Quick host-side dashboards for ARCP testing.

Run this on the client/driver-station computer, not on the roboRIO.

Fast local test source:
- `cargo run -p arcp-server --example sim_dashboard`

Binaries:
- `arcp-dashboard-cli`: terminal dashboard + control commands.
- `arcp-dashboard-web`: tiny HTTP dashboard with live polling.

Design goals:
- no async runtime
- no heavy framework dependencies
- one UDP receive loop with fixed packet buffer
- keeps only latest value per signal (no history)

CLI quality-of-life:
- resilient terminal teardown on errors/panics
- built-in help overlay (`?`) and keyboard shortcuts
- selected-signal workflow (`set <value>` / `action` can target selection)
- live process stats in header (`cpu`, `rss`)

CLI usage:
- `cargo run -p arcp-dashboard --bin arcp-dashboard-cli -- --host 127.0.0.1 --control-port 5805`

Web usage:
- `cargo run -p arcp-dashboard --bin arcp-dashboard-web -- --host 127.0.0.1 --control-port 5805 --http-bind 127.0.0.1 --http-port 5810`
