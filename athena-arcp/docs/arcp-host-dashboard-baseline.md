# ARCP Host Dashboard Baseline: Elastic + SmartDashboard

## Scope

This baseline is for a host-run dashboard (driver station/laptop) backed by ARCP control + telemetry, not a robot-side UI process.

## What existing systems do well

### Elastic Dashboard (FRC)

- Flexible widget composition and layout editing (`Add Widgets`, drag/resize workflow).
- Per-widget topic binding and settings (`Edit Widget` with topic selection).
- Supports custom widgets in HTML/CSS/JS (`Custom Widgets` docs).
- Exposes global dashboard settings including update period and robot address (`Settings` docs).

### SmartDashboard (WPILib)

- Simple out-of-box display for NetworkTables keys and sendables.
- Auto-displays sendable data and allows per-widget selection/config.
- Can persist and reload dashboard layout/state.
- Mature plugin model and many legacy team workflows built around it.

## Pitfalls in practice

### Elastic Dashboard pitfalls

- Widget binding is still topic-centric and operator-config heavy at scale.
- Topic selection is connection-dependent; disconnected state limits setup/discovery flow.
- No protocol-level typed metadata contract for richer controls by default (inference based on topic/widget model).

### SmartDashboard pitfalls

- Legacy UI/UX model and plugin architecture increase maintenance friction for modern workflows.
- WPILib marks SmartDashboard as legacy support only through 2027.
- Relies on NetworkTables topic semantics instead of a strongly typed manifest-first schema.

### Shared baseline gaps

- Control semantics (`SET`, `ACTION`, validation) are often implicit at the dashboard layer instead of explicit protocol contracts.
- Large signal counts become hard to manage without first-class filtering, typed roles, and reconnect-aware control state.

## ARCP Dashboard 2.0 direction (Tauri + Svelte)

## Baseline we should keep

- Elastic-style composable panels and dense telemetry views.
- SmartDashboard-like fast control operations for tuning and actions.

## Improvements on top

- Manifest-first typed signal model (`id`, `type`, `role`, `path`) from ARCP.
- Explicit control plane commands with acknowledgements (`SET`, `ACTION`) via Rust backend.
- Auto-reconnect + re-subscribe behavior built into control link management.
- Built-in scale tools: search/filter by type/role/path, inspector workflow, and high-rate update handling.
- Host-only runtime model to keep roboRIO CPU/memory budget focused on robot code.

## Minimum product slices

1. Connection + state health: connect/disconnect, control status, reconnect status.
2. Signal explorer: searchable/filterable table with latest values and typed metadata.
3. Inspector + control: selected signal details, value write, action trigger.
4. Performance visibility: app CPU/RSS plus update counters.
5. Layout evolution: user-configurable panels and saved workspaces.

## Sources

- Elastic Dashboard docs: https://frc-elastic.gitbook.io/docs
- Elastic `Add Widgets`: https://frc-elastic.gitbook.io/docs/add-widgets
- Elastic `Custom Widgets`: https://frc-elastic.gitbook.io/docs/custom-widgets
- Elastic `Settings`: https://frc-elastic.gitbook.io/docs/settings
- WPILib SmartDashboard intro: https://docs.wpilib.org/en/stable/docs/software/dashboards/smartdashboard/smartdashboard-intro.html
- WPILib SmartDashboard index (legacy support note): https://docs.wpilib.org/en/stable/docs/software/dashboards/smartdashboard/index.html
