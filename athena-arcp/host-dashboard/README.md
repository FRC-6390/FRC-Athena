# ARCP Host Dashboard (Tauri + Svelte)

Host-side desktop dashboard for ARCP.

## Run

1. `cd athena-arcp/host-dashboard`
2. `bun install`
3. `bun run tauri:dev`

Windows note:
- `tauri` launch scripts are cross-platform; no WSL/Bash is required.

Linux Wayland note:
- If your compositor/driver stack throws GTK/WebKit protocol errors, the launcher auto-falls back to X11 compatibility.
- To force native Wayland anyway: `ARCP_TAURI_WAYLAND=1 bun run tauri:dev`

For realistic runtime numbers, run your ARCP server and dashboard in release where applicable.

## Production Perf Check (No Vite Dev Server)

Use this to confirm whether lag is mainly from dev mode:

1. `cd athena-arcp/host-dashboard`
2. `bun install`
3. `bun run tauri:run:bin`

This builds a release binary with embedded frontend assets (`--no-bundle`) and runs it directly.

If you only want to build the binary and run it yourself:

- Build: `bun run tauri:build:bin`
- Linux run path: `./src-tauri/target/release/arcp-host-dashboard`

## Build Targets

Important constraints:
- Linux bundles are built on Linux hosts.
- Windows bundles (`msi`, `nsis`) are built on Windows hosts.
- macOS bundles (`app`, `dmg`) are built on macOS hosts.
- Cross-OS bundling is not a practical default path for Tauri packaging.

## Dockerized Builds (Linux)

If you want reproducible Linux builds without polluting your host toolchain, use the Docker builder:

- x64 deb + rpm: `bun run docker:build:linux:x64`
- x64 appimage: `bun run docker:build:linux:x64:appimage`
- x64 binary only: `bun run docker:build:linux:x64:bin`
- arm64 deb + rpm: `bun run docker:build:linux:arm64`
- arm64 appimage: `bun run docker:build:linux:arm64:appimage`
- arm64 binary only: `bun run docker:build:linux:arm64:bin`
- all Linux bundle targets: `bun run docker:build:linux:all`

Manual direct use:
- `bash ./scripts/docker-build.sh linux-x64`
- `bash ./scripts/docker-build.sh linux-x64-appimage`
- `bash ./scripts/docker-build.sh linux-arm64`

Notes:
- Docker is used only for Linux targets. Windows/macOS bundles still require native hosts.
- arm64 Docker builds on x64 hosts require QEMU/binfmt support (`docker buildx` handles this on most setups).
- Build artifacts still land in your local `src-tauri/target/...` tree because the repo is bind-mounted into the container.

### Linux (x86_64)
- `bun run tauri:build:linux:x64`
- Output bundles in `src-tauri/target/x86_64-unknown-linux-gnu/release/bundle/`
- AppImage target (separate): `bun run tauri:build:linux:x64:appimage`

### Linux (aarch64 / ARM64)
- `rustup target add aarch64-unknown-linux-gnu`
- `bun run tauri:build:linux:arm64`
- Output bundles in `src-tauri/target/aarch64-unknown-linux-gnu/release/bundle/`
- AppImage target (separate): `bun run tauri:build:linux:arm64:appimage`

AppImage note:
- On newer distros/toolchains, `linuxdeploy` can fail while stripping libs with `.relr.dyn` sections.
- If that happens, keep using `deb`/`rpm` on that host, or build AppImage in a container/VM with a compatible linuxdeploy/binutils combo.

### Windows (x86_64)
- Run on a Windows build host:
- `bun run tauri:build:windows:x64`

### Windows (ARM64)
- Run on a Windows build host:
- `bun run tauri:build:windows:arm64`

### macOS (Intel x86_64)
- Run on a macOS build host:
- `bun run tauri:build:macos:x64`

### macOS (Apple Silicon ARM64)
- Run on a macOS build host:
- `bun run tauri:build:macos:arm64`

## What it includes

- ARCP control + telemetry connection from Tauri backend Rust
- automatic control reconnection with re-subscribe
- live signal table with search/type/category filters
- inspector panel with `SET` and `ACTION` controls
- server CPU and RSS metrics in dashboard snapshot
- local UI process CPU and RSS metrics in dashboard snapshot
