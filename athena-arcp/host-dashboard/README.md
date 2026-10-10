# Athena Dashboard

Host-side desktop dashboard for the Athena robot runtime (ARCP and NetworkTables). It is a Tauri 2 app:
a Svelte + Vite frontend in `src/` and a Rust backend in `src-tauri/`.

App identity: product name `Athena Dashboard`, identifier `ca.frc6390.athena.dashboard`, version taken from
`package.json` (`src-tauri/tauri.conf.json` uses `"version": "../package.json"`).

## Prerequisites

- [Bun](https://bun.sh) 1.2.9 (the version in `package.json` `packageManager`)
- Rust stable (`rustup`), with the default host target
- Linux only: the WebKitGTK 4.1 development packages (see below)

The Rust backend uses path dependencies on `athena-arcp/rust`, so build from a full checkout of this repository.

## Build locally

```bash
cd athena-arcp/host-dashboard
bun install --frozen-lockfile
bun run package
```

`bun run package` (`scripts/package.mjs`) builds the release bundles for the **host OS only**. It runs
`tauri build --bundles ...` with:

| Host | Bundles | Notes |
| --- | --- | --- |
| Linux | `deb`, `appimage` | Needs the Linux packages below. Tauri downloads linuxdeploy at build time. The script sets `APPIMAGE_EXTRACT_AND_RUN=1` (no FUSE needed) and `NO_STRIP=1` (linuxdeploy's bundled strip cannot read newer system libraries, e.g. `.relr.dyn` on Fedora 44). The AppImage is larger for that reason; unset `NO_STRIP` to strip it. |
| Windows | `nsis`, `msi` | Tauri downloads NSIS and WiX as needed. |
| macOS | `app`, `dmg` | Host architecture only (arm64 on Apple Silicon, x86_64 on Intel). |

No cross-OS or cross-architecture targets are built. The `tauri:build:*` scripts in `package.json` are the
explicit cross-target builds and are not used by `package`.

### Where the outputs land

Without `CARGO_TARGET_DIR`:

```
athena-arcp/host-dashboard/src-tauri/target/release/bundle/
  deb/        Athena Dashboard_<version>_amd64.deb        (package name athena-dashboard)
  appimage/   Athena Dashboard_<version>_amd64.AppImage
```

On Windows the files go to `bundle/nsis/` and `bundle/msi/`; on macOS to `bundle/dmg/` and `bundle/macos/`.

If `CARGO_TARGET_DIR` is set (useful when `/tmp` is a small tmpfs), the output is under that directory
instead, in `<CARGO_TARGET_DIR>/release/bundle/`.

### Linux system packages

Debian and Ubuntu (Ubuntu 22.04 is the CI baseline):

```bash
sudo apt-get install -y libwebkit2gtk-4.1-dev libayatana-appindicator3-dev librsvg2-dev patchelf \
  libgtk-3-dev libssl-dev build-essential curl wget file libfuse2
```

Fedora:

```bash
sudo dnf install webkit2gtk4.1-devel libappindicator-gtk3-devel librsvg2-devel patchelf \
  openssl-devel gtk3-devel file fuse-libs
```

`libayatana-appindicator3` (or `libappindicator`) is needed for tray support. The current build does not
enable the tray feature, but CI installs it so that it stays buildable when the tray is added.

### Run the development build

```bash
bun run tauri:dev
```

`tauri:dev` uses `scripts/tauri-launch.mjs`, which switches a Wayland session to the X11 backend unless
`ARCP_TAURI_WAYLAND=1` is set.

## CI release

Workflow: `.github/workflows/dashboard-release.yml`.

- **Manual run** (`workflow_dispatch`): builds the matrix and uploads the bundles as workflow artifacts named
  `dashboard-<platform>`. Nothing is published.
- **Tag push** `dashboard-v<version>`: builds the same matrix and attaches the bundles to a **draft** GitHub
  release for the tag. Check the bundles, then publish the draft by hand.

| Matrix entry | Runner | Bundles |
| --- | --- | --- |
| `linux-x86_64` | `ubuntu-22.04` | deb, AppImage |
| `windows-x86_64` | `windows-latest` | nsis, msi |
| `macos-aarch64` | `macos-14` | dmg (arm64) |
| `macos-x86_64` | `macos-13` | dmg (x86_64) |

To cut a release:

1. Set `version` in `athena-arcp/host-dashboard/package.json` (the bundles use it).
2. Commit, then tag and push: `git tag dashboard-v<version> && git push origin dashboard-v<version>`.
3. Open the draft release, check the assets, and publish it.

Only the default `GITHUB_TOKEN` is used (`contents: write`, for the draft release).

### Code signing (not configured)

Signing is intentionally absent. The workflow has commented TODO blocks for:

- **Windows**: a PFX certificate, plus `certificateThumbprint`, `digestAlgorithm` and `timestampUrl` under
  `bundle.windows` in `src-tauri/tauri.conf.json`.
- **macOS**: Developer ID certificate, `APPLE_SIGNING_IDENTITY`, and notarization credentials
  (`APPLE_ID`, `APPLE_PASSWORD`, `APPLE_TEAM_ID`).

Until these are set, Windows shows an "unknown publisher" warning and macOS builds are not notarized.
