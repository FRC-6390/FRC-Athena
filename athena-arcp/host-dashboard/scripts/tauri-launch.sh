#!/usr/bin/env bash
set -euo pipefail

# Linux Wayland note:
# Some GTK/WebKit/WRY combinations can crash with protocol errors.
# Default to X11 backend for stability unless explicitly overridden.
if [[ "$(uname -s)" == "Linux" ]]; then
  session_type="${XDG_SESSION_TYPE:-}"
  force_wayland="${ARCP_TAURI_WAYLAND:-0}"

  if [[ "$session_type" == "wayland" && "$force_wayland" != "1" ]]; then
    export GDK_BACKEND=x11
    export WINIT_UNIX_BACKEND=x11
    export WEBKIT_DISABLE_DMABUF_RENDERER=1
    echo "[arcp-host-dashboard] Wayland session detected; launching Tauri via X11 compatibility path."
    echo "[arcp-host-dashboard] Set ARCP_TAURI_WAYLAND=1 to bypass this fallback."
  fi
fi

if command -v tauri >/dev/null 2>&1; then
  exec tauri "$@"
fi

if [[ -x "./node_modules/.bin/tauri" ]]; then
  exec ./node_modules/.bin/tauri "$@"
fi

exec bunx tauri "$@"
