#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT_DIR"

bash ./scripts/tauri-launch.sh build --no-bundle

case "$(uname -s)" in
  Linux)
    BIN_PATH="./src-tauri/target/release/arcp-host-dashboard"
    ;;
  Darwin)
    BIN_PATH="./src-tauri/target/release/arcp-host-dashboard"
    ;;
  CYGWIN*|MINGW*|MSYS*)
    BIN_PATH="./src-tauri/target/release/arcp-host-dashboard.exe"
    ;;
  *)
    echo "Unsupported OS for auto-run. Build finished; run binary from src-tauri/target/release."
    exit 0
    ;;
esac

if [[ ! -x "$BIN_PATH" ]]; then
  echo "Release binary not found at $BIN_PATH"
  exit 1
fi

exec "$BIN_PATH"
