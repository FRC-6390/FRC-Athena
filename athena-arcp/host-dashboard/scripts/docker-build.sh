#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
REPO_DIR="$(git -C "${ROOT_DIR}" rev-parse --show-toplevel)"
TARGET="${1:-linux-x64}"

case "$TARGET" in
  linux-x64)
    PLATFORM="linux/amd64"
    TAURI_TARGET="x86_64-unknown-linux-gnu"
    BUILD_KIND="bundles"
    BUNDLES="deb,rpm"
    EXTRA_ENV=""
    ;;
  linux-x64-appimage)
    PLATFORM="linux/amd64"
    TAURI_TARGET="x86_64-unknown-linux-gnu"
    BUILD_KIND="bundles"
    BUNDLES="appimage"
    # Work around linuxdeploy/binutils strip incompatibilities on some hosts.
    EXTRA_ENV="NO_STRIP=1"
    ;;
  linux-x64-bin)
    PLATFORM="linux/amd64"
    TAURI_TARGET="x86_64-unknown-linux-gnu"
    BUILD_KIND="bin"
    BUNDLES=""
    EXTRA_ENV=""
    ;;
  linux-arm64)
    PLATFORM="linux/arm64"
    TAURI_TARGET="aarch64-unknown-linux-gnu"
    BUILD_KIND="bundles"
    BUNDLES="deb,rpm"
    EXTRA_ENV=""
    ;;
  linux-arm64-appimage)
    PLATFORM="linux/arm64"
    TAURI_TARGET="aarch64-unknown-linux-gnu"
    BUILD_KIND="bundles"
    BUNDLES="appimage"
    EXTRA_ENV="NO_STRIP=1"
    ;;
  linux-arm64-bin)
    PLATFORM="linux/arm64"
    TAURI_TARGET="aarch64-unknown-linux-gnu"
    BUILD_KIND="bin"
    BUNDLES=""
    EXTRA_ENV=""
    ;;
  linux-all)
    "$0" linux-x64
    "$0" linux-x64-appimage
    "$0" linux-arm64
    "$0" linux-arm64-appimage
    exit 0
    ;;
  *)
    cat <<EOF
Usage: $(basename "$0") <target>

Targets:
  linux-x64             build deb + rpm bundles (x86_64)
  linux-x64-appimage    build appimage bundle (x86_64)
  linux-x64-bin         build release binary only (x86_64)
  linux-arm64           build deb + rpm bundles (aarch64)
  linux-arm64-appimage  build appimage bundle (aarch64)
  linux-arm64-bin       build release binary only (aarch64)
  linux-all             build all Linux bundle targets
EOF
    exit 1
    ;;
esac

if ! command -v docker >/dev/null 2>&1; then
  echo "docker is required but not installed."
  exit 1
fi

mkdir -p "${ROOT_DIR}/.docker-cache/cargo-registry"
mkdir -p "${ROOT_DIR}/.docker-cache/cargo-git"
mkdir -p "${ROOT_DIR}/.docker-cache/rustup"

USER_ID="$(id -u)"
GROUP_ID="$(id -g)"
IMAGE_TAG="arcp-host-dashboard-builder:${PLATFORM//\//-}"

echo "[docker-build] building image ${IMAGE_TAG} for ${PLATFORM}"
docker buildx build \
  --load \
  --platform "${PLATFORM}" \
  --build-arg USER_ID="${USER_ID}" \
  --build-arg GROUP_ID="${GROUP_ID}" \
  -f "${ROOT_DIR}/docker/Dockerfile.builder" \
  -t "${IMAGE_TAG}" \
  "${ROOT_DIR}"

if [[ "${BUILD_KIND}" == "bundles" ]]; then
  BUILD_CMD="bunx tauri build --target ${TAURI_TARGET} --bundles ${BUNDLES}"
else
  BUILD_CMD="bunx tauri build --target ${TAURI_TARGET} --no-bundle"
fi

ENV_PREFIX=""
if [[ -n "${EXTRA_ENV}" ]]; then
  ENV_PREFIX="${EXTRA_ENV} "
fi

echo "[docker-build] running ${TARGET} in container (${PLATFORM})"
docker run --rm \
  --platform "${PLATFORM}" \
  -e HOME=/home/builder \
  -e CARGO_HOME=/home/builder/.cargo \
  -e RUSTUP_HOME=/home/builder/.rustup \
  -v "${REPO_DIR}:/workspace" \
  -v "${ROOT_DIR}/.docker-cache/cargo-registry:/home/builder/.cargo/registry" \
  -v "${ROOT_DIR}/.docker-cache/cargo-git:/home/builder/.cargo/git" \
  -v "${ROOT_DIR}/.docker-cache/rustup:/home/builder/.rustup" \
  -w /workspace/athena-arcp/host-dashboard \
  "${IMAGE_TAG}" \
  bash -lc "\
    set -euo pipefail; \
    export PATH=/home/builder/.bun/bin:/home/builder/.cargo/bin:\$PATH; \
    bun --version; \
    rustup toolchain install stable --profile minimal; \
    rustup default stable; \
    cargo --version; \
    rustup target add ${TAURI_TARGET}; \
    bun install --frozen-lockfile; \
    ${ENV_PREFIX}${BUILD_CMD}; \
    echo '[docker-build] done: ${TARGET}' \
  "
