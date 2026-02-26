# Athena ARCP Java Layer

Thin Java facade over the Rust-native ARCP runtime.

Source root:
- `athena-arcp/java/src/main/java`

Current package:
- `ca.frc6390.athena.arcp`

## Build Requirements

`athena-arcp-java` supports two build paths:

1) Dockerized cross-build (recommended for vendordep publish):
- `docker`
- No local cross GCC toolchains required.

2) Native local build:
- `rustup`
- `cargo`
- Cross GCC toolchains for non-host Linux targets.

Install Rust from: https://rustup.rs

Notes:
- Teams consuming released vendordeps do not need Rust on the robot runtime.
- Maintainers/CI need either Docker (cross mode) or Rust toolchain (native mode).

## Native Packaging

`athena-arcp-java` now bundles `arcp_jni` binaries directly inside the jar under:

- `native/linux-x86_64/libarcp_jni.so`
- `native/linux-aarch64/libarcp_jni.so`
- `native/linux-arm32/libarcp_jni.so`
- `native/macos-*/libarcp_jni.dylib`
- `native/windows-*/arcp_jni.dll`

At runtime, `ArcpNative` tries to load the bundled binary first (extracts to temp), then falls back
to `System.loadLibrary("arcp_jni")`.

## Build Modes

Native build is driven by Gradle task `prepareArcpNativeResources`.

- `host` (default for normal builds): build current host target only.
- `vendor` (default when running publish tasks): cross-build vendor matrix.
- `full`: build full configured matrix.
- `none`: skip native staging.

Properties/env:

- `-ParcpNativeMode=<host|vendor|full|none>`
- `-ParcpJniTargets=triple1,triple2,...` (overrides mode matrix)
- `-ParcpUseDockerCross=<true|false>` (default `true` for `vendor/full`, `false` for `host`)
- `-ParcpDockerImage=<tag>` (default `athena-arcp-cross:latest`)
- `-ParcpDockerRebuild=<true|false>` (force rebuild of cross image)
- `-ParcpInstallRustTargets=<true|false>` (auto-run `rustup target add` for missing targets)
- `-ParcpOptionalTargets=triple1,triple2,...` (targets allowed to skip if toolchain is missing)
- `-ParcpRequireAllTargets=<true|false>` (when true, fail build instead of skipping optional targets)

Native-mode cross-linker prerequisites (non-Docker):

- `armv7-unknown-linux-gnueabihf` -> `arm-linux-gnueabihf-gcc`
- `aarch64-unknown-linux-gnu` -> `aarch64-linux-gnu-gcc`
- `x86_64-pc-windows-gnu` -> `x86_64-w64-mingw32-gcc` (required when directly building GNU Windows target)

Windows host behavior:

- With `-ParcpUseDockerCross=true`, Docker is detected with native Windows path lookup (`where docker`).
- With `-ParcpUseDockerCross=false`, requested `x86_64-pc-windows-gnu` is built as `x86_64-pc-windows-msvc` and still staged to `native/windows-x86_64/arcp_jni.dll`.

Vendor matrix targets:

- `armv7-unknown-linux-gnueabihf`
- `x86_64-unknown-linux-gnu`
- `aarch64-unknown-linux-gnu`
- `x86_64-pc-windows-gnu`

Examples:

- `./gradlew :athena-arcp-java:prepareArcpNativeResources -ParcpNativeMode=host`
- `./gradlew :athena-arcp-java:prepareArcpNativeResources -ParcpNativeMode=vendor` (docker cross by default)
- `./gradlew :athena-arcp-java:prepareArcpNativeResources -ParcpNativeMode=vendor -ParcpUseDockerCross=true`
- `./gradlew :athena-arcp-java:prepareArcpNativeResources -ParcpNativeMode=vendor -ParcpUseDockerCross=false`
- `./gradlew :athena-arcp-java:prepareArcpNativeResources -ParcpNativeMode=vendor -ParcpOptionalTargets=armv7-unknown-linux-gnueabihf`
- `./gradlew :athena-arcp-java:prepareArcpNativeResources -ParcpNativeMode=vendor -ParcpRequireAllTargets=true`
- `./gradlew :athena-arcp-java:publishToMavenLocal` (defaults to vendor mode unless overridden)
