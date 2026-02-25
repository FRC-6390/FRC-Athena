# Athena ARCP

Canonical home for the Athena Realtime Communication Protocol implementation.

- Rust backend: `athena-arcp/rust`
- Java thin interface: `athena-arcp/java`
- Host dashboard tooling (CLI + web, run on client computer): `athena-arcp/rust/crates/arcp-dashboard`
- Host desktop dashboard (Tauri + Svelte, run on client computer): `athena-arcp/host-dashboard`

## Build Requirement

For publishing JNI bundles (`athena-arcp-java`) you have two supported paths:

- Recommended: Dockerized cross-build (`docker` installed).
- Native local build: `rustup` + `cargo` (+ cross GCC toolchains for non-host targets).

Teams consuming released vendordeps do not need Rust on the robot runtime.
