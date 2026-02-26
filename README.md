# FRC-Athena

## Vendordep URLs

Athena vendordeps (install these from WPILib vendordep manager):
- https://raw.githubusercontent.com/FRC-6390/FRC-Athena/refs/heads/main/vendordeps/FRC6390-Athena-Core.json
- https://raw.githubusercontent.com/FRC-6390/FRC-Athena/refs/heads/main/vendordeps/FRC6390-Athena-CTRE.json
- https://raw.githubusercontent.com/FRC-6390/FRC-Athena/refs/heads/main/vendordeps/FRC6390-Athena-REV.json
- https://raw.githubusercontent.com/FRC-6390/FRC-Athena/refs/heads/main/vendordeps/FRC6390-Athena-PhotonVision.json
- https://raw.githubusercontent.com/FRC-6390/FRC-Athena/refs/heads/main/vendordeps/FRC6390-Athena-Limelight.json
- https://raw.githubusercontent.com/FRC-6390/FRC-Athena/refs/heads/main/vendordeps/FRC6390-Athena-Pathplanner.json
- https://raw.githubusercontent.com/FRC-6390/FRC-Athena/refs/heads/main/vendordeps/FRC6390-Athena-Choreo.json
- https://raw.githubusercontent.com/FRC-6390/FRC-Athena/refs/heads/main/vendordeps/FRC6390-Athena-Studica.json
- https://raw.githubusercontent.com/FRC-6390/FRC-Athena/refs/heads/main/vendordeps/Studica-2026.0.0.json

External vendordeps required by Athena modules:
- https://maven.ctr-electronics.com/release/com/ctre/phoenix6/latest/Phoenix6-frc2026-latest.json
- https://software-metadata.revrobotics.com/REVLib-2026.json
- https://maven.photonvision.org/repository/internal/org/photonvision/photonlib-json/1.0/photonlib-json-1.0.json
- https://3015rangerrobotics.github.io/pathplannerlib/PathplannerLib.json
- https://choreo.autos/lib/ChoreoLib2026.json

## First-Time Toolchain Setup (Docker + Rust)

This section is for Athena maintainers and contributors who build/publish ARCP JNI natives (`athena-arcp-java`).

If you only consume released Athena vendordeps in a robot project, you can skip this section.

### 1) Install Docker

Recommended path for cross-target JNI bundles is Dockerized cross-build.

- Linux: install Docker Engine from official docs for your distro: https://docs.docker.com/engine/install/
- macOS/Windows: install Docker Desktop: https://docs.docker.com/desktop/

Linux post-install (so `docker` runs without `sudo`):

```bash
sudo usermod -aG docker $USER
newgrp docker
```

Verify:

```bash
docker --version
docker run --rm hello-world
```

### 2) Install Rust (`rustup`, `cargo`, `rustc`)

Install Rust via rustup:

- macOS/Linux:
```bash
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
```
- Windows (PowerShell):
```powershell
winget install Rustlang.Rustup
```

Restart terminal, then verify:

```bash
rustup --version
rustc --version
cargo --version
rustup default stable
```

### 3) Install Required Rust Targets

ARCP vendor JNI matrix uses these targets:

- `armv7-unknown-linux-gnueabihf`
- `x86_64-unknown-linux-gnu`
- `aarch64-unknown-linux-gnu`
- `x86_64-pc-windows-gnu`

Install all required targets:

```bash
rustup target add \
  armv7-unknown-linux-gnueabihf \
  x86_64-unknown-linux-gnu \
  aarch64-unknown-linux-gnu \
  x86_64-pc-windows-gnu
```

Verify installed targets:

```bash
rustup target list --installed
```

### 4) Build Path You Should Use

Recommended (Docker cross-build):

```bash
./gradlew :athena-arcp-java:prepareArcpNativeResources -ParcpNativeMode=vendor -ParcpUseDockerCross=true
```

Native cross-build (without Docker) requires additional linker toolchains:

- `armv7-unknown-linux-gnueabihf` -> `arm-linux-gnueabihf-gcc`
- `aarch64-unknown-linux-gnu` -> `aarch64-linux-gnu-gcc`
- `x86_64-pc-windows-gnu` -> `x86_64-w64-mingw32-gcc` (for direct GNU Windows target builds)

Windows host note: without Docker cross-build, the Windows JNI target falls back to `x86_64-pc-windows-msvc` and still stages to `native/windows-x86_64/arcp_jni.dll`.

Host-only native build (fast local dev):

```bash
./gradlew :athena-arcp-java:prepareArcpNativeResources -ParcpNativeMode=host
```

## Blank WPILib Project Setup (Required Gradle Changes)

Minimum required for most teams:

Full guide:
- `docs/core/wpilib-project-setup.md`

- Install Athena vendordep(s).
- Keep WPILib executable/fat-jar wiring for rio deploy (especially if you edit `jar`):
```groovy
def ROBOT_MAIN_CLASS = "frc.robot.Main"
def deployArtifact = deploy.targets.roborio.artifacts.frcJava

jar {
    from { configurations.runtimeClasspath.collect { it.isDirectory() ? it : zipTree(it) } }
    from sourceSets.main.allSource
    manifest edu.wpi.first.gradlerio.GradleRIOPlugin.javaManifest(ROBOT_MAIN_CLASS)
    duplicatesStrategy = DuplicatesStrategy.INCLUDE
}

deployArtifact.jarTask = jar
wpi.java.configureExecutableTasks(jar)
```

Only if using Athena State DSL/compiler-plugin features:
- add Athena maven to `pluginManagement.repositories` in `settings.gradle`
- add `athena-plugin` classpath + `apply plugin: "ca.frc6390.athena.plugin"` in `build.gradle`

## Build Commands

Build changed Athena modules and downstream dependents (default `build` behavior):
```bash
./gradlew build
```

Run the changed-module task directly:
```bash
./gradlew buildChangedModules
```

Build changed modules from a specific git base:
```bash
./gradlew buildChangedModules -PchangedSince=origin/main
```

Force build all Athena modules:
```bash
./gradlew buildAllModules
```

Force all modules through root build:
```bash
./gradlew build -PbuildAllModules=true
```

Build each module directly (per dependency module):
```bash
./gradlew :athena-core:build
./gradlew :athena-ctre:build
./gradlew :athena-rev:build
./gradlew :athena-photonvision:build
./gradlew :athena-limelight:build
./gradlew :athena-pathplanner:build
./gradlew :athena-choreo:build
./gradlew :athena-studica:build
```

Publish all modules locally to WPILib:
```bash
./gradlew publish -PpublishMode=local
```

Publish all modules online:
```bash
./gradlew publish -PpublishMode=online
```

Publish a single module locally:
```bash
./gradlew :athena-core:publishToMavenLocal
./gradlew :athena-ctre:publishToMavenLocal
./gradlew :athena-rev:publishToMavenLocal
./gradlew :athena-photonvision:publishToMavenLocal
./gradlew :athena-limelight:publishToMavenLocal
./gradlew :athena-pathplanner:publishToMavenLocal
./gradlew :athena-choreo:publishToMavenLocal
./gradlew :athena-studica:publishToMavenLocal
```

Set explicit version/FRC year during build or publish:
```bash
./gradlew build -Pversion="2026.1.99" -PfrcYear="2026"
./gradlew publish -PpublishMode=local -Pversion="2026.1.99" -PfrcYear="2026"
```

Online publish credentials:
```bash
export MAVEN_USERNAME=<username>
export MAVEN_PASSWORD=<token-or-password>
./gradlew publish -PpublishMode=online
```
