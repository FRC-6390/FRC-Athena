# FRC Athena 2027

Athena is split into small Java artifacts for robot code, hardware declarations, mechanisms, drivetrain helpers, vision, localization, autonomous paths, simulation, WPILib lifecycle integration, and vendor adapters.

This branch is the 2027 rebuild. The API is not a 1:1 port of older Athena internals; the goal is feature coverage with cleaner boundaries.

## Requirements

- JDK 17
- Gradle wrapper from this repository
- WPILib/FRC vendor repositories from `settings.gradle`

Use the wrapper from the repo root:

```powershell
.\gradlew.bat test
.\gradlew.bat check
.\gradlew.bat build
```

On macOS/Linux:

```bash
./gradlew test
./gradlew check
./gradlew build
```

`test` runs unit tests. `check` runs verification and tests. `build` compiles, checks, and packages every module.

## Local Development

Publish local snapshots to Maven Local when testing Athena from a robot project on the same machine:

```powershell
.\gradlew.bat publishToMavenLocal
```

Then add `mavenLocal()` to the consuming robot project's repositories while testing local snapshots.

Do not use old root release helper commands such as `releaseChecklist`, `stageMavenPublication`, or `generateVendordep`. This repo now uses standard Gradle lifecycle and publishing tasks.

## Adding Athena To A Robot Project

The checked-in vendordep is:

```text
vendordeps/FRC6390-Athena.json
```

Published JSON URL:

```text
https://raw.githubusercontent.com/FRC-6390/FRC-Athena/refs/heads/main/vendordeps/FRC6390-Athena.json
```

The vendordep provides the default Athena modules:

- `athena-api`
- `athena-runtime`
- `athena-hardware`
- `athena-mechanisms`
- `athena-drivetrain`
- `athena-commands`

Most robot projects should also add the runtime host explicitly:

```groovy
dependencies {
    implementation "ca.frc6390.athena:athena-robot:2027.0.0-SNAPSHOT"
    implementation "ca.frc6390.athena:athena-wpilib:2027.0.0-SNAPSHOT"
}
```

Add optional modules only when used:

```groovy
dependencies {
    implementation "ca.frc6390.athena:athena-vision:2027.0.0-SNAPSHOT"
    implementation "ca.frc6390.athena:athena-localization:2027.0.0-SNAPSHOT"
    implementation "ca.frc6390.athena:athena-auto:2027.0.0-SNAPSHOT"
    implementation "ca.frc6390.athena:athena-simulation:2027.0.0-SNAPSHOT"
}
```

Add vendor adapters that match installed hardware/vendor libraries:

```groovy
dependencies {
    implementation "ca.frc6390.athena:athena-vendor-ctre:2027.0.0-SNAPSHOT"
    implementation "ca.frc6390.athena:athena-vendor-rev:2027.0.0-SNAPSHOT"
    implementation "ca.frc6390.athena:athena-vendor-studica:2027.0.0-SNAPSHOT"
    implementation "ca.frc6390.athena:athena-vendor-limelight:2027.0.0-SNAPSHOT"
    implementation "ca.frc6390.athena:athena-vendor-photonvision:2027.0.0-SNAPSHOT"
    implementation "ca.frc6390.athena:athena-vendor-pathplanner:2027.0.0-SNAPSHOT"
    implementation "ca.frc6390.athena:athena-vendor-choreo:2027.0.0-SNAPSHOT"
}
```

## Athena Gradle Plugin

The Athena Gradle plugin is build tooling, not a robot runtime dependency. It should not be added through the vendordep. When the plugin is available to the consuming build, it can add modules and vendor adapters for you:

```groovy
plugins {
    id "java"
    id "ca.frc6390.athena" version "2027.0.0-SNAPSHOT"
}

athena {
    setVersion("2027.0.0-SNAPSHOT")
    features "drivetrain", "vision", "localization", "auto", "wpilib", "simulation"
    vendors "ctre", "rev", "limelight", "photonvision", "pathplanner", "choreo"
}
```

Supported feature names:

- `drivetrain`
- `vision`
- `localization`
- `auto`
- `wpilib`
- `simulation`

Supported vendor names:

- `ctre`
- `rev`
- `studica`
- `limelight`
- `photonvision`
- `pathplanner`
- `choreo`

## Module Guide

- `athena-api`: shared kind catalogs such as motor, encoder, IMU, and camera kinds.
- `athena-runtime`: common runtime values, measurements, signals, and control helpers.
- `athena-hardware`: hardware declarations, backend SPI, hardware graph, and simulation declarations.
- `athena-mechanisms`: mechanism/state/output/control/event model.
- `athena-drivetrain`: drivetrain helper values and swerve module presets.
- `athena-commands`: command-style state lifecycle.
- `athena-auto`: autonomous routines, path providers, and marker graphs.
- `athena-vision`: camera declarations, pose/target signals, and camera adapter runtime.
- `athena-localization`: localization pipelines, filters, and estimators.
- `athena-simulation`: in-memory simulated hardware runtime.
- `athena-robot`: root Athena robot runtime.
- `athena-wpilib`: WPILib `TimedRobot` lifecycle bridge.
- `athena-vendor-*`: optional vendor backends/adapters.

## Current Architecture Notes

Mechanisms are the main robot behavior abstraction. Swerve is not a special runtime graph anymore: a drivetrain should be a normal `Mechanism`, and swerve modules are normal mechanism components. Hardware declarations resolve through `HardwareGraph`; vendor modules provide backend implementations through service loading.

Simulation currently supports in-memory hardware handles and simple model stepping. More WPILib-backed physics simulation is still a planned addition.
