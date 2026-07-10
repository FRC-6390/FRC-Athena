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

For runtime performance work, run the JMH benchmarks with GC allocation profiling:

```powershell
.\gradlew.bat :athena-benchmarks:jmhGc
```

## Local Development

Publish local snapshots to Maven Local when testing Athena from a robot project on the same machine:

```powershell
.\gradlew.bat publishToMavenLocal
```

Then add `mavenLocal()` to the consuming robot project's repositories while testing local snapshots.

To install the local snapshot through WPILib's local vendordep flow, use:

```powershell
.\gradlew.bat publishToLocalWpilib
```

That publishes Athena to Maven Local, mirrors the Athena artifacts into the local WPILib Maven folder, and installs `FRC6390-Athena.json` into the local WPILib vendordeps folder. By default it targets the WPILib year from `athenaFrcYear`. Override the install folder when needed:

```powershell
.\gradlew.bat publishToLocalWpilib -PathenaWpilibHome=C:\Users\Public\wpilib\2027
```

Do not use old root release helper commands such as `releaseChecklist`, `stageMavenPublication`, or `generateVendordep`. This repo now uses standard Gradle lifecycle and publishing tasks.

## Adding Athena To A Robot Project

The low-friction path is:

1. Install the Athena vendordep.
2. Keep the normal vendor vendordeps for the hardware and tools your robot actually uses.

```groovy
plugins {
    id "java"
    id "edu.wpi.first.GradleRIO" version "2027.x.x"
}
```

The checked-in vendordep is:

```text
vendordeps/FRC6390-Athena.json
```

The checked-in `example-projects` are currently pinned to the WPILib/GradleRIO 2026 project layout so they can compile against the available local toolchain while Athena 2027 API work continues. Treat those projects as API examples, not final 2027 WPILib release metadata.

Published JSON URL:

```text
https://raw.githubusercontent.com/FRC-6390/FRC-Athena/refs/heads/main/vendordeps/FRC6390-Athena.json
```

The vendordep provides the standard Athena robot stack for teams using WPILib's vendordep flow:

- `athena-api`
- `athena-runtime`
- `athena-hardware`
- `athena-mechanisms`
- `athena-drivetrain`
- `athena-vision`
- `athena-localization`
- `athena-auto`
- `athena-commands`
- `athena-robot`
- `athena-simulation`
- `athena-wpilib`
- `athena-vendor-ctre`
- `athena-vendor-rev`
- `athena-vendor-studica`
- `athena-vendor-limelight`
- `athena-vendor-photonvision`
- `athena-vendor-pathplanner`
- `athena-vendor-choreo`

The Athena vendor adapter artifacts are included by the single Athena vendordep, but they do not bring the real third-party vendor libraries with them. A robot that uses CTRE hardware still installs CTRE's vendordep, a robot that uses REV still installs REV's vendordep, and so on. If a matching third-party library is not present, Athena skips that adapter during service discovery.

## Hardware Connections

`HardwareBus` is the unified entry point for hardware declarations. Integer device IDs remain CAN shorthand. For other interfaces, select the connection from the bus and declare the device on it.

```java
HardwareBus RIO = HardwareBus.rio();
HardwareBus CANIVORE = HardwareBus.can("canivore");

MotorDevice drive = CANIVORE.motor(MotorKinds.KRAKEN_X60, 1);
EncoderDevice angle = CANIVORE.encoder(EncoderKinds.CANCODER, 9);
ImuDevice pigeon = RIO.imu(ImuKinds.PIGEON_2, 0);

EncoderDevice throughBore = RIO.dio(2).encoder(EncoderKinds.REV_THROUGH_BORE);
ImuDevice navx = RIO.spi(SpiPort.MXP).imu(ImuKinds.NAVX);
DigitalInputDevice home = RIO.dio(3).digitalInput("arm home");
```

The roboRIO bus exposes CAN, DIO, analog, SPI, I2C, serial, and USB. Named CAN buses expose CAN only. Requesting an unavailable interface, such as DIO from a CANivore, fails when the declaration is created. Vendor backends then validate that the selected device kind supports that connection.

## Module Guide

- `athena-api`: shared kind catalogs such as motor, encoder, IMU, and camera kinds.
- `athena-runtime`: common runtime values, measurements, signals, and control helpers.
- `athena-hardware`: hardware declarations, backend SPI, hardware graph, and simulation declarations.
- `athena-mechanisms`: mechanism/Action/output/control/event model.
- `athena-drivetrain`: drivetrain helper values and swerve module presets.
- `athena-commands`: command-style Action lifecycle.
- `athena-auto`: autonomous routines, path providers, and marker graphs.
- `athena-vision`: camera declarations, pose/target signals, and camera adapter runtime.
- `athena-localization`: localization pipelines, filters, and estimators.
- `athena-simulation`: simulation session, in-memory simulated hardware handles, and model stepping.
- `athena-robot`: root Athena robot runtime.
- `athena-wpilib`: WPILib `TimedRobot` lifecycle bridge.
- `athena-vendor-*`: optional vendor backends/adapters.

## Current Architecture Notes

Mechanisms are the main robot behavior abstraction. Swerve is not a special runtime graph anymore: a drivetrain should be a normal `Mechanism`, and swerve modules are normal mechanism components. Hardware declarations resolve through `HardwareGraph`; vendor modules provide backend implementations through service loading.

Simulation runs through the normal `RobotRuntime` with `SimulationSession` selecting simulated hardware backends. `SimModel` is one composable API for provider-backed leaves, nested models, and custom runtime rules. Production `SwerveKinematics` owns module targeting and forward kinematics and contributes its model automatically, so drive code and simulation share one layout without calling WPILib kinematics. Robot actions are evaluated by the normal runtime tick; simulation periodic advances physics from the already-applied hardware commands.
