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

Build, test, and publish Athena into the local WPILib Maven/vendordep installation with one command:

```powershell
.\gradlew.bat buildAndPublish
```

Publish only to Maven Local when testing Athena from a project that uses `mavenLocal()` directly:

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

## roboRIO System Tuning

`AthenaRobot` automatically monitors system and JVM health. On a positively identified low-memory
roboRIO, it prefers a verified 64 MiB compressed zram device, falls back to a verified 32 MiB swap
file only when enough disk headroom remains, stops NI's optional web services, and enables kernel
overcommit plus aggressive VFS-cache reclaim independently of optional swap support. The explicit
`lowMemory()` profile raises zram to 96 MiB, fallback swap to 64 MiB, and reacts earlier to pressure.
roboRIO 2, simulation, and unknown Linux targets are monitor-only by default. Tuning runs once on a
low-priority daemon and a failed operation is reported without stopping robot code.

No setup is required. A robot can explicitly select another policy in its constructor:

```java
systemTuning(SystemTuning.standard());   // Monitor only
systemTuning(SystemTuning.lowMemory());  // Force tuning on a real Linux robot
systemTuning(SystemTuning.restoreDefaults()); // Restore the captured pre-Athena state
systemTuning(SystemTuning.disabled());   // Disable tuning and periodic sampling
```

Sizes, disk reserve, pressure thresholds, and hysteresis are configurable without exposing shell
commands:

```java
systemTuning(SystemTuning.automatic()
        .zramMegabytes(64)
        .swapMegabytes(32)
        .minimumFreeDiskMegabytes(24)
        .pressureThresholds(0.14, 0.07)
        .pressureHysteresis(3, 6));
```

Athena records original sysctl and NI web-service state plus ownership of swap resources in
`/home/lvuser/athena/system-state.properties`. Each operation is idempotent and read back from the
kernel. `restoreDefaults()` removes only Athena-owned swap, restores captured sysctls, restarts the
web service only when it originally ran, and removes the state file after every restoration verifies.

The latest snapshot is available from `systemStatus()`. AdvantageScope and other NT4 clients can
inspect `/Athena/System`, including target/profile, hysteretic pressure and its reason, total/current/
lowest available RAM, process RSS, available-memory trend and exhaustion estimate, heap/non-heap/
direct-buffer usage, allocation rate, GC count/time/load, process and system CPU, live threads,
swap kind and usage, pressure transitions, tuning verification, applied changes, and failures.

At warning pressure Athena stops trace capture and changes read-only diagnostic telemetry from 10 Hz
to 2 Hz. At critical pressure it changes diagnostics to 1 Hz and pauses auto previews. Writable
dashboard controls and actions still run every robot cycle. Hardware refresh, actions, localization,
normal vision processing, and mechanism control are never shed.

System tuning cannot change the Java process's `-Xmx` after startup. On a roboRIO 1-class target,
`/Athena/System/JVM/RecommendedArguments` and `/Athena/System/JVM/GradleRioSnippet` publish a bounded
startup recommendation and `/Athena/System/JVM/ConfigurationHealthy` reports whether the running
heap limit is in range. Apply the emitted arguments inside the existing GradleRIO artifact:

```groovy
frcJava(getArtifactTypeClass('FRCJavaArtifact')) {
    jvmArgs.addAll("-Xms16m", "-Xmx96m", "-Xss512k",
            "-XX:MaxDirectMemorySize=24m", "-XX:ReservedCodeCacheSize=32m",
            "-XX:MinHeapFreeRatio=5", "-XX:MaxHeapFreeRatio=20")
}
```

GradleRIO already selects Serial GC by default. Athena does not silently rewrite `build.gradle` or
the deployed robot command.

## Hardware Connections

`HardwareBus` is the unified entry point for hardware declarations. Integer device IDs remain CAN shorthand. For other interfaces, select the connection from the bus and declare the device on it.

Device and camera failures default to `FailurePolicy.DISABLE_MECHANISM`: Athena reports an error to Driver Station, neutralizes the owning mechanism, and keeps the robot program running. Override individual declarations when a different response is required:

```java
MotorDevice requiredDrive = CANIVORE.motor(MotorKinds.KRAKEN_X60, 1)
        .failurePolicy(FailurePolicy.PANIC);
MotorDevice optionalRoller = RIO.motor(MotorKinds.NEO_550, 2)
        .failurePolicy(FailurePolicy.DISABLE_DEVICE);
CameraDevice temporaryCamera = Cameras.photonVision("front")
        .failurePolicy(FailurePolicy.WARN);
```

`WARN` retains the last valid hardware snapshot and retries each cycle. Cameras emit no measurements while unavailable, preventing stale frames from entering localization. `PANIC` is the only policy that intentionally propagates the failure out of the runtime loop.

```java
HardwareBus RIO = HardwareBus.rio();
HardwareBus CANIVORE = HardwareBus.can("canivore");

MotorDevice drive = CANIVORE.motor(MotorKinds.KRAKEN_X60, 1);
MotorDevice intake = RIO.motor(MotorKinds.NEO_550, 2);
MotorDevice flexNeo = RIO.motor(MotorControllerKinds.SPARK_FLEX, MotorKinds.NEO, 3);

MotorDevice leader = RIO.motor(MotorKinds.NEO, 10)
        .supplyCurrentLimit(40)
        .statorCurrentLimit(60);
MotorDevice followerA = RIO.motor(MotorKinds.NEO, 11).follow(leader);
MotorDevice followerB = RIO.motor(MotorKinds.NEO, 12).follow(leader);
MotorDevice followerC = RIO.motor(MotorKinds.NEO, 13).follow(leader);
MotorDevice followerD = RIO.motor(MotorKinds.NEO, 14).follow(leader);

MotorDevice configuredNeo = RIO.motor(MotorKinds.NEO, 15)
        .vendor(RevMotorOptions.class, rev -> rev
                .voltageCompensation(12.0)
                .telemetrySignalPeriodMs(20)
                .forwardSoftLimitRotations(100.0));

EncoderDevice angle = CANIVORE.encoder(EncoderKinds.CANCODER, 9);
ImuDevice pigeon = RIO.imu(ImuKinds.PIGEON_2, 0);

EncoderDevice throughBoreV1 = RIO.dio(2).encoder(EncoderKinds.REV_THROUGH_BORE);
EncoderDevice throughBoreV2 = RIO.dio(3).encoder(EncoderKinds.REV_THROUGH_BORE_V2);
EncoderDevice throughBoreQuadrature = RIO.quadrature(4, 5)
        .encoder(EncoderKinds.REV_THROUGH_BORE_QUADRATURE);
ImuDevice navx = RIO.spi(SpiPort.MXP).imu(ImuKinds.NAVX);
DigitalInputDevice home = RIO.dio(6).digitalInput("arm home");
```

Normalize an IMU once where it is declared, then pass the resulting `ImuSource` to drivetrain,
localization, and controls:

```java
ImuSource heading = pigeon
        .transform(ImuTransform.identity().yawInverted())
        .mounting(ImuMount.yawDegrees(90.0))
        .relative();

PositionSignal wrappedHeading = heading.heading();
PositionSignal accumulatedHeading = heading.continuousHeading();
VelocitySignal yawRate = heading.yawRate();
ScalarSignal forwardAcceleration = heading.accelerationX();

boolean usable = heading.isConnected()
        && !heading.isCalibrating()
        && heading.isFresh(0.1);
```

For a cardinal installation, describe which sensor directions point robot-forward and robot-up:

```java
ImuSource heading = pigeon.mounting(ImuMount
        .forward(ImuDirection.NEGATIVE_Y)
        .up(ImuDirection.POSITIVE_Z));
```

`ImuTransform` corrects a vendor or wiring convention. `ImuMount` corrects the sensor's physical
orientation using a 3D rotation. Keeping those separate prevents mounting corrections from being
repeated in odometry and control code.

For REV SPARK controllers, Athena maps CTRE-style supply/stator declarations onto REV's
motor-phase current limiter. If both are set, the lower value wins. A
`RevMotorOptions.smartCurrentLimit(...)` value explicitly overrides that mapping. Initial REV
configuration preserves undeclared parameters (including values set with the REV Hardware Client)
and persists Athena's declared settings by default. Use
`RevMotorOptions.resetSafeParameters(true)` to explicitly reset safe parameters first.

`MotorKinds` identifies the physical motor so simulation can select its real motor constants.
Each built-in motor has a usual controller (`KRAKEN_X60` uses Talon FX, `NEO` uses Spark MAX,
and `NEO_VORTEX` uses Spark Flex). The explicit controller overload supports valid alternate
pairings without losing the physical motor model.

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

Action ownership and resource partitions are compiled from the declared graph. Dynamic
computed actions provide required stable ownership with
`Actions.compute(resolver, ownership...)`; only the resolver runs each periodic cycle. Control
bindings retain one bound runtime while safely resetting controller state at neutral,
cancellation, arbitration loss, and disable boundaries.

The standalone robot examples are indexed in [`example-projects/README.md`](example-projects/README.md).
The feature coverage matrix is [`example-projects/COVERAGE.md`](example-projects/COVERAGE.md).
The scheduler's request, arbitration, completion, cancellation, and mode guarantees are defined in
[`docs/action-runtime-contract.md`](docs/action-runtime-contract.md).
