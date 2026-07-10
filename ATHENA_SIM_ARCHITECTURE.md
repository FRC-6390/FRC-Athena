# Athena Simulation Architecture

Athena simulation is a hardware backend swap, not a parallel robot runtime.

Robot code must declare real hardware kinds. Simulation code must not require teams to replace those declarations with simulation-only kinds.

```java
MotorDevice shooter = Constants.CANIVORE.motor(MotorKinds.KRAKEN_X60, 12);
EncoderDevice shooterEncoder = shooter.encoder();

SimModel shooterSim = SimModel.flywheel(shooter)
        .encoder(shooterEncoder)
        .gearRatio(GearRatio.reduction(18.0, 1.0))
        .momentOfInertia(0.004);
```

The same mechanism graph, `Action` graph, hardware declarations, controls, and `RobotRuntime` run in real and simulated modes. Only the hardware backend changes.

## Runtime Boundary

```text
Robot code
  -> real hardware declarations
  -> normal Athena RobotRuntime
  -> HardwareGraph
  -> BackendRegistry

Real robot:
  BackendRegistry -> CTRE / REV / Studica / vision vendor backends

Simulation:
  BackendRegistry -> Athena simulation backends for the same real hardware kinds
```

`athena-mechanisms` should execute behavior only. It should not own fake hardware handles or physics stepping.

`athena-simulation` owns provider-neutral simulation coordination, fallback simulated
device handles, and shared world state behind `SimulationSession`:

- motor command state
- motor sensor state
- encoder state
- IMU state
- digital input state
- field pose state
- registered, composable `SimModel` declarations
- deterministic step loop

`athena-wpilib` owns WPILib-backed physics adapters. It may use `DCMotorSim`, `FlywheelSim`, `SingleJointedArmSim`, `ElevatorSim`, `BatterySim`, `RoboRioSim`, and `Field2d` internally, but those types must not leak into user-facing Athena mechanism code.

`SimulationSession` is the public coordinator for that state. Robot code still runs
through the same `RobotRuntime`, mechanism scheduler, hook runtime, controls,
commands, autos, and localization path used on the real robot. Simulation only
changes the selected `HardwareGraph` backends and adds coordinated physics/world
stepping.

`athena-simulation` exposes a narrow `SimPhysicsEngine` SPI so the default in-memory runner can be replaced by a host-specific engine. `athena-wpilib` installs `WpilibSimPhysicsEngine` in `AthenaRobot` simulation mode.

`WpilibSimPhysicsEngine` estimates loaded battery voltage with WPILib `BatterySim`. Publishing that value into `RoboRioSim` must remain an optional HAL-backed voltage sink; direct RoboRIO voltage writes are not part of the default path because plain unit tests and non-HAL hosts may not have the required native simulation runtime loaded.

## SimModel Contract

`SimModel` is the single simulation composition API. One model can contain provider-backed
physics leaves, other models, and custom per-session runtime rules. A declaration implementing
`SimModel.Source` contributes its model automatically when discovered in a mechanism.

Good public API:

```java
SimModel.arm(pivotMotor)
        .encoder(pivotEncoder)
        .gearRatio(GearRatio.reduction(125.0, 1.0))
        .lengthMeters(0.42)
        .momentOfInertia(0.018)
        .range(Range.of(-0.25, 1.75))
        .limit(lowerLimit, 0.0);
```

Not public API:

```java
new SingleJointedArmSim(...);
DCMotor.getKrakenX60(...);
```

Custom behavior uses the same model rather than creating another simulation runtime:

```java
SimModel custom = SimModel.builder()
        .motor(motor)
        .encoder(encoder)
        .runtime(context -> seconds -> {
            double velocity = context.command(motor).value() * 5.0;
            double position = context.encoderPosition(encoder) + velocity * seconds;
            context.encoderState(encoder, position, position, velocity);
        })
        .build();
```

Provider-specific physics remains behind the provider-backed leaves and does not change the
model/session contract.

## Automatic Selection

`AthenaRobot` should create a simulation-backed hardware graph when WPILib reports simulation mode. Robot code should not change.

```java
if (RobotBase.isSimulation()) {
    SimulationSession simulation = SimulationSession.create()
            .physicsEngine(new WpilibSimPhysicsEngine());
    runtime = RobotRuntime.simulated(simulation);
} else {
    runtime = RobotRuntime.create();
}
```

Tests can force simulation directly with a session/harness even when WPILib simulation detection is unavailable.

Normal robot periodic still owns action, hook, command, vision, and localization evaluation. WPILib simulation periodic only advances the selected `SimulationSession` physics from the hardware commands already applied by the normal runtime tick; it must not execute mechanisms a second time.

## Pose And Drivetrain State

`SimulationSession` owns the provider-neutral robot pose as a `PoseSnapshot`. Tests and future WPILib adapters can reset/read that pose without leaking `Pose2d`, `Field2d`, or WPILib simulation classes into Athena mechanism code.

```java
SimulationSession session = SimulationSession.create()
        .resetPose(new PoseSnapshot(0.0, 0.0, 0.0));

session.drivePose(1.0, 0.0, 0.0, 0.02);
PoseSnapshot pose = session.pose();
```

Drivetrain kinematics are production declarations, not simulation selections. `SwerveKinematics`
owns the real module layout and is used both to create module target Actions and to solve measured
chassis velocity. Because it implements `SimModel.Source`, its composed motor/steer/pose model is
discovered automatically in simulation.

```java
SwerveModule frontLeft = new SwerveModules.SDS.MK5N.R3();
frontLeft.drive.fill(driveMotor)
        .steer.fill(steerMotor)
        .angle.fill(absoluteEncoder);

private final SwerveKinematics kinematics = SwerveKinematics.rectangular(
        0.58,
        0.58,
        4.5,
        frontLeft,
        frontRight,
        backLeft,
        backRight);

public Action drive(RobotVelocity velocity) {
    return kinematics.drive(velocity);
}
```

The rectangular constructor is only a layout convenience. Arbitrary module positions use the
normal `SwerveKinematics` constructor. Simulation does not inspect motors to guess a drivetrain
type and no longer calls WPILib swerve kinematics.

## Vision Field Simulation

Vision simulation uses Athena-owned field metadata through `VisionSimulationField` and `VisionSimulationTarget`. A `SimulationSession` can be configured with `visionField(...)`; root runtime camera discovery passes that field into optional vision simulation providers. PhotonVision support binds unbound Athena camera declarations to deterministic target and pose measurements from that field, while native Photon simulation object creation is opt-in with `-Dathena.photonvision.sim.native=true` so teams can avoid vendor sim startup failures when they only need Athena measurements.

## Scoped Digital Inputs

Digital input declarations remain real hardware declarations. In simulation, `SimulationSession` owns a scoped digital input runtime that binds those declarations to session-local `SimDigitalInputHandle` state. Root `RobotRuntime.simulated(session)` evaluates mechanisms inside that scope, so two sessions can use the same DIO declaration without leaking raw values or latched signal edges into each other. Manual `DigitalInputDevice.bindRuntime(...)` remains a simple non-session helper for tests and direct bindings.

When the discovered kinematics model runs, module speed/angle commands advance
`SimulationSession.pose()` and synchronize simulated IMU yaw through the shared runtime pose state.

## Current Coverage

- Robot code declares real hardware kinds; there are no public `SIM` motor, encoder, or IMU kinds.
- `SimulationSession` provides simulated hardware handles, model registration, pose state, and test readback.
- `athena-simulation` owns generic in-memory handle/model stepping and custom model runtimes.
- `athena-wpilib` owns WPILib-backed motor, flywheel, arm, elevator, and battery simulation.
- `AthenaRobot` creates a simulation-backed `RobotRuntime` under WPILib simulation.
- Simulation periodic steps physics only; robot actions are evaluated once through the normal runtime path.
- Vision simulation is bridged through provider discovery and currently uses PhotonVision when that vendor dependency is present.

Remaining coverage work is test depth and tuning: broaden real-kind backend coverage, expand mechanism physics assertions, add more whole-robot sim fixtures, and keep benchmark coverage growing around large robot projects.

## Vision Simulation Boundary

Vision simulation is optional and vendor-backed. Core Athena should expose only provider-neutral discovery and update hooks; it should not depend on PhotonVision or WPILib camera simulation classes.

The intended implementation is the same shape as Athena 2026:

- `athena-vision` defines the provider-neutral `VisionSimulation` / `VisionSimulationProvider` SPI.
- `athena-vendor-photonvision` contributes the ServiceLoader provider.
- The Photon provider uses PhotonVision's simulation engine internally.
- The provider registers every Athena `CameraDevice` it can simulate, not only `PhotonVisionDevice`, because PhotonVision supports custom camera setups.
- If the PhotonVision vendor artifact and PhotonLib are not on the robot classpath, no camera simulation provider is discovered and camera sim is simply unavailable.

User robot code should still declare normal cameras:

```java
CameraDevice front = Cameras.limelight("front")
        .mount(0.25, 0.0, 0.5, 0.0, -10.0, 0.0);
```

Simulation code may use PhotonVision types internally, but those types must not leak into Athena camera declarations.

## Performance Guardrail

Do not move runtime code to Rust until Java performance is measured.

First proof step:

- add an `athena-benchmarks` JMH project
- benchmark mechanism scheduling
- benchmark full robot periodic
- benchmark hardware graph refresh
- benchmark simulation stepping

Optimize Java hot paths first: allocation-free action accessors, cached output buffers, REV input snapshots, cached vision adapters/signals, and duplicate sim/auto stepping fixes.

## Worker Boundary

The default Athena runtime path is deterministic and single-threaded. Optional workers are configured explicitly through `RobotRuntime`; no background thread is created by default.

Workers are for snapshot-producing tasks such as fast signal sampling, vision/localization input refresh, or other read/compute work. They must not own motor writes, Action routing, hook execution, or autonomous lifecycle. Those stay in the single `RobotRuntime` periodic path.

```java
runtime.fastSignalSampling(0.005, scheduler);
```

The scheduler is caller-owned. Athena cancels the worker tasks it starts, but it does not shut down a scheduler it did not create.
