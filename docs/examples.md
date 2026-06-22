# Example Catalog

The example project is the primary reference for the new Athena syntax.
Every example uses simulation hardware so it can compile and validate without
installing CTRE, REV, PhotonVision, or other vendor libraries.

## Hardware Map

File: `example-project/src/main/java/ca/frc6390/athena/examples/RobotHardware.java`

`RobotHardware` keeps device identity in one place:

```java
public static final MotorId INTAKE_ROLLER = MotorId.of(AthenaMotor.SIM, 1);
public static final EncoderId SHOOTER_ENCODER = EncoderId.of(AthenaEncoder.SIM, 20);
public static final ImuId ROBOT_IMU = ImuId.of(AthenaImu.SIM, 0);
public static final CameraId FRONT_CAMERA = CameraId.of(AthenaCamera.SIM, "frontCam");
```

This is the recommended pattern for robot projects. Mechanisms reference
hardware aliases instead of repeating IDs throughout the codebase.

## Intake Mechanism

File: `example-project/src/main/java/ca/frc6390/athena/examples/IntakeExample.java`

The intake demonstrates the smallest useful mechanism:

```java
Mechanisms.simple("intake")
        .motor("roller", motor -> motor
                .hardware(RobotHardware.INTAKE_ROLLER)
                .brake()
                .currentLimit(35))
        .input("beamBreak", input -> input.digital(0))
        .control(control -> control.percentOutput())
        .state("stopped", state -> state.target(0.0))
        .state("feed", state -> state.target(0.65));
```

It covers:

- motor declaration
- digital input declaration
- percent-output control
- named state targets

## Shooter Mechanism

File: `example-project/src/main/java/ca/frc6390/athena/examples/ShooterExample.java`

The shooter demonstrates the richer mechanism surface:

```java
Mechanisms.flywheel("shooter")
        .motor("leader", motor -> motor
                .hardware(RobotHardware.SHOOTER_LEADER)
                .coast()
                .currentLimit(60)
                .integratedEncoder())
        .encoder("flywheelEncoder", encoder -> encoder
                .hardware(RobotHardware.SHOOTER_ENCODER)
                .velocity()
                .gearRatio(1.0))
        .velocitySource("flywheelEncoder")
        .input("targetRpm", input -> input.runtimeNumber("dashboard/shooterTargetRpm"))
        .control(control -> control
                .velocity(pid -> pid
                        .p(0.14)
                        .i(0.0)
                        .d(0.001))
                .feedforward(ff -> ff
                        .staticGain(0.18)
                        .velocity(0.12)))
        .state("idle", state -> state.target(0.0))
        .state("speaker", state -> state.target(4600.0))
        .state("amp", state -> state.target(1800.0));
```

It covers:

- named encoder declaration
- velocity source selection
- runtime input declaration
- PID gains
- feedforward gains
- multiple named state targets

## V2 Mechanism Shapes

File: `example-project/src/main/java/ca/frc6390/athena/examples/MechanismV2Example.java`

The v2 mechanism examples are represented as fluent declarations while the
annotation layer remains deferred:

```java
Mechanisms.simple("armPivot")
        .motor("armMotor", motor -> motor
                .hardware(AthenaMotor.SIM, 30)
                .brake()
                .currentLimit(45))
        .encoder("armAbsolute", encoder -> encoder
                .hardware(AthenaEncoder.SIM, 31)
                .absolutePosition())
        .positionSource("armAbsolute")
        .input("homeSwitch", input -> input.digital(4))
        .control(control -> control
                .position(pid -> pid.p(0.16))
                .feedforward(ff -> ff.gravity(0.08)))
        .state("home", state -> state.target(0.0))
        .state("score", state -> state.target(72.0));
```

It covers:

- simple roller percent output
- angular arm position control
- linear elevator position control with runtime inputs
- flywheel velocity control
- turret position control with target inputs
- runtime state application through `MechanismController`

```java
MechanismSpec spec = MechanismV2Example.armPivot().toSpec();
MechanismController.of(spec, List.of(pivotMotor)).applyState("score");
```

## Differential Drivetrain

File: `example-project/src/main/java/ca/frc6390/athena/examples/DriveExample.java`

The drivetrain example keeps drive declarations separate from mechanism
declarations:

```java
Drivetrains.differential("drive")
        .leftMotor("leftLeader", motor -> motor
                .hardware(RobotHardware.DRIVE_LEFT_LEADER)
                .brake()
                .currentLimit(45))
        .rightMotor("rightLeader", motor -> motor
                .hardware(RobotHardware.DRIVE_RIGHT_LEADER)
                .brake()
                .currentLimit(45))
        .trackWidth(TrackWidth.meters(0.71));
```

It covers:

- left/right motor groups
- shared motor config syntax
- physical drivetrain dimensions

## Swerve Drivetrain

File: `example-project/src/main/java/ca/frc6390/athena/examples/SwerveExample.java`

The swerve example declares module hardware, geometry, steer PID, and drive
feedforward without importing vendor classes:

```java
Drivetrains.swerve("swerve")
        .trackWidth(TrackWidth.meters(0.58))
        .wheelBase(WheelBase.meters(0.62))
        .module("frontLeft", module -> module
                .location(0.31, 0.29)
                .driveMotor(motor -> motor.hardware(RobotHardware.SWERVE_FRONT_LEFT_DRIVE))
                .steerMotor(motor -> motor.hardware(RobotHardware.SWERVE_FRONT_LEFT_STEER))
                .steerEncoder(encoder -> encoder
                        .hardware(RobotHardware.SWERVE_FRONT_LEFT_ENCODER)
                        .absolutePosition())
                .steerPid(4.5, 0.0, 0.12)
                .driveFeedforward(0.18, 2.1, 0.3));
```

It covers:

- module-level drive, steer, and encoder declarations
- robot-relative module locations
- drivetrain and module inversion resolution
- steer PID and drive feedforward gains

## Superstructure

File: `example-project/src/main/java/ca/frc6390/athena/examples/SuperstructureExample.java`

The superstructure coordinates mechanisms by name:

```java
Superstructures.create("robot")
        .part("intake", IntakeExample.CONFIG)
        .part("shooter", ShooterExample.CONFIG)
        .state("idle", state -> state
                .part("intake", "stopped")
                .part("shooter", "idle"))
        .state("score", state -> state
                .part("intake", "feed")
                .part("shooter", "speaker"));
```

It covers:

- named mechanism parts
- cross-mechanism state targets
- validation that state targets reference declared parts

File: `example-project/src/main/java/ca/frc6390/athena/examples/CompositeSuperstructureExample.java`

Nested superstructures let a top-level robot state target an assembly state:

```java
Superstructures.create("robotComposite")
        .part("handoff", CompositeSuperstructureExample.HANDOFF)
        .state("scoreSpeaker", state -> state
                .part("handoff", "feeding"));

CommandSpec score = CompositeSuperstructureExample.scoreSpeakerCommand(targets);
```

It covers:

- superstructure parts backed by another superstructure
- parent states targeting child superstructure states
- validation that nested child state names exist
- runtime transition planning into leaf mechanism targets
- runtime application through mechanism controllers
- command factory for applying a named superstructure state

File: `example-project/src/main/java/ca/frc6390/athena/examples/TurretSuperstructureExample.java`

The turret assembly coordinates turret angle, hood angle, and shooter velocity:

```java
Superstructures.create("turretAssembly")
        .part("turret", TurretSuperstructureExample.TURRET)
        .part("hood", TurretSuperstructureExample.HOOD)
        .part("shooter", ShooterExample.CONFIG)
        .state("speaker", state -> state
                .part("turret", "speaker")
                .part("hood", "speaker")
                .part("shooter", "speaker"));
```

It covers:

- turret-style position mechanism declarations
- assembly-level state names that map to several child mechanisms
- child target validation for mechanism state names
- transition guard hooks before target execution

## Commands

File: `example-project/src/main/java/ca/frc6390/athena/examples/RobotCommands.java`

Commands are currently WPILib-independent descriptors:

```java
CommandSpec.create("runIntake")
        .onExecute(intakeRunning::run)
        .toSpec();

CommandGroups.sequence("scoreSequence", spinUp, feed);
```

This keeps the authoring shape testable before a WPILib adapter is added, while
still covering simple command composition.

## Drive Commands

File: `example-project/src/main/java/ca/frc6390/athena/examples/DriveCommandExample.java`

Drive commands can be declared without importing WPILib command classes:

```java
RobotSpeeds speeds = DriveCommandExample.speeds();

CommandSpec tank = DriveCommandExample.tankDrive(speeds, leftVelocity, rightVelocity);
CommandSpec field = DriveCommandExample.fieldRelativeDrive(speeds, xVelocity, yVelocity, omega);
CommandSpec distance = DriveCommandExample.driveToLine(speeds, measuredMeters);
CommandSpec path = DriveCommandExample.followSimplePath(speeds, robotPose);
CommandSpec align = DriveCommandExample.alignToTarget(speeds, targetVisible, yawErrorRadians);
CommandSpec cameraAlign = DriveCommandExample.alignToVisionFrame(speeds, photonCamera::latestFrame);
CommandSpec score = CommandSpec.create("score").requires("shooter").toSpec();
```

Those commands write into `RobotSpeeds`, so driver, autonomous, and feedback
requests can still be blended before the real WPILib adapter turns the same
descriptor shape into robot commands. Camera-frame alignment consumes the same
generic `VisionFrame` produced by PhotonVision and Limelight adapters.

It covers:

- tank and field-relative velocity commands
- distance and waypoint-following autonomous commands
- target and camera-frame alignment feedback
- named subsystem requirements for WPILib adaptation

## Control Utilities

File: `example-project/src/main/java/ca/frc6390/athena/examples/ControlUtilityExample.java`

Controller helpers can stay dependency-free for tests and can bind directly to
WPILib HID devices in the WPILib tier:

```java
new ModifiedAxis(() -> 0.5, 0.1)
        .squared(true)
        .inverted(false);

ManualClock clock = new ManualClock();
Debouncer debouncer = new Debouncer(() -> true, 0.25, clock);

ModifiedAxis turn = ControlUtilityExample.xboxTurnAxis(driverController);

WpilibDriverStationProfile profile = ControlUtilityExample.driverStationProfile();
ModifiedAxis profiledTurn = ControlUtilityExample.profiledDriverTurnAxis(driverController::getRawAxis);
```

Filter pipelines support scalar values and simple pose snapshots:

```java
new FilteredValue(() -> 1.5).addOffset(0.25);

AtomicReference<PoseSnapshot> pose = new AtomicReference<>(new PoseSnapshot(2.0, 0.0, 0.5));
new FilteredPose(pose::get).addMovingAverage(2);
```

It covers:

- deadzone shaping
- optional axis squaring and inversion
- named driver-station profiles
- WPILib Xbox controller axis construction
- deterministic boolean gating
- cached filtered values
- axis-wise pose filtering

## Motion Limits

File: `example-project/src/main/java/ca/frc6390/athena/examples/MotionLimitsExample.java`

Motion limits combine base values and runtime providers conservatively:

```java
new MotionLimits()
        .baseDriveLimits(new MotionLimits.DriveLimits(4.5, 3.0, 2.2, 7.0))
        .driveProvider(() -> new MotionLimits.DriveLimits(3.2, 4.0, 1.9, 6.0));

TimedRunner.periodicMs(task, 20.0);
```

It covers:

- drive motion-limit aggregation
- named axis motion-limit aggregation
- invalid value normalization
- deterministic timed runner behavior

## Robot Speeds

File: `example-project/src/main/java/ca/frc6390/athena/examples/RobotSpeedsExample.java`

Robot speed blending combines named drive, autonomous, feedback, and custom
sources before drivetrain code consumes the output:

```java
new RobotSpeeds(10.0, 10.0)
        .blend(RobotSpeeds.DRIVE_SOURCE, RobotSpeeds.DRIVE_SOURCE, RobotSpeeds.AUTO_SOURCE,
                RobotSpeeds.BlendMode.AVERAGE, RobotSpeeds.SpeedAxis.X, RobotSpeeds.SpeedAxis.Y)
        .blendToOutput(RobotSpeeds.FEEDBACK_SOURCE, RobotSpeeds.BlendMode.ADD, RobotSpeeds.SpeedAxis.THETA);
```

It covers:

- default drive/auto/feedback source blending
- translation averaging
- heading-assist theta override
- field-relative to robot-relative conversion
- source update timestamps

## Autonomous

File: `example-project/src/main/java/ca/frc6390/athena/examples/AutoExample.java`

Autonomous routines are generic chooser descriptors. They can use explicit
commands or load commands from a registered source key:

```java
Autos.chooser()
        .routine("leave", routine -> routine
                .displayName("Leave Community")
                .command(CommandSpec.create("leave").toSpec()))
        .routine("score", routine -> routine
                .displayName("Score Preload")
                .command(CommandSpec.create("score").toSpec()))
        .defaultRoutine("leave")
        .toSpec();
```

Source-backed routine:

```java
AutoRegistry.get().register("sim", path -> CommandSpec.create("sim:" + path).toSpec());

Autos.chooser()
        .routine("leave", routine -> routine.fromSource("sim", "leave-path"))
        .toSpec();
```

Scoped input handoff:

```java
AutoExecution execution = AutoExample.handoffChooser().prepare();

AutoExample.handoffToConsumer(execution).select("consumer");

String targetMode = AutoExample.selectedTargetMode(execution);
boolean fire = AutoExample.selectedFireTrigger(execution);
```

It covers:

- chooser default validation
- routine metadata
- command retrieval from selected routine
- source-key lookup with clear missing dependency guidance
- scoped string, boolean, and numeric input handoff between routines

## Auto Vendor Adapters

File: `example-project/src/main/java/ca/frc6390/athena/examples/AutoVendorAdapterExample.java`

PathPlanner and Choreo adapters register named auto sources. PathPlanner can
load real WPILib commands through PathPlannerLib `AutoBuilder`; Choreo can load
real trajectories through ChoreoLib `Choreo.loadTrajectory(String)` and wrap
Choreo `AutoFactory` commands. Generic auto declarations still reference source
keys:

```java
PathPlannerAutos.register(AutoRegistry.get());
ChoreoAutos.register(AutoRegistry.get());

Autos.chooser()
        .routine("pathplannerLeave", routine -> routine
                .fromSource(PathPlannerAutoSource.KEY, "LeaveCommunity"))
        .routine("choreoScore", routine -> routine
                .fromSource(ChoreoAutoSource.KEY, "ScorePreload"))
        .defaultRoutine("pathplannerLeave")
        .toSpec();

Command trajectory = AutoVendorAdapterExample.choreoTrajectoryCommand(choreoFactoryAdapter);
Command routine = AutoVendorAdapterExample.choreoRoutineCommand(choreoFactoryAdapter);
```

It covers:

- PathPlanner auto source registration and real command-loading adapter
- Choreo auto source registration, real trajectory-loading adapter, and
  AutoFactory command wrapper
- shared `AutoRegistry` integration
- source-backed chooser syntax

## Telemetry

File: `example-project/src/main/java/ca/frc6390/athena/examples/RobotTelemetry.java`

Telemetry declarations use typed keys:

```java
public static final TelemetryKey INTAKE_RUNNING = TelemetryKey.bool("intake/running");
public static final TelemetryKey SHOOTER_TARGET_RPM = TelemetryKey.number("shooter/targetRpm");
```

The registry can publish to a sink or capture a snapshot for tests. The
NetworkTables-shaped sink normalizes paths under `/Athena`:

```java
InMemoryNetworkTableWriter writer = new InMemoryNetworkTableWriter();
RobotTelemetry.create(true, 4500.0)
        .publishAll(new NetworkTablesTelemetrySink(writer));
```

## Diagnostics

File: `example-project/src/main/java/ca/frc6390/athena/examples/DiagnosticsExample.java`

Diagnostics channels combine summary fields with bounded event history:

```java
DiagnosticsChannel channel = new DiagnosticsChannel("shooter", 8)
        .summary("state", "speaker")
        .summary("targetRpm", 4600);

channel.info("enabled");
channel.warn("below target velocity");
channel.error("flywheel encoder disconnected");

DiagnosticsSnapshot snapshot = channel.snapshot(4);
```

It covers:

- bounded event logs
- monotonic event sequencing
- summary metadata
- health snapshots for dashboards and tests

## Dashboard Bridge

File: `example-project/src/main/java/ca/frc6390/athena/examples/DashboardBridgeExample.java`

Dashboard integrations consume telemetry and diagnostics snapshots through a
separate bridge artifact:

```java
DashboardPublisher publisher = new DashboardPublisher(packets::add);
publisher.publish(
        RobotTelemetry.create(true, 4500.0).snapshot(),
        List.of(DiagnosticsExample.shooterSnapshot()));

String payload = DashboardBridgeExample.encodeSnapshot();
```

Control messages are explicit named payloads:

```java
new DashboardControlRegistry()
        .on("setMode", message -> selectedMode.set(message.fields().get("mode")))
        .dispatch(DashboardControlMessage.of("setMode", "mode", "score"));
```

The optional TCP transport keeps dashboard dependencies out of default robot
projects while still carrying the same packet/control JSON:

```java
DashboardTcpServer server = DashboardTcpServer.start(
        DashboardTcpServer.DEFAULT_PORT,
        controls::dispatch);

new DashboardPublisher(server).publish(
        RobotTelemetry.create(true, 4500.0).snapshot(),
        List.of(DiagnosticsExample.shooterSnapshot()));
```

It covers:

- dashboard packets
- telemetry snapshot publishing
- diagnostics snapshot publishing
- explicit dashboard control dispatch
- dependency-free JSON payload encoding
- optional TCP dashboard transport

## Sensors

File: `example-project/src/main/java/ca/frc6390/athena/examples/SensorExample.java`

Digital sensor wrappers lower to generic input specs while preserving
sensor-specific metadata:

```java
Sensors.sensor("arm", "lowerLimit", sensor -> sensor
        .limitSwitch(2)
        .hardstop(BlockDirection.NEGATIVE, -12.5));

Sensors.sensor("intake", "loaded", sensor -> sensor.beamBreak(0));
```

Camera targeting uses the same `VisionFrame` data model through a target view:

```java
CameraTargetView target = new CameraTargetView(VisionExample.sampleFrame());
target.tagId();
target.yawDegrees();
```

It covers:

- limit-switch hardstop metadata
- block-direction multipliers
- inverted button and beam-break semantics
- no-target camera accessors returning empty optionals

## Vision

File: `example-project/src/main/java/ca/frc6390/athena/examples/VisionExample.java`

The vision example declares a generic camera and selects the best target from a
frame without importing PhotonVision or Limelight classes:

```java
CameraSpec front = Cameras.camera("front", camera -> camera
        .hardware(RobotHardware.FRONT_CAMERA)
        .mountPose(0.24, 0.0, 0.62, 0.0, -18.0, 0.0));

VisionFrame frame = VisionFrame.of(
        VisionObservation.tag(3, 8.5, -2.0, 4.1, 0.7),
        VisionObservation.tag(7, -3.2, -1.5, 2.4, 0.92));
```

It covers:

- camera aliases
- robot-relative camera mount pose
- finite target validation
- best-target selection by confidence, distance, then translation

## Vision Vendor Adapters

File: `example-project/src/main/java/ca/frc6390/athena/examples/VisionVendorAdapterExample.java`

Vendor camera adapters translate vendor data into the generic `VisionFrame`
model used by the rest of Athena. PhotonVision can read real PhotonLib pipeline
results, and Limelight can read real NetworkTables entries. The static target
helpers keep examples hardware-free:

```java
VisionFrame photon = PhotonVisionCameraAdapter.frameFromTargets(List.of(
        PhotonVisionTarget.aprilTag(7, -3.2, -1.5, 2.4, 0.08)));

VisionFrame limelight = LimelightCameraAdapter.frameFromTarget(
        LimelightTarget.aprilTag(7, -3.2, -1.5, 2.4, 0.92));

VisionFrame liveLimelight = new LimelightCameraAdapter("limelight-front").latestFrame();
```

It covers:

- PhotonVision target translation
- real PhotonLib pipeline-result conversion
- Limelight target translation
- real Limelight NetworkTables conversion
- generic `VisionFrame` output
- vendor adapter artifact boundaries

## Localization

File: `example-project/src/main/java/ca/frc6390/athena/examples/LocalizationExample.java`

Localization declarations configure pose-estimation behavior without importing
WPILib estimator classes or camera vendor APIs:

```java
Localizations.localization("robotPose", localization -> localization
        .vision(vision -> vision
                .standardDeviations(0.8, 0.8, 0.65)
                .multiTagScale(0.45))
        .slip(slip -> slip.enabled(0.22, 0.4))
        .fieldBounds("chargedUp", 0.0, 0.0, 16.54, 8.02)
        .poseAlias("subwooferCenter", 1.36, 5.55, 0.0));

VisionPoseEstimate cameraEstimate = LocalizationExample.cameraEstimate();
```

It covers:

- vision standard deviations
- multi-tag measurement scaling
- slip detection thresholds
- named field bounds
- autonomous pose aliases
- camera-derived pose estimates from generic `VisionFrame` observations
- WPILib pose-estimator measurement weights through the optional adapter

## Simulation

File: `example-project/src/main/java/ca/frc6390/athena/examples/SimulationExample.java`

The simulation example creates a lightweight world with motor, IMU, vision,
mechanism, and drivetrain state:

```java
SimWorld world = new SimWorld();
world.motor("drive.left").percentOutput(0.5).velocityPerSecond(2.0);
world.imu("robot").yawRateDegreesPerSecond(45.0);
world.camera(VisionExample.FRONT_CAMERA)
        .frame(VisionFrame.of(VisionObservation.tag(7, -3.2, -1.5, 2.4, 0.92)));
new SimMechanism(world, ShooterExample.CONFIG.toSpec()).applyState("speaker");
new SimDifferentialDrive(world, DriveExample.CONFIG.toSpec()).tankVelocity(2.0, 1.5);
new SimSwerveDrive(world, SwerveExample.CONFIG.toSpec()).driveAll(3.0, 45.0);
world.step(0.5);
```

It covers:

- stateful motor simulation
- IMU yaw integration
- simulated vision frames
- mechanism state coupling into simulated motor state
- differential and swerve drivetrain targets
- deterministic test-time stepping

## Vendor Options

File: `example-project/src/main/java/ca/frc6390/athena/examples/VendorOptionsExample.java`

Vendor-specific options are typed extension objects. They compile only when the
matching Athena vendor adapter artifact is present, but generic Athena hardware
code stores them without importing vendor classes:

```java
MotorConfig.create()
        .hardware(AthenaMotor.TALON_FX, 12)
        .canbus("canivore")
        .vendor(CtreMotorOptions.class, ctre -> ctre
                .supplyCurrentLimit(50)
                .statorCurrentLimit(80)
                .torqueCurrentLimit(120))
        .toSpec("shooter", "leader");
```

REV options use the same shape:

```java
MotorConfig.create()
        .hardware(AthenaMotor.SPARK_FLEX_BRUSHLESS, 21)
        .vendor(RevMotorOptions.class, rev -> rev
                .smartCurrentLimit(50)
                .openLoopRampSeconds(0.2))
        .toSpec("arm", "pivot");
```

CTRE CANcoder declarations stay in the generic encoder spec shape while the CTRE
adapter owns the runtime device:

```java
EncoderConfig.create()
        .hardware(AthenaEncoder.CANCODER, 22)
        .canbus("canivore")
        .absolutePosition()
        .offset(0.125)
        .toSpec("swerve.frontLeft", "steer");
```

REV through-bore encoders use the same encoder spec shape; the REV adapter reads
them as WPILib duty-cycle inputs:

```java
EncoderConfig.create()
        .hardware(AthenaEncoder.REV_THROUGH_BORE, 7)
        .absolutePosition()
        .toSpec("arm", "absolute");
```

It covers:

- typed vendor-specific configuration
- compile-time artifact boundaries for vendor escape hatches
- real CTRE Phoenix 6 TalonFX and REVLib Spark motor adapter placement
- real TalonFX and Spark integrated encoder read placement
- real Spark attached absolute encoder read placement
- real CTRE Phoenix 6 CANcoder encoder adapter placement
- real REV through-bore duty-cycle encoder adapter placement
- generic spec storage through `VendorOptions`

## IMU Vendor Adapters

File: `example-project/src/main/java/ca/frc6390/athena/examples/StudicaImuAdapterExample.java`

IMU adapters keep declarations generic until the optional adapter creates the
runtime vendor-backed device:

```java
ImuSpec navx = ImuConfig.create()
        .hardware(AthenaImu.NAVX, 0)
        .mountPose(0.0, 0.0, 0.0)
        .toSpec("robot", "navx");

ImuDevice device = new StudicaImuBackend().create(navx);

ImuSpec pigeon = ImuConfig.create()
        .hardware(AthenaImu.PIGEON_2, 30)
        .canbus("canivore")
        .mountPose(0.0, 0.0, 0.0)
        .toSpec("robot", "pigeon");

ImuDevice pigeonDevice = new CtreImuBackend().create(pigeon);
```

It covers:

- `AthenaImu.NAVX` as a common hardware key
- `AthenaImu.PIGEON_2` as a common hardware key
- optional Studica and CTRE adapter artifact placement
- real Studica/NavX yaw reads without importing Studica classes into generic
  hardware declarations
- real CTRE Pigeon2 yaw reads without importing Phoenix classes into generic
  hardware declarations

## WPILib Boundary

File: `example-project/src/main/java/ca/frc6390/athena/examples/WpilibBoundaryExample.java`

The WPILib adapter converts generic Athena specs into real WPILib objects
inside the optional `athena-wpilib` artifact:

```java
CommandSpec scoreSpec = CommandSpec.create("score")
        .requires("shooter")
        .onExecute(cycles::incrementAndGet)
        .until(() -> cycles.get() >= 2)
        .toSpec();

Command command = WpilibCommandAdapter.adapt(scoreSpec, Map.of("shooter", shooterSubsystem));
Command scheduled = new WpilibCommandScheduler().schedule(scoreSpec, Map.of("shooter", shooterSubsystem));
Command bound = WpilibTriggerBindings.onTrue(driverA, scoreSpec, Map.of("shooter", shooterSubsystem));

RobotLifecycleSpec lifecycle = new RobotLifecycleConfig()
        .onInit(RobotMode.AUTONOMOUS, events::incrementAndGet)
        .onPeriodic(RobotMode.TELEOP, events::incrementAndGet)
        .toSpec();

AthenaTimedRobot robot = new AthenaTimedRobot(lifecycle);
```

Telemetry can publish to a real WPILib NetworkTables instance:

```java
registry.publishAll(new NetworkTablesTelemetrySink(
        WpilibNetworkTableWriter.forDefaultInstance()));
```

Drive commands can also bind `RobotSpeeds` output to a WPILib differential drive
or swerve drive runtime adapter:

```java
WpilibDifferentialDriveAdapter adapter = WpilibDifferentialDriveAdapter.create(
        speeds,
        DriveExample.CONFIG.toSpec(),
        leftMotor,
        rightMotor,
        4.5);

adapter.periodic();
```

```java
WpilibSwerveDriveAdapter swerve = WpilibSwerveDriveAdapter
        .builder(speeds, SwerveExample.CONFIG.toSpec(), 4.5)
        .module((module, state) -> applyModuleState(module.name(), state))
        .module((module, state) -> applyModuleState(module.name(), state))
        .module((module, state) -> applyModuleState(module.name(), state))
        .module((module, state) -> applyModuleState(module.name(), state))
        .build();

swerve.periodic();
```

Localization specs can also configure WPILib estimator vision measurements:

```java
WpilibPoseEstimatorAdapter localization = new WpilibPoseEstimatorAdapter(
        LocalizationExample.ROBOT_POSE,
        poseEstimator);

localization.addVisionMeasurement(cameraPose, timestampSeconds, visibleTagCount);
```

It covers:

- real WPILib command objects
- scheduler facade for `CommandSpec`
- trigger binding helpers for command specs
- named subsystem requirement mapping
- differential-drive runtime output
- swerve module-state runtime output
- pose-estimator vision measurement standard deviations
- TimedRobot mode init and periodic hooks
- NetworkTables writer backed by `NetworkTableInstance`
- optional adapter-artifact placement

## Coverage Roadmap

The broader implementation status is tracked in
[Coverage Roadmap](./coverage-roadmap.md).

## Validation Pattern

All examples lower through `toSpec()` and validate with an explicit simulation
backend in tests:

```java
var context = AthenaValidationContext.withBackends(BackendRegistry.of(new SimMotorBackend()));
var report = ShooterExample.CONFIG.toSpec().validate(context);
report.assertValid();
```

This proves the syntax, lowerer path, and backend capability checks without
needing robot hardware or vendor installs.
