package ca.frc6390.athena.examples;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;

import org.junit.jupiter.api.Test;

import ca.frc6390.athena.commands.CommandRunner;
import ca.frc6390.athena.auto.AutoRegistry;
import ca.frc6390.athena.hardware.backend.BackendRegistry;
import ca.frc6390.athena.hardware.spec.AthenaValidationContext;
import ca.frc6390.athena.mechanism.spec.ControlMode;
import ca.frc6390.athena.runtime.control.ManualClock;
import ca.frc6390.athena.runtime.filter.PoseSnapshot;
import ca.frc6390.athena.sim.hardware.SimMotorBackend;
import ca.frc6390.athena.telemetry.TelemetryValue;
import ca.frc6390.athena.vendor.choreo.ChoreoAutoFactoryAdapter;
import ca.frc6390.athena.vendor.ctre.CtreMotorOptions;
import ca.frc6390.athena.vendor.rev.RevMotorOptions;
import ca.frc6390.athena.vision.spec.VisionFrame;
import ca.frc6390.athena.vision.spec.VisionObservation;
import ca.frc6390.athena.wpilib.commands.WpilibCommandScheduler;
import ca.frc6390.athena.wpilib.commands.WpilibTriggerBindings;
import edu.wpi.first.wpilibj2.command.Command;

class ExampleProjectTest {
    @Test
    void intakeExampleValidatesWithSimulationBackend() {
        var context = AthenaValidationContext.withBackends(BackendRegistry.of(new SimMotorBackend()));
        var report = IntakeExample.CONFIG.toSpec().validate(context);

        assertFalse(report.hasErrors());
        report.assertValid();
    }

    @Test
    void shooterExampleValidatesWithSimulationBackend() {
        var context = AthenaValidationContext.withBackends(BackendRegistry.of(new SimMotorBackend()));
        var report = ShooterExample.CONFIG.toSpec().validate(context);

        assertFalse(report.hasErrors());
    }

    @Test
    void mechanismV2ExampleCoversFluentMechanismShapes() {
        var context = AthenaValidationContext.withBackends(BackendRegistry.of(new SimMotorBackend()));

        for (var mechanism : MechanismV2Example.all()) {
            assertFalse(mechanism.toSpec().validate(context).hasErrors());
        }

        var roller = MechanismV2Example.simpleRoller().toSpec();
        var arm = MechanismV2Example.armPivot().toSpec();
        var elevator = MechanismV2Example.elevator().toSpec();
        var flywheel = MechanismV2Example.flywheel().toSpec();
        var turret = MechanismV2Example.turret().toSpec();

        assertEquals(ControlMode.PERCENT_OUTPUT, roller.controlMode());
        assertEquals("armAbsolute", arm.positionSource());
        assertEquals("carriageEncoder", elevator.positionSource());
        assertEquals("flywheelEncoder", flywheel.velocitySource());
        assertEquals("targetVisible", turret.inputs().get(1).name());

        var runtimeTargets = MechanismV2Example.runtimeStateTargets();
        assertEquals(0.65, runtimeTargets.get("roller"), 1.0e-9);
        assertEquals(72.0, runtimeTargets.get("arm"), 1.0e-9);
        assertEquals(544.5, runtimeTargets.get("flywheel"), 1.0e-9);
    }

    @Test
    void drivetrainExampleValidatesWithSimulationBackend() {
        var context = AthenaValidationContext.withBackends(BackendRegistry.of(new SimMotorBackend()));
        var report = DriveExample.CONFIG.toSpec().validate(context);

        assertFalse(report.hasErrors());
    }

    @Test
    void swerveExampleValidatesWithSimulationBackend() {
        var context = AthenaValidationContext.withBackends(BackendRegistry.of(new SimMotorBackend()));
        var report = SwerveExample.CONFIG.toSpec().validate(context);

        assertFalse(report.hasErrors());
    }

    @Test
    void superstructureExampleValidatesWithSimulationBackend() {
        var context = AthenaValidationContext.withBackends(BackendRegistry.of(new SimMotorBackend()));
        var report = SuperstructureExample.CONFIG.toSpec().validate(context);

        assertFalse(report.hasErrors());
    }

    @Test
    void compositeSuperstructureExampleValidatesNestedTargets() {
        var context = AthenaValidationContext.withBackends(BackendRegistry.of(new SimMotorBackend()));
        var spec = CompositeSuperstructureExample.ROBOT.toSpec();
        var report = spec.validate(context);
        var plan = CompositeSuperstructureExample.scoreSpeakerPlan();

        assertFalse(report.hasErrors());
        assertEquals("handoff", spec.parts().get(0).name());
        assertEquals("feeding", spec.states().get(1).partTargets().get("handoff"));
        assertEquals("handoff.intake", plan.targets().get(0).path());
        assertEquals("feed", plan.targets().get(0).stateName());
        assertEquals("handoff.shooter", plan.targets().get(1).path());
        var applied = CompositeSuperstructureExample.applyScoreSpeaker();
        assertEquals(0.65, applied.get("handoff.intake"), 1.0e-9);
        assertEquals(4600.0, applied.get("handoff.shooter"), 1.0e-9);

        var commandTargets = new java.util.LinkedHashMap<String, Double>();
        var command = CompositeSuperstructureExample.scoreSpeakerCommand(commandTargets);
        assertTrue(command.requirements().contains("superstructure"));
        assertTrue(new CommandRunner(command).step());
        assertEquals(0.65, commandTargets.get("handoff.intake"), 1.0e-9);
        assertEquals(4600.0, commandTargets.get("handoff.shooter"), 1.0e-9);
    }

    @Test
    void turretSuperstructureExampleCoordinatesAssemblyTargets() {
        var context = AthenaValidationContext.withBackends(BackendRegistry.of(new SimMotorBackend()));
        var spec = TurretSuperstructureExample.ASSEMBLY.toSpec();
        var report = spec.validate(context);

        assertFalse(report.hasErrors());
        assertEquals(3, spec.parts().size());
        assertEquals("speaker", spec.states().get(1).partTargets().get("turret"));
        assertEquals("speaker", spec.states().get(1).partTargets().get("hood"));
        assertEquals("speaker", spec.states().get(1).partTargets().get("shooter"));
        assertEquals(3, TurretSuperstructureExample.speakerPlan((current, target) -> true).targets().size());
    }

    @Test
    void commandExampleRuns() {
        AtomicInteger cycles = new AtomicInteger();
        CommandRunner runner = new CommandRunner(RobotCommands.runIntake(cycles::incrementAndGet));

        runner.step();
        runner.step();

        assertEquals(2, cycles.get());
    }

    @Test
    void commandExampleComposesScoreSequence() {
        AtomicInteger shooter = new AtomicInteger();
        AtomicInteger feeder = new AtomicInteger();
        CommandRunner runner = new CommandRunner(RobotCommands.scoreSequence(
                shooter::incrementAndGet,
                feeder::incrementAndGet));

        assertFalse(runner.step());
        assertEquals(1, shooter.get());
        assertEquals(0, feeder.get());

        assertEquals(true, runner.step());
        assertEquals(1, shooter.get());
        assertEquals(1, feeder.get());
    }

    @Test
    void driveCommandExampleWritesRobotSpeeds() {
        var speeds = DriveCommandExample.speeds();
        var runner = new CommandRunner(DriveCommandExample.tankDrive(speeds, () -> 1.0, () -> 2.24));

        runner.step();

        assertEquals(1.62, speeds.calculate().xMetersPerSecond(), 1.0e-9);
        assertEquals(2.0, speeds.calculate().angularRadiansPerSecond(), 1.0e-9);
    }

    @Test
    void driveCommandExampleSupportsFieldRelativeDistanceAndVisionCommands() {
        var fieldSpeeds = DriveCommandExample.speeds();
        var distanceSpeeds = DriveCommandExample.speeds();
        var pathSpeeds = DriveCommandExample.speeds();
        var visionSpeeds = DriveCommandExample.speeds();
        AtomicReference<Double> measured = new AtomicReference<>(0.0);
        AtomicReference<Double> yaw = new AtomicReference<>(0.2);
        AtomicReference<PoseSnapshot> pose = new AtomicReference<>(new PoseSnapshot(0.0, 0.0, 0.0));

        new CommandRunner(DriveCommandExample.fieldRelativeDrive(fieldSpeeds, () -> 1.5, () -> 0.0, () -> 0.4)).step();
        assertEquals(0.0, fieldSpeeds.calculate(Math.PI / 2.0).xMetersPerSecond(), 1.0e-9);
        assertEquals(-1.5, fieldSpeeds.calculate(Math.PI / 2.0).yMetersPerSecond(), 1.0e-9);

        var distanceRunner = new CommandRunner(DriveCommandExample.driveToLine(distanceSpeeds, measured::get));
        assertFalse(distanceRunner.step());
        assertEquals(1.2, distanceSpeeds.calculate().xMetersPerSecond(), 1.0e-9);
        measured.set(2.4);
        assertEquals(true, distanceRunner.step());

        var pathRunner = new CommandRunner(DriveCommandExample.followSimplePath(pathSpeeds, pose::get));
        assertFalse(pathRunner.step());
        assertEquals(1.2, pathSpeeds.calculate().xMetersPerSecond(), 1.0e-9);
        pose.set(new PoseSnapshot(1.5, 0.0, 0.0));
        assertFalse(pathRunner.step());
        assertFalse(pathRunner.step());
        assertEquals(1.0, pathSpeeds.calculate().yMetersPerSecond(), 1.0e-9);

        var visionRunner = new CommandRunner(DriveCommandExample.alignToTarget(visionSpeeds, () -> true, yaw::get));
        assertFalse(visionRunner.step());
        assertEquals(-0.5, visionSpeeds.calculate().angularRadiansPerSecond(), 1.0e-9);
        yaw.set(0.02);
        assertEquals(true, visionRunner.step());

        var frameSpeeds = DriveCommandExample.speeds();
        var frameRunner = new CommandRunner(DriveCommandExample.alignToVisionFrame(
                frameSpeeds,
                () -> VisionFrame.of(VisionObservation.tag(4, 8.0, 0.0, 3.0, 0.9))));
        assertFalse(frameRunner.step());
        assertEquals(Math.toRadians(8.0) * -2.5, frameSpeeds.calculate().angularRadiansPerSecond(), 1.0e-9);
    }

    @Test
    void controlUtilityExampleShapesGatesAndFiltersInputs() {
        double shaped = ControlUtilityExample.driverTurnAxis().getAsDouble();
        ManualClock clock = new ManualClock();
        var debouncer = ControlUtilityExample.heldButton(clock);
        var filteredValue = ControlUtilityExample.offsetTarget();
        AtomicReference<PoseSnapshot> pose = new AtomicReference<>(new PoseSnapshot(2.0, 0.0, 0.5));
        var filteredPose = ControlUtilityExample.smoothedPose(pose);

        assertEquals(Math.pow((0.5 - 0.1) / (1.0 - 0.1), 2.0), shaped, 1.0e-9);
        assertEquals(Math.pow((0.55 - 0.1) / (1.0 - 0.1), 2.0),
                ControlUtilityExample.hidTurnAxis(axis -> 0.55).getAsDouble(),
                1.0e-9);
        assertEquals(0, ControlUtilityExample.driverStationProfile().port("driver"));
        assertEquals(1, ControlUtilityExample.driverStationProfile().port("operator"));
        assertEquals(Math.pow((0.55 - 0.1) / (1.0 - 0.1), 2.0),
                ControlUtilityExample.profiledDriverTurnAxis(axis -> 0.55).getAsDouble(),
                1.0e-9);
        assertFalse(debouncer.getAsBoolean());
        clock.advance(0.25);
        assertEquals(true, debouncer.getAsBoolean());
        assertEquals(1.75, filteredValue.getFiltered(), 1.0e-9);
        assertEquals(new PoseSnapshot(2.0, 0.0, 0.5), filteredPose.getFiltered());
    }

    @Test
    void motionLimitsExampleResolvesLimitsAndTimedRunner() {
        var drive = MotionLimitsExample.driveLimitProfile().resolveDrive();
        var arm = MotionLimitsExample.axisLimitProfile().resolveAxis("arm");
        AtomicInteger runs = new AtomicInteger();
        var runner = MotionLimitsExample.every20Ms(runs::incrementAndGet);

        assertEquals(3.2, drive.maxLinearVelocity(), 1.0e-9);
        assertEquals(2.5, drive.maxLinearAcceleration(), 1.0e-9);
        assertEquals(1.5, arm.maxVelocity(), 1.0e-9);
        assertEquals(2.0, arm.maxAcceleration(), 1.0e-9);
        assertEquals(true, runner.run(Runnable::run, 0.0));
        assertEquals(false, runner.run(Runnable::run, 0.01));
        assertEquals(1, runs.get());
    }

    @Test
    void robotSpeedsExampleBlendsSourcesAndFieldRelativeInput() {
        var averaged = RobotSpeedsExample.driveAutoAverageForTranslation()
                .setSpeeds(ca.frc6390.athena.runtime.control.RobotSpeeds.DRIVE_SOURCE, 2.0, 4.0, 0.6)
                .setSpeeds(ca.frc6390.athena.runtime.control.RobotSpeeds.AUTO_SOURCE, 0.0, 2.0, 1.0)
                .setSpeeds(ca.frc6390.athena.runtime.control.RobotSpeeds.FEEDBACK_SOURCE, 0.0, 0.0, 0.25)
                .calculate();
        var assist = RobotSpeedsExample.headingAssistOverride()
                .setSpeeds(ca.frc6390.athena.runtime.control.RobotSpeeds.DRIVE_SOURCE, 0.0, 0.0, 0.4)
                .setSpeeds("assist", 0.0, 0.0, 1.3)
                .calculate();
        var fieldRelative = RobotSpeedsExample.fieldRelativeExample();

        assertEquals(1.0, averaged.xMetersPerSecond(), 1.0e-9);
        assertEquals(3.0, averaged.yMetersPerSecond(), 1.0e-9);
        assertEquals(0.85, averaged.angularRadiansPerSecond(), 1.0e-9);
        assertEquals(1.3, assist.angularRadiansPerSecond(), 1.0e-9);
        assertEquals(0.0, fieldRelative.xMetersPerSecond(), 1.0e-9);
        assertEquals(-2.0, fieldRelative.yMetersPerSecond(), 1.0e-9);
    }

    @Test
    void autoExampleSelectsDefaultCommand() {
        var execution = AutoExample.chooser().prepare();

        assertEquals("leave", execution.selectedRoutine().id());
        assertEquals("leave", execution.selectedCommand().name());
    }

    @Test
    void autoExampleLoadsFromRegisteredSource() {
        AutoRegistry.get().clear();
        try {
            var execution = AutoExample.sourcedChooser().prepare();

            assertEquals("sim:leave-path", execution.selectedCommand().name());
        } finally {
            AutoRegistry.get().clear();
        }
    }

    @Test
    void autoExamplePassesScopedInputsBetweenRoutines() {
        var execution = AutoExample.handoffChooser().prepare();

        AutoExample.handoffToConsumer(execution).select("consumer");

        assertEquals("consumer", execution.selectedRoutine().id());
        assertEquals("amp", AutoExample.selectedTargetMode(execution));
        assertEquals(true, AutoExample.selectedFireTrigger(execution));

        execution.inputs().clearScope("consumer");
        assertEquals("speaker", AutoExample.selectedTargetMode(execution));
        assertEquals(false, AutoExample.selectedFireTrigger(execution));
    }

    @Test
    void autoVendorAdapterExampleLoadsPathToolCommands() {
        AutoRegistry.get().clear();
        try {
            var execution = AutoVendorAdapterExample.chooser().prepare();

            assertEquals("pathplannerLeave", execution.selectedRoutine().id());
            assertEquals("pathplanner:LeaveCommunity", execution.selectedCommand().name());
            assertEquals("choreo:ScorePreload", AutoRegistry.get().require("choreo").load("ScorePreload").name());
        } finally {
            AutoRegistry.get().clear();
        }
    }

    @Test
    void autoVendorAdapterExampleCreatesChoreoFactoryCommands() {
        var client = new RecordingChoreoFactoryClient();
        var adapter = new ChoreoAutoFactoryAdapter(client);

        assertSame(client.trajectoryCommand, AutoVendorAdapterExample.choreoTrajectoryCommand(adapter));
        assertEquals("ScorePreload", client.trajectoryName);

        assertSame(client.routineCommand, AutoVendorAdapterExample.choreoRoutineCommand(adapter));
        assertEquals("TwoPiece", client.routineName);
    }

    @Test
    void telemetryExampleCapturesValues() {
        var snapshot = RobotTelemetry.create(true, 4500.0).snapshot();

        assertEquals(TelemetryValue.of(true), snapshot.find(RobotTelemetry.INTAKE_RUNNING).orElseThrow());
        assertEquals(TelemetryValue.of(4500.0), snapshot.find(RobotTelemetry.SHOOTER_TARGET_RPM).orElseThrow());
    }

    @Test
    void telemetryExamplePublishesNetworkTablePaths() {
        var writer = RobotTelemetry.publishToNetworkTables(true, 4500.0);

        assertEquals(true, writer.values().get("/Athena/intake/running"));
        assertEquals(4500.0, writer.values().get("/Athena/shooter/targetRpm"));
    }

    @Test
    void diagnosticsExampleCapturesBoundedHealthSnapshot() {
        var snapshot = DiagnosticsExample.shooterSnapshot();

        assertEquals("shooter", snapshot.channel());
        assertEquals("speaker", snapshot.summary().get("state"));
        assertEquals(3, snapshot.events().size());
        assertEquals("flywheel encoder disconnected", snapshot.events().get(2).message());
    }

    @Test
    void dashboardBridgeExamplePublishesSnapshotsAndHandlesControls() {
        var packets = DashboardBridgeExample.publishSnapshot();

        assertEquals(1, packets.size());
        assertEquals(true, packets.get(0).telemetry().find(RobotTelemetry.INTAKE_RUNNING).orElseThrow().value());
        assertEquals(true, packets.get(0).hasErrors());
        assertEquals("score", DashboardBridgeExample.dispatchControl());
        assertEquals(true, DashboardBridgeExample.encodeSnapshot().contains("\"hasErrors\":true"));
        assertEquals("score", DashboardBridgeExample.decodeControlPayload().fields().get("mode"));
    }

    @Test
    void sensorExampleCapturesDigitalAndCameraSensorViews() {
        var lowerLimit = SensorExample.armLowerLimit();
        var loaded = SensorExample.intakeLoaded();
        var target = SensorExample.cameraTarget();

        assertEquals(-1, lowerLimit.blockDirection().multiplier());
        assertEquals(-12.5, lowerLimit.position());
        assertEquals(0, loaded.input().channel());
        assertEquals(7, target.tagId().orElseThrow());
    }

    @Test
    void visionExampleSelectsBestTarget() {
        assertFalse(VisionExample.FRONT_CAMERA.validate().hasErrors());

        var frame = VisionExample.sampleFrame();

        assertEquals(7, frame.tagId().orElseThrow());
        assertEquals(-3.2, frame.yawDegrees().orElseThrow());
    }

    @Test
    void localizationExampleCapturesPoseEstimatorSettings() {
        var spec = LocalizationExample.ROBOT_POSE;
        var cameraEstimate = LocalizationExample.cameraEstimate();

        assertFalse(spec.validate().hasErrors());
        assertEquals(0.36, spec.visionWeight().forTagCount(2).xStdDevMeters(), 1.0e-9);
        assertEquals(0.22, spec.slipDetection().lateralVelocityMetersPerSecond(), 1.0e-9);
        assertEquals(1.36, spec.findPoseAlias("subwooferCenter").orElseThrow().pose().xMeters(), 1.0e-9);
        assertEquals(2.7, cameraEstimate.pose().xMeters(), 1.0e-9);
        assertEquals(2.0, cameraEstimate.pose().yMeters(), 1.0e-9);
        assertEquals(12.5, cameraEstimate.timestampSeconds(), 1.0e-9);
        assertEquals(0.8, cameraEstimate.standardDeviations().xStdDevMeters(), 1.0e-9);
    }

    @Test
    void visionVendorAdapterExampleConvertsVendorTargets() {
        var photon = VisionVendorAdapterExample.photonFrame();
        var limelight = VisionVendorAdapterExample.limelightFrame();

        assertEquals(7, photon.tagId().orElseThrow());
        assertEquals(0.92, photon.bestTarget().orElseThrow().confidence(), 1.0e-9);
        assertEquals(7, limelight.tagId().orElseThrow());
        assertEquals(-3.2, limelight.yawDegrees().orElseThrow(), 1.0e-9);
    }

    @Test
    void simulationExampleStepsWorldState() {
        var world = SimulationExample.createWorld();

        world.step(0.5);

        assertEquals(1.0, world.motor("drive.left").position());
        assertEquals(22.5, world.imu("robot").yawDegrees());
        assertEquals(7, world.findCamera("vision.front").orElseThrow().frame().tagId().orElseThrow());
    }

    @Test
    void simulationExampleAppliesMechanismStates() {
        var world = SimulationExample.createMechanismWorld();

        world.step(0.25);

        assertEquals(1150.0, world.motor("shooter.leader").position(), 1.0e-9);
        assertEquals(4600.0, world.motor("shooter.leader").velocityPerSecond(), 1.0e-9);
    }

    @Test
    void simulationExampleAppliesDrivetrainTargets() {
        var differential = SimulationExample.createDifferentialDriveWorld();
        var swerve = SimulationExample.createSwerveDriveWorld();

        differential.step(0.5);
        swerve.step(0.25);

        assertEquals(1.0, differential.motor("drive.left.leftLeader").position(), 1.0e-9);
        assertEquals(0.75, differential.motor("drive.right.rightLeader").position(), 1.0e-9);
        assertEquals(0.75, swerve.motor("swerve.frontLeft.drive").position(), 1.0e-9);
        assertEquals(45.0, swerve.motor("swerve.frontLeft.steer").position(), 1.0e-9);
    }

    @Test
    void studicaImuAdapterExampleDeclaresNavxDevice() {
        var navx = StudicaImuAdapterExample.NAVX;
        var pigeon = StudicaImuAdapterExample.PIGEON;

        assertEquals("studica:navx", navx.kind().key());
        assertEquals("robot.navx", navx.path());
        assertEquals("ctre:pigeon-2", pigeon.kind().key());
        assertEquals("canivore", pigeon.canbus());
    }

    @Test
    void vendorOptionsExampleKeepsTypedVendorOptionsInSpecs() {
        var ctreSpec = VendorOptionsExample.ctreShooterLeader();
        var ctre = ctreSpec
                .vendorOptions()
                .find(CtreMotorOptions.class)
                .orElseThrow();
        var rev = VendorOptionsExample.revArmPivot()
                .vendorOptions()
                .find(RevMotorOptions.class)
                .orElseThrow();
        var encoder = VendorOptionsExample.ctreSteerEncoder();

        assertEquals("ctre:kraken-x60", ctreSpec.kind().key());
        assertEquals(80, ctre.statorCurrentLimitAmps());
        assertEquals(50, ctre.supplyCurrentLimitAmps());
        assertEquals(50, rev.smartCurrentLimitAmps());
        assertEquals(0.2, rev.openLoopRampSeconds(), 1.0e-9);
        assertEquals("ctre:cancoder", encoder.kind().key());
        assertEquals(0.125, encoder.offset(), 1.0e-9);
    }

    @Test
    void wpilibBoundaryExampleAdaptsCommandsLifecycleAndTelemetry() {
        AtomicInteger cycles = new AtomicInteger();
        var command = WpilibBoundaryExample.adaptedCommand(cycles);

        command.initialize();
        command.execute();
        command.execute();

        assertEquals(2, cycles.get());
        assertEquals(true, command.isFinished());
        assertEquals(1, command.getRequirements().size());

        var schedulerClient = new RecordingSchedulerClient();
        Command scheduled = WpilibBoundaryExample.scheduledCommand(cycles, schedulerClient);
        assertEquals("scoreScheduled", scheduled.getName());
        assertSame(scheduled, schedulerClient.scheduled);

        var triggerBinder = new RecordingTriggerBinder();
        Command triggerBound = WpilibBoundaryExample.triggerBoundCommand(cycles, triggerBinder);
        assertEquals("scoreOnTrigger", triggerBound.getName());
        assertSame(triggerBound, triggerBinder.onTrueCommand);

        AtomicInteger events = new AtomicInteger();
        var lifecycle = WpilibBoundaryExample.lifecycle(events);
        lifecycle.runInit(ca.frc6390.athena.wpilib.lifecycle.RobotMode.AUTONOMOUS);
        lifecycle.runPeriodic(ca.frc6390.athena.wpilib.lifecycle.RobotMode.TELEOP);

        assertEquals(2, events.get());
        assertEquals(true, WpilibBoundaryExample.publishTelemetry().get("/Athena/robot/enabled"));
        var driveOutputs = WpilibBoundaryExample.applyDifferentialDriveSpeeds();
        assertEquals((2.25 - 0.355) / 4.5, driveOutputs.get("left"), 1.0e-9);
        assertEquals((2.25 + 0.355) / 4.5, driveOutputs.get("right"), 1.0e-9);
        var swerveOutputs = WpilibBoundaryExample.applySwerveDriveSpeeds();
        assertEquals(2.0, swerveOutputs.get("frontLeft"), 1.0e-9);
        assertEquals(2.0, swerveOutputs.get("frontRight"), 1.0e-9);
        assertEquals(2.0, swerveOutputs.get("backLeft"), 1.0e-9);
        assertEquals(2.0, swerveOutputs.get("backRight"), 1.0e-9);
        var visionStdDevs = WpilibBoundaryExample.applyVisionMeasurement();
        assertEquals(0.8 * 0.45, visionStdDevs.get("x"), 1.0e-9);
        assertEquals(0.8 * 0.45, visionStdDevs.get("y"), 1.0e-9);
        assertEquals(0.65 * 0.45, visionStdDevs.get("heading"), 1.0e-9);
    }

    private static final class RecordingChoreoFactoryClient implements ChoreoAutoFactoryAdapter.FactoryClient {
        private final Command trajectoryCommand = new RecordingCommand("trajectory");
        private final Command splitTrajectoryCommand = new RecordingCommand("splitTrajectory");
        private final Command resetCommand = new RecordingCommand("reset");
        private final Command splitResetCommand = new RecordingCommand("splitReset");
        private final Command routineCommand = new RecordingCommand("routine");
        private final Command warmupCommand = new RecordingCommand("warmup");
        private String trajectoryName;
        private String routineName;

        @Override
        public Command trajectoryCommand(String trajectoryName) {
            this.trajectoryName = trajectoryName;
            return trajectoryCommand;
        }

        @Override
        public Command trajectoryCommand(String trajectoryName, int splitIndex) {
            this.trajectoryName = trajectoryName;
            return splitTrajectoryCommand;
        }

        @Override
        public Command resetOdometryCommand(String trajectoryName) {
            this.trajectoryName = trajectoryName;
            return resetCommand;
        }

        @Override
        public Command resetOdometryCommand(String trajectoryName, int splitIndex) {
            this.trajectoryName = trajectoryName;
            return splitResetCommand;
        }

        @Override
        public Command routineCommand(String routineName) {
            this.routineName = routineName;
            return routineCommand;
        }

        @Override
        public Command warmupCommand() {
            return warmupCommand;
        }
    }

    private static final class RecordingCommand extends Command {
        private RecordingCommand(String name) {
            setName(name);
        }
    }

    private static final class RecordingSchedulerClient implements WpilibCommandScheduler.SchedulerClient {
        private Command scheduled;

        @Override
        public void schedule(Command command) {
            scheduled = command;
        }

        @Override
        public void run() {
        }

        @Override
        public void cancel(Command command) {
        }
    }

    private static final class RecordingTriggerBinder implements WpilibTriggerBindings.TriggerBinder {
        private Command onTrueCommand;

        @Override
        public void onTrue(Command command) {
            onTrueCommand = command;
        }

        @Override
        public void whileTrue(Command command) {
        }

        @Override
        public void toggleOnTrue(Command command) {
        }
    }
}
