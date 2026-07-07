package ca.frc6390.athena.wpilib;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.LinkedHashMap;
import java.util.Map;
import java.util.concurrent.atomic.AtomicInteger;

import org.junit.jupiter.api.Test;

import ca.frc6390.athena.commands.CommandSpec;
import ca.frc6390.athena.localization.config.Localizations;
import ca.frc6390.athena.mechanism.core.MechanismContext;
import ca.frc6390.athena.mechanism.core.PathRef;
import ca.frc6390.athena.mechanism.core.PathRuntime;
import ca.frc6390.athena.mechanism.core.Paths;
import ca.frc6390.athena.runtime.control.ManualClock;
import ca.frc6390.athena.runtime.control.RobotSpeeds;
import ca.frc6390.athena.telemetry.TelemetryKey;
import ca.frc6390.athena.telemetry.TelemetryRegistry;
import ca.frc6390.athena.telemetry.networktables.NetworkTablesTelemetrySink;
import ca.frc6390.athena.wpilib.commands.WpilibCommandAdapter;
import ca.frc6390.athena.wpilib.commands.WpilibCommandPathRuntime;
import ca.frc6390.athena.wpilib.commands.WpilibCommandScheduler;
import ca.frc6390.athena.wpilib.commands.WpilibTriggerBindings;
import ca.frc6390.athena.wpilib.controllers.WpilibControllerBindings;
import ca.frc6390.athena.wpilib.controllers.WpilibDriverStationProfile;
import ca.frc6390.athena.wpilib.drivetrain.WpilibDifferentialDriveAdapter;
import ca.frc6390.athena.wpilib.drivetrain.WpilibSwerveDriveAdapter;
import ca.frc6390.athena.wpilib.lifecycle.RobotLifecycleConfig;
import ca.frc6390.athena.wpilib.lifecycle.RobotMode;
import ca.frc6390.athena.wpilib.localization.WpilibPoseEstimatorAdapter;
import ca.frc6390.athena.wpilib.networktables.WpilibNetworkTableWriter;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

class WpilibAdapterBoundaryTest {
    @Test
    void commandSpecAdaptsToWpilibCommand() {
        AtomicInteger events = new AtomicInteger();
        var command = CommandSpec.create("score")
                .requires("shooter")
                .onInitialize(events::incrementAndGet)
                .onExecute(events::incrementAndGet)
                .until(() -> events.get() >= 2)
                .onEnd(events::incrementAndGet)
                .toSpec();
        Subsystem shooter = new RecordingSubsystem();

        Command wpilibCommand = WpilibCommandAdapter.adapt(command, Map.of("shooter", shooter));

        wpilibCommand.initialize();
        wpilibCommand.execute();
        assertTrue(wpilibCommand.isFinished());
        wpilibCommand.end(false);
        assertEquals("score", wpilibCommand.getName());
        assertEquals(3, events.get());
        assertTrue(wpilibCommand.getRequirements().contains(shooter));
    }

    @Test
    void commandPathRuntimeRunsWpilibCommandLifecycle() {
        RecordingPathCommand command = new RecordingPathCommand();
        PathRef path = Paths.choreo("leave");
        PathRuntime runtime = WpilibCommandPathRuntime.of(ignored -> command);

        runtime.initialize(path, MechanismContext.empty());
        runtime.execute(path, MechanismContext.empty());
        assertFalse(runtime.isFinished(path, MechanismContext.empty()));
        command.finished = true;
        assertTrue(runtime.isFinished(path, MechanismContext.empty()));
        runtime.end(path, MechanismContext.empty(), false);

        assertEquals(1, command.initialized);
        assertEquals(1, command.executed);
        assertEquals(1, command.ended);
        assertFalse(command.interrupted);
    }

    @Test
    void commandSchedulerAdaptsSchedulesRunsAndCancelsSpecs() {
        AtomicInteger events = new AtomicInteger();
        var schedulerClient = new RecordingSchedulerClient();
        var scheduler = new WpilibCommandScheduler(schedulerClient);
        Subsystem shooter = new RecordingSubsystem();
        var spec = CommandSpec.create("score")
                .requires("shooter")
                .onExecute(events::incrementAndGet)
                .toSpec();

        Command command = scheduler.schedule(spec, Map.of("shooter", shooter));
        scheduler.run();
        scheduler.cancel(command);

        assertEquals(command, schedulerClient.scheduled);
        assertEquals(command, schedulerClient.cancelled);
        assertEquals(1, schedulerClient.runCount);
        assertEquals("score", command.getName());
        assertTrue(command.getRequirements().contains(shooter));
    }

    @Test
    void triggerBindingsAdaptSpecsByMode() {
        Subsystem shooter = new RecordingSubsystem();
        var spec = CommandSpec.create("score")
                .requires("shooter")
                .toSpec();
        var binder = new RecordingTriggerBinder();

        Command onTrue = WpilibTriggerBindings.bind(
                binder,
                WpilibTriggerBindings.TriggerMode.ON_TRUE,
                spec,
                Map.of("shooter", shooter));
        Command whileTrue = WpilibTriggerBindings.bind(
                binder,
                WpilibTriggerBindings.TriggerMode.WHILE_TRUE,
                spec,
                Map.of("shooter", shooter));
        Command toggle = WpilibTriggerBindings.bind(
                binder,
                WpilibTriggerBindings.TriggerMode.TOGGLE_ON_TRUE,
                spec,
                Map.of("shooter", shooter));

        assertEquals(onTrue, binder.onTrueCommand);
        assertEquals(whileTrue, binder.whileTrueCommand);
        assertEquals(toggle, binder.toggleCommand);
        assertEquals("score", toggle.getName());
        assertTrue(toggle.getRequirements().contains(shooter));
    }

    @Test
    void controllerBindingsWrapRuntimeHelpers() {
        ManualClock clock = new ManualClock();
        var axis = WpilibControllerBindings.axis(() -> -0.5, 0.1).squared(true).inverted(true);
        var button = WpilibControllerBindings.button(() -> true, 0.2, clock);

        assertEquals(Math.pow((0.5 - 0.1) / (1.0 - 0.1), 2.0), axis.getAsDouble(), 1.0e-9);
        assertFalse(button.getAsBoolean());
        clock.advance(0.25);
        assertTrue(button.getAsBoolean());
    }

    @Test
    void controllerBindingsWrapWpilibHidDevices() {
        ManualClock clock = new ManualClock();
        RecordingHid hid = new RecordingHid();
        hid.axis = -0.6;
        hid.button = true;

        var rawAxis = WpilibControllerBindings.axis(hid::readAxis, 2, 0.1).inverted(true);
        var xboxAxis = WpilibControllerBindings.axis(hid::readAxis, XboxController.Axis.kLeftX.value, 0.1);
        var rawButton = WpilibControllerBindings.button(hid::readButton, 1, 0.2, clock);
        var xboxButton = WpilibControllerBindings.button(hid::readButton, XboxController.Button.kA.value, 0.2, clock);

        assertEquals((0.6 - 0.1) / (1.0 - 0.1), rawAxis.getAsDouble(), 1.0e-9);
        assertEquals((-0.6 + 0.1) / (1.0 - 0.1), xboxAxis.getAsDouble(), 1.0e-9);
        assertFalse(rawButton.getAsBoolean());
        assertFalse(xboxButton.getAsBoolean());
        clock.advance(0.25);
        assertTrue(rawButton.getAsBoolean());
        assertTrue(xboxButton.getAsBoolean());
    }

    @Test
    void driverStationProfileNamesPortsAndUsesDefaultInputShaping() {
        ManualClock clock = new ManualClock();
        RecordingHid hid = new RecordingHid();
        hid.axis = 0.55;
        hid.button = true;
        var profile = WpilibDriverStationProfile.standard(0, 1)
                .role("mechanisms", 2)
                .deadzone(0.1)
                .debounceSeconds(0.2);

        var axis = profile.driverAxis(hid::readAxis, XboxController.Axis.kRightX).squared(true);
        var button = profile.operatorButton(hid::readButton, XboxController.Button.kA, clock);

        assertEquals(0, profile.port(WpilibDriverStationProfile.DRIVER));
        assertEquals(1, profile.port(WpilibDriverStationProfile.OPERATOR));
        assertEquals(2, profile.port("mechanisms"));
        assertEquals(Math.pow((0.55 - 0.1) / (1.0 - 0.1), 2.0), axis.getAsDouble(), 1.0e-9);
        assertFalse(button.getAsBoolean());
        clock.advance(0.25);
        assertTrue(button.getAsBoolean());
    }

    @Test
    void lifecycleRunsModeHooks() {
        AtomicInteger events = new AtomicInteger();
        var lifecycle = new RobotLifecycleConfig()
                .onInit(RobotMode.AUTONOMOUS, events::incrementAndGet)
                .onPeriodic(RobotMode.AUTONOMOUS, events::incrementAndGet)
                .toSpec();

        lifecycle.runInit(RobotMode.AUTONOMOUS);
        lifecycle.runPeriodic(RobotMode.AUTONOMOUS);
        lifecycle.runPeriodic(RobotMode.TELEOP);

        assertEquals(2, events.get());
    }

    @Test
    void networkTablesWriterBridgesTelemetrySink() {
        Map<String, Object> values = new LinkedHashMap<>();
        var writer = new WpilibNetworkTableWriter(new RecordingTableSink(values));
        var registry = new TelemetryRegistry()
                .booleanValue(TelemetryKey.bool("enabled"), () -> true)
                .numberValue(TelemetryKey.number("rpm"), () -> 4500.0)
                .stringValue(TelemetryKey.string("mode"), () -> "auto");

        registry.publishAll(new NetworkTablesTelemetrySink(writer));

        assertEquals(true, values.get("/Athena/enabled"));
        assertEquals(4500.0, values.get("/Athena/rpm"));
        assertEquals("auto", values.get("/Athena/mode"));
    }

    @Test
    void differentialDriveAdapterAppliesRobotSpeedsToWpilibDrive() {
        var speeds = new RobotSpeeds(4.0, 3.0);
        var left = new RecordingOutput();
        var right = new RecordingOutput();
        var adapter = WpilibDifferentialDriveAdapter.create(speeds, 0.6, left::accept, right::accept, 4.0);

        speeds.setSpeeds(RobotSpeeds.DRIVE_SOURCE, 2.0, 0.0, 1.0);
        adapter.periodic();

        assertEquals((2.0 - 0.3) / 4.0, left.value(), 1.0e-9);
        assertEquals((2.0 + 0.3) / 4.0, right.value(), 1.0e-9);

        adapter.stop();

        assertEquals(0.0, left.value(), 1.0e-9);
        assertEquals(0.0, right.value(), 1.0e-9);
    }

    @Test
    void swerveDriveAdapterAppliesRobotSpeedsToModuleStates() {
        var speeds = new RobotSpeeds(4.0, 3.0);
        var frontLeft = new RecordingModuleOutput();
        var frontRight = new RecordingModuleOutput();
        var backLeft = new RecordingModuleOutput();
        var backRight = new RecordingModuleOutput();
        var spec = ca.frc6390.athena.drivetrain.config.Drivetrains.swerve("swerve")
                .module("frontLeft", module -> module.location(0.3, 0.3)
                        .driveMotor(motor -> motor.hardware(ca.frc6390.athena.api.hardware.AthenaMotor.SIM, 1))
                        .steerMotor(motor -> motor.hardware(ca.frc6390.athena.api.hardware.AthenaMotor.SIM, 2))
                        .steerEncoder(encoder -> encoder.hardware(ca.frc6390.athena.api.hardware.AthenaEncoder.SIM, 3)))
                .module("frontRight", module -> module.location(0.3, -0.3)
                        .driveMotor(motor -> motor.hardware(ca.frc6390.athena.api.hardware.AthenaMotor.SIM, 4))
                        .steerMotor(motor -> motor.hardware(ca.frc6390.athena.api.hardware.AthenaMotor.SIM, 5))
                        .steerEncoder(encoder -> encoder.hardware(ca.frc6390.athena.api.hardware.AthenaEncoder.SIM, 6)))
                .module("backLeft", module -> module.location(-0.3, 0.3)
                        .driveMotor(motor -> motor.hardware(ca.frc6390.athena.api.hardware.AthenaMotor.SIM, 7))
                        .steerMotor(motor -> motor.hardware(ca.frc6390.athena.api.hardware.AthenaMotor.SIM, 8))
                        .steerEncoder(encoder -> encoder.hardware(ca.frc6390.athena.api.hardware.AthenaEncoder.SIM, 9)))
                .module("backRight", module -> module.location(-0.3, -0.3)
                        .driveMotor(motor -> motor.hardware(ca.frc6390.athena.api.hardware.AthenaMotor.SIM, 10))
                        .steerMotor(motor -> motor.hardware(ca.frc6390.athena.api.hardware.AthenaMotor.SIM, 11))
                        .steerEncoder(encoder -> encoder.hardware(ca.frc6390.athena.api.hardware.AthenaEncoder.SIM, 12)))
                .toSpec();
        var adapter = WpilibSwerveDriveAdapter.builder(speeds, spec, 4.0)
                .module(frontLeft)
                .module(frontRight)
                .module(backLeft)
                .module(backRight)
                .build();

        speeds.setSpeeds(RobotSpeeds.DRIVE_SOURCE, 2.0, 0.0, 0.0);
        adapter.periodic();

        assertEquals(2.0, frontLeft.state().speedMetersPerSecond, 1.0e-9);
        assertEquals(0.0, frontLeft.state().angle.getRadians(), 1.0e-9);
        assertEquals(2.0, frontRight.state().speedMetersPerSecond, 1.0e-9);
        assertEquals(2.0, backLeft.state().speedMetersPerSecond, 1.0e-9);
        assertEquals(2.0, backRight.state().speedMetersPerSecond, 1.0e-9);

        adapter.stop();

        assertEquals(0.0, frontLeft.state().speedMetersPerSecond, 1.0e-9);
    }

    @Test
    void poseEstimatorAdapterAppliesLocalizationVisionWeights() {
        var sink = new RecordingVisionSink();
        var adapter = new WpilibPoseEstimatorAdapter(
                Localizations.localization("robotPose", localization -> localization
                        .vision(vision -> vision
                                .standardDeviations(0.8, 0.8, 0.65)
                                .multiTagScale(0.45))),
                sink);

        adapter.addVisionMeasurement(new Pose2d(1.0, 2.0, new Rotation2d(0.4)), 12.5, 2);

        assertEquals(1.0, sink.pose.getX(), 1.0e-9);
        assertEquals(12.5, sink.timestamp, 1.0e-9);
        assertEquals(0.8 * 0.45, sink.standardDeviations.get(0, 0), 1.0e-9);
        assertEquals(0.8 * 0.45, sink.standardDeviations.get(1, 0), 1.0e-9);
        assertEquals(0.65 * 0.45, sink.standardDeviations.get(2, 0), 1.0e-9);
    }

    private record RecordingTableSink(Map<String, Object> values)
            implements ca.frc6390.athena.wpilib.networktables.WpilibNetworkTableSink {
        @Override
        public void setBoolean(String path, boolean value) {
            values.put(path, value);
        }

        @Override
        public void setDouble(String path, double value) {
            values.put(path, value);
        }

        @Override
        public void setString(String path, String value) {
            values.put(path, value);
        }
    }

    private static final class RecordingOutput {
        private double value;

        void accept(double value) {
            this.value = value;
        }

        double value() {
            return value;
        }
    }

    private static final class RecordingSubsystem implements Subsystem {
    }

    private static final class RecordingPathCommand extends Command {
        private int initialized;
        private int executed;
        private int ended;
        private boolean finished;
        private boolean interrupted;

        @Override
        public void initialize() {
            initialized++;
        }

        @Override
        public void execute() {
            executed++;
        }

        @Override
        public boolean isFinished() {
            return finished;
        }

        @Override
        public void end(boolean interrupted) {
            ended++;
            this.interrupted = interrupted;
        }
    }

    private static final class RecordingSchedulerClient implements WpilibCommandScheduler.SchedulerClient {
        private Command scheduled;
        private Command cancelled;
        private int runCount;

        @Override
        public void schedule(Command command) {
            scheduled = command;
        }

        @Override
        public void run() {
            runCount++;
        }

        @Override
        public void cancel(Command command) {
            cancelled = command;
        }
    }

    private static final class RecordingTriggerBinder implements WpilibTriggerBindings.TriggerBinder {
        private Command onTrueCommand;
        private Command whileTrueCommand;
        private Command toggleCommand;

        @Override
        public void onTrue(Command command) {
            onTrueCommand = command;
        }

        @Override
        public void whileTrue(Command command) {
            whileTrueCommand = command;
        }

        @Override
        public void toggleOnTrue(Command command) {
            toggleCommand = command;
        }
    }

    private static final class RecordingModuleOutput implements WpilibSwerveDriveAdapter.ModuleOutput {
        private SwerveModuleState state = new SwerveModuleState();

        @Override
        public void apply(
                ca.frc6390.athena.drivetrain.spec.SwerveModuleSpec module,
                SwerveModuleState state) {
            this.state = state;
        }

        SwerveModuleState state() {
            return state;
        }
    }

    private static final class RecordingVisionSink implements WpilibPoseEstimatorAdapter.VisionMeasurementSink {
        private Pose2d pose;
        private double timestamp;
        private Matrix<N3, N1> standardDeviations;

        @Override
        public void addVisionMeasurement(
                Pose2d pose,
                double timestampSeconds,
                Matrix<N3, N1> standardDeviations) {
            this.pose = pose;
            timestamp = timestampSeconds;
            this.standardDeviations = standardDeviations;
        }
    }

    private static final class RecordingHid {
        private double axis;
        private boolean button;

        public double readAxis(int axis) {
            return this.axis;
        }

        public boolean readButton(int button) {
            return this.button;
        }
    }
}
