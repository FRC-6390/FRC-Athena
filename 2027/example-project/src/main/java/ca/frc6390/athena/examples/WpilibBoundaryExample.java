package ca.frc6390.athena.examples;

import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.atomic.AtomicInteger;

import ca.frc6390.athena.commands.CommandSpec;
import ca.frc6390.athena.runtime.control.RobotSpeeds;
import ca.frc6390.athena.telemetry.TelemetryKey;
import ca.frc6390.athena.telemetry.TelemetryRegistry;
import ca.frc6390.athena.telemetry.networktables.NetworkTablesTelemetrySink;
import ca.frc6390.athena.wpilib.commands.WpilibCommandAdapter;
import ca.frc6390.athena.wpilib.commands.WpilibCommandScheduler;
import ca.frc6390.athena.wpilib.commands.WpilibTriggerBindings;
import ca.frc6390.athena.wpilib.drivetrain.WpilibDifferentialDriveAdapter;
import ca.frc6390.athena.wpilib.drivetrain.WpilibSwerveDriveAdapter;
import ca.frc6390.athena.wpilib.lifecycle.AthenaTimedRobot;
import ca.frc6390.athena.wpilib.lifecycle.RobotLifecycleConfig;
import ca.frc6390.athena.wpilib.lifecycle.RobotLifecycleSpec;
import ca.frc6390.athena.wpilib.lifecycle.RobotMode;
import ca.frc6390.athena.wpilib.localization.WpilibPoseEstimatorAdapter;
import ca.frc6390.athena.wpilib.networktables.WpilibNetworkTableSink;
import ca.frc6390.athena.wpilib.networktables.WpilibNetworkTableWriter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Example WPILib adapter usage.
 */
public final class WpilibBoundaryExample {
    private WpilibBoundaryExample() {
    }

    /**
     * Adapts an Athena command spec to a real WPILib command.
     *
     * @param cycles execution counter
     * @return WPILib command
     */
    public static Command adaptedCommand(AtomicInteger cycles) {
        return WpilibCommandAdapter.adapt(CommandSpec.create("score")
                .requires("shooter")
                .onExecute(cycles::incrementAndGet)
                .until(() -> cycles.get() >= 2)
                .toSpec(), Map.of("shooter", new ExampleSubsystem()));
    }

    /**
     * Schedules an Athena command spec through the WPILib scheduler facade.
     *
     * @param cycles execution counter
     * @param schedulerClient scheduler client
     * @return scheduled WPILib command
     */
    public static Command scheduledCommand(
            AtomicInteger cycles,
            WpilibCommandScheduler.SchedulerClient schedulerClient) {
        return new WpilibCommandScheduler(schedulerClient).schedule(CommandSpec.create("scoreScheduled")
                .requires("shooter")
                .onExecute(cycles::incrementAndGet)
                .toSpec(), Map.of("shooter", new ExampleSubsystem()));
    }

    /**
     * Binds an Athena command spec to a WPILib trigger binder.
     *
     * @param cycles execution counter
     * @param binder trigger binder
     * @return bound WPILib command
     */
    public static Command triggerBoundCommand(
            AtomicInteger cycles,
            WpilibTriggerBindings.TriggerBinder binder) {
        return WpilibTriggerBindings.bind(
                binder,
                WpilibTriggerBindings.TriggerMode.ON_TRUE,
                CommandSpec.create("scoreOnTrigger")
                        .requires("shooter")
                        .onExecute(cycles::incrementAndGet)
                        .toSpec(),
                Map.of("shooter", new ExampleSubsystem()));
    }

    /**
     * Declares robot lifecycle hooks.
     *
     * @param events event counter
     * @return lifecycle spec
     */
    public static RobotLifecycleSpec lifecycle(AtomicInteger events) {
        return new RobotLifecycleConfig()
                .onInit(RobotMode.AUTONOMOUS, events::incrementAndGet)
                .onPeriodic(RobotMode.AUTONOMOUS, events::incrementAndGet)
                .onPeriodic(RobotMode.TELEOP, events::incrementAndGet)
                .toSpec();
    }

    /**
     * Creates a real WPILib TimedRobot adapter.
     *
     * @param events event counter
     * @return TimedRobot adapter
     */
    public static AthenaTimedRobot timedRobot(AtomicInteger events) {
        return new AthenaTimedRobot(lifecycle(events));
    }

    /**
     * Publishes telemetry through a test sink that matches the WPILib writer contract.
     *
     * @return recorded table values
     */
    public static Map<String, Object> publishTelemetry() {
        Map<String, Object> values = new ConcurrentHashMap<>();
        WpilibNetworkTableSink table = new RecordingSink(values);
        var registry = new TelemetryRegistry()
                .booleanValue(TelemetryKey.bool("robot/enabled"), () -> true)
                .stringValue(TelemetryKey.string("robot/mode"), () -> "auto");

        registry.publishAll(new NetworkTablesTelemetrySink(new WpilibNetworkTableWriter(table)));
        return values;
    }

    /**
     * Applies Athena drive speeds to a WPILib differential drive output.
     *
     * @return left and right motor output values
     */
    public static Map<String, Double> applyDifferentialDriveSpeeds() {
        Map<String, Double> outputs = new ConcurrentHashMap<>();
        RobotSpeeds speeds = DriveCommandExample.speeds();
        var adapter = WpilibDifferentialDriveAdapter.create(
                speeds,
                DriveExample.CONFIG.toSpec(),
                value -> outputs.put("left", value),
                value -> outputs.put("right", value),
                4.5);

        speeds.setSpeeds(RobotSpeeds.DRIVE_SOURCE, 2.25, 0.0, 1.0);
        adapter.periodic();
        return outputs;
    }

    /**
     * Applies Athena drive speeds to WPILib swerve module states.
     *
     * @return module speeds keyed by module name
     */
    public static Map<String, Double> applySwerveDriveSpeeds() {
        Map<String, Double> speedsByModule = new ConcurrentHashMap<>();
        RobotSpeeds speeds = DriveCommandExample.speeds();
        var adapter = WpilibSwerveDriveAdapter.builder(speeds, SwerveExample.CONFIG.toSpec(), 4.5)
                .module((module, state) -> speedsByModule.put(module.name(), state.speedMetersPerSecond))
                .module((module, state) -> speedsByModule.put(module.name(), state.speedMetersPerSecond))
                .module((module, state) -> speedsByModule.put(module.name(), state.speedMetersPerSecond))
                .module((module, state) -> speedsByModule.put(module.name(), state.speedMetersPerSecond))
                .build();

        speeds.setSpeeds(RobotSpeeds.DRIVE_SOURCE, 2.0, 0.0, 0.0);
        adapter.periodic();
        return speedsByModule;
    }

    /**
     * Applies Athena localization weights to a WPILib vision measurement.
     *
     * @return x, y, and heading standard deviations
     */
    public static Map<String, Double> applyVisionMeasurement() {
        Map<String, Double> standardDeviations = new ConcurrentHashMap<>();
        var adapter = new WpilibPoseEstimatorAdapter(LocalizationExample.ROBOT_POSE, (pose, timestamp, stdDevs) -> {
            standardDeviations.put("x", stdDevs.get(0, 0));
            standardDeviations.put("y", stdDevs.get(1, 0));
            standardDeviations.put("heading", stdDevs.get(2, 0));
        });

        adapter.addVisionMeasurement(new Pose2d(1.0, 2.0, new Rotation2d()), 0.25, 2);
        return standardDeviations;
    }

    private record RecordingSink(Map<String, Object> values) implements WpilibNetworkTableSink {
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

    private static final class ExampleSubsystem implements Subsystem {
    }
}
