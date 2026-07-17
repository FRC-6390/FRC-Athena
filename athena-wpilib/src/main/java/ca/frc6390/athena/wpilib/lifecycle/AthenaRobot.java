package ca.frc6390.athena.wpilib.lifecycle;

import ca.frc6390.athena.auto.AutoChooser;
import ca.frc6390.athena.commands.CommandAction;
import ca.frc6390.athena.mechanism.core.EventContext;
import ca.frc6390.athena.mechanism.core.LifecycleMode;
import ca.frc6390.athena.mechanism.core.LifecyclePhase;
import ca.frc6390.athena.mechanism.core.Mechanism;
import ca.frc6390.athena.mechanism.core.MechanismContext;
import ca.frc6390.athena.mechanism.core.TelemetrySchema;
import ca.frc6390.athena.mechanism.core.Action;
import ca.frc6390.athena.mechanism.core.ActionRequests;
import ca.frc6390.athena.robot.RobotRuntime;
import ca.frc6390.athena.wpilib.commands.WpilibCommands;
import ca.frc6390.athena.wpilib.simulation.WpilibSimPhysicsEngine;
import ca.frc6390.athena.sim.runtime.SimulationSession;
import ca.frc6390.athena.wpilib.telemetry.MechanismTracePublisher;
import ca.frc6390.athena.wpilib.telemetry.MechanismTelemetryPublisher;
import ca.frc6390.athena.wpilib.telemetry.AutoPreviewPublisher;
import ca.frc6390.athena.wpilib.telemetry.SystemTelemetryPublisher;
import ca.frc6390.athena.wpilib.system.MemoryPressure;
import ca.frc6390.athena.wpilib.system.SystemRuntime;
import ca.frc6390.athena.wpilib.system.SystemStatus;
import ca.frc6390.athena.wpilib.system.SystemTuning;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.util.Objects;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

/**
 * Subclassable WPILib robot host backed by Athena mechanisms.
 */
public abstract class AthenaRobot extends TimedRobot implements Mechanism {
    private RobotRuntime runtime;
    private double lastTimestampSeconds;
    private double lastSimulationTimestampSeconds;
    private MechanismTracePublisher tracePublisher;
    private MechanismTelemetryPublisher mechanismTelemetryPublisher;
    private TelemetrySchema mechanismTelemetrySchema;
    private AutoPreviewPublisher autoPreviewPublisher;
    private SystemRuntime systemRuntime;
    private SystemTelemetryPublisher systemTelemetryPublisher;
    private final List<SendableChooser<String>> dashboardAutoChoosers = new ArrayList<>();
    private final Map<Integer, DigitalInput> digitalInputs = new ConcurrentHashMap<>();
    private MechanismTracePublisher.Profile traceProfile = MechanismTracePublisher.Profile.OFF;
    private SystemTuning systemTuning = SystemTuning.automatic();
    private int reportedSystemFailures;
    private MemoryPressure reportedPressure = MemoryPressure.NORMAL;
    private boolean reportedJvmConfiguration;

    /**
     * Returns the owned Athena mechanism runtime.
     *
     * @return runtime
     */
    public final RobotRuntime athena() {
        if (runtime == null) {
            throw new IllegalStateException("Athena has not been created yet.");
        }
        return runtime;
    }

    public final void set(Action Action) {
        athena().request(Objects.requireNonNull(Action, "Action"));
    }

    /** Selects the live mechanism telemetry profile. Safe to call from the robot constructor. */
    public final void traceTelemetry(MechanismTracePublisher.Profile profile) {
        traceProfile = profile == null ? MechanismTracePublisher.Profile.OFF : profile;
        if (tracePublisher != null) {
            tracePublisher.profile(traceProfile);
        }
    }

    /** Selects roboRIO memory monitoring and tuning. Safe to call from the robot constructor. */
    public final void systemTuning(SystemTuning tuning) {
        if (runtime != null) {
            throw new IllegalStateException("System tuning must be selected before robotInit.");
        }
        systemTuning = Objects.requireNonNull(tuning, "tuning");
    }

    /** Returns the latest JVM and roboRIO system-health snapshot. */
    public final SystemStatus systemStatus() {
        if (systemRuntime == null) {
            throw new IllegalStateException("System monitoring has not started yet.");
        }
        return systemRuntime.status();
    }

    public final CommandAction Action(Action Action) {
        return WpilibCommands.run("Action:robot", () -> set(Action));
    }

    @Override
    public final void robotInit() {
        lastTimestampSeconds = timestampSeconds();
        lastSimulationTimestampSeconds = lastTimestampSeconds;
        runtime = createRuntime(RobotBase.isSimulation(), digitalInputs);
        runtime.failureReporter(new ca.frc6390.athena.robot.RuntimeFailureReporter() {
            @Override public void error(String message, Throwable cause) {
                DriverStation.reportError(message, cause == null ? new StackTraceElement[0] : cause.getStackTrace());
            }

            @Override public void warning(String message, Throwable cause) {
                DriverStation.reportWarning(message, cause == null ? new StackTraceElement[0] : cause.getStackTrace());
            }
        });
        ActionRequests.bind(runtime::request);
        runtime.register(this);
        mechanismTelemetrySchema = runtime.mechanismTelemetrySchema();
        configureAutoChoosers();
        tracePublisher = new MechanismTracePublisher().profile(
                RobotBase.isSimulation() && traceProfile == MechanismTracePublisher.Profile.SUMMARY
                        ? MechanismTracePublisher.Profile.CAPTURE
                        : traceProfile);
        mechanismTelemetryPublisher = new MechanismTelemetryPublisher();
        autoPreviewPublisher = new AutoPreviewPublisher();
        systemRuntime = SystemRuntime.create(systemTuning, RobotBase.isReal());
        systemTelemetryPublisher = new SystemTelemetryPublisher();
        systemRuntime.start();
        publishSystemTelemetry();
        publishMechanismTelemetry();
        run(LifecycleMode.ROBOT, LifecyclePhase.INIT, true, simulationActive());
        publishAutoPreview();
    }

    @Override
    public final void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public final void disabledInit() {
        run(LifecycleMode.DISABLED, LifecyclePhase.INIT, false, simulationActive());
    }

    @Override
    public final void disabledPeriodic() {
        run(LifecycleMode.DISABLED, LifecyclePhase.PERIODIC, false, simulationActive());
        publishTraces();
    }

    @Override
    public final void disabledExit() {
        run(LifecycleMode.DISABLED, LifecyclePhase.EXIT, false, simulationActive());
    }

    @Override
    public final void autonomousInit() {
        run(LifecycleMode.AUTONOMOUS, LifecyclePhase.INIT, true, simulationActive());
    }

    @Override
    public final void autonomousPeriodic() {
        run(LifecycleMode.AUTONOMOUS, LifecyclePhase.PERIODIC, true, simulationActive());
        publishTraces();
    }

    @Override
    public final void autonomousExit() {
        run(LifecycleMode.AUTONOMOUS, LifecyclePhase.EXIT, true, simulationActive());
    }

    @Override
    public final void teleopInit() {
        CommandScheduler.getInstance().cancelAll();
        run(LifecycleMode.TELEOP, LifecyclePhase.INIT, true, simulationActive());
    }

    @Override
    public final void teleopPeriodic() {
        run(LifecycleMode.TELEOP, LifecyclePhase.PERIODIC, true, simulationActive());
        publishTraces();
    }

    @Override
    public final void teleopExit() {
        run(LifecycleMode.TELEOP, LifecyclePhase.EXIT, true, simulationActive());
    }

    @Override
    public final void testInit() {
        CommandScheduler.getInstance().cancelAll();
        run(LifecycleMode.TEST, LifecyclePhase.INIT, true, simulationActive());
    }

    @Override
    public final void testPeriodic() {
        run(LifecycleMode.TEST, LifecyclePhase.PERIODIC, true, simulationActive());
        publishTraces();
    }

    @Override
    public final void testExit() {
        run(LifecycleMode.TEST, LifecyclePhase.EXIT, true, simulationActive());
    }

    @Override
    public final void simulationInit() {
        run(LifecycleMode.SIMULATION, LifecyclePhase.INIT, true, true);
    }

    @Override
    public final void simulationPeriodic() {
        double timestamp = timestampSeconds();
        athena().simulationPeriodic(timestamp, simulationElapsed(timestamp));
    }

    private void run(LifecycleMode mode, LifecyclePhase phase, boolean enabled, boolean simulation) {
        double timestamp = timestampSeconds();
        double dtSeconds = elapsed(timestamp);
        if (tracePublisher != null) {
            tracePublisher.profile(effectiveTraceProfile());
            athena().mechanismTraceLevel(tracePublisher.traceLevel());
        }
        EventContext eventContext = new EventContext(timestamp, dtSeconds, mode, phase, enabled, simulation);
        athena().periodic(
                new MechanismContext(timestamp, 0.0, dtSeconds, enabled, mode == LifecycleMode.AUTONOMOUS, simulation),
                eventContext);
    }

    private void publishTraces() {
        publishSystemTelemetry();
        if (tracePublisher != null && pressure() == MemoryPressure.NORMAL) {
            tracePublisher.publish(athena().mechanismTraces());
        }
        publishMechanismTelemetry();
        publishAutoPreview();
    }

    private void publishMechanismTelemetry() {
        if (mechanismTelemetryPublisher != null) {
            mechanismTelemetryPublisher.publish(mechanismTelemetrySchema);
        }
    }

    private void publishAutoPreview() {
        if (autoPreviewPublisher != null && pressure() != MemoryPressure.CRITICAL) {
            autoPreviewPublisher.publish(athena().selectedAutoPreviews(), athena().runningAutoName());
        }
    }

    private void publishSystemTelemetry() {
        if (systemRuntime == null || systemTelemetryPublisher == null) return;
        systemRuntime.update();
        SystemStatus status = systemRuntime.status();
        systemTelemetryPublisher.publish(status);
        mechanismTelemetryPublisher.readOnlyPeriodSeconds(switch (status.pressure()) {
            case NORMAL -> 0.10;
            case WARNING -> 0.50;
            case CRITICAL -> 1.00;
        });
        while (reportedSystemFailures < status.failures().size()) {
            DriverStation.reportWarning(
                    "Athena system tuning: " + status.failures().get(reportedSystemFailures++), false);
        }
        if (status.pressure() != reportedPressure) {
            DriverStation.reportWarning(
                    "Athena memory pressure is " + status.pressure() + ": " + status.pressureReason(), false);
            reportedPressure = status.pressure();
        }
        if (!reportedJvmConfiguration && !status.jvmConfigurationHealthy()
                && !status.recommendedJvmArguments().isEmpty()) {
            reportedJvmConfiguration = true;
            DriverStation.reportWarning(
                    "Athena recommends roboRIO JVM arguments: "
                            + String.join(" ", status.recommendedJvmArguments()), false);
        }
    }

    private MemoryPressure pressure() {
        return systemRuntime == null ? MemoryPressure.NORMAL : systemRuntime.status().pressure();
    }

    private MechanismTracePublisher.Profile effectiveTraceProfile() {
        if (pressure() != MemoryPressure.NORMAL) return MechanismTracePublisher.Profile.OFF;
        return RobotBase.isSimulation() && traceProfile == MechanismTracePublisher.Profile.SUMMARY
                ? MechanismTracePublisher.Profile.CAPTURE
                : traceProfile;
    }

    private void configureAutoChoosers() {
        dashboardAutoChoosers.clear();
        for (AutoChooser model : athena().autoChoosers()) {
            SendableChooser<String> dashboard = new SendableChooser<>();
            dashboard.setDefaultOption(model.defaultName(), model.defaultName());
            model.optionNames().stream()
                    .filter(name -> !name.equals(model.defaultName()))
                    .forEach(name -> dashboard.addOption(name, name));
            dashboard.onChange(model::selectIfPresent);
            SmartDashboard.putData(model.dashboardName(), dashboard);
            dashboardAutoChoosers.add(dashboard);
        }
    }

    private boolean simulationActive() {
        return runtime != null && runtime.simulationSession() != null;
    }

    private double elapsed(double timestampSeconds) {
        double dtSeconds = timestampSeconds - lastTimestampSeconds;
        lastTimestampSeconds = timestampSeconds;
        return Double.isFinite(dtSeconds) && dtSeconds >= 0.0 ? dtSeconds : 0.0;
    }

    private double simulationElapsed(double timestampSeconds) {
        double dtSeconds = timestampSeconds - lastSimulationTimestampSeconds;
        lastSimulationTimestampSeconds = timestampSeconds;
        return Double.isFinite(dtSeconds) && dtSeconds >= 0.0 ? dtSeconds : 0.0;
    }

    private static double timestampSeconds() {
        return Timer.getFPGATimestamp();
    }

    static RobotRuntime createRuntime(boolean simulation) {
        return createRuntime(simulation, new ConcurrentHashMap<>());
    }

    private static RobotRuntime createRuntime(boolean simulation, Map<Integer, DigitalInput> inputs) {
        RobotRuntime runtime = simulation
                ? RobotRuntime.simulated(SimulationSession.create().physicsEngine(new WpilibSimPhysicsEngine()))
                : RobotRuntime.create();
        return runtime.digitalInputs(device -> {
            DigitalInput input = inputs.computeIfAbsent(device.channel(), DigitalInput::new);
            return input::get;
        });
    }

    @Override
    public void close() {
        if (tracePublisher != null) tracePublisher.close();
        if (mechanismTelemetryPublisher != null) mechanismTelemetryPublisher.close();
        if (autoPreviewPublisher != null) autoPreviewPublisher.close();
        if (systemTelemetryPublisher != null) systemTelemetryPublisher.close();
        if (systemRuntime != null) systemRuntime.close();
        digitalInputs.values().forEach(DigitalInput::close);
        digitalInputs.clear();
        super.close();
    }

}
