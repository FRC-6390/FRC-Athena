package ca.frc6390.athena.wpilib.lifecycle;

import ca.frc6390.athena.commands.CommandAction;
import ca.frc6390.athena.mechanism.core.EventContext;
import ca.frc6390.athena.mechanism.core.LifecycleMode;
import ca.frc6390.athena.mechanism.core.LifecyclePhase;
import ca.frc6390.athena.mechanism.core.Mechanism;
import ca.frc6390.athena.mechanism.core.MechanismContext;
import ca.frc6390.athena.mechanism.core.Action;
import ca.frc6390.athena.mechanism.core.ActionRequests;
import ca.frc6390.athena.robot.RobotRuntime;
import ca.frc6390.athena.wpilib.commands.WpilibCommands;
import ca.frc6390.athena.wpilib.simulation.WpilibSimPhysicsEngine;
import ca.frc6390.athena.sim.runtime.SimulationSession;
import ca.frc6390.athena.wpilib.telemetry.MechanismTracePublisher;
import ca.frc6390.athena.wpilib.telemetry.MechanismTelemetryPublisher;
import ca.frc6390.athena.wpilib.telemetry.AutoPreviewPublisher;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.util.Objects;
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
    private AutoPreviewPublisher autoPreviewPublisher;
    private final Map<Integer, DigitalInput> digitalInputs = new ConcurrentHashMap<>();
    private MechanismTracePublisher.Profile traceProfile = MechanismTracePublisher.Profile.SUMMARY;

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
        traceProfile = profile == null ? MechanismTracePublisher.Profile.SUMMARY : profile;
        if (tracePublisher != null) {
            tracePublisher.profile(traceProfile);
        }
    }

    public final CommandAction Action(Action Action) {
        return WpilibCommands.run("Action:robot", () -> set(Action));
    }

    @Override
    public final void robotInit() {
        lastTimestampSeconds = timestampSeconds();
        lastSimulationTimestampSeconds = lastTimestampSeconds;
        runtime = createRuntime(RobotBase.isSimulation(), digitalInputs);
        ActionRequests.bind(runtime::request);
        runtime.register(this);
        tracePublisher = new MechanismTracePublisher().profile(
                RobotBase.isSimulation() && traceProfile == MechanismTracePublisher.Profile.SUMMARY
                        ? MechanismTracePublisher.Profile.CAPTURE
                        : traceProfile);
        mechanismTelemetryPublisher = new MechanismTelemetryPublisher();
        autoPreviewPublisher = new AutoPreviewPublisher();
        publishMechanismTelemetry();
        run(LifecycleMode.ROBOT, LifecyclePhase.INIT, true, simulationActive());
        publishAutoPreview();
    }

    @Override
    public final void robotPeriodic() {
        CommandScheduler.getInstance().run();
        run(LifecycleMode.ROBOT, LifecyclePhase.PERIODIC, true, simulationActive());
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
        EventContext eventContext = new EventContext(timestamp, dtSeconds, mode, phase, enabled, simulation);
        athena().periodic(
                new MechanismContext(timestamp, 0.0, dtSeconds, enabled, mode == LifecycleMode.AUTONOMOUS, simulation),
                eventContext);
    }

    private void publishTraces() {
        if (tracePublisher != null) {
            tracePublisher.publish(athena().mechanismTraces());
        }
        publishMechanismTelemetry();
        publishAutoPreview();
    }

    private void publishMechanismTelemetry() {
        if (mechanismTelemetryPublisher != null) {
            mechanismTelemetryPublisher.publish(athena().mechanismTelemetry());
        }
    }

    private void publishAutoPreview() {
        if (autoPreviewPublisher != null) {
            autoPreviewPublisher.publish(athena().selectedAutoPreviews());
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
        if (autoPreviewPublisher != null) autoPreviewPublisher.close();
        digitalInputs.values().forEach(DigitalInput::close);
        digitalInputs.clear();
        super.close();
    }

}
