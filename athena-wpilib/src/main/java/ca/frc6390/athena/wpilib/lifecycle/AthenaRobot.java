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
import ca.frc6390.athena.wpilib.controls.Controls;
import ca.frc6390.athena.wpilib.simulation.WpilibSimPhysicsEngine;
import ca.frc6390.athena.sim.runtime.SimulationSession;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.util.Objects;

/**
 * Subclassable WPILib robot host backed by Athena mechanisms.
 */
public abstract class AthenaRobot extends TimedRobot implements Mechanism {
    private RobotRuntime runtime;
    private double lastTimestampSeconds;
    private double lastSimulationTimestampSeconds;

    /**
     * Registers mechanisms, bindings, and robot controls.
     */
    protected abstract void configure();

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

    protected final void register(Mechanism mechanism) {
        athena().register(Objects.requireNonNull(mechanism, "mechanism"));
    }

    public final void set(Action Action) {
        athena().request(Objects.requireNonNull(Action, "Action"));
    }

    public final CommandAction Action(Action Action) {
        return Controls.run("Action:robot", () -> set(Action));
    }

    @Override
    public final void robotInit() {
        lastTimestampSeconds = timestampSeconds();
        lastSimulationTimestampSeconds = lastTimestampSeconds;
        runtime = createRuntime(RobotBase.isSimulation());
        ActionRequests.bind(runtime::request);
        runtime.register(this);
        configure();
        run(LifecycleMode.ROBOT, LifecyclePhase.INIT, true, false);
    }

    @Override
    public final void robotPeriodic() {
        CommandScheduler.getInstance().run();
        run(LifecycleMode.ROBOT, LifecyclePhase.PERIODIC, true, false);
    }

    @Override
    public final void disabledInit() {
        run(LifecycleMode.DISABLED, LifecyclePhase.INIT, false, false);
    }

    @Override
    public final void disabledPeriodic() {
        run(LifecycleMode.DISABLED, LifecyclePhase.PERIODIC, false, false);
    }

    @Override
    public final void disabledExit() {
        run(LifecycleMode.DISABLED, LifecyclePhase.EXIT, false, false);
    }

    @Override
    public final void autonomousInit() {
        run(LifecycleMode.AUTONOMOUS, LifecyclePhase.INIT, true, false);
    }

    @Override
    public final void autonomousPeriodic() {
        run(LifecycleMode.AUTONOMOUS, LifecyclePhase.PERIODIC, true, false);
    }

    @Override
    public final void autonomousExit() {
        run(LifecycleMode.AUTONOMOUS, LifecyclePhase.EXIT, true, false);
    }

    @Override
    public final void teleopInit() {
        CommandScheduler.getInstance().cancelAll();
        run(LifecycleMode.TELEOP, LifecyclePhase.INIT, true, false);
    }

    @Override
    public final void teleopPeriodic() {
        run(LifecycleMode.TELEOP, LifecyclePhase.PERIODIC, true, false);
    }

    @Override
    public final void teleopExit() {
        run(LifecycleMode.TELEOP, LifecyclePhase.EXIT, true, false);
    }

    @Override
    public final void testInit() {
        CommandScheduler.getInstance().cancelAll();
        run(LifecycleMode.TEST, LifecyclePhase.INIT, true, false);
    }

    @Override
    public final void testPeriodic() {
        run(LifecycleMode.TEST, LifecyclePhase.PERIODIC, true, false);
    }

    @Override
    public final void testExit() {
        run(LifecycleMode.TEST, LifecyclePhase.EXIT, true, false);
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
        if (!simulation) {
            return RobotRuntime.create();
        }
        return RobotRuntime.simulated(SimulationSession.create().physicsEngine(new WpilibSimPhysicsEngine()));
    }

}
