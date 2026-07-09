package ca.frc6390.athena.wpilib.lifecycle;

import ca.frc6390.athena.mechanism.core.EventContext;
import ca.frc6390.athena.mechanism.core.LifecycleMode;
import ca.frc6390.athena.mechanism.core.LifecyclePhase;
import ca.frc6390.athena.mechanism.core.MechanismContext;
import java.util.Objects;
import java.util.function.DoubleSupplier;

final class AthenaRobotLifecycle {
    private final DoubleSupplier timestampSeconds;
    private final RuntimeDispatcher dispatcher;
    private double lastTimestampSeconds;

    AthenaRobotLifecycle(DoubleSupplier timestampSeconds, RuntimeDispatcher dispatcher) {
        this.timestampSeconds = Objects.requireNonNull(timestampSeconds, "timestampSeconds");
        this.dispatcher = Objects.requireNonNull(dispatcher, "dispatcher");
        this.lastTimestampSeconds = this.timestampSeconds.getAsDouble();
    }

    void robotInit() {
        run(LifecycleMode.ROBOT, LifecyclePhase.INIT, true, false, false);
    }

    void robotPeriodic() {
        run(LifecycleMode.ROBOT, LifecyclePhase.PERIODIC, true, false, false);
    }

    void disabledInit() {
        run(LifecycleMode.DISABLED, LifecyclePhase.INIT, false, false, false);
    }

    void disabledPeriodic() {
        run(LifecycleMode.DISABLED, LifecyclePhase.PERIODIC, false, false, false);
    }

    void autonomousInit() {
        run(LifecycleMode.AUTONOMOUS, LifecyclePhase.INIT, true, true, false);
    }

    void autonomousPeriodic() {
        run(LifecycleMode.AUTONOMOUS, LifecyclePhase.PERIODIC, true, true, false);
    }

    void teleopInit() {
        run(LifecycleMode.TELEOP, LifecyclePhase.INIT, true, false, false);
    }

    void teleopPeriodic() {
        run(LifecycleMode.TELEOP, LifecyclePhase.PERIODIC, true, false, false);
    }

    void testInit() {
        run(LifecycleMode.TEST, LifecyclePhase.INIT, true, false, false);
    }

    void testPeriodic() {
        run(LifecycleMode.TEST, LifecyclePhase.PERIODIC, true, false, false);
    }

    void simulationInit() {
        run(LifecycleMode.SIMULATION, LifecyclePhase.INIT, true, false, true);
    }

    void simulationPeriodic() {
        run(LifecycleMode.SIMULATION, LifecyclePhase.PERIODIC, true, false, true);
    }

    private void run(
            LifecycleMode mode,
            LifecyclePhase phase,
            boolean enabled,
            boolean autonomous,
            boolean simulation) {
        double now = timestampSeconds.getAsDouble();
        double dt = elapsed(now);
        dispatcher.periodic(
                new MechanismContext(now, 0.0, dt, enabled, autonomous, simulation),
                new EventContext(now, dt, mode, phase, enabled, simulation));
    }

    private double elapsed(double nowSeconds) {
        double dtSeconds = nowSeconds - lastTimestampSeconds;
        lastTimestampSeconds = nowSeconds;
        return Double.isFinite(dtSeconds) && dtSeconds >= 0.0 ? dtSeconds : 0.0;
    }

    @FunctionalInterface
    interface RuntimeDispatcher {
        void periodic(MechanismContext mechanismContext, EventContext eventContext);
    }
}
