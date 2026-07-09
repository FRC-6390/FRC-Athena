package ca.frc6390.athena.wpilib.lifecycle;

import java.util.Objects;

import ca.frc6390.athena.robot.RobotRuntime;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;

/**
 * WPILib robot host backed by Athena's robot runtime.
 */
public class AthenaRobot extends TimedRobot {
    private final RobotRuntime runtime;
    private final AthenaRobotLifecycle lifecycle;

    /**
     * Creates a robot with a new Athena runtime.
     */
    public AthenaRobot() {
        this(RobotRuntime.create());
    }

    /**
     * Creates a robot backed by an existing Athena runtime.
     *
     * @param runtime robot runtime
     */
    public AthenaRobot(RobotRuntime runtime) {
        this.runtime = Objects.requireNonNull(runtime, "runtime");
        this.lifecycle = new AthenaRobotLifecycle(AthenaRobot::timestampSeconds, runtime::periodic);
    }

    /**
     * Returns the runtime owned by this robot host.
     *
     * @return robot runtime
     */
    public final RobotRuntime runtime() {
        return runtime;
    }

    @Override
    public void robotInit() {
        lifecycle.robotInit();
    }

    @Override
    public void robotPeriodic() {
        lifecycle.robotPeriodic();
    }

    @Override
    public void disabledInit() {
        lifecycle.disabledInit();
    }

    @Override
    public void disabledPeriodic() {
        lifecycle.disabledPeriodic();
    }

    @Override
    public void autonomousInit() {
        lifecycle.autonomousInit();
    }

    @Override
    public void autonomousPeriodic() {
        lifecycle.autonomousPeriodic();
    }

    @Override
    public void teleopInit() {
        lifecycle.teleopInit();
    }

    @Override
    public void teleopPeriodic() {
        lifecycle.teleopPeriodic();
    }

    @Override
    public void testInit() {
        lifecycle.testInit();
    }

    @Override
    public void testPeriodic() {
        lifecycle.testPeriodic();
    }

    @Override
    public void simulationInit() {
        lifecycle.simulationInit();
    }

    @Override
    public void simulationPeriodic() {
        lifecycle.simulationPeriodic();
    }

    private static double timestampSeconds() {
        return Timer.getFPGATimestamp();
    }
}
