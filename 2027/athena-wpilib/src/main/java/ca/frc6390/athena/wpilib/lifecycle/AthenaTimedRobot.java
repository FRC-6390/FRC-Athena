package ca.frc6390.athena.wpilib.lifecycle;

import java.util.Objects;

import edu.wpi.first.wpilibj.TimedRobot;

/**
 * WPILib {@link TimedRobot} implementation backed by Athena lifecycle hooks.
 */
public class AthenaTimedRobot extends TimedRobot {
    private final RobotLifecycleSpec lifecycle;

    /**
     * Creates a TimedRobot adapter.
     *
     * @param lifecycle lifecycle hooks
     */
    public AthenaTimedRobot(RobotLifecycleSpec lifecycle) {
        this.lifecycle = Objects.requireNonNull(lifecycle, "lifecycle");
    }

    @Override
    public void disabledInit() {
        lifecycle.runInit(RobotMode.DISABLED);
    }

    @Override
    public void disabledPeriodic() {
        lifecycle.runPeriodic(RobotMode.DISABLED);
    }

    @Override
    public void autonomousInit() {
        lifecycle.runInit(RobotMode.AUTONOMOUS);
    }

    @Override
    public void autonomousPeriodic() {
        lifecycle.runPeriodic(RobotMode.AUTONOMOUS);
    }

    @Override
    public void teleopInit() {
        lifecycle.runInit(RobotMode.TELEOP);
    }

    @Override
    public void teleopPeriodic() {
        lifecycle.runPeriodic(RobotMode.TELEOP);
    }

    @Override
    public void testInit() {
        lifecycle.runInit(RobotMode.TEST);
    }

    @Override
    public void testPeriodic() {
        lifecycle.runPeriodic(RobotMode.TEST);
    }

    @Override
    public void simulationInit() {
        lifecycle.runInit(RobotMode.SIMULATION);
    }

    @Override
    public void simulationPeriodic() {
        lifecycle.runPeriodic(RobotMode.SIMULATION);
    }
}
