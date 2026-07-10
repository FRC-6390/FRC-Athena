package frc.robot;

import ca.frc6390.athena.wpilib.lifecycle.AthenaRobot;
import frc.robot.mechanisms.ExperimentalArm;
import frc.robot.mechanisms.Flywheel;

public final class Robot extends AthenaRobot {
    public final ExperimentalArm arm = new ExperimentalArm();
    public final Flywheel flywheel = new Flywheel();
    public final Controls controls = new Controls(arm, flywheel);

    @Override
    protected void configure() {
        register(arm);
        register(flywheel);
        register(controls);
    }
}
