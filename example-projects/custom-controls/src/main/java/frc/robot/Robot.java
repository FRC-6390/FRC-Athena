package frc.robot;

import ca.frc6390.athena.wpilib.lifecycle.AthenaRobot;
import frc.robot.mechanisms.AdvancedFeedback;
import frc.robot.mechanisms.ExperimentalArm;
import frc.robot.mechanisms.Flywheel;

public final class Robot extends AthenaRobot {
    public final ExperimentalArm arm = new ExperimentalArm();
    public final Flywheel flywheel = new Flywheel();
    public final AdvancedFeedback advancedFeedback = new AdvancedFeedback();
    public final Controls controls = new Controls(arm, flywheel, advancedFeedback);

    @Override
    protected void configure() {
        register(arm);
        register(flywheel);
        register(advancedFeedback);
        register(controls);
    }
}
