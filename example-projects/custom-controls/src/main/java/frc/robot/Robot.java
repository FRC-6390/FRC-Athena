package frc.robot;

import ca.frc6390.athena.mechanism.core.Events;
import ca.frc6390.athena.mechanism.core.HookBinding;
import ca.frc6390.athena.wpilib.lifecycle.AthenaRobot;
import frc.robot.mechanisms.ExperimentalArm;
import frc.robot.mechanisms.Flywheel;

public final class Robot extends AthenaRobot {
    public final ExperimentalArm arm = new ExperimentalArm();
    public final Flywheel flywheel = new Flywheel();

    @SuppressWarnings("unused")
    public final HookBinding runControls = Events.teleopPeriodic().whileActive(() -> {
        arm.motionProfiled.request();
        flywheel.withArbitraryFeedforward.request();
    });

    @Override
    protected void configure() {
        register(arm);
        register(flywheel);
    }
}
