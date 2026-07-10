package frc.robot;

import ca.frc6390.athena.mechanism.core.Events;
import ca.frc6390.athena.mechanism.core.HookBinding;
import ca.frc6390.athena.wpilib.lifecycle.AthenaRobot;
import frc.robot.mechanisms.Conveyor;
import frc.robot.mechanisms.GuardedArm;

public final class Robot extends AthenaRobot {
    public final GuardedArm arm = new GuardedArm();
    public final Conveyor conveyor = new Conveyor();

    @SuppressWarnings("unused")
    public final HookBinding teleop = Events.teleopPeriodic().whileActive(() -> {
        arm.score.request();
        conveyor.feedWhenLoaded.request();
    });

    @Override
    protected void configure() {
        register(arm);
        register(conveyor);
    }
}
