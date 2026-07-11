package frc.robot;

import ca.frc6390.athena.wpilib.lifecycle.AthenaRobot;
import frc.robot.mechanisms.Conveyor;
import frc.robot.mechanisms.GuardedArm;

public final class Robot extends AthenaRobot {
    public final GuardedArm arm = new GuardedArm();
    public final Conveyor conveyor = new Conveyor(arm.atScore);
    public final Controls controls = new Controls(this);
}
