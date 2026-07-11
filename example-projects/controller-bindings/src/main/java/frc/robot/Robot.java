package frc.robot;

import ca.frc6390.athena.wpilib.lifecycle.AthenaRobot;

public final class Robot extends AthenaRobot {
    public final DriveTrain driveTrain = new DriveTrain();
    public final Intake intake = new Intake();
    public final Controls controls = new Controls(this);
}
