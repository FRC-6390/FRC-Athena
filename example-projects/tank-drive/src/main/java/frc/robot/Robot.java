package frc.robot;

import ca.frc6390.athena.wpilib.lifecycle.AthenaRobot;

public final class Robot extends AthenaRobot {
    public final DriveTrain driveTrain = new DriveTrain();
    public final Controls controls = new Controls(driveTrain);

    @Override
    protected void configure() {
        register(driveTrain);
        register(controls);
    }
}
