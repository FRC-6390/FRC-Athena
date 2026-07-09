package frc.robot;

import ca.frc6390.athena.mechanism.core.Events;
import ca.frc6390.athena.mechanism.core.HookBinding;
import ca.frc6390.athena.wpilib.controls.Controls;
import ca.frc6390.athena.wpilib.controls.Controls.Controller;
import ca.frc6390.athena.wpilib.lifecycle.AthenaRobot;

public final class Robot extends AthenaRobot {
    public final DriveTrain driveTrain = new DriveTrain();
    public final Controller driver = Controls.xbox(Constants.Driver.PORT)
            .deadband(Constants.Driver.DEADBAND)
            .invertLeftY();

    public final HookBinding teleopDrive = Events.teleopPeriodic().whileActive(() -> {
        driveTrain.drive(driver.leftY(), driver.leftX(), driver.rightX()).request();
    });

    @Override
    protected void configure() {
        register(driveTrain);
    }
}
