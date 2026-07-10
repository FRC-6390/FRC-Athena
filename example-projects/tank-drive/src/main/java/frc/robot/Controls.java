package frc.robot;

import ca.frc6390.athena.mechanism.core.Events;
import ca.frc6390.athena.mechanism.core.HookBinding;
import ca.frc6390.athena.mechanism.core.Mechanism;
import ca.frc6390.athena.wpilib.controls.Controllers;
import ca.frc6390.athena.wpilib.controls.Gamepad;
import java.util.function.DoubleSupplier;

public final class Controls implements Mechanism {
    public final Gamepad driver = Controllers.xbox(Constants.Driver.PORT);
    public final DoubleSupplier forward = driver.leftY()
            .deadband(Constants.Driver.DEADBAND)
            .inverted()
            .toSupplier();
    public final DoubleSupplier turn = driver.rightX()
            .deadband(Constants.Driver.DEADBAND)
            .toSupplier();
    public final HookBinding drive;

    public Controls(DriveTrain driveTrain) {
        drive = Events.teleopPeriodic().whileActive(() -> {
            driveTrain.arcade(forward.getAsDouble(), turn.getAsDouble());
            driveTrain.drive.request();
        });
    }
}
