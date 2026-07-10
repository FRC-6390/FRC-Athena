package frc.robot;

import ca.frc6390.athena.mechanism.core.Events;
import ca.frc6390.athena.mechanism.core.HookBinding;
import ca.frc6390.athena.mechanism.core.Mechanism;
import ca.frc6390.athena.wpilib.controls.Controllers;
import ca.frc6390.athena.wpilib.controls.Gamepad;
import java.util.function.DoubleSupplier;

public final class Controls implements Mechanism {
    private boolean fieldOriented = true;
    public final Gamepad driver = Controllers.xbox(Constants.Driver.PORT);
    public final DoubleSupplier forward = driver.leftY()
            .deadband(Constants.Driver.DEADBAND)
            .inverted()
            .toSupplier();
    public final DoubleSupplier strafe = driver.leftX()
            .deadband(Constants.Driver.DEADBAND)
            .toSupplier();
    public final DoubleSupplier rotation = driver.rightX()
            .deadband(Constants.Driver.DEADBAND)
            .toSupplier();
    public final HookBinding drive;

    public Controls(DriveTrain driveTrain) {
        driver.y().onActive(() -> fieldOriented = true);
        driver.a().onActive(() -> fieldOriented = false);

        drive = Events.teleopPeriodic().whileActive(() -> {
            double forwardValue = forward.getAsDouble();
            double strafeValue = strafe.getAsDouble();
            double rotationValue = rotation.getAsDouble();
            if (fieldOriented) {
                driveTrain.fieldOrientedDrive(forwardValue, strafeValue, rotationValue).request();
            } else {
                driveTrain.robotOrientedDrive(forwardValue, strafeValue, rotationValue).request();
            }
        });
    }
}
