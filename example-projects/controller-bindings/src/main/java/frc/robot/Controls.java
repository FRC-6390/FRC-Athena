package frc.robot;

import ca.frc6390.athena.mechanism.core.Action;
import ca.frc6390.athena.mechanism.core.Mechanism;
import ca.frc6390.athena.wpilib.controls.ControlSignal;
import ca.frc6390.athena.wpilib.controls.Controllers;
import ca.frc6390.athena.wpilib.controls.Gamepad;
import ca.frc6390.athena.wpilib.controls.ToggleSignal;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.function.DoubleSupplier;

public final class Controls implements Mechanism {
    private double driveScale = 1.0;
    public final Gamepad driver = Controllers.xbox(Constants.Driver.PORT);
    private final DoubleSupplier forward = driver.leftY()
            .deadband(Constants.Driver.DEADBAND)
            .inverted()
            .squared()
            .toSupplier();
    private final DoubleSupplier turn = driver.rightX()
            .deadband(Constants.Driver.DEADBAND)
            .squared()
            .toSupplier();

    public Controls(Robot robot) {
        ControlSignal teleop = driver.signal("teleop", DriverStation::isTeleopEnabled);
        Action arcadeDrive = robot.driveTrain.arcade(
                () -> scale(forward.getAsDouble()),
                () -> scale(turn.getAsDouble()));
        teleop.whileTrue(arcadeDrive);

        driver.a().pressed().onlyIf(teleop).onTrue(robot.driveTrain.stop);
        ToggleSignal slowMode = driver.y().onlyIf(teleop).toggle();
        slowMode
                .onTrue(() -> driveScale = Constants.Driver.SLOW_SCALE)
                .onFalse(() -> driveScale = 1.0);

        ControlSignal collect = driver.rightBumper()
                .and(driver.leftBumper().negate())
                .onlyIf(teleop);
        ControlSignal eject = driver.leftBumper()
                .and(driver.rightBumper().negate())
                .onlyIf(teleop);
        collect.whileTrue(robot.intake.collect);
        eject.whileTrue(robot.intake.eject);
    }

    private double scale(double value) {
        return value * driveScale;
    }
}
