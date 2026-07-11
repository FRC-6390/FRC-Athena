package frc.robot;

import ca.frc6390.athena.mechanism.core.Events;
import ca.frc6390.athena.mechanism.core.HookBinding;
import ca.frc6390.athena.mechanism.core.Mechanism;
import ca.frc6390.athena.wpilib.controls.Controllers;
import ca.frc6390.athena.wpilib.controls.Gamepad;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.localization.LocalizationExamples;
import java.util.function.DoubleSupplier;

public final class Controls implements Mechanism {
    private boolean fieldOriented = true;
    public final Gamepad driver = Controllers.xbox(Constants.Driver.PORT);
    public final DoubleSupplier forward = driver.leftY()
            .deadband(Constants.Driver.DEADBAND)
            .inverted()
            .squared()
            .toSupplier();
    public final DoubleSupplier strafe = driver.leftX()
            .deadband(Constants.Driver.DEADBAND)
            .squared()
            .toSupplier();
    public final DoubleSupplier rotation = driver.rightX()
            .deadband(Constants.Driver.DEADBAND)
            .squared()
            .toSupplier();
    public final HookBinding drive;

    public Controls(DriveTrain driveTrain, LocalizationExamples localization) {
        driver.y().pressed().onlyIf(DriverStation::isTeleopEnabled)
                .onTrue(() -> fieldOriented = true);
        driver.a().pressed().onlyIf(DriverStation::isTeleopEnabled)
                .onTrue(() -> fieldOriented = false);
        driver.start().pressed().onlyIf(DriverStation::isTeleopEnabled)
                .onTrue(localization.resetHeading());
        driver.back().pressed().onlyIf(DriverStation::isTeleopEnabled)
                .onTrue(localization.estimatedFieldPose.reset(
                        new Pose2d(2.0, 4.0, Rotation2d.kZero)));

        drive = Events.teleopPeriodic().whileActive(driveTrain.drive(
                forward,
                strafe,
                rotation,
                () -> fieldOriented,
                () -> localization.pose().getRotation().getRadians()));
    }
}
