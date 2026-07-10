package frc.robot;

import ca.frc6390.athena.mechanism.core.Events;
import ca.frc6390.athena.mechanism.core.HookBinding;
import ca.frc6390.athena.mechanism.core.Mechanism;
import ca.frc6390.athena.wpilib.controls.Controllers;
import ca.frc6390.athena.wpilib.controls.Gamepad;
import java.util.Objects;
import java.util.function.DoubleSupplier;

public final class Controls implements Mechanism {
    private final DriveTrain driveTrain;
    private final Intake intake;

    public final Gamepad driver = Controllers.xbox(Constants.Driver.PORT);
    public final DoubleSupplier forward = driver.leftY()
            .deadband(Constants.Driver.DEADBAND)
            .inverted()
            .squared()
            .toSupplier();
    public final DoubleSupplier turn = driver.rightX()
            .deadband(Constants.Driver.DEADBAND)
            .squared()
            .toSupplier();

    public final HookBinding arcadeDrive;

    public Controls(DriveTrain driveTrain, Intake intake) {
        this.driveTrain = Objects.requireNonNull(driveTrain, "driveTrain");
        this.intake = Objects.requireNonNull(intake, "intake");

        arcadeDrive = Events.teleopPeriodic().whileActive(() -> {
            this.driveTrain.arcade(forward.getAsDouble(), turn.getAsDouble());
            this.driveTrain.drive.request();
        });

        driver.a().onActive(this.driveTrain.stop);
        driver.rightBumper()
                .whileActive(this.intake.collect)
                .onDeactive(this.intake.stop);
        driver.leftBumper()
                .whileActive(this.intake.eject)
                .onDeactive(this.intake.stop);
    }
}
