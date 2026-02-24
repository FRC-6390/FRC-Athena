package ca.frc6390.athena.commands.examples;

import java.util.function.DoubleSupplier;

import ca.frc6390.athena.commands.control.TankDriveCommand;
import ca.frc6390.athena.drivetrains.differential.DifferentialDrivetrain;
import edu.wpi.first.math.MathUtil;

/**
 * Examples showing common setup patterns for {@link TankDriveCommand}.
 */
public final class TankDriveCommandExamples {
    private TankDriveCommandExamples() {}

    public static TankDriveCommand arcadeDrive(
            DifferentialDrivetrain drivetrain,
            DoubleSupplier throttleInput,
            DoubleSupplier turnInput) {
        return new TankDriveCommand(drivetrain, throttleInput, turnInput);
    }

    public static TankDriveCommand arcadeDriveWithDeadband(
            DifferentialDrivetrain drivetrain,
            DoubleSupplier throttleInput,
            DoubleSupplier turnInput,
            double deadband) {
        double sanitizedDeadband = MathUtil.clamp(Math.abs(deadband), 0.0, 1.0);
        return arcadeDrive(
                drivetrain,
                () -> MathUtil.applyDeadband(throttleInput.getAsDouble(), sanitizedDeadband),
                () -> MathUtil.applyDeadband(turnInput.getAsDouble(), sanitizedDeadband));
    }
}
