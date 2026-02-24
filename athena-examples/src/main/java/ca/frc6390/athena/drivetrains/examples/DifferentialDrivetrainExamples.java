package ca.frc6390.athena.drivetrains.examples;

import java.util.function.DoubleSupplier;

import ca.frc6390.athena.drivetrains.differential.DifferentialDrivetrain;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Examples for configuring and driving a differential drivetrain.
 */
public final class DifferentialDrivetrainExamples {
    private DifferentialDrivetrainExamples() {}

    public static void configureDriveFeedforward(
            DifferentialDrivetrain drivetrain,
            double kS,
            double kV,
            double kA,
            boolean enabled) {
        drivetrain.driveFeedforward(new SimpleMotorFeedforward(kS, kV, kA));
        drivetrain.driveFeedforwardEnabled(enabled);
    }

    public static Command arcadeDriveCommand(
            DifferentialDrivetrain drivetrain,
            DoubleSupplier throttleInput,
            DoubleSupplier turnInput) {
        return drivetrain.control().command(throttleInput, () -> 0.0, turnInput);
    }

    public static void resetControlState(DifferentialDrivetrain drivetrain) {
        drivetrain.control().reset();
    }

    public static void setSimulationPose(DifferentialDrivetrain drivetrain, Pose2d pose) {
        drivetrain.simulation().pose(pose);
    }
}
