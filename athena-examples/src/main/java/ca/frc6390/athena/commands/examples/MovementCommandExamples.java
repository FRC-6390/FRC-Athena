package ca.frc6390.athena.commands.examples;

import ca.frc6390.athena.commands.movement.TranslateToPoint;
import ca.frc6390.athena.core.RobotDrivetrain;
import edu.wpi.first.math.controller.PIDController;

/**
 * Examples for configuring movement commands that use translation feedback.
 */
public final class MovementCommandExamples {
    private MovementCommandExamples() {}

    public static PIDController createTranslationPid(double kp, double ki, double kd, double toleranceMeters) {
        PIDController controller = new PIDController(kp, ki, kd);
        controller.setTolerance(Math.abs(toleranceMeters));
        return controller;
    }

    public static TranslateToPoint translateToPoint(RobotDrivetrain<?> drivetrain, double xMeters, double yMeters) {
        return new TranslateToPoint(drivetrain, xMeters, yMeters);
    }

    public static TranslateToPoint translateToPointWithPid(
            RobotDrivetrain<?> drivetrain,
            double xMeters,
            double yMeters,
            PIDController translationPid) {
        return translateToPoint(drivetrain, xMeters, yMeters).setPID(translationPid);
    }
}
