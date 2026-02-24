package ca.frc6390.athena.commands.examples;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import ca.frc6390.athena.commands.vision.RotateToTagCommand;
import ca.frc6390.athena.core.RobotSpeeds;
import ca.frc6390.athena.sensors.camera.ConfigurableCamera.CameraSoftware;
import ca.frc6390.athena.sensors.camera.VisionCamera;
import ca.frc6390.athena.sensors.camera.VisionCamera.CoordinateSpace;
import ca.frc6390.athena.sensors.camera.VisionCameraConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Examples for vision-assisted command configuration and camera stubs.
 */
public final class VisionCommandExamples {
    private VisionCommandExamples() {}

    public static PIDController createHeadingController(double kp, double ki, double kd, double toleranceDegrees) {
        PIDController controller = new PIDController(kp, ki, kd);
        controller.setTolerance(Math.abs(toleranceDegrees));
        return controller;
    }

    public static VisionCamera createYawOnlyCamera(
            String table,
            BooleanSupplier hasTargets,
            DoubleSupplier yawDegreesSupplier) {
        VisionCameraConfig config = VisionCameraConfig.create(table, CameraSoftware.PhotonVision);
        config.config(section -> section
                .hasTargetsSupplier(hasTargets)
                .targetYawSupplier(yawDegreesSupplier));
        return new VisionCamera(config);
    }

    public static RotateToTagCommand rotateToTag(
            RobotSpeeds robotSpeeds,
            VisionCamera camera,
            PIDController headingController,
            double headingOffsetDegrees,
            double maxAngularVelocityRadPerSec,
            double timeoutSeconds) {
        return new RotateToTagCommand(
                robotSpeeds,
                camera,
                headingController,
                headingOffsetDegrees,
                maxAngularVelocityRadPerSec,
                timeoutSeconds);
    }

    public static VisionCamera.TargetObservation targetObservation(
            double yawDegrees,
            double distanceMeters,
            double xMeters,
            double yMeters,
            double confidence) {
        return new VisionCamera.TargetObservation(
                -1,
                yawDegrees,
                distanceMeters,
                new Translation2d(xMeters, yMeters),
                CoordinateSpace.ROBOT,
                null,
                confidence);
    }
}
