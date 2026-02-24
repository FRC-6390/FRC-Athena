package ca.frc6390.athena.commands.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

import ca.frc6390.athena.commands.examples.VisionCommandExamples;
import ca.frc6390.athena.core.RobotSpeeds;
import ca.frc6390.athena.sensors.camera.VisionCamera;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.junit.jupiter.api.Test;

final class VisionCommandExamplesTest {

    @Test
    void rotateToTagAppliesCorrectionAndClamp() {
        RobotSpeeds speeds = new RobotSpeeds(4.0, 3.0);
        AtomicBoolean hasTargets = new AtomicBoolean(true);
        AtomicReference<Double> yaw = new AtomicReference<>(30.0);
        VisionCamera camera = VisionCommandExamples.createYawOnlyCamera("front", hasTargets::get, yaw::get);
        PIDController headingController = VisionCommandExamples.createHeadingController(0.5, 0.0, 0.0, 1.0);
        RotateToTagCommand command = VisionCommandExamples.rotateToTag(
                speeds,
                camera,
                headingController,
                0.0,
                0.2,
                0.0);

        command.initialize();
        command.execute();

        ChassisSpeeds feedback = speeds.getInputSpeeds(RobotSpeeds.FEEDBACK_SOURCE);
        assertEquals(0.0, feedback.vxMetersPerSecond, 1e-9);
        assertEquals(0.0, feedback.vyMetersPerSecond, 1e-9);
        assertEquals(0.2, feedback.omegaRadiansPerSecond, 1e-9);
    }

    @Test
    void rotateToTagStopsFeedbackWhenNoTargetIsPresent() {
        RobotSpeeds speeds = new RobotSpeeds(4.0, 3.0);
        speeds.setSpeeds(RobotSpeeds.FEEDBACK_SOURCE, 0.0, 0.0, 1.5);

        VisionCamera camera = VisionCommandExamples.createYawOnlyCamera("front", () -> false, () -> Double.NaN);
        PIDController headingController = VisionCommandExamples.createHeadingController(0.4, 0.0, 0.0, 1.0);
        RotateToTagCommand command = VisionCommandExamples.rotateToTag(
                speeds,
                camera,
                headingController,
                0.0,
                1.5,
                0.0);

        command.initialize();
        command.execute();

        ChassisSpeeds feedback = speeds.getInputSpeeds(RobotSpeeds.FEEDBACK_SOURCE);
        assertEquals(0.0, feedback.vxMetersPerSecond, 1e-9);
        assertEquals(0.0, feedback.vyMetersPerSecond, 1e-9);
        assertEquals(0.0, feedback.omegaRadiansPerSecond, 1e-9);
    }

    @Test
    void rotateToTagFinishesWhenHeadingIsWithinTolerance() {
        RobotSpeeds speeds = new RobotSpeeds(4.0, 3.0);
        VisionCamera camera = VisionCommandExamples.createYawOnlyCamera("front", () -> true, () -> 0.0);
        PIDController headingController = VisionCommandExamples.createHeadingController(0.4, 0.0, 0.0, 0.25);
        RotateToTagCommand command = VisionCommandExamples.rotateToTag(
                speeds,
                camera,
                headingController,
                0.0,
                1.0,
                0.0);

        command.initialize();
        command.execute();

        assertTrue(command.isFinished());
    }
}
