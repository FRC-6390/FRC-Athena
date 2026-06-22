package ca.frc6390.athena.examples;

import ca.frc6390.athena.localization.config.Localizations;
import ca.frc6390.athena.localization.runtime.VisionPoseEstimate;
import ca.frc6390.athena.localization.runtime.VisionPoseEstimator;
import ca.frc6390.athena.localization.spec.LocalizationSpec;
import ca.frc6390.athena.runtime.filter.PoseSnapshot;
import ca.frc6390.athena.vision.spec.CameraMountPose;
import ca.frc6390.athena.vision.spec.VisionFrame;
import ca.frc6390.athena.vision.spec.VisionObservation;
import java.util.Map;

/**
 * Example localization declaration with no WPILib or camera vendor imports.
 */
public final class LocalizationExample {
    /**
     * Robot pose-estimation configuration.
     */
    public static final LocalizationSpec ROBOT_POSE = Localizations.localization("robotPose", localization -> localization
            .vision(vision -> vision
                    .standardDeviations(0.8, 0.8, 0.65)
                    .multiTagScale(0.45))
            .slip(slip -> slip.enabled(0.22, 0.4))
            .fieldBounds("chargedUp", 0.0, 0.0, 16.54, 8.02)
            .poseAlias("subwooferCenter", 1.36, 5.55, 0.0)
            .poseAlias("ampScore", 1.82, 7.66, Math.PI / 2.0));

    private LocalizationExample() {
    }

    /**
     * Estimates a robot pose from a generic AprilTag camera frame.
     *
     * @return camera-derived pose measurement
     */
    public static VisionPoseEstimate cameraEstimate() {
        var estimator = new VisionPoseEstimator(
                ROBOT_POSE,
                new CameraMountPose(0.3, 0.0, 0.45, 0.0, -15.0, 0.0),
                Map.of(7, new PoseSnapshot(5.0, 2.0, 0.0)),
                () -> 0.0,
                () -> 12.5);
        return estimator.estimate(VisionFrame.of(VisionObservation.tag(7, 0.0, 0.0, 2.0, 0.92)))
                .orElseThrow();
    }
}
