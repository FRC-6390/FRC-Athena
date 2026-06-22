package ca.frc6390.athena.localization;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.localization.config.Localizations;
import ca.frc6390.athena.localization.runtime.VisionPoseEstimator;
import ca.frc6390.athena.runtime.filter.PoseSnapshot;
import ca.frc6390.athena.vision.spec.CameraMountPose;
import ca.frc6390.athena.vision.spec.VisionFrame;
import ca.frc6390.athena.vision.spec.VisionObservation;
import java.util.Map;
import org.junit.jupiter.api.Test;

class VisionPoseEstimatorTest {
    @Test
    void estimatesRobotPoseFromKnownTagAndCameraMount() {
        var estimator = new VisionPoseEstimator(
                Localizations.localization("robotPose", localization -> localization
                        .vision(vision -> vision.standardDeviations(0.8, 0.9, 0.7))),
                new CameraMountPose(0.3, 0.0, 0.4, 0.0, -15.0, 0.0),
                Map.of(4, new PoseSnapshot(5.0, 2.0, 0.0)),
                () -> 0.0,
                () -> 12.5);

        var estimate = estimator.estimate(VisionFrame.of(VisionObservation.tag(4, 0.0, 0.0, 2.0, 0.9)))
                .orElseThrow();

        assertEquals(2.7, estimate.pose().xMeters(), 1.0e-9);
        assertEquals(2.0, estimate.pose().yMeters(), 1.0e-9);
        assertEquals(0.0, estimate.pose().headingRadians(), 1.0e-9);
        assertEquals(12.5, estimate.timestampSeconds(), 1.0e-9);
        assertEquals(0.8, estimate.standardDeviations().xStdDevMeters(), 1.0e-9);
        assertEquals(1, estimate.tagCount());
    }

    @Test
    void rejectsUnknownTagsAndOutOfBoundsEstimates() {
        var bounded = Localizations.localization("robotPose", localization -> localization
                .fieldBounds("field", 0.0, 0.0, 1.0, 1.0));
        var estimator = new VisionPoseEstimator(
                bounded,
                CameraMountPose.identity(),
                Map.of(4, new PoseSnapshot(5.0, 2.0, 0.0)),
                () -> 0.0,
                () -> 0.0);

        assertTrue(estimator.estimate(VisionFrame.of(VisionObservation.tag(7, 0.0, 0.0, 1.0, 0.9))).isEmpty());
        assertTrue(estimator.estimate(VisionFrame.of(VisionObservation.tag(4, 0.0, 0.0, 1.0, 0.9))).isEmpty());
    }

    @Test
    void scalesVisionWeightsForMultiTagFrames() {
        var estimator = new VisionPoseEstimator(
                Localizations.localization("robotPose", localization -> localization
                        .vision(vision -> vision.standardDeviations(1.0, 1.2, 0.8).multiTagScale(0.4))),
                CameraMountPose.identity(),
                Map.of(
                        4, new PoseSnapshot(5.0, 2.0, 0.0),
                        5, new PoseSnapshot(5.0, 3.0, 0.0)),
                () -> 0.0,
                () -> 0.0);

        var estimate = estimator.estimate(VisionFrame.of(
                        VisionObservation.tag(4, 0.0, 0.0, 2.0, 0.9),
                        VisionObservation.tag(5, 0.0, 0.0, 2.5, 0.8)))
                .orElseThrow();

        assertEquals(2, estimate.tagCount());
        assertEquals(0.4, estimate.standardDeviations().xStdDevMeters(), 1.0e-9);
        assertEquals(0.48, estimate.standardDeviations().yStdDevMeters(), 1.0e-9);
        assertEquals(0.32, estimate.standardDeviations().headingStdDevRadians(), 1.0e-9);
    }
}
