package ca.frc6390.athena.vendor.limelight;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import java.util.Map;
import org.junit.jupiter.api.Test;

import ca.frc6390.athena.runtime.filter.PoseSnapshot;
import ca.frc6390.athena.runtime.measurement.Measurement;
import ca.frc6390.athena.runtime.measurement.PoseMeasurementSample;
import ca.frc6390.athena.runtime.measurement.TargetMeasurementSample;
import ca.frc6390.athena.vision.config.Cameras;
import ca.frc6390.athena.vision.device.LimelightDevice;
import ca.frc6390.athena.vision.signal.LimelightPoseSignal;
import ca.frc6390.athena.vision.runtime.VisionGraph;

class LimelightCameraAdapterTest {
    @Test
    void poseSignalMetadataRecordsModeTagsAndTrustDistance() {
        LimelightDevice device = Cameras.limelight("limelight-front");
        LimelightPoseSignal signal = device.pose()
                .megatag2Blue()
                .tags(3, 4)
                .trustDistance(5.5);
        Map<String, Object> metadata = signal.metadata();

        assertEquals("megatag2-blue", signal.poseMode());
        assertEquals(List.of(3, 4), signal.tags());
        assertEquals(5.5, signal.trustDistanceMeters(), 1.0e-9);
        assertEquals("megatag2-blue", metadata.get("poseMode"));
        assertEquals(List.of(3, 4), metadata.get("tags"));
        assertEquals(5.5, (Double) metadata.get("trustDistanceMeters"), 1.0e-9);
    }

    @Test
    void targetValuesClampInvalidNumbersAndNoTargetDefaults() {
        LimelightTarget noTarget = LimelightTarget.noTarget();
        LimelightTarget target = LimelightTarget.aprilTag(
                5,
                Double.NaN,
                Double.POSITIVE_INFINITY,
                -2.0,
                -1.0);

        assertEquals(-1, noTarget.tagId());
        assertTrue(!noTarget.hasTarget());
        assertEquals(0.0, target.txDegrees(), 1.0e-9);
        assertEquals(0.0, target.tyDegrees(), 1.0e-9);
        assertEquals(0.0, target.distanceMeters(), 1.0e-9);
        assertEquals(0.0, target.targetArea(), 1.0e-9);
    }

    @Test
    void fakeFrameConvertsBotPoseAndTargetToMeasurements() {
        Object source = new Object();
        LimelightCameraAdapter.LimelightFrame frame = frame(
                LimelightTarget.aprilTag(11, 6.0, -2.0, 3.5, 0.45),
                new double[] {1.0, 2.0, 0.0, 0.0, 0.0, 90.0, 25.0});

        List<Measurement> poseMeasurements = invokeMeasurements("measurementsFromPose", source, frame);
        List<Measurement> targetMeasurements = invokeMeasurements("measurementsFromTarget", source, frame);

        assertEquals(1, poseMeasurements.size());
        PoseMeasurementSample pose = (PoseMeasurementSample) poseMeasurements.get(0);
        assertPose(pose.pose(), 1.0, 2.0, Math.PI / 2.0);
        assertEquals(0.025, pose.latencySeconds(), 1.0e-9);
        assertEquals(1, pose.targetCount());
        assertEquals(source, pose.source());

        assertEquals(1, targetMeasurements.size());
        TargetMeasurementSample target = (TargetMeasurementSample) targetMeasurements.get(0);
        assertEquals(11, target.targetId());
        assertEquals(6.0, target.yawDegrees(), 1.0e-9);
        assertEquals(-2.0, target.pitchDegrees(), 1.0e-9);
        assertEquals(3.5, target.distanceMeters(), 1.0e-9);
        assertEquals(0.45, target.confidence(), 1.0e-9);
        assertEquals(source, target.source());
    }

    @Test
    void missingPoseOrNoTargetProducesNoMeasurements() {
        Object source = new Object();
        LimelightCameraAdapter.LimelightFrame frame = frame(LimelightTarget.noTarget(), new double[] {1.0, 2.0});

        assertTrue(invokeMeasurements("measurementsFromPose", source, frame).isEmpty());
        assertTrue(invokeMeasurements("measurementsFromTarget", source, frame).isEmpty());
    }

    @Test
    void boundDeviceSharesOneFramePerVisionGraphRefresh() {
        CountingClient client = new CountingClient();
        LimelightDevice bound = new LimelightCameraAdapter(client).bind(Cameras.limelight("front"));
        VisionGraph graph = VisionGraph.of(bound);

        graph.refresh();

        assertEquals(1, client.calls);
        assertEquals(1, graph.poseMeasurements().size());
        assertEquals(1, graph.targetMeasurements().size());
    }

    private static LimelightCameraAdapter.LimelightFrame frame(LimelightTarget target, double[] botPoseBlue) {
        return new LimelightCameraAdapter.LimelightFrame(target, botPoseBlue);
    }

    private static List<Measurement> invokeMeasurements(
            String methodName,
            Object source,
            LimelightCameraAdapter.LimelightFrame frame) {
        return switch (methodName) {
            case "measurementsFromPose" -> LimelightCameraAdapter.measurementsFromPose(source, frame);
            case "measurementsFromTarget" -> LimelightCameraAdapter.measurementsFromTarget(source, frame);
            default -> List.of();
        };
    }

    private static List<Measurement> castMeasurements(Object value) {
        if (!(value instanceof List<?> values)) {
            return List.of();
        }
        return values.stream()
                .map(Measurement.class::cast)
                .toList();
    }

    private static void assertPose(PoseSnapshot pose, double xMeters, double yMeters, double headingRadians) {
        assertEquals(xMeters, pose.xMeters(), 1.0e-9);
        assertEquals(yMeters, pose.yMeters(), 1.0e-9);
        assertEquals(headingRadians, pose.headingRadians(), 1.0e-9);
    }

    private static final class CountingClient implements LimelightCameraAdapter.LimelightClient {
        private int calls;

        @Override
        public LimelightCameraAdapter.LimelightFrame latestFrame() {
            calls++;
            return frame(
                    LimelightTarget.aprilTag(7, 1.0, 2.0, 3.0, 0.4),
                    new double[] {1.0, 2.0, 0.0, 0.0, 0.0, 45.0, 20.0});
        }
    }
}
