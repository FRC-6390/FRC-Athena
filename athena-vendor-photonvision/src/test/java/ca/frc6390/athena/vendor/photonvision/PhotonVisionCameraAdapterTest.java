package ca.frc6390.athena.vendor.photonvision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import java.util.Optional;
import org.junit.jupiter.api.Test;

import ca.frc6390.athena.api.hardware.CameraKinds;
import ca.frc6390.athena.runtime.filter.PoseSnapshot;
import ca.frc6390.athena.runtime.measurement.Measurement;
import ca.frc6390.athena.runtime.measurement.PoseMeasurementSample;
import ca.frc6390.athena.runtime.measurement.TargetMeasurementSample;
import ca.frc6390.athena.vision.config.Cameras;
import ca.frc6390.athena.vision.device.PhotonVisionDevice;
import ca.frc6390.athena.vision.signal.PhotonVisionPoseSignal;

class PhotonVisionCameraAdapterTest {
    @Test
    void poseSignalMetadataRecordsConfiguredStrategy() {
        PhotonVisionDevice device = Cameras.photonVision("front");

        PhotonVisionPoseSignal defaultSignal = device.pose();
        PhotonVisionPoseSignal multiTag = defaultSignal.multiTagOnCoprocessor();
        PhotonVisionPoseSignal lowestAmbiguity = defaultSignal.lowestAmbiguity();

        assertTrue(defaultSignal.metadata().isEmpty());
        assertEquals("multi-tag-coprocessor", multiTag.strategy());
        assertEquals("multi-tag-coprocessor", multiTag.metadata().get("strategy"));
        assertEquals("lowest-ambiguity", lowestAmbiguity.strategy());
        assertEquals("lowest-ambiguity", lowestAmbiguity.metadata().get("strategy"));
    }

    @Test
    void targetValuesConvertToTargetMeasurements() {
        Object source = new Object();
        List<Measurement> measurements = PhotonVisionCameraAdapter.measurementsFromTargets(
                List.of(
                        PhotonVisionTarget.aprilTag(7, 12.5, -3.0, 4.25, 0.2),
                        PhotonVisionTarget.aprilTag(8, -2.0, 1.0, 1.5, Double.POSITIVE_INFINITY)),
                source);

        assertEquals(2, measurements.size());
        TargetMeasurementSample first = (TargetMeasurementSample) measurements.get(0);
        TargetMeasurementSample second = (TargetMeasurementSample) measurements.get(1);

        assertEquals(7, first.targetId());
        assertEquals(12.5, first.yawDegrees(), 1.0e-9);
        assertEquals(-3.0, first.pitchDegrees(), 1.0e-9);
        assertEquals(4.25, first.distanceMeters(), 1.0e-9);
        assertEquals(0.8, first.confidence(), 1.0e-9);
        assertEquals(source, first.source());
        assertEquals(8, second.targetId());
        assertEquals(0.0, second.confidence(), 1.0e-9);
    }

    @Test
    void emptyOrNullTargetsConvertToNoMeasurements() {
        assertTrue(PhotonVisionCameraAdapter.measurementsFromTargets(null, this).isEmpty());
        assertTrue(PhotonVisionCameraAdapter.measurementsFromTargets(List.of(), this).isEmpty());
    }

    @Test
    void resultTimestampsPropagateThroughFakeableResultSeam() {
        Object source = new Object();
        PhotonVisionCameraAdapter.PhotonVisionResult result = new PhotonVisionCameraAdapter.PhotonVisionResult(
                42.25,
                0.035,
                List.of(PhotonVisionTarget.aprilTag(7, 12.5, -3.0, 4.25, 0.2)),
                Optional.of(new PhotonVisionCameraAdapter.PhotonVisionPoseEstimate(
                        new PoseSnapshot(2.0, 3.0, 0.5),
                        42.25,
                        0.2,
                        2)));

        TargetMeasurementSample target =
                (TargetMeasurementSample) PhotonVisionCameraAdapter.targetMeasurementsFromResult(result, source).get(0);
        PoseMeasurementSample pose =
                (PoseMeasurementSample) PhotonVisionCameraAdapter.poseMeasurementsFromResult(result, source).get(0);

        assertEquals(42.25, target.timestampSeconds(), 1.0e-9);
        assertEquals(0.035, target.latencySeconds(), 1.0e-9);
        assertEquals(42.25, pose.timestampSeconds(), 1.0e-9);
        assertEquals(0.035, pose.latencySeconds(), 1.0e-9);
        assertEquals(2.0, pose.pose().xMeters(), 1.0e-9);
        assertEquals(3.0, pose.pose().yMeters(), 1.0e-9);
        assertEquals(0.5, pose.pose().headingRadians(), 1.0e-9);
        assertEquals(0.2, pose.ambiguity(), 1.0e-9);
        assertEquals(2, pose.targetCount());
        assertEquals(source, pose.source());
    }

    @Test
    void poseSignalUsesLatestUnreadEstimatedPose() {
        PhotonVisionDevice device = Cameras.photonVision("front");
        FakePhotonClient client = new FakePhotonClient(List.of(
                new PhotonVisionCameraAdapter.PhotonVisionResult(
                        1.0,
                        0.010,
                        List.of(),
                        Optional.of(new PhotonVisionCameraAdapter.PhotonVisionPoseEstimate(
                                new PoseSnapshot(1.0, 2.0, 0.25),
                                1.0,
                                0.4,
                                1))),
                new PhotonVisionCameraAdapter.PhotonVisionResult(
                        2.0,
                        0.020,
                        List.of(),
                        Optional.of(new PhotonVisionCameraAdapter.PhotonVisionPoseEstimate(
                                new PoseSnapshot(3.0, 4.0, 0.75),
                                2.0,
                                0.1,
                                3)))));
        PhotonVisionCameraAdapter adapter = new PhotonVisionCameraAdapter(device, client, result -> Optional.empty());

        List<Measurement> measurements = adapter.poseSignal().measurements();

        assertEquals(1, measurements.size());
        PoseMeasurementSample pose = (PoseMeasurementSample) measurements.get(0);
        assertEquals(3.0, pose.pose().xMeters(), 1.0e-9);
        assertEquals(4.0, pose.pose().yMeters(), 1.0e-9);
        assertEquals(0.75, pose.pose().headingRadians(), 1.0e-9);
        assertEquals(2.0, pose.timestampSeconds(), 1.0e-9);
        assertEquals(0.020, pose.latencySeconds(), 1.0e-9);
        assertEquals(0.1, pose.ambiguity(), 1.0e-9);
        assertEquals(3, pose.targetCount());
        assertEquals(device, pose.source());
    }

    @Test
    void supportAndUnconfiguredAdapterBehaviorAreExplicit() {
        PhotonVisionCameraAdapter adapter = new PhotonVisionCameraAdapter();

        assertTrue(adapter.supports(CameraKinds.PHOTONVISION));
        assertTrue(adapter.supports(Cameras.photonVision("front")));
        assertThrows(IllegalStateException.class, adapter::latestTargets);
        assertThrows(IllegalStateException.class, adapter::targetSignal);
        assertThrows(IllegalStateException.class, adapter::latestPoses);
        assertThrows(IllegalStateException.class, adapter::poseSignal);
    }

    @Test
    void poseSignalRequiresPoseEstimatorConfiguration() {
        PhotonVisionCameraAdapter adapter = new PhotonVisionCameraAdapter(
                Cameras.photonVision("front"),
                new FakePhotonClient(List.of()));

        assertThrows(IllegalStateException.class, adapter::latestPoses);
        assertThrows(IllegalStateException.class, adapter::poseSignal);
    }

    private record FakePhotonClient(List<PhotonVisionCameraAdapter.PhotonVisionResult> results)
            implements PhotonVisionCameraAdapter.PhotonClient {
        @Override
        public List<PhotonVisionCameraAdapter.PhotonVisionResult> unreadResults(
                PhotonVisionCameraAdapter.PhotonPoseClient poseClient) {
            return results;
        }

        @Override
        public void close() {
        }
    }
}
