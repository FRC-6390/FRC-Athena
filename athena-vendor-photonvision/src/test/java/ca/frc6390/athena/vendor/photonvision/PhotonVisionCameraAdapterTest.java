package ca.frc6390.athena.vendor.photonvision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicBoolean;
import org.junit.jupiter.api.Test;

import ca.frc6390.athena.api.hardware.CameraKinds;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;
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
    void multiTagStrategyFallsBackToSingleTagLowestAmbiguityPose() {
        AtomicBoolean fallbackRead = new AtomicBoolean();

        Optional<String> estimate = PhotonVisionCameraAdapter.multiTagThenSingleTag(
                Optional.empty(),
                () -> {
                    fallbackRead.set(true);
                    return Optional.of("single-tag");
                });

        assertEquals(Optional.of("single-tag"), estimate);
        assertTrue(fallbackRead.get());
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

    @Test
    void singleTagFallbackReportsOnlyTheTagThatProducedThePose() {
        Pose3d robot = new Pose3d(2.0, 0.0, 0.0, new Rotation3d());
        Pose3d tagOne = new Pose3d(5.0, 0.0, 1.0, new Rotation3d(0.0, 0.0, Math.PI));
        Pose3d tagTwo = new Pose3d(5.0, 1.0, 1.0, new Rotation3d(0.0, 0.0, Math.PI));
        List<PhotonTrackedTarget> visible = List.of(
                target(1, new Transform3d(robot, tagOne), 0.30),
                target(2, new Transform3d(robot, tagTwo), 0.10));
        // PhotonLib's lowest-ambiguity fallback reports every visible target in targetsUsed.
        EstimatedRobotPose fallback = new EstimatedRobotPose(
                robot, 1.0, visible, PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);

        PhotonVisionCameraAdapter.PhotonVisionPoseEstimate estimate =
                PhotonVisionCameraAdapter.PhotonPoseEstimatorClient.toEstimate(
                        fallback, layout(tagOne, tagTwo), new Transform3d());

        assertEquals(1, estimate.targetCount());
        assertEquals(0.10, estimate.ambiguity(), 1.0e-9);
        assertEquals(new Transform3d(robot, tagTwo).getTranslation().getNorm(),
                estimate.averageTargetDistanceMeters(), 1.0e-9);
    }

    @Test
    void multiTagEstimateKeepsAllTargets() {
        Pose3d robot = new Pose3d(2.0, 0.0, 0.0, new Rotation3d());
        Pose3d tagOne = new Pose3d(5.0, 0.0, 1.0, new Rotation3d(0.0, 0.0, Math.PI));
        Pose3d tagTwo = new Pose3d(5.0, 1.0, 1.0, new Rotation3d(0.0, 0.0, Math.PI));
        EstimatedRobotPose multi = new EstimatedRobotPose(
                robot,
                1.0,
                List.of(target(1, new Transform3d(robot, tagOne), 0.30),
                        target(2, new Transform3d(robot, tagTwo), 0.10)),
                PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR);

        PhotonVisionCameraAdapter.PhotonVisionPoseEstimate estimate =
                PhotonVisionCameraAdapter.PhotonPoseEstimatorClient.toEstimate(
                        multi, layout(tagOne, tagTwo), new Transform3d());

        assertEquals(2, estimate.targetCount());
        assertTrue(Double.isNaN(estimate.averageTargetDistanceMeters()));
    }

    @Test
    void referencePoseStrategyReportsTheTagWhoseSolveProducedThePose() {
        Pose3d robot = new Pose3d(2.0, 0.0, 0.0, new Rotation3d());
        Pose3d wrongRobot = new Pose3d(4.0, 3.0, 0.0, new Rotation3d());
        Pose3d tagOne = new Pose3d(5.0, 0.0, 1.0, new Rotation3d(0.0, 0.0, Math.PI));
        Pose3d tagTwo = new Pose3d(5.0, 1.0, 1.0, new Rotation3d(0.0, 0.0, Math.PI));
        // Tag one's alternate solve produced the pose; tag two has the lower ambiguity.
        EstimatedRobotPose closest = new EstimatedRobotPose(
                robot,
                1.0,
                List.of(target(2, new Transform3d(wrongRobot, tagTwo), new Transform3d(wrongRobot, tagTwo), 0.05),
                        target(1, new Transform3d(wrongRobot, tagOne), new Transform3d(robot, tagOne), 0.25)),
                PhotonPoseEstimator.PoseStrategy.CLOSEST_TO_REFERENCE_POSE);

        PhotonVisionCameraAdapter.PhotonVisionPoseEstimate estimate =
                PhotonVisionCameraAdapter.PhotonPoseEstimatorClient.toEstimate(
                        closest, layout(tagOne, tagTwo), new Transform3d());

        assertEquals(1, estimate.targetCount());
        assertEquals(0.25, estimate.ambiguity(), 1.0e-9);
    }

    @Test
    void singleTagEstimateDistanceOverridesAverageOfAllVisibleTargets() {
        PhotonVisionCameraAdapter.PhotonVisionResult result = new PhotonVisionCameraAdapter.PhotonVisionResult(
                1.0,
                0.01,
                List.of(PhotonVisionTarget.aprilTag(1, 0.0, 0.0, 2.0, 0.3),
                        PhotonVisionTarget.aprilTag(2, 0.0, 0.0, 6.0, 0.1)),
                Optional.of(new PhotonVisionCameraAdapter.PhotonVisionPoseEstimate(
                        new PoseSnapshot(1.0, 1.0, 0.0), 1.0, 0.1, 1, 6.0)));

        PoseMeasurementSample pose =
                (PoseMeasurementSample) PhotonVisionCameraAdapter.poseMeasurementsFromResult(result, this).get(0);

        assertEquals(6.0, pose.averageTargetDistanceMeters(), 1.0e-9);
        assertEquals(1, pose.targetCount());
    }

    private static AprilTagFieldLayout layout(Pose3d... tags) {
        List<AprilTag> layoutTags = new java.util.ArrayList<>();
        for (int index = 0; index < tags.length; index++) {
            layoutTags.add(new AprilTag(index + 1, tags[index]));
        }
        return new AprilTagFieldLayout(layoutTags, 16.0, 8.0);
    }

    private static PhotonTrackedTarget target(int id, Transform3d cameraToTarget, double ambiguity) {
        return target(id, cameraToTarget, cameraToTarget, ambiguity);
    }

    private static PhotonTrackedTarget target(
            int id, Transform3d best, Transform3d alternate, double ambiguity) {
        return new PhotonTrackedTarget(
                0.0, 0.0, 1.0, 0.0, id, -1, -1.0f, best, alternate, ambiguity, corners(), corners());
    }

    private static List<TargetCorner> corners() {
        return List.of(new TargetCorner(0, 0), new TargetCorner(1, 0), new TargetCorner(1, 1), new TargetCorner(0, 1));
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
