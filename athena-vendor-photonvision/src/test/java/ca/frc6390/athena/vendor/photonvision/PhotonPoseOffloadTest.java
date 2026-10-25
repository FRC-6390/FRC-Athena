package ca.frc6390.athena.vendor.photonvision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.assertTimeoutPreemptively;
import ca.frc6390.athena.runtime.filter.PoseSnapshot;
import ca.frc6390.athena.vision.device.CameraMountPose;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.Units;
import java.time.Duration;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicInteger;
import org.junit.jupiter.api.Test;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

class PhotonPoseOffloadTest {
    @Test
    void heldSolverLeavesTargetReadsResponsiveAndKeepsOnlyTheLatestPendingPose() throws Exception {
        ConcurrentLinkedQueue<PhotonPipelineResult> frames = new ConcurrentLinkedQueue<>();
        CountDownLatch entered = new CountDownLatch(1);
        CountDownLatch release = new CountDownLatch(1);
        ConcurrentLinkedQueue<Double> solved = new ConcurrentLinkedQueue<>();
        var source = new PhotonVisionCameraAdapter.PhotonCameraClient.FrameSource() {
            @Override public List<PhotonPipelineResult> unreadResults() {
                PhotonPipelineResult frame = frames.poll();
                return frame == null ? List.of() : List.of(frame);
            }
            @Override public boolean isConnected() { return true; }
            @Override public void close() { }
        };
        var client = new PhotonVisionCameraAdapter.PhotonCameraClient(source, true);
        PhotonVisionCameraAdapter.PhotonPoseClient pose = frame -> {
            double timestamp = frame.getTimestampSeconds();
            solved.add(timestamp);
            assertTrue(Thread.currentThread().getName().equals("athena-photonvision-solver"));
            if (timestamp == 1.0) {
                entered.countDown();
                try {
                    if (!release.await(5, TimeUnit.SECONDS)) throw new IllegalStateException("test solver gate timed out");
                } catch (InterruptedException exception) {
                    Thread.currentThread().interrupt();
                    throw new IllegalStateException(exception);
                }
            }
            return Optional.of(new PhotonVisionCameraAdapter.PhotonVisionPoseEstimate(
                    new PoseSnapshot(timestamp, 0, 0), timestamp, 0.1, 1, 2.0));
        };
        try {
            frames.add(frame(1.0));
            assertEquals(1.0, nextTarget(client, pose).timestampSeconds());
            assertTrue(entered.await(2, TimeUnit.SECONDS));
            frames.add(frame(2.0));
            assertEquals(2.0, nextTarget(client, pose).timestampSeconds());
            frames.add(frame(3.0));
            assertEquals(3.0, nextTarget(client, pose).timestampSeconds());
            assertTimeoutPreemptively(Duration.ofMillis(200), () -> {
                for (int cycle = 0; cycle < 1_000; cycle++) {
                    assertTrue(client.unreadResults(pose).isEmpty());
                    assertTrue(client.completedPoseResults().isEmpty());
                    assertTrue(client.isConnected());
                }
            });
            assertEquals(List.of(1.0), List.copyOf(solved));
            assertEquals(1L, client.droppedFrames(), "intermediate pending pose replaced by newest frame");
            release.countDown();
            var completed = newestPose(client, 3.0);
            assertEquals(List.of(1.0, 3.0), List.copyOf(solved));
            assertEquals(3.0, completed.pose().orElseThrow().timestampSeconds());
            assertEquals(0.020, completed.latencySeconds(), 1e-9, "pose keeps its source frame's latency");
        } finally {
            release.countDown();
            client.close();
        }
    }

    @Test
    void robotCapturesMovingMountReferenceAndHeadingBeforeWorkerTouchesTheEstimator() throws Exception {
        Thread robotThread = Thread.currentThread();
        RecordingEstimator estimator = new RecordingEstimator();
        Pose2d[] reference = {new Pose2d(1.0, 2.0, new Rotation2d(0.25))};
        double[] mountX = {1.0};
        AtomicInteger supplierReads = new AtomicInteger();
        var client = new PhotonVisionCameraAdapter.PhotonPoseEstimatorClient(estimator,
                PhotonPoseEstimator.PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
                () -> {
                    assertSame(robotThread, Thread.currentThread());
                    supplierReads.incrementAndGet();
                    return new CameraMountPose(Units.Meters.of(mountX[0]), Units.Meters.zero(), Units.Meters.zero(),
                            Units.Radians.zero(), Units.Radians.zero(), Units.Radians.zero());
                },
                () -> {
                    assertSame(robotThread, Thread.currentThread());
                    supplierReads.incrementAndGet();
                    return reference[0];
                },
                () -> { assertSame(robotThread, Thread.currentThread()); return true; });
        client.beginCycle(10.0);
        var first = client.snapshot();
        reference[0] = new Pose2d(3.0, 4.0, new Rotation2d(0.5));
        mountX[0] = 2.0;
        client.beginCycle(10.02);
        var second = client.snapshot();
        reference[0] = new Pose2d(99.0, 99.0, new Rotation2d(2.0));
        mountX[0] = 99.0;
        assertTrue(estimator.mutationThreads.isEmpty(), "capture never mutates the shared Photon estimator");
        int readsBeforeSolve = supplierReads.get();
        var worker = Executors.newSingleThreadExecutor(runnable -> new Thread(runnable, "test-photon-worker"));
        try {
            var firstPose = worker.submit(() -> first.estimate(frame(9.99))).get(2, TimeUnit.SECONDS).orElseThrow();
            assertEquals(1.0, firstPose.pose().xMeters());
            assertEquals(1.0, estimator.getRobotToCameraTransform().getX());
            assertEquals(List.of(10.0), List.copyOf(estimator.headingTimes));
            var secondPose = worker.submit(() -> second.estimate(frame(10.01))).get(2, TimeUnit.SECONDS).orElseThrow();
            assertEquals(3.0, secondPose.pose().xMeters());
            assertEquals(2.0, estimator.getRobotToCameraTransform().getX());
            assertEquals(List.of(10.0, 10.02), List.copyOf(estimator.headingTimes));
            assertEquals(List.of(0.25, 0.5), List.copyOf(estimator.headings));
            assertEquals(readsBeforeSolve, supplierReads.get(), "solver never reads live robot suppliers");
            assertTrue(estimator.mutationThreads.stream().allMatch(thread -> thread.equals("test-photon-worker")));
            assertEquals(0.375, interpolatedHeading(estimator, 10.01), 1e-8,
                    "actual Photon heading buffer preserves capture-time interpolation");
        } finally {
            worker.shutdownNow();
        }
    }

    @Test
    void cyclesWithoutAReportedHeadingDropTheHeadingHistory() {
        RecordingEstimator estimator = new RecordingEstimator();
        double[] reported = {0.25};
        var client = new PhotonVisionCameraAdapter.PhotonPoseEstimatorClient(estimator,
                PhotonPoseEstimator.PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
                null,
                () -> null,
                () -> true,
                () -> reported[0]);
        client.beginCycle(10.0);
        reported[0] = Double.NaN;
        client.beginCycle(10.02);
        reported[0] = 0.5;
        client.beginCycle(10.04);
        client.snapshot().estimate(frame(10.03));

        // Photon's resetHeadingData records its sample through addHeadingData: only 10.04 is left. Keeping the
        // 10.0 sample from before the cycle without a heading would give [10.0, 10.04].
        assertEquals(List.of(10.04), List.copyOf(estimator.headingTimes),
                "a cycle without a trusted heading must not leave older samples for the trig solve");
    }

    private static final class RecordingEstimator extends PhotonPoseEstimator {
        final List<String> mutationThreads = new ArrayList<>();
        final List<Double> headingTimes = new ArrayList<>();
        final List<Double> headings = new ArrayList<>();

        RecordingEstimator() { super(new AprilTagFieldLayout(List.of(), 16.0, 8.0), new Transform3d()); }

        @Override public void setRobotToCameraTransform(Transform3d transform) {
            mutationThreads.add(Thread.currentThread().getName());
            super.setRobotToCameraTransform(transform);
        }
        @Override public void resetHeadingData(double timestamp, Rotation2d heading) {
            headingTimes.clear();
            headings.clear();
            super.resetHeadingData(timestamp, heading);
        }
        @Override public void addHeadingData(double timestamp, Rotation2d heading) {
            mutationThreads.add(Thread.currentThread().getName());
            headingTimes.add(timestamp);
            headings.add(heading.getRadians());
            super.addHeadingData(timestamp, heading);
        }
        @Override public Optional<EstimatedRobotPose> estimateClosestToReferencePose(PhotonPipelineResult result, Pose3d reference) {
            mutationThreads.add(Thread.currentThread().getName());
            return Optional.of(new EstimatedRobotPose(reference, result.getTimestampSeconds(), List.of(),
                    PhotonPoseEstimator.PoseStrategy.CLOSEST_TO_REFERENCE_POSE));
        }
    }

    @SuppressWarnings("unchecked")
    private static double interpolatedHeading(PhotonPoseEstimator estimator, double timestamp) throws Exception {
        var field = PhotonPoseEstimator.class.getDeclaredField("headingBuffer");
        field.setAccessible(true);
        var buffer = (edu.wpi.first.math.interpolation.TimeInterpolatableBuffer<Rotation2d>) field.get(estimator);
        return buffer.getSample(timestamp).orElseThrow().getRadians();
    }

    private static PhotonPipelineResult frame(double seconds) {
        PhotonPipelineResult result = new PhotonPipelineResult();
        result.metadata.captureTimestampMicros = (long) (seconds * 1_000_000L);
        result.metadata.publishTimestampMicros = result.metadata.captureTimestampMicros + 20_000L;
        return result;
    }

    private static PhotonVisionCameraAdapter.PhotonVisionResult nextTarget(
            PhotonVisionCameraAdapter.PhotonCameraClient client, PhotonVisionCameraAdapter.PhotonPoseClient pose) {
        return assertTimeoutPreemptively(Duration.ofSeconds(2), () -> {
            while (true) {
                var results = client.unreadResults(pose);
                if (!results.isEmpty()) return results.get(0);
                Thread.sleep(2);
            }
        });
    }

    private static PhotonVisionCameraAdapter.PhotonVisionResult newestPose(
            PhotonVisionCameraAdapter.PhotonCameraClient client, double timestamp) {
        return assertTimeoutPreemptively(Duration.ofSeconds(2), () -> {
            while (true) {
                var results = client.completedPoseResults();
                if (!results.isEmpty() && results.get(0).timestampSeconds() == timestamp) return results.get(0);
                Thread.sleep(2);
            }
        });
    }
}
