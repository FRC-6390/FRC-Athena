package ca.frc6390.athena.vision.device;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotSame;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;
import org.junit.jupiter.api.Test;

import ca.frc6390.athena.api.hardware.CameraKinds;
import ca.frc6390.athena.api.FailurePolicy;
import ca.frc6390.athena.runtime.filter.PoseSnapshot;
import ca.frc6390.athena.runtime.measurement.Measurement;
import ca.frc6390.athena.runtime.measurement.Measurements;
import ca.frc6390.athena.runtime.measurement.PoseMeasurementSample;
import ca.frc6390.athena.runtime.measurement.TargetMeasurementSample;
import ca.frc6390.athena.vision.config.Cameras;
import ca.frc6390.athena.vision.runtime.VisionGraph;
import ca.frc6390.athena.vision.signal.PoseSignal;
import ca.frc6390.athena.vision.signal.TargetSignal;
class CameraDeviceTest {
    @Test
    void defaultsNameToKindKeyAndUsesIdentityMount() {
        CameraDevice camera = Cameras.camera(CameraKinds.LIMELIGHT, " ");

        assertEquals(CameraKinds.LIMELIGHT, camera.kind());
        assertEquals(CameraKinds.LIMELIGHT.key(), camera.name());
        assertEquals(CameraMountPose.identity(), camera.mountPose());
    }

    @Test
    void mountCopiesConfigurationAndRuntimeBindingReachesPreviouslyDeclaredSignals() {
        CameraDevice original = Cameras.photonVision("front");
        PoseSignal declaredSignal = original.pose();
        CameraMountPose mount = new CameraMountPose(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
        Measurement pose = Measurements.pose(new PoseSnapshot(2.0, 3.0, 0.5));

        CameraDevice mounted = original.mount(mount);
        CameraDevice bound = mounted.bindPose(() -> List.of(pose));

        assertNotSame(original, mounted);
        assertNotSame(mounted, bound);
        assertEquals(CameraMountPose.identity(), original.mountPose());
        assertEquals(mount, mounted.mountPose());
        assertEquals(List.of(pose), declaredSignal.measurements());
        assertEquals(List.of(pose), bound.pose().measurements());
    }

    @Test
    void dynamicMountIsEvaluatedWhenTheCameraPoseIsRead() {
        CameraMountPose[] mount = {CameraMountPose.identity()};
        PhotonVisionDevice camera = Cameras.photonVision("turret").mount(() -> mount[0]);

        assertEquals(CameraMountPose.identity(), camera.mountPose());

        mount[0] = new CameraMountPose(0.1, -0.2, 0.3, 45.0, 0.0, 0.0);

        assertEquals(mount[0], camera.mountPose());
    }

    @Test
    void nullSuppliersAndNullSupplierResultsReadAsEmpty() {
        CameraDevice nullSuppliers = Cameras.photonVision("front").bindPose(null).bindTargets(null);
        CameraDevice nullResults = Cameras.photonVision("rear")
                .bindPose(() -> null)
                .bindTargets(() -> null);

        assertTrue(nullSuppliers.pose().measurements().isEmpty());
        assertTrue(nullSuppliers.targets().measurements().isEmpty());
        assertTrue(nullResults.pose().measurements().isEmpty());
        assertTrue(nullResults.targets().measurements().isEmpty());
        assertTrue(nullResults.pose().measurement().isEmpty());
        assertTrue(nullResults.pose().value().isEmpty());
        assertTrue(nullResults.targets().latest().isEmpty());
    }

    @Test
    void visionGraphCachesPoseAndTargetSignalsUntilRefresh() {
        AtomicInteger poseReads = new AtomicInteger();
        AtomicInteger targetReads = new AtomicInteger();
        PoseMeasurementSample firstPose = Measurements.pose(new PoseSnapshot(1.0, 0.0, 0.0));
        PoseMeasurementSample secondPose = Measurements.pose(new PoseSnapshot(2.0, 0.0, 0.0));
        TargetMeasurementSample firstTarget = Measurements.target(1, 10.0, 2.0, 3.0, 0.9);
        TargetMeasurementSample secondTarget = Measurements.target(2, 20.0, 4.0, 6.0, 0.8);
        CameraDevice camera = Cameras.photonVision("cache")
                .bindPose(() -> List.of(poseReads.incrementAndGet() == 1 ? firstPose : secondPose))
                .bindTargets(() -> List.of(targetReads.incrementAndGet() == 1 ? firstTarget : secondTarget));
        VisionGraph graph = VisionGraph.of(camera);
        VisionGraph.CameraRuntime runtime = graph.camera(camera);
        PoseSignal cachedPoseSignal = runtime.poseSignal();
        TargetSignal cachedTargetSignal = runtime.targetSignal();

        assertSame(graph.cameras(), graph.cameras());
        assertSame(cachedPoseSignal, runtime.poseSignal());
        assertSame(cachedTargetSignal, runtime.targetSignal());
        assertTrue(cachedPoseSignal.measurements().isEmpty());
        assertTrue(cachedTargetSignal.measurements().isEmpty());
        assertTrue(camera.pose().value().isEmpty());
        assertTrue(camera.targets().latest().isEmpty());
        assertEquals(0, poseReads.get());
        assertEquals(0, targetReads.get());

        graph.refresh();
        List<Measurement> graphTargets = graph.targetMeasurements();
        assertEquals(List.of(firstPose), cachedPoseSignal.measurements());
        assertEquals(List.of(firstTarget), graphTargets);
        assertEquals(firstPose, camera.pose().measurement().orElseThrow());
        assertEquals(firstPose.pose(), camera.pose().value().orElseThrow());
        assertEquals(firstTarget, camera.targets().latest().orElseThrow());
        assertEquals(List.of(firstTarget), camera.targets().values());
        assertTrue(camera.targets().hasTarget());
        assertSame(graphTargets, graph.targetMeasurements());
        assertEquals(1, poseReads.get());
        assertEquals(1, targetReads.get());

        assertEquals(List.of(firstPose), cachedPoseSignal.measurements());
        assertEquals(List.of(firstTarget), cachedTargetSignal.measurements());
        assertEquals(1, poseReads.get());
        assertEquals(1, targetReads.get());

        runtime.refresh();
        assertEquals(List.of(secondPose), graph.poseMeasurements());
        assertEquals(List.of(secondTarget), cachedTargetSignal.measurements());
        assertEquals(secondPose, camera.pose().measurement().orElseThrow());
        assertEquals(secondTarget, camera.targets().latest().orElseThrow());
        assertEquals(2, poseReads.get());
        assertEquals(2, targetReads.get());
        assertSame(camera, cachedPoseSignal.camera());
        assertSame(camera, cachedTargetSignal.camera());
    }

    @Test
    void oneCameraFailureDoesNotDiscardHealthyCameraMeasurements() {
        AtomicInteger failedReads = new AtomicInteger();
        PoseMeasurementSample pose = Measurements.pose(new PoseSnapshot(3.0, 2.0, 0.25));
        CameraDevice failed = Cameras.photonVision("failed")
                .bindPose(() -> {
                    failedReads.incrementAndGet();
                    throw new IllegalStateException("camera offline");
                });
        CameraDevice healthy = Cameras.photonVision("healthy").bindPose(() -> List.of(pose));
        VisionGraph graph = VisionGraph.of(failed, healthy);

        graph.refresh();

        assertEquals(List.of(pose), graph.poseMeasurements());
        assertEquals(1, graph.refreshFailures().size());
        assertSame(failed, graph.refreshFailures().get(0).camera());

        graph.refresh();
        assertEquals(2, failedReads.get());
        assertEquals(List.of(pose), graph.poseMeasurements());
    }

    @Test
    void warningCameraRetriesAfterFailure() {
        AtomicInteger reads = new AtomicInteger();
        PoseMeasurementSample pose = Measurements.pose(new PoseSnapshot(1.0, 1.0, 0.0));
        CameraDevice camera = Cameras.photonVision("retry")
                .failurePolicy(FailurePolicy.WARN)
                .bindPose(() -> {
                    if (reads.incrementAndGet() == 1) throw new IllegalStateException("temporary outage");
                    return List.of(pose);
                });
        VisionGraph graph = VisionGraph.of(camera);

        graph.refresh();
        assertTrue(graph.poseMeasurements().isEmpty());
        graph.refresh();

        assertEquals(List.of(pose), graph.poseMeasurements());
        assertEquals(2, reads.get());
    }

    @Test
    void disabledDevicePolicyCameraRetriesAndRecoversAfterAnOutage() {
        AtomicInteger reads = new AtomicInteger();
        PoseMeasurementSample pose = Measurements.pose(new PoseSnapshot(2.0, 3.0, 0.5));
        CameraDevice camera = Cameras.photonVision("recover")
                .failurePolicy(FailurePolicy.DISABLE_DEVICE)
                .bindPose(() -> {
                    if (reads.incrementAndGet() == 1) throw new IllegalStateException("temporary outage");
                    return List.of(pose);
                });
        VisionGraph graph = VisionGraph.of(camera);

        graph.refresh();
        assertTrue(graph.poseMeasurements().isEmpty());
        graph.refresh();

        assertEquals(List.of(pose), graph.poseMeasurements());
        assertTrue(graph.refreshFailures().isEmpty());
    }
}
