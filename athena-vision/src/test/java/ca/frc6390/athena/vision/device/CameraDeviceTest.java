package ca.frc6390.athena.vision.device;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotSame;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;
import org.junit.jupiter.api.Test;

import ca.frc6390.athena.api.hardware.CameraKinds;
import ca.frc6390.athena.runtime.filter.PoseSnapshot;
import ca.frc6390.athena.runtime.measurement.Measurement;
import ca.frc6390.athena.runtime.measurement.Measurements;
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
    void nullSuppliersAndNullSupplierResultsReadAsEmpty() {
        CameraDevice nullSuppliers = Cameras.photonVision("front").bindPose(null).bindTargets(null);
        CameraDevice nullResults = Cameras.photonVision("rear")
                .bindPose(() -> null)
                .bindTargets(() -> null);

        assertTrue(nullSuppliers.pose().measurements().isEmpty());
        assertTrue(nullSuppliers.targets().measurements().isEmpty());
        assertTrue(nullResults.pose().measurements().isEmpty());
        assertTrue(nullResults.targets().measurements().isEmpty());
    }

    @Test
    void visionGraphCachesPoseAndTargetSignalsUntilRefresh() {
        AtomicInteger poseReads = new AtomicInteger();
        AtomicInteger targetReads = new AtomicInteger();
        Measurement firstPose = Measurements.pose(new PoseSnapshot(1.0, 0.0, 0.0));
        Measurement secondPose = Measurements.pose(new PoseSnapshot(2.0, 0.0, 0.0));
        Measurement firstTarget = Measurements.custom("target-1", null);
        Measurement secondTarget = Measurements.custom("target-2", null);
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

        graph.refresh();
        List<Measurement> graphTargets = graph.targetMeasurements();
        assertEquals(List.of(firstPose), cachedPoseSignal.measurements());
        assertEquals(List.of(firstTarget), graphTargets);
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
        assertSame(camera, cachedPoseSignal.camera());
        assertSame(camera, cachedTargetSignal.camera());
    }
}
