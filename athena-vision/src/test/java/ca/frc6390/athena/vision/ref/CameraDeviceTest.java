package ca.frc6390.athena.vision.ref;

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
class CameraDeviceTest {
    @Test
    void defaultsNameToKindKeyAndUsesIdentityMount() {
        CameraDevice camera = Cameras.camera(CameraKinds.SIM, " ");

        assertEquals(CameraKinds.SIM, camera.kind());
        assertEquals(CameraKinds.SIM.key(), camera.name());
        assertEquals(CameraMountPose.identity(), camera.mountPose());
    }

    @Test
    void mountAndBindingsReturnCopiesWithoutChangingOriginal() {
        CameraDevice original = Cameras.sim("front");
        CameraMountPose mount = new CameraMountPose(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
        Measurement pose = Measurements.pose(new PoseSnapshot(2.0, 3.0, 0.5));

        CameraDevice mounted = original.mount(mount);
        CameraDevice bound = mounted.bindPose(() -> List.of(pose));

        assertNotSame(original, mounted);
        assertNotSame(mounted, bound);
        assertEquals(CameraMountPose.identity(), original.mountPose());
        assertEquals(mount, mounted.mountPose());
        assertTrue(original.pose().measurements().isEmpty());
        assertEquals(List.of(pose), bound.pose().measurements());
    }

    @Test
    void nullSuppliersAndNullSupplierResultsReadAsEmpty() {
        CameraDevice nullSuppliers = Cameras.sim("sim").bindPose(null).bindTargets(null);
        CameraDevice nullResults = Cameras.sim("sim")
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
        CameraDevice camera = Cameras.sim("cache")
                .bindPose(() -> List.of(poseReads.incrementAndGet() == 1 ? firstPose : secondPose))
                .bindTargets(() -> List.of(targetReads.incrementAndGet() == 1 ? firstTarget : secondTarget));
        VisionGraph graph = VisionGraph.of(camera);
        VisionGraph.CameraRuntime runtime = graph.camera(camera);

        assertTrue(runtime.poseSignal().measurements().isEmpty());
        assertTrue(runtime.targetSignal().measurements().isEmpty());

        graph.refresh();
        assertEquals(List.of(firstPose), runtime.poseSignal().measurements());
        assertEquals(List.of(firstTarget), graph.targetMeasurements());
        assertEquals(1, poseReads.get());
        assertEquals(1, targetReads.get());

        assertEquals(List.of(firstPose), runtime.poseSignal().measurements());
        assertEquals(List.of(firstTarget), runtime.targetSignal().measurements());
        assertEquals(1, poseReads.get());
        assertEquals(1, targetReads.get());

        runtime.refresh();
        assertEquals(List.of(secondPose), graph.poseMeasurements());
        assertEquals(List.of(secondTarget), runtime.targetSignal().measurements());
        assertSame(camera, runtime.poseSignal().camera());
        assertSame(camera, runtime.targetSignal().camera());
    }
}
