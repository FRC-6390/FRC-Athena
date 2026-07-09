package ca.frc6390.athena.vision.runtime;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.runtime.measurement.Measurement;
import ca.frc6390.athena.runtime.measurement.Measurements;
import ca.frc6390.athena.vision.config.Cameras;
import ca.frc6390.athena.vision.device.CameraDevice;
import java.util.List;
import org.junit.jupiter.api.Test;

class VisionSimulationsTest {
    @Test
    void discoveryIsEmptyWhenNoVendorSimulationProviderIsInstalled() {
        assertTrue(VisionSimulations.discover().isEmpty());
        assertTrue(VisionSimulations.createDiscovered(List.of(Cameras.limelight("front"))).isEmpty());
    }

    @Test
    void createDiscoveredSkipsProvidersThatThrowDuringSupportChecks() {
        assertTrue(assertDoesNotThrow(() -> VisionSimulations.create(List.of(
                new ThrowingSupportProvider(),
                new EmptyProvider()), List.of(Cameras.photonVision("front")))).isEmpty());
    }

    @Test
    void createDiscoveredSkipsProvidersThatThrowDuringCreation() {
        assertTrue(assertDoesNotThrow(() -> VisionSimulations.create(List.of(
                new ThrowingCreateProvider(),
                new EmptyProvider()), List.of(Cameras.photonVision("front")))).isEmpty());
    }

    @Test
    void simulationCanBindUnboundCameraMeasurementsBeforeGraphSnapshotsSignals() {
        Measurement target = Measurements.custom("target", null);
        VisionSimulation simulation = new BindingSimulation(target);
        VisionGraph graph = VisionGraph.of(simulation.bind(Cameras.photonVision("front")));

        graph.refresh();

        assertTrue(graph.poseMeasurements().isEmpty());
        assertTrue(graph.targetMeasurements().contains(target));
    }

    @Test
    void visionGraphCanRebindUnboundCameraDeclarations() {
        Measurement target = Measurements.custom("target", null);
        VisionGraph graph = VisionGraph.of(Cameras.photonVision("front"));

        VisionGraph rebound = graph.bind(camera -> camera.bindTargets(() -> List.of(target)));
        rebound.refresh();

        assertTrue(graph.targetMeasurements().isEmpty());
        assertTrue(rebound.targetMeasurements().contains(target));
    }

    private static final class ThrowingSupportProvider implements VisionSimulationProvider {
        @Override
        public boolean supports(List<CameraDevice> cameras) {
            throw new IllegalStateException("support failed");
        }

        @Override
        public VisionSimulation create(List<CameraDevice> cameras) {
            throw new AssertionError("create should not be called");
        }
    }

    private static final class ThrowingCreateProvider implements VisionSimulationProvider {
        @Override
        public boolean supports(List<CameraDevice> cameras) {
            return true;
        }

        @Override
        public VisionSimulation create(List<CameraDevice> cameras) {
            throw new IllegalStateException("create failed");
        }
    }

    private static final class EmptyProvider implements VisionSimulationProvider {
        @Override
        public boolean supports(List<CameraDevice> cameras) {
            return false;
        }

        @Override
        public VisionSimulation create(List<CameraDevice> cameras) {
            return robotPose -> {
            };
        }
    }

    private record BindingSimulation(Measurement target) implements VisionSimulation {
        @Override
        public CameraDevice bind(CameraDevice camera) {
            return camera.bindTargets(() -> List.of(target));
        }

        @Override
        public void update(ca.frc6390.athena.runtime.filter.PoseSnapshot robotPose) {
        }
    }
}
