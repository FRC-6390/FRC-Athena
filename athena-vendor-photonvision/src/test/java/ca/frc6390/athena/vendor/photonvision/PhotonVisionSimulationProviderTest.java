package ca.frc6390.athena.vendor.photonvision;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.runtime.filter.PoseSnapshot;
import ca.frc6390.athena.hardware.runtime.ActionContext;
import ca.frc6390.athena.localization.pipeline.Localization;
import ca.frc6390.athena.localization.pipeline.Localizations;
import ca.frc6390.athena.runtime.measurement.Measurement;
import ca.frc6390.athena.runtime.measurement.PoseMeasurementSample;
import ca.frc6390.athena.runtime.measurement.TargetMeasurementSample;
import ca.frc6390.athena.vision.config.Cameras;
import ca.frc6390.athena.vision.device.CameraDevice;
import ca.frc6390.athena.vision.runtime.VisionSimNoise;
import ca.frc6390.athena.vision.runtime.VisionSimulation;
import ca.frc6390.athena.vision.runtime.VisionSimulationField;
import ca.frc6390.athena.vision.runtime.VisionSimulationProvider;
import ca.frc6390.athena.vision.runtime.VisionSimulationTarget;
import ca.frc6390.athena.vision.runtime.VisionGraph;
import ca.frc6390.athena.vision.runtime.VisionSimulations;
import java.util.List;
import org.junit.jupiter.api.Test;

class PhotonVisionSimulationProviderTest {
    @Test
    void photonProviderIsDiscoverableAndSupportsMixedAthenaCameras() {
        List<VisionSimulationProvider> providers = VisionSimulations.discover();

        assertTrue(providers.stream().anyMatch(PhotonVisionSimulationProvider.class::isInstance));

        VisionSimulationProvider provider = providers.stream()
                .filter(PhotonVisionSimulationProvider.class::isInstance)
                .findFirst()
                .orElseThrow();
        List<CameraDevice> cameras = List.of(
                Cameras.photonVision("photon-front").mount(Meters.of(0.25), Meters.of(0.0), Meters.of(0.5), Degrees.of(0.0), Degrees.of(-10.0), Degrees.of(0.0)),
                Cameras.limelight("limelight-front").mount(Meters.of(0.2), Meters.of(0.1), Meters.of(0.45), Degrees.of(5.0), Degrees.of(-8.0), Degrees.of(0.0)),
                Cameras.helios("helios-front"));

        assertTrue(provider.supports(cameras));

        assertEquals(1, providers.stream().filter(PhotonVisionSimulationProvider.class::isInstance).count());
    }

    @Test
    void photonProviderSupportsAllAthenaCameraKindsAsCustomPhotonSims() {
        PhotonVisionSimulationProvider provider = new PhotonVisionSimulationProvider();

        assertFalse(provider.supports(null));
        assertFalse(provider.supports(List.of()));
        assertTrue(provider.supports(List.of(Cameras.photonVision("photon-front"))));
        assertTrue(provider.supports(List.of(Cameras.limelight("limelight-front"))));
        assertTrue(provider.supports(List.of(Cameras.helios("helios-front"))));
    }

    @Test
    void simulatedTargetConversionProducesAthenaTargetMeasurementsWhenTargetIsInView() {
        CameraDevice camera = Cameras.photonVision("front").mount(Meters.of(0.0), Meters.of(0.0), Meters.of(0.5), Degrees.of(0.0), Degrees.of(0.0), Degrees.of(0.0));

        List<Measurement> measurements = PhotonVisionSimulationProvider.PhotonVisionSimulation.simulatedTargets(
                new PoseSnapshot(0.0, 0.0, 0.0),
                camera);

        assertEquals(1, measurements.size());
        TargetMeasurementSample target = (TargetMeasurementSample) measurements.get(0);
        assertEquals(1, target.targetId());
        assertEquals(0.0, target.yawDegrees(), 1.0e-9);
        assertTrue(target.distanceMeters() > 0.0);
    }

    @Test
    void simulatedTargetConversionUsesConfiguredFieldTargets() {
        CameraDevice camera = Cameras.photonVision("front").mount(Meters.of(0.0), Meters.of(0.0), Meters.of(0.5), Degrees.of(0.0), Degrees.of(0.0), Degrees.of(0.0));
        VisionSimulationField field = VisionSimulationField.of(
                VisionSimulationTarget.aprilTag(4, Meters.of(2.0), Meters.of(0.0), Meters.of(1.0), Radians.of(0.0)),
                VisionSimulationTarget.aprilTag(7, Meters.of(-2.0), Meters.of(0.0), Meters.of(1.0), Radians.of(0.0)));

        List<Measurement> measurements = PhotonVisionSimulationProvider.PhotonVisionSimulation.simulatedTargets(
                new PoseSnapshot(0.0, 0.0, 0.0),
                camera,
                field);

        assertEquals(1, measurements.size());
        TargetMeasurementSample target = (TargetMeasurementSample) measurements.get(0);
        assertEquals(4, target.targetId());
    }

    @Test
    void simulationBindsTargetAndPoseMeasurementsFromConfiguredField() {
        CameraDevice camera = Cameras.photonVision("front").mount(Meters.of(0.0), Meters.of(0.0), Meters.of(0.5), Degrees.of(0.0), Degrees.of(0.0), Degrees.of(0.0));
        VisionSimulation simulation = new PhotonVisionSimulationProvider().create(
                List.of(camera),
                VisionSimulationField.of(VisionSimulationTarget.aprilTag(11, Meters.of(3.0), Meters.of(0.0), Meters.of(1.0), Radians.of(0.0)))
                        .noise(VisionSimNoise.perfect()));
        CameraDevice bound = simulation.bind(camera);

        simulation.update(new PoseSnapshot(0.0, 0.0, 0.0), 1.25);

        assertEquals(1, bound.targets().measurements().size());
        assertEquals(11, ((TargetMeasurementSample) bound.targets().measurements().get(0)).targetId());
        assertEquals(1, bound.pose().measurements().size());
        PoseMeasurementSample pose = (PoseMeasurementSample) bound.pose().measurements().get(0);
        assertEquals(1, pose.targetCount());
        assertEquals(0.0, pose.pose().xMeters(), 1.0e-9);
        assertEquals(1.25, pose.timestampSeconds(), 1.0e-9);
    }

    @Test
    void simulatedCameraCapturedBeforeBindingFeedsLocalizationThroughRuntimeCache() {
        CameraDevice camera = Cameras.photonVision("localization-front")
                .mount(Meters.of(0.0), Meters.of(0.0), Meters.of(0.5), Degrees.of(0.0), Degrees.of(0.0), Degrees.of(0.0));
        var declaredPose = camera.pose().singleTagStdDevs(Meters.of(0.2), Meters.of(0.2), Radians.of(0.1));
        Localization localization = Localizations.latestValid().input(declaredPose);
        VisionSimulation simulation = new PhotonVisionSimulationProvider().create(
                List.of(camera),
                VisionSimulationField.of(VisionSimulationTarget.aprilTag(11, Meters.of(5.0), Meters.of(2.0), Meters.of(1.0), Radians.of(0.0)))
                        .noise(VisionSimNoise.perfect()));
        CameraDevice bound = simulation.bind(camera);
        VisionGraph graph = VisionGraph.of(bound);

        simulation.update(new PoseSnapshot(2.0, 2.0, 0.0));
        graph.refresh();
        localization.refresh(ActionContext.empty(), 1.0, 0.02);

        assertEquals(2.0, localization.pose().getX(), 1.0e-9);
        assertEquals(2.0, localization.pose().getY(), 1.0e-9);
        assertEquals(1, localization.acceptedMeasurements().size());
    }
}
