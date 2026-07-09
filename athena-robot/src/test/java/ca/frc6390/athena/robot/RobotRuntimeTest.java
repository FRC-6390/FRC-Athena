package ca.frc6390.athena.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.auto.AutoRuntime;
import ca.frc6390.athena.auto.Autos;
import ca.frc6390.athena.commands.CommandState;
import ca.frc6390.athena.api.hardware.EncoderKinds;
import ca.frc6390.athena.api.hardware.ImuKinds;
import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.backend.BackendRegistry;
import ca.frc6390.athena.localization.ref.LocalizationPipeline;
import ca.frc6390.athena.localization.ref.Localizations;
import ca.frc6390.athena.runtime.control.RobotVelocity;
import ca.frc6390.athena.runtime.filter.PoseSnapshot;
import ca.frc6390.athena.runtime.measurement.Measurement;
import ca.frc6390.athena.runtime.measurement.MeasurementStdDevs;
import ca.frc6390.athena.runtime.measurement.Measurements;
import ca.frc6390.athena.runtime.measurement.PoseMeasurementSample;
import ca.frc6390.athena.sim.runtime.SimRuntime;
import ca.frc6390.athena.vision.config.Cameras;
import ca.frc6390.athena.vision.ref.CameraDevice;
import ca.frc6390.athena.vision.runtime.CameraAdapters;
import ca.frc6390.athena.vision.runtime.VisionGraph;
import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;
import org.junit.jupiter.api.Test;

class RobotRuntimeTest {
    @Test
    void discoversHardwareVendorBackendsFromServiceDescriptorsOnRootClasspath() {
        BackendRegistry.setGlobal(null);
        BackendRegistry registry = BackendRegistry.discover();

        assertTrue(registry.motorBackendFor(MotorKinds.TALON_FX).isPresent());
        assertTrue(registry.motorBackendFor(MotorKinds.SPARK_MAX_BRUSHLESS).isPresent());
        assertTrue(registry.encoderBackendFor(EncoderKinds.CANCODER).isPresent());
        assertTrue(registry.encoderBackendFor(EncoderKinds.REV_THROUGH_BORE).isPresent());
        assertTrue(registry.imuBackendFor(ImuKinds.PIGEON_2).isPresent());
        assertTrue(registry.imuBackendFor(ImuKinds.NAVX).isPresent());
    }

    @Test
    void discoversVisionVendorAdaptersFromServiceDescriptorsOnRootClasspath() {
        List<String> adapterNames = CameraAdapters.discover().stream()
                .map(adapter -> adapter.getClass().getName())
                .toList();

        assertTrue(adapterNames.contains("ca.frc6390.athena.vendor.limelight.LimelightCameraAdapter"));
        assertTrue(adapterNames.contains("ca.frc6390.athena.vendor.photonvision.PhotonVisionCameraAdapter"));
    }

    @Test
    void ownsCommandAndAutoLifecycleFromOneRoot() {
        AtomicInteger commandExecute = new AtomicInteger();
        AtomicInteger commandEnd = new AtomicInteger();
        AtomicInteger autoExecute = new AtomicInteger();
        AtomicInteger autoEnd = new AtomicInteger();
        CommandState command = CommandState.create("drive")
                .onExecute(commandExecute::incrementAndGet)
                .onEnd(commandEnd::incrementAndGet)
                .build();
        CommandState auto = CommandState.create("auto")
                .onExecute(autoExecute::incrementAndGet)
                .onEnd(autoEnd::incrementAndGet)
                .build();
        AutoRuntime autoRuntime = Autos.runtime(Autos.routine("auto", auto));
        RobotRuntime runtime = RobotRuntime.simulated(new SimRuntime())
                .schedule(command)
                .auto(autoRuntime, null);

        runtime.robotPeriodic(0.0, 0.02);
        runtime.autoPeriodic(0.02, 0.02);
        runtime.disabledPeriodic(0.04, 0.02);

        assertEquals(2, commandExecute.get());
        assertEquals(1, commandEnd.get());
        assertEquals(1, autoExecute.get());
        assertEquals(1, autoEnd.get());
    }

    @Test
    void exposesSimulationBackedHardwareGraph() {
        RobotRuntime runtime = RobotRuntime.simulated(new SimRuntime());

        assertNotNull(runtime.hardwareGraph());
        assertNotNull(runtime.simRuntime());
    }

    @Test
    void refreshesVisionGraphsDuringRobotPeriodic() {
        AtomicInteger reads = new AtomicInteger();
        Measurement measurement = Measurements.custom("target", null);
        CameraDevice camera = Cameras.sim("front").bindTargets(() -> {
            reads.incrementAndGet();
            return List.of(measurement);
        });
        VisionGraph vision = VisionGraph.of(camera);
        RobotRuntime runtime = RobotRuntime.simulated(new SimRuntime()).vision(vision);

        runtime.robotPeriodic(0.0, 0.02);

        assertEquals(1, reads.get());
        assertEquals(List.of(measurement), vision.targetMeasurements());
    }

    @Test
    void mountsVendorKindCameraDeclarationsThroughRootRuntime() {
        AtomicInteger limelightReads = new AtomicInteger();
        AtomicInteger photonReads = new AtomicInteger();
        Measurement limelightTarget = Measurements.custom("limelight-target", null);
        Measurement photonTarget = Measurements.custom("photon-target", null);
        CameraDevice limelight = Cameras.limelight("limelight-front").bindTargets(() -> {
            limelightReads.incrementAndGet();
            return List.of(limelightTarget);
        });
        CameraDevice photon = Cameras.photonVision("photon-front").bindTargets(() -> {
            photonReads.incrementAndGet();
            return List.of(photonTarget);
        });
        RobotRuntime runtime = RobotRuntime.simulated(new SimRuntime()).cameras(limelight, photon);

        runtime.robotPeriodic(0.0, 0.02);

        assertEquals(1, limelightReads.get());
        assertEquals(1, photonReads.get());
    }

    @Test
    void refreshesLocalizationSnapshotsDuringRobotPeriodic() {
        AtomicInteger reads = new AtomicInteger();
        LocalizationPipeline localization = Localizations.latestValid()
                .input(() -> {
                    reads.incrementAndGet();
                    return List.of(Measurements.pose(new PoseSnapshot(2.0, 3.0, 0.5)));
                });
        RobotRuntime runtime = RobotRuntime.simulated(new SimRuntime()).localization(localization);

        runtime.robotPeriodic(0.0, 0.02);

        assertEquals(1, reads.get());
        assertEquals(2.0, localization.pose().xMeters(), 1.0e-9);
        assertEquals(2, reads.get());
    }

    @Test
    void localizationRefreshPolicyAppliesRootAgeAndDisabledRules() {
        AtomicInteger reads = new AtomicInteger();
        LocalizationPipeline localization = Localizations.latestValid()
                .input(() -> {
                    reads.incrementAndGet();
                    return List.of(new TestPoseMeasurement(new PoseSnapshot(4.0, 5.0, 0.25), 1.0));
                });
        RobotRuntime runtime = RobotRuntime.simulated(new SimRuntime())
                .localization(localization)
                .localizationMaxAge(1.0);

        runtime.robotPeriodic(5.0, 0.02);
        runtime.disabledPeriodic(5.02, 0.02);
        runtime.localizationRefreshWhileDisabled(true).disabledPeriodic(5.04, 0.02);

        assertTrue(runtime.localizationMeasurements().isEmpty());
        assertEquals(2, reads.get());
    }

    private record TestPoseMeasurement(PoseSnapshot pose, double timestampSeconds) implements PoseMeasurementSample {
        @Override
        public RobotVelocity speeds() {
            return RobotVelocity.zero();
        }

        @Override
        public double latencySeconds() {
            return 0.0;
        }

        @Override
        public double ambiguity() {
            return 0.0;
        }

        @Override
        public int targetCount() {
            return 1;
        }

        @Override
        public MeasurementStdDevs stdDevs() {
            return null;
        }

        @Override
        public Object source() {
            return null;
        }
    }
}
