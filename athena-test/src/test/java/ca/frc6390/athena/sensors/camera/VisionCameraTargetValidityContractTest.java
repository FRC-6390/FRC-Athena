package ca.frc6390.athena.sensors.camera;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import java.util.Optional;
import java.util.OptionalDouble;
import java.util.OptionalInt;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;

import ca.frc6390.athena.sensors.camera.ConfigurableCamera.CameraSoftware;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.junit.jupiter.api.Test;

final class VisionCameraTargetValidityContractTest {

    @Test
    void targetAccessorsReturnEmptyWhenCameraReportsNoTarget() {
        AtomicBoolean connected = new AtomicBoolean(true);
        AtomicBoolean hasTargets = new AtomicBoolean(false);
        AtomicReference<Double> yaw = new AtomicReference<>(12.5);
        AtomicReference<Double> pitch = new AtomicReference<>(-3.0);
        AtomicReference<Double> distance = new AtomicReference<>(2.1);
        AtomicInteger tagId = new AtomicInteger(7);

        VisionCamera camera = createCamera(connected, hasTargets, yaw, pitch, distance, tagId);

        assertFalse(camera.hasValidTarget());
        assertEquals(OptionalDouble.empty(), camera.getTargetYawDegrees());
        assertEquals(OptionalDouble.empty(), camera.getTargetPitchDegrees());
        assertEquals(OptionalDouble.empty(), camera.getTargetDistanceMeters());
        assertEquals(OptionalInt.empty(), camera.getLatestTagId());
        assertEquals(List.of(), camera.getTargetMeasurements());
        assertEquals(Optional.empty(), camera.getCameraRelativeTranslation());
        assertEquals(Optional.empty(), camera.getLatestObservation());
        assertEquals(Optional.empty(), camera.getTargetTranslation(VisionCamera.CoordinateSpace.CAMERA));
        assertEquals(List.of(), camera.getObservations(VisionCamera.CoordinateSpace.CAMERA));
        assertEquals(
                Optional.empty(),
                camera.projectToFieldTranslation(new Pose2d(1.0, 2.0, Rotation2d.fromDegrees(20.0)), null));
        assertEquals(
                Optional.empty(),
                camera.estimateFieldPoseFromTag(new Pose2d(3.0, 1.0, Rotation2d.fromDegrees(180.0)), null));
    }

    @Test
    void targetAccessorsExposeValuesWhenCameraReportsTarget() {
        AtomicBoolean connected = new AtomicBoolean(true);
        AtomicBoolean hasTargets = new AtomicBoolean(true);
        AtomicReference<Double> yaw = new AtomicReference<>(12.5);
        AtomicReference<Double> pitch = new AtomicReference<>(-3.0);
        AtomicReference<Double> distance = new AtomicReference<>(2.1);
        AtomicInteger tagId = new AtomicInteger(7);

        VisionCamera camera = createCamera(connected, hasTargets, yaw, pitch, distance, tagId);

        assertTrue(camera.hasValidTarget());
        assertEquals(12.5, camera.getTargetYawDegrees().orElseThrow(), 1e-9);
        assertEquals(-3.0, camera.getTargetPitchDegrees().orElseThrow(), 1e-9);
        assertEquals(2.1, camera.getTargetDistanceMeters().orElseThrow(), 1e-9);
        assertEquals(7, camera.getLatestTagId().orElseThrow());
        assertEquals(1, camera.getTargetMeasurements().size());
        assertEquals(2.0, camera.getCameraRelativeTranslation().orElseThrow().getX(), 1e-9);
        assertEquals(-0.5, camera.getCameraRelativeTranslation().orElseThrow().getY(), 1e-9);
        assertTrue(camera.getLatestObservation().isPresent());
        assertTrue(camera.getTargetTranslation(VisionCamera.CoordinateSpace.CAMERA).isPresent());
        assertEquals(1, camera.getObservations(VisionCamera.CoordinateSpace.CAMERA).size());
    }

    private static VisionCamera createCamera(
            AtomicBoolean connected,
            AtomicBoolean hasTargets,
            AtomicReference<Double> yaw,
            AtomicReference<Double> pitch,
            AtomicReference<Double> distance,
            AtomicInteger tagId) {
        VisionCameraConfig config = VisionCameraConfig.create("test-camera", CameraSoftware.PhotonVision);
        config.config(section -> section
                .connectedSupplier(connected::get)
                .hasTargetsSupplier(hasTargets::get)
                .targetYawSupplier(yaw::get)
                .targetPitchSupplier(pitch::get)
                .tagDistanceSupplier(distance::get)
                .tagIdSupplier(tagId::get)
                .targetMeasurementsSupplier(() -> List.of(new VisionCamera.TargetMeasurement(
                        new Translation2d(2.0, -0.5),
                        yaw.get(),
                        distance.get(),
                        tagId.get(),
                        0.9))));
        return new VisionCamera(config);
    }
}
