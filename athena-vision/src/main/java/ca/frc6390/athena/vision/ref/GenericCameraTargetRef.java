package ca.frc6390.athena.vision.ref;

import java.util.List;
import java.util.Objects;

import ca.frc6390.athena.runtime.measurement.Measurement;
import ca.frc6390.athena.runtime.measurement.TargetMeasurement;
import ca.frc6390.athena.vision.spec.VisionObservation;

/**
 * Generic camera target measurement stream.
 */
public final class GenericCameraTargetRef implements CameraTargetRef {
    private final CameraRef camera;

    GenericCameraTargetRef(CameraRef camera) {
        this.camera = Objects.requireNonNull(camera, "camera");
    }

    @Override
    public CameraRef camera() {
        return camera;
    }

    @Override
    public List<Measurement> measurements() {
        return camera.frame().validObservations().stream()
                .map(this::toMeasurement)
                .map(Measurement.class::cast)
                .toList();
    }

    private TargetMeasurement toMeasurement(VisionObservation observation) {
        return new TargetMeasurement(
                observation.tagId(),
                observation.yawDegrees(),
                observation.pitchDegrees(),
                observation.distanceMeters(),
                observation.xMeters(),
                observation.yMeters(),
                observation.confidence(),
                0.0,
                0.0,
                camera);
    }
}
