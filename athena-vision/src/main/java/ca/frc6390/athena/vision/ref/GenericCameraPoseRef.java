package ca.frc6390.athena.vision.ref;

import java.util.List;
import java.util.Objects;
import java.util.function.Supplier;

import ca.frc6390.athena.runtime.measurement.Measurement;
import ca.frc6390.athena.runtime.measurement.PoseMeasurement;

/**
 * Generic camera pose measurement stream.
 */
public final class GenericCameraPoseRef implements CameraPoseRef {
    private final CameraRef camera;
    private final Supplier<? extends List<? extends PoseMeasurement>> poseMeasurements;

    GenericCameraPoseRef(
            CameraRef camera,
            Supplier<? extends List<? extends PoseMeasurement>> poseMeasurements) {
        this.camera = Objects.requireNonNull(camera, "camera");
        this.poseMeasurements = poseMeasurements == null ? List::of : poseMeasurements;
    }

    @Override
    public CameraRef camera() {
        return camera;
    }

    @Override
    public List<Measurement> measurements() {
        List<? extends PoseMeasurement> values = poseMeasurements.get();
        return values == null ? List.of() : values.stream().map(Measurement.class::cast).toList();
    }
}
