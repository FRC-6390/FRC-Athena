package ca.frc6390.athena.vision.ref;

import java.util.List;
import java.util.Objects;
import java.util.function.Supplier;

import ca.frc6390.athena.runtime.measurement.Measurement;

/**
 * Generic camera pose stream.
 */
public final class GenericPoseSignal implements PoseSignal {
    private final CameraDevice camera;
    private final Supplier<? extends List<? extends Measurement>> poseMeasurements;

    GenericPoseSignal(
            CameraDevice camera,
            Supplier<? extends List<? extends Measurement>> poseMeasurements) {
        this.camera = Objects.requireNonNull(camera, "camera");
        this.poseMeasurements = poseMeasurements == null ? List::of : poseMeasurements;
    }

    @Override
    public CameraDevice camera() {
        return camera;
    }

    @Override
    public List<Measurement> measurements() {
        List<? extends Measurement> values = poseMeasurements.get();
        return values == null ? List.of() : List.copyOf(values);
    }
}
