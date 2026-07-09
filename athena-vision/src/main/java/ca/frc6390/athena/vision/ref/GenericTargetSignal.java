package ca.frc6390.athena.vision.ref;

import java.util.List;
import java.util.Objects;
import java.util.function.Supplier;

import ca.frc6390.athena.runtime.measurement.Measurement;

/**
 * Generic camera target stream.
 */
public final class GenericTargetSignal implements TargetSignal {
    private final CameraDevice camera;
    private final Supplier<? extends List<? extends Measurement>> targetMeasurements;

    GenericTargetSignal(
            CameraDevice camera,
            Supplier<? extends List<? extends Measurement>> targetMeasurements) {
        this.camera = Objects.requireNonNull(camera, "camera");
        this.targetMeasurements = targetMeasurements == null ? List::of : targetMeasurements;
    }

    @Override
    public CameraDevice camera() {
        return camera;
    }

    @Override
    public List<Measurement> measurements() {
        List<? extends Measurement> values = targetMeasurements.get();
        return values == null ? List.of() : List.copyOf(values);
    }
}
