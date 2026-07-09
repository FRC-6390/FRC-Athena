package ca.frc6390.athena.vision.ref;

import java.util.List;
import java.util.Objects;

import ca.frc6390.athena.runtime.measurement.Measurement;

/**
 * HeliOS pose stream.
 */
public final class HeliosPoseSignal implements PoseSignal {
    private final HeliosDevice camera;
    private final PoseSignal delegate;

    HeliosPoseSignal(HeliosDevice camera, PoseSignal delegate) {
        this.camera = Objects.requireNonNull(camera, "camera");
        this.delegate = Objects.requireNonNull(delegate, "delegate");
    }

    @Override
    public HeliosDevice camera() {
        return camera;
    }

    @Override
    public List<Measurement> measurements() {
        return delegate.measurements();
    }
}
