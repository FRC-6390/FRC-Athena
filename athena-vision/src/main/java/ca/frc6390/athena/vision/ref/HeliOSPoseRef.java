package ca.frc6390.athena.vision.ref;

import java.util.List;
import java.util.Objects;

import ca.frc6390.athena.runtime.measurement.Measurement;

/**
 * HeliOS pose measurement stream.
 */
public final class HeliOSPoseRef implements CameraPoseRef {
    private final HeliOSRef camera;
    private final CameraPoseRef delegate;

    HeliOSPoseRef(HeliOSRef camera, CameraPoseRef delegate) {
        this.camera = Objects.requireNonNull(camera, "camera");
        this.delegate = Objects.requireNonNull(delegate, "delegate");
    }

    @Override
    public HeliOSRef camera() {
        return camera;
    }

    @Override
    public List<Measurement> measurements() {
        return delegate.measurements();
    }
}
