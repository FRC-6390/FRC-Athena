package ca.frc6390.athena.vision.ref;

import java.util.List;
import java.util.Objects;

import ca.frc6390.athena.runtime.measurement.Measurement;

/**
 * PhotonVision pose measurement stream.
 */
public final class PhotonPoseRef implements CameraPoseRef {
    private final PhotonRef camera;
    private final CameraPoseRef delegate;
    private final String strategy;

    PhotonPoseRef(PhotonRef camera, CameraPoseRef delegate) {
        this(camera, delegate, "");
    }

    private PhotonPoseRef(PhotonRef camera, CameraPoseRef delegate, String strategy) {
        this.camera = Objects.requireNonNull(camera, "camera");
        this.delegate = Objects.requireNonNull(delegate, "delegate");
        this.strategy = strategy == null ? "" : strategy;
    }

    /**
     * Uses a coprocessor multi-tag solve when available.
     *
     * @return updated ref
     */
    public PhotonPoseRef multiTagOnCoprocessor() {
        return new PhotonPoseRef(camera, delegate, "multi-tag-coprocessor");
    }

    /**
     * Uses the lowest ambiguity pose.
     *
     * @return updated ref
     */
    public PhotonPoseRef lowestAmbiguity() {
        return new PhotonPoseRef(camera, delegate, "lowest-ambiguity");
    }

    @Override
    public PhotonRef camera() {
        return camera;
    }

    @Override
    public List<Measurement> measurements() {
        return delegate.measurements();
    }

    /**
     * Returns the configured pose strategy.
     *
     * @return strategy
     */
    public String strategy() {
        return strategy;
    }
}
