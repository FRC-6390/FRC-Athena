package ca.frc6390.athena.vision.signal;

import java.util.List;
import java.util.Map;
import java.util.Objects;

import ca.frc6390.athena.runtime.measurement.Measurement;
import ca.frc6390.athena.vision.device.PhotonVisionDevice;

/**
 * PhotonVision pose stream with source-specific strategy metadata.
 */
public final class PhotonVisionPoseSignal implements PoseSignal {
    private final PhotonVisionDevice camera;
    private final PoseSignal delegate;
    private final String strategy;

    public PhotonVisionPoseSignal(PhotonVisionDevice camera, PoseSignal delegate) {
        this(camera, delegate, "");
    }

    private PhotonVisionPoseSignal(PhotonVisionDevice camera, PoseSignal delegate, String strategy) {
        this.camera = Objects.requireNonNull(camera, "camera");
        this.delegate = Objects.requireNonNull(delegate, "delegate");
        this.strategy = strategy == null ? "" : strategy;
    }

    /**
     * Uses a coprocessor multi-tag solve when available.
     *
     * @return updated signal
     */
    public PhotonVisionPoseSignal multiTagOnCoprocessor() {
        return new PhotonVisionPoseSignal(camera, delegate, "multi-tag-coprocessor");
    }

    /**
     * Uses the lowest ambiguity pose.
     *
     * @return updated signal
     */
    public PhotonVisionPoseSignal lowestAmbiguity() {
        return new PhotonVisionPoseSignal(camera, delegate, "lowest-ambiguity");
    }

    @Override
    public PhotonVisionDevice camera() {
        return camera;
    }

    @Override
    public List<Measurement> measurements() {
        return delegate.measurements();
    }

    @Override
    public Map<String, Object> metadata() {
        return strategy.isBlank() ? delegate.metadata() : Map.of("strategy", strategy);
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
