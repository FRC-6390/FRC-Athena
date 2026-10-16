package ca.frc6390.athena.vision.signal;

import java.util.LinkedHashMap;
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
    private final boolean singleTagUsesHeading;

    public PhotonVisionPoseSignal(PhotonVisionDevice camera, PoseSignal delegate) {
        this(camera, delegate, "", false);
    }

    private PhotonVisionPoseSignal(
            PhotonVisionDevice camera, PoseSignal delegate, String strategy, boolean singleTagUsesHeading) {
        this.camera = Objects.requireNonNull(camera, "camera");
        this.delegate = Objects.requireNonNull(delegate, "delegate");
        this.strategy = strategy == null ? "" : strategy;
        this.singleTagUsesHeading = singleTagUsesHeading;
    }

    /**
     * Uses a coprocessor multi-tag solve when available.
     *
     * @return updated signal
     */
    public PhotonVisionPoseSignal multiTagOnCoprocessor() {
        return new PhotonVisionPoseSignal(camera, delegate, "multi-tag-coprocessor", singleTagUsesHeading);
    }

    /**
     * Uses the lowest ambiguity pose.
     *
     * @return updated signal
     */
    public PhotonVisionPoseSignal lowestAmbiguity() {
        return new PhotonVisionPoseSignal(camera, delegate, "lowest-ambiguity", singleTagUsesHeading);
    }

    /**
     * Solves single-tag frames with the robot heading instead of an ambiguous single-tag PnP.
     *
     * <p>Multi-tag frames keep using the coprocessor multi-tag result. Frames with one tag use
     * PhotonLib's {@code PNP_DISTANCE_TRIG_SOLVE} with the robot heading, and fall back to the
     * lowest-ambiguity solve while no heading is available. The option is stored on the shared
     * camera declaration, so it applies to the adapter bound from that declaration.</p>
     *
     * @return updated signal
     */
    public PhotonVisionPoseSignal singleTagUsesHeading() {
        camera.configureSingleTagHeading(true);
        return new PhotonVisionPoseSignal(camera, delegate, strategy, true);
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
        if (strategy.isBlank() && !singleTagUsesHeading) {
            return delegate.metadata();
        }
        Map<String, Object> metadata = new LinkedHashMap<>();
        if (!strategy.isBlank()) {
            metadata.put("strategy", strategy);
        }
        if (singleTagUsesHeading) {
            metadata.put("singleTagUsesHeading", true);
        }
        return Map.copyOf(metadata);
    }

    /**
     * Returns the configured pose strategy.
     *
     * @return strategy
     */
    public String strategy() {
        return strategy;
    }

    /**
     * Returns whether single-tag frames use the heading-constrained solve.
     *
     * @return true when enabled
     */
    public boolean headingSolveEnabled() {
        return singleTagUsesHeading;
    }
}
