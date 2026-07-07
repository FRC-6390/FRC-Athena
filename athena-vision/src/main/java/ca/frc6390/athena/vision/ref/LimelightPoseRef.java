package ca.frc6390.athena.vision.ref;

import java.util.Arrays;
import java.util.List;
import java.util.Objects;

import ca.frc6390.athena.runtime.measurement.Measurement;

/**
 * Limelight pose measurement stream.
 */
public final class LimelightPoseRef implements CameraPoseRef {
    private final LimelightRef camera;
    private final CameraPoseRef delegate;
    private final String poseMode;
    private final List<Integer> tags;
    private final double trustDistanceMeters;

    LimelightPoseRef(LimelightRef camera, CameraPoseRef delegate) {
        this(camera, delegate, "", List.of(), Double.POSITIVE_INFINITY);
    }

    private LimelightPoseRef(
            LimelightRef camera,
            CameraPoseRef delegate,
            String poseMode,
            List<Integer> tags,
            double trustDistanceMeters) {
        this.camera = Objects.requireNonNull(camera, "camera");
        this.delegate = Objects.requireNonNull(delegate, "delegate");
        this.poseMode = poseMode == null ? "" : poseMode;
        this.tags = tags == null ? List.of() : List.copyOf(tags);
        this.trustDistanceMeters = Double.isFinite(trustDistanceMeters)
                ? Math.max(0.0, trustDistanceMeters)
                : Double.POSITIVE_INFINITY;
    }

    /**
     * Uses MegaTag 1 blue-alliance pose output.
     *
     * @return updated ref
     */
    public LimelightPoseRef megatag1Blue() {
        return withPoseMode("megatag1-blue");
    }

    /**
     * Uses MegaTag 2 blue-alliance pose output.
     *
     * @return updated ref
     */
    public LimelightPoseRef megatag2Blue() {
        return withPoseMode("megatag2-blue");
    }

    /**
     * Uses botpose blue output.
     *
     * @return updated ref
     */
    public LimelightPoseRef botPoseBlue() {
        return withPoseMode("botpose-blue");
    }

    /**
     * Restricts accepted tag ids.
     *
     * @param tags tag ids
     * @return updated ref
     */
    public LimelightPoseRef tags(int... tags) {
        if (tags == null) {
            return new LimelightPoseRef(camera, delegate, poseMode, List.of(), trustDistanceMeters);
        }
        return new LimelightPoseRef(camera, delegate, poseMode, Arrays.stream(tags).boxed().toList(),
                trustDistanceMeters);
    }

    /**
     * Sets a distance hint for downstream trust policies.
     *
     * @param meters trust distance
     * @return updated ref
     */
    public LimelightPoseRef trustDistance(double meters) {
        return new LimelightPoseRef(camera, delegate, poseMode, tags, meters);
    }

    @Override
    public LimelightRef camera() {
        return camera;
    }

    @Override
    public List<Measurement> measurements() {
        return delegate.measurements();
    }

    /**
     * Returns the configured pose mode.
     *
     * @return pose mode
     */
    public String poseMode() {
        return poseMode;
    }

    /**
     * Returns restricted tag ids.
     *
     * @return tag ids
     */
    public List<Integer> tags() {
        return tags;
    }

    /**
     * Returns trust-distance metadata.
     *
     * @return trust distance
     */
    public double trustDistanceMeters() {
        return trustDistanceMeters;
    }

    private LimelightPoseRef withPoseMode(String mode) {
        return new LimelightPoseRef(camera, delegate, mode, tags, trustDistanceMeters);
    }
}
