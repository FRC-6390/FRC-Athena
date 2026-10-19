package ca.frc6390.athena.vision.signal;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Distance;
import java.util.Arrays;
import java.util.List;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Objects;

import ca.frc6390.athena.runtime.measurement.Measurement;
import ca.frc6390.athena.vision.device.LimelightDevice;

/**
 * Limelight pose stream with source-specific selection metadata.
 */
public final class LimelightPoseSignal implements PoseSignal {
    private final LimelightDevice camera;
    private final PoseSignal delegate;
    private final String poseMode;
    private final List<Integer> tags;
    private final Distance trustDistance;

    public LimelightPoseSignal(LimelightDevice camera, PoseSignal delegate) {
        this(camera, delegate, "", List.of(), Meters.of(Double.POSITIVE_INFINITY));
    }

    private LimelightPoseSignal(
            LimelightDevice camera,
            PoseSignal delegate,
            String poseMode,
            List<Integer> tags,
            Distance trustDistance) {
        this.camera = Objects.requireNonNull(camera, "camera");
        this.delegate = Objects.requireNonNull(delegate, "delegate");
        this.poseMode = poseMode == null ? "" : poseMode;
        this.tags = tags == null ? List.of() : List.copyOf(tags);
        double meters = trustDistance == null ? Double.POSITIVE_INFINITY : trustDistance.in(Meters);
        this.trustDistance = Double.isFinite(meters)
                ? Meters.of(Math.max(0.0, meters))
                : Meters.of(Double.POSITIVE_INFINITY);
    }

    /**
     * Uses MegaTag 1 blue-alliance pose output.
     *
     * @return updated signal
     */
    public LimelightPoseSignal megatag1Blue() {
        return withPoseMode("megatag1-blue");
    }

    /**
     * Uses MegaTag 2 blue-alliance pose output.
     *
     * @return updated signal
     */
    public LimelightPoseSignal megatag2Blue() {
        return withPoseMode("megatag2-blue");
    }

    /**
     * Uses botpose blue output.
     *
     * @return updated signal
     */
    public LimelightPoseSignal botPoseBlue() {
        return withPoseMode("botpose-blue");
    }

    /**
     * Restricts accepted tag ids.
     *
     * @param tags tag ids
     * @return updated signal
     */
    public LimelightPoseSignal tags(int... tags) {
        if (tags == null) {
            return new LimelightPoseSignal(camera, delegate, poseMode, List.of(), trustDistance);
        }
        return new LimelightPoseSignal(camera, delegate, poseMode, Arrays.stream(tags).boxed().toList(),
                trustDistance);
    }

    /**
     * Sets a distance hint for downstream trust policies.
     *
     * @param distance trust distance
     * @return updated signal
     */
    public LimelightPoseSignal trustDistance(Distance distance) {
        return new LimelightPoseSignal(camera, delegate, poseMode, tags, distance);
    }

    @Override
    public LimelightDevice camera() {
        return camera;
    }

    @Override
    public List<Measurement> measurements() {
        return delegate.measurements();
    }

    @Override
    public Map<String, Object> metadata() {
        Map<String, Object> metadata = new LinkedHashMap<>(delegate.metadata());
        if (!poseMode.isBlank()) {
            metadata.put("poseMode", poseMode);
        }
        if (!tags.isEmpty()) {
            metadata.put("tags", tags);
        }
        if (Double.isFinite(trustDistance.in(Meters))) {
            metadata.put("trustDistanceMeters", trustDistance.in(Meters));
        }
        return Map.copyOf(metadata);
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
    public Distance trustDistance() {
        return trustDistance;
    }

    private LimelightPoseSignal withPoseMode(String mode) {
        return new LimelightPoseSignal(camera, delegate, mode, tags, trustDistance);
    }
}
