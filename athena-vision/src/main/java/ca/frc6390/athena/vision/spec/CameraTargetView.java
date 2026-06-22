package ca.frc6390.athena.vision.spec;

import java.util.List;
import java.util.OptionalDouble;
import java.util.OptionalInt;

/**
 * Convenience view over a camera frame for target-style sensor access.
 *
 * @param frame source frame
 */
public record CameraTargetView(VisionFrame frame) {
    public CameraTargetView {
        frame = frame == null ? VisionFrame.noTarget() : frame;
    }

    /**
     * Returns whether the frame has at least one valid target.
     *
     * @return true when a valid target exists
     */
    public boolean hasValidTarget() {
        return frame.bestTarget().isPresent();
    }

    /**
     * Returns valid observations.
     *
     * @return valid observations
     */
    public List<VisionObservation> observations() {
        return frame.validObservations();
    }

    /**
     * Returns yaw for the best target.
     *
     * @return yaw when present
     */
    public OptionalDouble yawDegrees() {
        return frame.yawDegrees();
    }

    /**
     * Returns pitch for the best target.
     *
     * @return pitch when present
     */
    public OptionalDouble pitchDegrees() {
        return frame.pitchDegrees();
    }

    /**
     * Returns distance for the best target.
     *
     * @return distance when present
     */
    public OptionalDouble distanceMeters() {
        return frame.distanceMeters();
    }

    /**
     * Returns tag id for the best target.
     *
     * @return tag id when present
     */
    public OptionalInt tagId() {
        return frame.tagId();
    }
}
