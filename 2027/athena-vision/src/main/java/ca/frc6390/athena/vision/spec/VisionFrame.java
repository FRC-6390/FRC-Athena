package ca.frc6390.athena.vision.spec;

import java.util.Comparator;
import java.util.List;
import java.util.Optional;
import java.util.OptionalDouble;
import java.util.OptionalInt;

/**
 * Target observations captured at one camera update.
 *
 * @param observations raw observations
 */
public record VisionFrame(List<VisionObservation> observations) {
    private static final Comparator<VisionObservation> BEST_TARGET_ORDER = Comparator
            .comparingDouble(VisionObservation::confidence)
            .reversed()
            .thenComparingDouble(VisionObservation::distanceMeters)
            .thenComparingDouble(VisionObservation::translationMagnitude);

    public VisionFrame {
        observations = observations == null ? List.of() : List.copyOf(observations);
    }

    /**
     * Creates an empty frame.
     *
     * @return empty frame
     */
    public static VisionFrame noTarget() {
        return new VisionFrame(List.of());
    }

    /**
     * Creates a frame from observations.
     *
     * @param observations observations
     * @return frame
     */
    public static VisionFrame of(VisionObservation... observations) {
        if (observations == null) {
            return noTarget();
        }
        return new VisionFrame(List.of(observations));
    }

    /**
     * Returns valid observations only.
     *
     * @return valid observations
     */
    public List<VisionObservation> validObservations() {
        return observations.stream()
                .filter(VisionObservation::isValid)
                .toList();
    }

    /**
     * Returns whether a valid target is available.
     *
     * @return true if a valid target exists
     */
    public boolean hasValidTarget() {
        return bestTarget().isPresent();
    }

    /**
     * Returns the best target using confidence, distance, then translation.
     *
     * @return best target
     */
    public Optional<VisionObservation> bestTarget() {
        return validObservations().stream().min(BEST_TARGET_ORDER);
    }

    /**
     * Returns yaw for the best target.
     *
     * @return yaw when a valid target exists
     */
    public OptionalDouble yawDegrees() {
        Optional<VisionObservation> target = bestTarget();
        if (target.isEmpty()) {
            return OptionalDouble.empty();
        }
        return OptionalDouble.of(target.get().yawDegrees());
    }

    /**
     * Returns pitch for the best target.
     *
     * @return pitch when a valid target exists
     */
    public OptionalDouble pitchDegrees() {
        Optional<VisionObservation> target = bestTarget();
        if (target.isEmpty()) {
            return OptionalDouble.empty();
        }
        return OptionalDouble.of(target.get().pitchDegrees());
    }

    /**
     * Returns distance for the best target.
     *
     * @return distance when a valid target exists
     */
    public OptionalDouble distanceMeters() {
        Optional<VisionObservation> target = bestTarget();
        if (target.isEmpty()) {
            return OptionalDouble.empty();
        }
        return OptionalDouble.of(target.get().distanceMeters());
    }

    /**
     * Returns tag id for the best target.
     *
     * @return tag id when a valid target exists
     */
    public OptionalInt tagId() {
        Optional<VisionObservation> target = bestTarget();
        if (target.isEmpty()) {
            return OptionalInt.empty();
        }
        return OptionalInt.of(target.get().tagId());
    }
}
