package ca.frc6390.athena.vision.spec;

/**
 * Single target observation from a camera frame.
 *
 * @param tagId AprilTag id, or {@code -1} when the target is not tag-backed
 * @param yawDegrees target yaw in degrees
 * @param pitchDegrees target pitch in degrees
 * @param distanceMeters estimated distance in meters
 * @param xMeters camera-relative target x translation
 * @param yMeters camera-relative target y translation
 * @param confidence confidence score where larger values are preferred
 */
public record VisionObservation(
        int tagId,
        double yawDegrees,
        double pitchDegrees,
        double distanceMeters,
        double xMeters,
        double yMeters,
        double confidence) {
    /**
     * Creates an AprilTag-backed observation.
     *
     * @param tagId AprilTag id
     * @param yawDegrees target yaw
     * @param pitchDegrees target pitch
     * @param distanceMeters distance
     * @param confidence confidence
     * @return observation
     */
    public static VisionObservation tag(
            int tagId,
            double yawDegrees,
            double pitchDegrees,
            double distanceMeters,
            double confidence) {
        return new VisionObservation(tagId, yawDegrees, pitchDegrees, distanceMeters, 0.0, 0.0, confidence);
    }

    /**
     * Returns true when targeting values are usable.
     *
     * @return true if valid
     */
    public boolean isValid() {
        return tagId >= -1
                && Double.isFinite(yawDegrees)
                && Double.isFinite(pitchDegrees)
                && Double.isFinite(distanceMeters)
                && distanceMeters >= 0.0
                && Double.isFinite(xMeters)
                && Double.isFinite(yMeters)
                && Double.isFinite(confidence);
    }

    /**
     * Returns camera-relative translation magnitude.
     *
     * @return translation magnitude
     */
    public double translationMagnitude() {
        return Math.hypot(xMeters, yMeters);
    }
}
