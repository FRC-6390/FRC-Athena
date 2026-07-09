package ca.frc6390.athena.vendor.photonvision;

/**
 * PhotonVision target values needed by Athena.
 *
 * @param fiducialId AprilTag id, or {@code -1} when unavailable
 * @param yawDegrees target yaw
 * @param pitchDegrees target pitch
 * @param distanceMeters estimated distance
 * @param poseAmbiguity PhotonVision pose ambiguity where lower is better
 */
record PhotonVisionTarget(
        int fiducialId,
        double yawDegrees,
        double pitchDegrees,
        double distanceMeters,
        double poseAmbiguity) {
    /**
     * Creates an AprilTag-backed target.
     *
     * @param fiducialId AprilTag id
     * @param yawDegrees target yaw
     * @param pitchDegrees target pitch
     * @param distanceMeters estimated distance
     * @param poseAmbiguity pose ambiguity
     * @return target
     */
    public static PhotonVisionTarget aprilTag(
            int fiducialId,
            double yawDegrees,
            double pitchDegrees,
            double distanceMeters,
            double poseAmbiguity) {
        return new PhotonVisionTarget(fiducialId, yawDegrees, pitchDegrees, distanceMeters, poseAmbiguity);
    }

    /**
     * Returns a confidence estimate where higher is better.
     *
     * @return confidence from 0 to 1
     */
    double confidence() {
        return Double.isFinite(poseAmbiguity)
                ? Math.max(0.0, 1.0 - poseAmbiguity)
                : 0.0;
    }
}
