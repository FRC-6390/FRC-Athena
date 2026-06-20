package ca.frc6390.athena.vendor.photonvision;

import ca.frc6390.athena.vision.spec.VisionObservation;

/**
 * PhotonVision target values needed by Athena.
 *
 * @param fiducialId AprilTag id, or {@code -1} when unavailable
 * @param yawDegrees target yaw
 * @param pitchDegrees target pitch
 * @param distanceMeters estimated distance
 * @param poseAmbiguity PhotonVision pose ambiguity where lower is better
 */
public record PhotonVisionTarget(
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
     * Converts this target to Athena's generic observation model.
     *
     * @return vision observation
     */
    public VisionObservation toObservation() {
        double confidence = Double.isFinite(poseAmbiguity)
                ? Math.max(0.0, 1.0 - poseAmbiguity)
                : 0.0;
        return VisionObservation.tag(fiducialId, yawDegrees, pitchDegrees, distanceMeters, confidence);
    }
}
