package ca.frc6390.athena.vision.device;

/**
 * Robot-relative camera mount pose.
 *
 * @param xMeters forward-positive position
 * @param yMeters left-positive position
 * @param zMeters up-positive position
 * @param yawDegrees yaw offset
 * @param pitchDegrees pitch offset
 * @param rollDegrees roll offset
 */
public record CameraMountPose(
        double xMeters,
        double yMeters,
        double zMeters,
        double yawDegrees,
        double pitchDegrees,
        double rollDegrees) {
    /**
     * Returns a zero-offset camera pose.
     *
     * @return identity mount pose
     */
    public static CameraMountPose identity() {
        return new CameraMountPose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    }

    /**
     * Returns whether every pose value is finite.
     *
     * @return true if finite
     */
    public boolean isFinite() {
        return Double.isFinite(xMeters)
                && Double.isFinite(yMeters)
                && Double.isFinite(zMeters)
                && Double.isFinite(yawDegrees)
                && Double.isFinite(pitchDegrees)
                && Double.isFinite(rollDegrees);
    }
}
