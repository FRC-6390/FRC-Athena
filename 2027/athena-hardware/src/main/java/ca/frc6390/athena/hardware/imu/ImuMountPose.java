package ca.frc6390.athena.hardware.imu;

/**
 * Robot-relative IMU mount pose in degrees.
 *
 * @param yawDegrees yaw offset
 * @param pitchDegrees pitch offset
 * @param rollDegrees roll offset
 */
public record ImuMountPose(double yawDegrees, double pitchDegrees, double rollDegrees) {
    /**
     * Returns a zero-offset mount pose.
     *
     * @return zero mount pose
     */
    public static ImuMountPose identity() {
        return new ImuMountPose(0.0, 0.0, 0.0);
    }

    /**
     * Returns whether all offsets are finite.
     *
     * @return true if valid
     */
    public boolean isFinite() {
        return Double.isFinite(yawDegrees) && Double.isFinite(pitchDegrees) && Double.isFinite(rollDegrees);
    }
}
