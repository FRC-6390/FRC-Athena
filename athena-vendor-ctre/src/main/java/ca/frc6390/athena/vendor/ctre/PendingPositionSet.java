package ca.frc6390.athena.vendor.ctre;

/**
 * Bridges the gap between a Phoenix {@code setPosition} call and the first status frame that
 * reflects it.
 *
 * <p>{@code setPosition} returns once the device acknowledges the new position, but the cached
 * position status signal still holds the frame received before the set until the next frame
 * arrives. Without this bridge a refresh in the following loop would report the pre-set position
 * for one cycle.</p>
 */
final class PendingPositionSet {
    private boolean pending;
    private double rotations;
    private double setTimeSeconds;

    /**
     * Records a successful position set.
     *
     * @param rotations position written to the device
     * @param nowSeconds time the set completed, in the status-signal timebase
     */
    void set(double rotations, double nowSeconds) {
        this.rotations = rotations;
        this.setTimeSeconds = nowSeconds;
        pending = true;
    }

    /**
     * Returns the measured position, or the pending set value while the newest frame predates the set.
     *
     * @param measuredRotations position from the newest status frame
     * @param frameTimeSeconds receive time of the newest status frame
     * @return position to report
     */
    double resolve(double measuredRotations, double frameTimeSeconds) {
        if (!pending) {
            return measuredRotations;
        }
        if (frameTimeSeconds > setTimeSeconds) {
            pending = false;
            return measuredRotations;
        }
        return rotations;
    }
}
