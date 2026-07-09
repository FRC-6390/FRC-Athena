package ca.frc6390.athena.runtime.measurement;

/**
 * Typed target-capable measurement sample.
 */
public interface TargetMeasurementSample extends Measurement {
    /**
     * Returns the target id, or {@code -1} when unavailable.
     *
     * @return target id
     */
    int targetId();

    /**
     * Returns target yaw.
     *
     * @return yaw in degrees
     */
    double yawDegrees();

    /**
     * Returns target pitch.
     *
     * @return pitch in degrees
     */
    double pitchDegrees();

    /**
     * Returns estimated target distance.
     *
     * @return distance in meters
     */
    double distanceMeters();

    /**
     * Returns target confidence where higher is better.
     *
     * @return confidence
     */
    double confidence();
}
