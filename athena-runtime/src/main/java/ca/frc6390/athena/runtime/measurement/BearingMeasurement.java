package ca.frc6390.athena.runtime.measurement;

/**
 * Bearing measurement to a target.
 *
 * @param yawDegrees yaw angle
 * @param pitchDegrees pitch angle
 * @param timestampSeconds timestamp
 * @param latencySeconds latency
 * @param source source object
 */
public record BearingMeasurement(
        double yawDegrees,
        double pitchDegrees,
        double timestampSeconds,
        double latencySeconds,
        Object source) implements Measurement {
    public BearingMeasurement {
        yawDegrees = Double.isFinite(yawDegrees) ? yawDegrees : 0.0;
        pitchDegrees = Double.isFinite(pitchDegrees) ? pitchDegrees : 0.0;
        timestampSeconds = Double.isFinite(timestampSeconds) ? timestampSeconds : 0.0;
        latencySeconds = Double.isFinite(latencySeconds) ? Math.max(0.0, latencySeconds) : 0.0;
    }
}
