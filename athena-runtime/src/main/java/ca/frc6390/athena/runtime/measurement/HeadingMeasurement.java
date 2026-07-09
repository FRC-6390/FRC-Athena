package ca.frc6390.athena.runtime.measurement;

/**
 * Heading measurement.
 *
 * @param headingRadians heading in radians
 * @param timestampSeconds timestamp
 * @param latencySeconds latency
 * @param source source object
 */
record HeadingMeasurement(
        double headingRadians,
        double timestampSeconds,
        double latencySeconds,
        Object source) implements Measurement {
    public HeadingMeasurement {
        headingRadians = Double.isFinite(headingRadians) ? headingRadians : 0.0;
        timestampSeconds = Double.isFinite(timestampSeconds) ? timestampSeconds : 0.0;
        latencySeconds = Double.isFinite(latencySeconds) ? Math.max(0.0, latencySeconds) : 0.0;
    }
}
