package ca.frc6390.athena.runtime.measurement;

/**
 * Scalar distance measurement.
 *
 * @param distanceMeters distance in meters
 * @param timestampSeconds timestamp
 * @param latencySeconds latency
 * @param source source object
 */
record DistanceMeasurement(
        double distanceMeters,
        double timestampSeconds,
        double latencySeconds,
        Object source) implements Measurement {
    public DistanceMeasurement {
        distanceMeters = Double.isFinite(distanceMeters) ? Math.max(0.0, distanceMeters) : 0.0;
        timestampSeconds = Double.isFinite(timestampSeconds) ? timestampSeconds : 0.0;
        latencySeconds = Double.isFinite(latencySeconds) ? Math.max(0.0, latencySeconds) : 0.0;
    }
}
