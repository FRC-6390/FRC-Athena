package ca.frc6390.athena.runtime.measurement;

/**
 * Robot acceleration measurement.
 *
 * @param xMetersPerSecondSquared x acceleration
 * @param yMetersPerSecondSquared y acceleration
 * @param angularRadiansPerSecondSquared angular acceleration
 * @param timestampSeconds timestamp
 * @param latencySeconds latency
 * @param source source object
 */
record AccelerationMeasurement(
        double xMetersPerSecondSquared,
        double yMetersPerSecondSquared,
        double angularRadiansPerSecondSquared,
        double timestampSeconds,
        double latencySeconds,
        Object source) implements Measurement {
    public AccelerationMeasurement {
        xMetersPerSecondSquared = finiteOrZero(xMetersPerSecondSquared);
        yMetersPerSecondSquared = finiteOrZero(yMetersPerSecondSquared);
        angularRadiansPerSecondSquared = finiteOrZero(angularRadiansPerSecondSquared);
        timestampSeconds = finiteOrZero(timestampSeconds);
        latencySeconds = Math.max(0.0, finiteOrZero(latencySeconds));
    }

    private static double finiteOrZero(double value) {
        return Double.isFinite(value) ? value : 0.0;
    }
}
