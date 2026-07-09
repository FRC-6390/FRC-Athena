package ca.frc6390.athena.runtime.measurement;

/**
 * Camera-relative target measurement.
 *
 * @param targetId target id, or -1 when not id-backed
 * @param yawDegrees yaw angle
 * @param pitchDegrees pitch angle
 * @param distanceMeters distance
 * @param xMeters camera-relative x
 * @param yMeters camera-relative y
 * @param confidence confidence score
 * @param timestampSeconds timestamp
 * @param latencySeconds latency
 * @param source source object
 */
record TargetMeasurement(
        int targetId,
        double yawDegrees,
        double pitchDegrees,
        double distanceMeters,
        double xMeters,
        double yMeters,
        double confidence,
        double timestampSeconds,
        double latencySeconds,
        Object source) implements TargetMeasurementSample {
    public TargetMeasurement {
        targetId = Math.max(-1, targetId);
        yawDegrees = finiteOrZero(yawDegrees);
        pitchDegrees = finiteOrZero(pitchDegrees);
        distanceMeters = Math.max(0.0, finiteOrZero(distanceMeters));
        xMeters = finiteOrZero(xMeters);
        yMeters = finiteOrZero(yMeters);
        confidence = finiteOrZero(confidence);
        timestampSeconds = finiteOrZero(timestampSeconds);
        latencySeconds = Math.max(0.0, finiteOrZero(latencySeconds));
    }

    private static double finiteOrZero(double value) {
        return Double.isFinite(value) ? value : 0.0;
    }
}
