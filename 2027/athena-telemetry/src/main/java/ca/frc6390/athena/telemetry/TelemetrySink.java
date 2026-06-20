package ca.frc6390.athena.telemetry;

/**
 * Destination for telemetry values.
 */
@FunctionalInterface
public interface TelemetrySink {
    /**
     * Publishes one telemetry value.
     *
     * @param key telemetry key
     * @param value telemetry value
     */
    void publish(TelemetryKey key, TelemetryValue value);
}
