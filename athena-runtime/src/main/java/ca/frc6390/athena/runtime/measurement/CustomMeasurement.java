package ca.frc6390.athena.runtime.measurement;

import java.util.Map;

/**
 * Custom measurement payload for data Athena does not model directly.
 *
 * @param value value object
 * @param metadata metadata map
 * @param timestampSeconds timestamp
 * @param latencySeconds latency
 * @param source source object
 */
public record CustomMeasurement(
        Object value,
        Map<String, Object> metadata,
        double timestampSeconds,
        double latencySeconds,
        Object source) implements Measurement {
    public CustomMeasurement {
        metadata = metadata == null ? Map.of() : Map.copyOf(metadata);
        timestampSeconds = Double.isFinite(timestampSeconds) ? timestampSeconds : 0.0;
        latencySeconds = Double.isFinite(latencySeconds) ? Math.max(0.0, latencySeconds) : 0.0;
    }
}
