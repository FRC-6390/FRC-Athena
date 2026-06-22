package ca.frc6390.athena.telemetry;

import java.util.Map;
import java.util.Optional;

/**
 * Immutable telemetry snapshot.
 *
 * @param values values by key
 */
public record TelemetrySnapshot(Map<TelemetryKey, TelemetryValue> values) {
    public TelemetrySnapshot {
        values = Map.copyOf(values);
    }

    /**
     * Finds a value by key.
     *
     * @param key telemetry key
     * @return value if present
     */
    public Optional<TelemetryValue> find(TelemetryKey key) {
        return Optional.ofNullable(values.get(key));
    }
}
