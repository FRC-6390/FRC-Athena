package ca.frc6390.athena.telemetry;

import java.util.Objects;

/**
 * Typed telemetry key.
 *
 * @param path dashboard/network path
 * @param type telemetry value type
 */
public record TelemetryKey(String path, TelemetryType type) {
    public TelemetryKey {
        Objects.requireNonNull(path, "path");
        Objects.requireNonNull(type, "type");
        if (path.isBlank()) {
            throw new IllegalArgumentException("Telemetry path cannot be blank.");
        }
    }

    /**
     * Creates a boolean key.
     *
     * @param path path
     * @return telemetry key
     */
    public static TelemetryKey bool(String path) {
        return new TelemetryKey(path, TelemetryType.BOOLEAN);
    }

    /**
     * Creates a numeric key.
     *
     * @param path path
     * @return telemetry key
     */
    public static TelemetryKey number(String path) {
        return new TelemetryKey(path, TelemetryType.NUMBER);
    }

    /**
     * Creates a string key.
     *
     * @param path path
     * @return telemetry key
     */
    public static TelemetryKey string(String path) {
        return new TelemetryKey(path, TelemetryType.STRING);
    }
}
