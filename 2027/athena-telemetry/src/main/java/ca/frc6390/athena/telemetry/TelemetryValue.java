package ca.frc6390.athena.telemetry;

import java.util.Objects;

/**
 * Typed telemetry value.
 *
 * @param type value type
 * @param value boxed value
 */
public record TelemetryValue(TelemetryType type, Object value) {
    public TelemetryValue {
        Objects.requireNonNull(type, "type");
        Objects.requireNonNull(value, "value");
    }

    /**
     * Creates a boolean value.
     *
     * @param value value
     * @return telemetry value
     */
    public static TelemetryValue of(boolean value) {
        return new TelemetryValue(TelemetryType.BOOLEAN, value);
    }

    /**
     * Creates a numeric value.
     *
     * @param value value
     * @return telemetry value
     */
    public static TelemetryValue of(double value) {
        return new TelemetryValue(TelemetryType.NUMBER, value);
    }

    /**
     * Creates a string value.
     *
     * @param value value
     * @return telemetry value
     */
    public static TelemetryValue of(String value) {
        return new TelemetryValue(TelemetryType.STRING, value);
    }
}
