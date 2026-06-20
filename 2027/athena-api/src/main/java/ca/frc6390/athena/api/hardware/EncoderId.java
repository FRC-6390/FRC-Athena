package ca.frc6390.athena.api.hardware;

import java.util.Objects;

/**
 * Reusable encoder identity for robot-wide hardware maps.
 *
 * @param kind encoder hardware kind
 * @param id device id or channel
 * @param canbus CAN bus name, or {@code rio} by default
 */
public record EncoderId(EncoderKind kind, int id, String canbus) {
    /**
     * Creates an encoder identity on the default bus.
     *
     * @param kind encoder kind
     * @param id device id
     * @return encoder identity
     */
    public static EncoderId of(EncoderKind kind, int id) {
        return new EncoderId(kind, id, "rio");
    }

    public EncoderId {
        Objects.requireNonNull(kind, "kind");
        canbus = canbus == null || canbus.isBlank() ? "rio" : canbus;
    }

    /**
     * Returns a copy on another CAN bus.
     *
     * @param canbus CAN bus name
     * @return updated identity
     */
    public EncoderId canbus(String canbus) {
        return new EncoderId(kind, id, canbus);
    }
}
