package ca.frc6390.athena.api.hardware;

import java.util.Objects;

/**
 * Reusable IMU identity for robot-wide hardware maps.
 *
 * @param kind IMU hardware kind
 * @param id device id
 * @param canbus CAN bus name, or {@code rio} by default
 */
public record ImuId(ImuKind kind, int id, String canbus) {
    /**
     * Creates an IMU identity on the default bus.
     *
     * @param kind IMU kind
     * @param id device id
     * @return IMU identity
     */
    public static ImuId of(ImuKind kind, int id) {
        return new ImuId(kind, id, "rio");
    }

    public ImuId {
        Objects.requireNonNull(kind, "kind");
        canbus = canbus == null || canbus.isBlank() ? "rio" : canbus;
    }

    /**
     * Returns a copy on another CAN bus.
     *
     * @param canbus CAN bus name
     * @return updated identity
     */
    public ImuId canbus(String canbus) {
        return new ImuId(kind, id, canbus);
    }
}
