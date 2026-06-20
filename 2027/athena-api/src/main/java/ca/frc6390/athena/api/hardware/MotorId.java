package ca.frc6390.athena.api.hardware;

import java.util.Objects;

/**
 * Reusable motor identity for robot-wide hardware maps.
 *
 * @param kind motor hardware kind
 * @param id device id on the selected bus
 * @param canbus CAN bus name, or {@code rio} by default
 */
public record MotorId(MotorKind kind, int id, String canbus) {
    /**
     * Creates a motor identity on the default roboRIO CAN bus.
     *
     * @param kind motor hardware kind
     * @param id device id
     * @return motor identity
     */
    public static MotorId of(MotorKind kind, int id) {
        return new MotorId(kind, id, "rio");
    }

    public MotorId {
        Objects.requireNonNull(kind, "kind");
        canbus = canbus == null || canbus.isBlank() ? "rio" : canbus;
    }

    /**
     * Returns a copy on another CAN bus.
     *
     * @param canbus CAN bus name
     * @return updated identity
     */
    public MotorId canbus(String canbus) {
        return new MotorId(kind, id, canbus);
    }
}
