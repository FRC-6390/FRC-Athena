package ca.frc6390.athena.hardware.ref;

import java.util.Objects;

import ca.frc6390.athena.api.hardware.EncoderKind;
import ca.frc6390.athena.api.hardware.ImuId;
import ca.frc6390.athena.api.hardware.ImuKind;
import ca.frc6390.athena.api.hardware.MotorKind;

/**
 * Named hardware bus used to declare devices without repeating the bus on every ref.
 */
public record HardwareBus(String name) {
    public static HardwareBus rio() {
        return new HardwareBus("rio");
    }

    public static HardwareBus can(String name) {
        return new HardwareBus(name);
    }

    public HardwareBus {
        name = name == null || name.isBlank() ? "rio" : name;
    }

    public MotorRef motor(MotorKind kind, int id) {
        Objects.requireNonNull(kind, "kind");
        return MotorRef.of(kind, id).canbus(name);
    }

    public EncoderRef encoder(EncoderKind kind, int id) {
        Objects.requireNonNull(kind, "kind");
        return EncoderRef.of(kind, id).canbus(name);
    }

    public ImuId imu(ImuKind kind, int id) {
        Objects.requireNonNull(kind, "kind");
        return ImuId.of(kind, id).canbus(name);
    }
}
