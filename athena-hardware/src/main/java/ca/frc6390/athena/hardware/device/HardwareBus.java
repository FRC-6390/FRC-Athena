package ca.frc6390.athena.hardware.device;

import java.util.Objects;

import ca.frc6390.athena.api.hardware.EncoderKind;
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

    public MotorDevice motor(MotorKind kind, int id) {
        Objects.requireNonNull(kind, "kind");
        return MotorDevice.of(kind, id).canbus(name);
    }

    public EncoderDevice encoder(EncoderKind kind, int id) {
        Objects.requireNonNull(kind, "kind");
        return EncoderDevice.of(kind, id).canbus(name);
    }

    public ImuDevice imu(ImuKind kind, int id) {
        Objects.requireNonNull(kind, "kind");
        return ImuDevice.of(kind, id).canbus(name);
    }
}
