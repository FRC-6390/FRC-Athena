package ca.frc6390.athena.hardware.device;

import java.util.EnumSet;
import java.util.Objects;
import java.util.Set;

import ca.frc6390.athena.api.hardware.EncoderKind;
import ca.frc6390.athena.api.hardware.ImuKind;
import ca.frc6390.athena.api.hardware.MotorKind;

/**
 * Named hardware bus used to declare devices without repeating the bus on every ref.
 */
public record HardwareBus(String name, Set<HardwareInterface> interfaces) {
    public static HardwareBus rio() {
        return new HardwareBus("rio", EnumSet.allOf(HardwareInterface.class));
    }

    public static HardwareBus can(String name) {
        return new HardwareBus(name, EnumSet.of(HardwareInterface.CAN));
    }

    public HardwareBus {
        name = name == null || name.isBlank() ? "rio" : name;
        interfaces = interfaces == null || interfaces.isEmpty()
                ? Set.of(HardwareInterface.CAN)
                : Set.copyOf(interfaces);
    }

    public HardwareBus(String name) {
        this(name, Set.of(HardwareInterface.CAN));
    }

    public MotorDevice motor(MotorKind kind, int id) {
        Objects.requireNonNull(kind, "kind");
        return MotorDevice.of(kind, id).canbus(name);
    }

    public EncoderDevice encoder(EncoderKind kind, int id) {
        return encoder(kind, HardwarePort.can(id));
    }

    public EncoderDevice encoder(EncoderKind kind, HardwarePort port) {
        Objects.requireNonNull(kind, "kind");
        requireSupported(port);
        return EncoderDevice.connected(kind, name, port);
    }

    public ImuDevice imu(ImuKind kind, int id) {
        return imu(kind, HardwarePort.can(id));
    }

    public ImuDevice imu(ImuKind kind, HardwarePort port) {
        Objects.requireNonNull(kind, "kind");
        requireSupported(port);
        return ImuDevice.connected(kind, name, port);
    }

    public boolean supports(HardwareInterface hardwareInterface) {
        return interfaces.contains(Objects.requireNonNull(hardwareInterface, "hardwareInterface"));
    }

    private void requireSupported(HardwarePort port) {
        Objects.requireNonNull(port, "port");
        if (!supports(port.hardwareInterface())) {
            throw new IllegalArgumentException(
                    "Hardware bus \"" + name + "\" does not provide " + port.hardwareInterface() + ".");
        }
    }
}
