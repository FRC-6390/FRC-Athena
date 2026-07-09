package ca.frc6390.athena.hardware.device;

import java.util.Locale;
import java.util.Objects;

import ca.frc6390.athena.api.hardware.ImuKind;

/**
 * Reusable IMU declaration.
 */
public record ImuDevice(
        ImuKind kind,
        String bus,
        HardwarePort port) {
    public static ImuDevice of(ImuKind kind, int id) {
        return connected(kind, "rio", HardwarePort.can(id));
    }

    static ImuDevice connected(ImuKind kind, String bus, HardwarePort port) {
        return new ImuDevice(kind, bus, port);
    }

    public ImuDevice {
        Objects.requireNonNull(kind, "kind");
        bus = bus == null || bus.isBlank() ? "rio" : bus;
        Objects.requireNonNull(port, "port");
    }

    public int id() {
        return requireCan().id();
    }

    public String canbus() {
        requireCan();
        return bus;
    }

    public ImuDevice canbus(String canbus) {
        requireCan();
        return new ImuDevice(kind, canbus, port);
    }

    public String defaultName() {
        return sanitize(kind.key()) + "_" + sanitize(port.identity()) + "_" + port.primaryAddress();
    }

    private static String sanitize(String key) {
        return key.toLowerCase(Locale.ROOT).replace(':', '_').replace('-', '_');
    }

    private HardwarePort.Can requireCan() {
        if (port instanceof HardwarePort.Can can) {
            return can;
        }
        throw new IllegalStateException("IMU " + kind.key() + " is not connected over CAN.");
    }
}
