package ca.frc6390.athena.hardware.device;

import java.util.Locale;
import java.util.Objects;

import ca.frc6390.athena.api.hardware.ImuKind;

/**
 * Reusable IMU declaration.
 */
public record ImuDevice(
        ImuKind kind,
        int id,
        String canbus) {
    public static ImuDevice of(ImuKind kind, int id) {
        return new ImuDevice(kind, id, "rio");
    }

    public ImuDevice {
        Objects.requireNonNull(kind, "kind");
        canbus = canbus == null || canbus.isBlank() ? "rio" : canbus;
    }

    public ImuDevice canbus(String canbus) {
        return new ImuDevice(kind, id, canbus);
    }

    public String defaultName() {
        return sanitize(kind.key()) + "_" + id;
    }

    private static String sanitize(String key) {
        return key.toLowerCase(Locale.ROOT).replace(':', '_').replace('-', '_');
    }
}
