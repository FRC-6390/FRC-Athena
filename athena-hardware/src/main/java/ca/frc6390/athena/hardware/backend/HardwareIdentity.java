package ca.frc6390.athena.hardware.backend;

import java.util.Locale;
import java.util.Objects;

import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.device.ImuDevice;
import ca.frc6390.athena.hardware.device.MotorDevice;

/**
 * Stable runtime identity for hardware declarations.
 */
public record HardwareIdentity(String category, String kindKey, String bus, int id, String detail) {
    public HardwareIdentity {
        category = normalize(category, "hardware");
        kindKey = normalize(kindKey, "unknown");
        bus = normalize(bus, "rio");
        detail = normalize(detail, "");
    }

    /**
     * Returns a stable string key suitable for runtime maps and diagnostics.
     *
     * @return identity key
     */
    public String key() {
        String suffix = detail.isBlank() ? "" : ":" + detail;
        return category + ":" + sanitize(kindKey) + ":" + sanitize(bus) + ":" + id + suffix;
    }

    /**
     * Creates a motor identity.
     *
     * @param device motor declaration
     * @return identity
     */
    public static HardwareIdentity motor(MotorDevice device) {
        Objects.requireNonNull(device, "device");
        return new HardwareIdentity("motor", device.kind().key(), device.canbus(), device.id(), "");
    }

    /**
     * Creates an encoder identity.
     *
     * @param device encoder declaration
     * @return identity
     */
    public static HardwareIdentity encoder(EncoderDevice device) {
        Objects.requireNonNull(device, "device");
        if (device.source() instanceof EncoderDevice.EncoderSource.IntegratedMotor integrated) {
            MotorDevice motor = integrated.motor();
            return new HardwareIdentity("encoder", device.kind().key(), motor.canbus(), motor.id(), "integrated");
        }
        if (device.source() instanceof EncoderDevice.EncoderSource.MotorAbsolute absolute) {
            MotorDevice motor = absolute.motor();
            return new HardwareIdentity("encoder", device.kind().key(), motor.canbus(), motor.id(), "absolute");
        }
        return new HardwareIdentity(
                "encoder",
                device.kind().key(),
                device.bus(),
                device.port().primaryAddress(),
                device.port() instanceof ca.frc6390.athena.hardware.device.HardwarePort.Can
                        ? ""
                        : device.port().identity());
    }

    /**
     * Creates an IMU identity.
     *
     * @param device IMU declaration
     * @return identity
     */
    public static HardwareIdentity imu(ImuDevice device) {
        Objects.requireNonNull(device, "device");
        return new HardwareIdentity(
                "imu",
                device.kind().key(),
                device.bus(),
                device.port().primaryAddress(),
                device.port() instanceof ca.frc6390.athena.hardware.device.HardwarePort.Can
                        ? ""
                        : device.port().identity());
    }

    private static String normalize(String value, String fallback) {
        return value == null || value.isBlank() ? fallback : value.trim();
    }

    private static String sanitize(String value) {
        return value.toLowerCase(Locale.ROOT).replace(':', '_').replace('-', '_').replace(' ', '_');
    }
}
