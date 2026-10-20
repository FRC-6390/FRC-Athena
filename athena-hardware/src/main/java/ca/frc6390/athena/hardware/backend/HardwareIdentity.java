package ca.frc6390.athena.hardware.backend;

import java.util.Locale;
import java.util.Map;
import java.util.Objects;
import java.util.concurrent.ConcurrentHashMap;
import java.util.function.Function;

import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.device.ImuDevice;
import ca.frc6390.athena.hardware.device.MotorDevice;

/**
 * Stable runtime identity for hardware declarations.
 */
public record HardwareIdentity(String category, String kindKey, String bus, int id, String detail) {
    // Identities derived from declarations, computed once per declaration (or encoder source) and bounded so ad hoc
    // declarations cannot grow the caches without limit. Identities are immutable values.
    private static final int MAX_CACHED_IDENTITIES = 1024;
    private static final Map<MotorDevice, HardwareIdentity> MOTORS = new ConcurrentHashMap<>();
    private static final Map<EncoderDevice.EncoderSource, HardwareIdentity> ENCODERS = new ConcurrentHashMap<>();
    private static final Map<ImuDevice, HardwareIdentity> IMUS = new ConcurrentHashMap<>();

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
        return cached(MOTORS, device, HardwareIdentity::deriveMotor);
    }

    private static HardwareIdentity deriveMotor(MotorDevice device) {
        String controllerKey = device.kind().controllerKind() == null
                ? device.kind().key()
                : device.kind().controllerKind().key();
        return new HardwareIdentity("motor", controllerKey, device.canbus(), device.id(), "");
    }

    /**
     * Creates an encoder identity.
     *
     * @param device encoder declaration
     * @return identity
     */
    public static HardwareIdentity encoder(EncoderDevice device) {
        Objects.requireNonNull(device, "device");
        // The identity depends only on the encoder source (kind, bus and connection, or the owning motor).
        HardwareIdentity cached = ENCODERS.get(device.source());
        if (cached != null) {
            return cached;
        }
        HardwareIdentity identity = deriveEncoder(device);
        if (ENCODERS.size() < MAX_CACHED_IDENTITIES) {
            HardwareIdentity previous = ENCODERS.putIfAbsent(device.source(), identity);
            if (previous != null) {
                return previous;
            }
        }
        return identity;
    }

    private static HardwareIdentity deriveEncoder(EncoderDevice device) {
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
                device.connection().primaryAddress(),
                device.connection() instanceof ca.frc6390.athena.hardware.device.HardwareAddress.Can
                        ? ""
                        : device.connection().identity());
    }

    /**
     * Creates an IMU identity.
     *
     * @param device IMU declaration
     * @return identity
     */
    public static HardwareIdentity imu(ImuDevice device) {
        Objects.requireNonNull(device, "device");
        return cached(IMUS, device, HardwareIdentity::deriveImu);
    }

    private static HardwareIdentity deriveImu(ImuDevice device) {
        return new HardwareIdentity(
                "imu",
                device.kind().key(),
                device.bus(),
                device.connection().primaryAddress(),
                device.connection() instanceof ca.frc6390.athena.hardware.device.HardwareAddress.Can
                        ? ""
                        : device.connection().identity());
    }

    private static <K> HardwareIdentity cached(
            Map<K, HardwareIdentity> cache, K key, Function<K, HardwareIdentity> derive) {
        HardwareIdentity cached = cache.get(key);
        if (cached != null) {
            return cached;
        }
        HardwareIdentity identity = derive.apply(key);
        if (cache.size() < MAX_CACHED_IDENTITIES) {
            HardwareIdentity previous = cache.putIfAbsent(key, identity);
            if (previous != null) {
                return previous;
            }
        }
        return identity;
    }

    private static String normalize(String value, String fallback) {
        return value == null || value.isBlank() ? fallback : value.trim();
    }

    private static String sanitize(String value) {
        return value.toLowerCase(Locale.ROOT).replace(':', '_').replace('-', '_').replace(' ', '_');
    }
}
