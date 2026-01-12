package ca.frc6390.athena.hardware.imu;

import java.util.HashMap;
import java.util.Map;
import java.util.Objects;
import java.util.ServiceLoader;

/**
 * Discovers vendor-specific IMU types via {@link ServiceLoader}.
 */
public final class ImuRegistry {
    /** Implemented by vendor modules to register IMU types. */
    public interface Provider {
        void register(ImuRegistry registry);
    }

    private static final ImuRegistry INSTANCE = new ImuRegistry();

    static {
        ServiceLoader.load(Provider.class).forEach(p -> p.register(INSTANCE));
    }

    private final Map<String, ImuType> imus = new HashMap<>();

    private ImuRegistry() {}

    public static ImuRegistry get() {
        return INSTANCE;
    }

    public ImuRegistry add(ImuType type) {
        imus.put(type.getKey(), type);
        return this;
    }

    public ImuType imu(String key) {
        return Objects.requireNonNull(imus.get(key), missing(key));
    }

    private static String missing(String key) {
        return "Missing provider for IMU '" + key
                + "'. Add the appropriate athena-* vendor module to dependencies.";
    }
}
