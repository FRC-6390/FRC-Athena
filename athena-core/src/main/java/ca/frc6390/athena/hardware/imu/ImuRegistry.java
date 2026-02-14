package ca.frc6390.athena.hardware.imu;

import java.util.Objects;
import java.util.ServiceLoader;

import ca.frc6390.athena.core.registry.PluginRegistryBase;

/**
 * Discovers vendor-specific IMU types via {@link ServiceLoader}.
 */
public final class ImuRegistry extends PluginRegistryBase<ImuType> {
    /** Implemented by vendor modules to register IMU types. */
    public interface Provider {
        void register(ImuRegistry registry);
    }

    private static final ImuRegistry INSTANCE = new ImuRegistry();

    static {
        loadProviders(Provider.class, ImuRegistry.class, p -> p.register(INSTANCE));
    }

    private ImuRegistry() {}

    public static ImuRegistry get() {
        return INSTANCE;
    }

    public ImuRegistry add(ImuType type) {
        Objects.requireNonNull(type, "type");
        put(type.getKey(), type);
        return this;
    }

    public ImuType imu(String key) {
        return require(key);
    }

    @Override
    protected String missingMessage(String key) {
        return "Missing provider for IMU '" + key
                + "'. Add the appropriate athena-* vendor module to dependencies.";
    }
}
