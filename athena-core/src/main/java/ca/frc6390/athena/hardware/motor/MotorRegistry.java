package ca.frc6390.athena.hardware.motor;

import java.util.Objects;
import java.util.ServiceLoader;

import ca.frc6390.athena.core.registry.PluginRegistryBase;

/**
 * Discovers vendor-specific motor controller types via {@link ServiceLoader}. Missing providers
 * surface clear guidance to add the appropriate athena-* module.
 */
public final class MotorRegistry extends PluginRegistryBase<MotorControllerType> {
    /** Implemented by vendor modules to register motor controller types. */
    public interface Provider {
        void register(MotorRegistry registry);
    }

    private static final MotorRegistry INSTANCE = new MotorRegistry();

    static {
        loadProviders(Provider.class, MotorRegistry.class, p -> p.register(INSTANCE));
    }

    private MotorRegistry() {}

    public static MotorRegistry get() {
        return INSTANCE;
    }

    public MotorRegistry add(MotorControllerType type) {
        Objects.requireNonNull(type, "type");
        put(type.getKey(), type);
        return this;
    }

    public MotorControllerType motor(String key) {
        return require(key);
    }

    @Override
    protected String missingMessage(String key) {
        return "Missing provider for motor '" + key
                + "'. Add the appropriate athena-* vendor module to dependencies.";
    }
}
