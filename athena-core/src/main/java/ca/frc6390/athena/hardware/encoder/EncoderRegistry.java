package ca.frc6390.athena.hardware.encoder;

import java.util.Objects;
import java.util.ServiceLoader;

import ca.frc6390.athena.core.registry.PluginRegistryBase;

/**
 * Discovers vendor-specific encoder types via {@link ServiceLoader}.
 */
public final class EncoderRegistry extends PluginRegistryBase<EncoderType> {
    /** Implemented by vendor modules to register encoder types. */
    public interface Provider {
        void register(EncoderRegistry registry);
    }

    private static final EncoderRegistry INSTANCE = new EncoderRegistry();

    static {
        loadProviders(Provider.class, EncoderRegistry.class, p -> p.register(INSTANCE));
    }

    private EncoderRegistry() {}

    public static EncoderRegistry get() {
        return INSTANCE;
    }

    public EncoderRegistry add(EncoderType type) {
        Objects.requireNonNull(type, "type");
        put(type.getKey(), type);
        return this;
    }

    public EncoderType encoder(String key) {
        return require(key);
    }

    @Override
    protected String missingMessage(String key) {
        return "Missing provider for encoder '" + key
                + "'. Add the appropriate athena-* vendor module to dependencies.";
    }
}
