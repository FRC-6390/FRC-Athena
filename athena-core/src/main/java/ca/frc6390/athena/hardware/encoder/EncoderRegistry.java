package ca.frc6390.athena.hardware.encoder;

import java.util.HashMap;
import java.util.Map;
import java.util.Objects;
import java.util.ServiceLoader;

/**
 * Discovers vendor-specific encoder types via {@link ServiceLoader}.
 */
public final class EncoderRegistry {
    /** Implemented by vendor modules to register encoder types. */
    public interface Provider {
        void register(EncoderRegistry registry);
    }

    private static final EncoderRegistry INSTANCE = new EncoderRegistry();

    static {
        ClassLoader contextLoader = Thread.currentThread().getContextClassLoader();
        loadProviders(contextLoader);
        ClassLoader registryLoader = EncoderRegistry.class.getClassLoader();
        if (registryLoader != contextLoader) {
            loadProviders(registryLoader);
        }
    }

    private final Map<String, EncoderType> encoders = new HashMap<>();

    private EncoderRegistry() {}

    public static EncoderRegistry get() {
        return INSTANCE;
    }

    public EncoderRegistry add(EncoderType type) {
        encoders.put(type.getKey(), type);
        return this;
    }

    public EncoderType encoder(String key) {
        return Objects.requireNonNull(encoders.get(key), missing(key));
    }

    private static void loadProviders(ClassLoader loader) {
        ServiceLoader<Provider> providers = loader == null
                ? ServiceLoader.load(Provider.class)
                : ServiceLoader.load(Provider.class, loader);
        providers.forEach(p -> p.register(INSTANCE));
    }

    private static String missing(String key) {
        return "Missing provider for encoder '" + key
                + "'. Add the appropriate athena-* vendor module to dependencies.";
    }
}
