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
        ServiceLoader.load(Provider.class).forEach(p -> p.register(INSTANCE));
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

    private static String missing(String key) {
        return "Missing provider for encoder '" + key
                + "'. Add the appropriate athena-* vendor module to dependencies.";
    }
}
