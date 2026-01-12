package ca.frc6390.athena.core;

import java.util.HashMap;
import java.util.Map;
import java.util.Objects;
import java.util.ServiceLoader;

/**
 * Discovers autonomous engines (PathPlanner, Choreo, etc.) via {@link ServiceLoader}. Users pass
 * {@link AthenaAutoEngine} and receive a clear error if the corresponding athena-* module is
 * missing.
 */
public final class AutoRegistry {
    /** Implemented by vendor/engine modules to register their auto engines. */
    public interface Provider {
        void register(AutoRegistry registry);
    }

    private static final AutoRegistry INSTANCE = new AutoRegistry();

    static {
        ServiceLoader.load(Provider.class).forEach(p -> p.register(INSTANCE));
    }

    private final Map<String, RobotAuto.AutoSource> engines = new HashMap<>();

    private AutoRegistry() {}

    public static AutoRegistry get() {
        return INSTANCE;
    }

    public AutoRegistry add(String key, RobotAuto.AutoSource source) {
        engines.put(key, source);
        return this;
    }

    public RobotAuto.AutoSource engine(String key) {
        return Objects.requireNonNull(engines.get(key), missing(key));
    }

    private static String missing(String key) {
        return "Missing provider for auto engine '" + key
                + "'. Add the appropriate athena-* auto module to dependencies.";
    }
}
