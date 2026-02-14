package ca.frc6390.athena.core;

import java.util.ServiceLoader;

import ca.frc6390.athena.core.registry.PluginRegistryBase;

/**
 * Discovers autonomous engines (PathPlanner, Choreo, etc.) via {@link ServiceLoader}. Users pass
 * {@link AthenaAutoEngine} and receive a clear error if the corresponding athena-* module is
 * missing.
 */
public final class AutoRegistry extends PluginRegistryBase<RobotAuto.AutoSource> {
    /** Implemented by vendor/engine modules to register their auto engines. */
    public interface Provider {
        void register(AutoRegistry registry);
    }

    private static final AutoRegistry INSTANCE = new AutoRegistry();

    static {
        loadProviders(Provider.class, AutoRegistry.class, p -> p.register(INSTANCE));
    }

    private AutoRegistry() {}

    public static AutoRegistry get() {
        return INSTANCE;
    }

    public AutoRegistry add(String key, RobotAuto.AutoSource source) {
        put(key, source);
        return this;
    }

    public RobotAuto.AutoSource engine(String key) {
        return require(key);
    }

    @Override
    protected String missingMessage(String key) {
        return "Missing provider for auto engine '" + key
                + "'. Add the appropriate athena-* auto module to dependencies.";
    }
}
