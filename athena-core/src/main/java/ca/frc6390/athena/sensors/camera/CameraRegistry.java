package ca.frc6390.athena.sensors.camera;

import java.util.ServiceLoader;

import ca.frc6390.athena.core.registry.PluginRegistryBase;

/**
 * Discovers vendor-specific camera software implementations via {@link ServiceLoader}. This lets
 * user code stay vendor-agnostic (use {@link AthenaCamera}) while surfacing a clear error if the
 * required athena-* camera module is not on the classpath.
 */
public final class CameraRegistry extends PluginRegistryBase<ConfigurableCamera.CameraSoftware> {
    /** Implemented by vendor modules to register their camera software keys. */
    public interface Provider {
        void register(CameraRegistry registry);
    }

    private static final CameraRegistry INSTANCE = new CameraRegistry();

    static {
        loadProviders(Provider.class, CameraRegistry.class, p -> p.register(INSTANCE));
    }

    private CameraRegistry() {}

    public static CameraRegistry get() {
        return INSTANCE;
    }

    public CameraRegistry add(String key, ConfigurableCamera.CameraSoftware software) {
        put(key, software);
        return this;
    }

    public ConfigurableCamera.CameraSoftware camera(String key) {
        return require(key);
    }

    @Override
    protected String missingMessage(String key) {
        return "Missing vendor provider for camera '" + key
                + "'. Add the appropriate athena-* camera module to dependencies.";
    }
}
