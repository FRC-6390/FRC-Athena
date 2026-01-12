package ca.frc6390.athena.sensors.camera;

import java.util.HashMap;
import java.util.Map;
import java.util.Objects;
import java.util.ServiceLoader;

/**
 * Discovers vendor-specific camera software implementations via {@link ServiceLoader}. This lets
 * user code stay vendor-agnostic (use {@link AthenaCamera}) while surfacing a clear error if the
 * required athena-* camera module is not on the classpath.
 */
public final class CameraRegistry {
    /** Implemented by vendor modules to register their camera software keys. */
    public interface Provider {
        void register(CameraRegistry registry);
    }

    private static final CameraRegistry INSTANCE = new CameraRegistry();

    static {
        ServiceLoader.load(Provider.class).forEach(p -> p.register(INSTANCE));
    }

    private final Map<String, ConfigurableCamera.CameraSoftware> cameras = new HashMap<>();

    private CameraRegistry() {}

    public static CameraRegistry get() {
        return INSTANCE;
    }

    public CameraRegistry add(String key, ConfigurableCamera.CameraSoftware software) {
        cameras.put(key, software);
        return this;
    }

    public ConfigurableCamera.CameraSoftware camera(String key) {
        return Objects.requireNonNull(cameras.get(key), missing(key));
    }

    private static String missing(String key) {
        return "Missing vendor provider for camera '" + key
                + "'. Add the appropriate athena-* camera module to dependencies.";
    }
}
