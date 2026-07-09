package ca.frc6390.athena.vision.runtime;

import ca.frc6390.athena.vision.ref.CameraDevice;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.Objects;
import java.util.ServiceConfigurationError;
import java.util.ServiceLoader;

/**
 * Camera adapter discovery and binding helpers.
 */
public final class CameraAdapters {
    private static volatile List<CameraAdapter> discovered;

    private CameraAdapters() {
    }

    /**
     * Discovers camera adapters through {@link ServiceLoader}.
     *
     * @return discovered adapters
     */
    public static List<CameraAdapter> discover() {
        List<CameraAdapter> cached = discovered;
        if (cached != null) {
            return cached;
        }
        synchronized (CameraAdapters.class) {
            if (discovered == null) {
                discovered = load();
            }
            return discovered;
        }
    }

    /**
     * Clears cached adapter discovery. Intended for tests that change the context class loader.
     */
    static void clearCache() {
        synchronized (CameraAdapters.class) {
            discovered = null;
        }
    }

    private static List<CameraAdapter> load() {
        List<CameraAdapter> adapters = new ArrayList<>();
        Iterator<CameraAdapter> iterator = ServiceLoader.load(CameraAdapter.class).iterator();
        while (true) {
            try {
                if (!iterator.hasNext()) {
                    break;
                }
                adapters.add(iterator.next());
            } catch (ServiceConfigurationError error) {
                // Adapter jars may be present while their real vendor libraries are not.
            }
        }
        return List.copyOf(adapters);
    }

    /**
     * Binds a camera using the first discovered supporting adapter.
     *
     * @param camera camera declaration
     * @return bound camera, or the original declaration if no adapter supports it
     */
    public static CameraDevice bindDiscovered(CameraDevice camera) {
        Objects.requireNonNull(camera, "camera");
        for (CameraAdapter adapter : discover()) {
            if (adapter.supports(camera)) {
                return adapter.bind(camera);
            }
        }
        return camera;
    }
}
