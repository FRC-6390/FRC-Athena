package ca.frc6390.athena.vision.runtime;

import ca.frc6390.athena.vision.ref.CameraDevice;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.ServiceLoader;

/**
 * Camera adapter discovery and binding helpers.
 */
public final class CameraAdapters {
    private CameraAdapters() {
    }

    /**
     * Discovers camera adapters through {@link ServiceLoader}.
     *
     * @return discovered adapters
     */
    public static List<CameraAdapter> discover() {
        List<CameraAdapter> adapters = new ArrayList<>();
        ServiceLoader.load(CameraAdapter.class).forEach(adapters::add);
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
