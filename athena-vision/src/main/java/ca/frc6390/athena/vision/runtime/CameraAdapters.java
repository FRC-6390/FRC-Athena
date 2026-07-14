package ca.frc6390.athena.vision.runtime;

import ca.frc6390.athena.vision.device.CameraDevice;
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
    private static volatile List<String> discoveryFailures = List.of();

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
            discoveryFailures = List.of();
        }
    }

    private static List<CameraAdapter> load() {
        List<CameraAdapter> adapters = new ArrayList<>();
        List<String> failures = new ArrayList<>();
        Iterator<CameraAdapter> iterator = ServiceLoader.load(CameraAdapter.class).iterator();
        while (true) {
            try {
                if (!iterator.hasNext()) {
                    break;
                }
                adapters.add(iterator.next());
            } catch (ServiceConfigurationError error) {
                failures.add(describe(error));
            }
        }
        discoveryFailures = List.copyOf(failures);
        return List.copyOf(adapters);
    }

    public static List<String> discoveryFailures() {
        discover();
        return discoveryFailures;
    }

    public static String missingAdapterMessage(CameraDevice camera) {
        String failures = discoveryFailures().isEmpty()
                ? "No camera adapter advertised support for this camera kind."
                : "Vendor adapter loading failed: " + String.join("; ", discoveryFailures());
        return "Athena cannot start camera '" + camera.name() + "' (" + camera.kind().key() + "). "
                + failures + " Add the vendor's WPILib vendordep, or disable dependency checks only for debugging.";
    }

    private static String describe(Throwable error) {
        Throwable cause = error;
        while (cause.getCause() != null) {
            cause = cause.getCause();
        }
        String provider = error.getMessage();
        String missing = cause.getMessage();
        String detail = cause.getClass().getSimpleName()
                + (missing == null || missing.isBlank() ? "" : ": " + missing);
        return provider == null || provider.isBlank() ? detail : provider + " (" + detail + ")";
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
