package ca.frc6390.athena.vision.runtime;

import ca.frc6390.athena.vision.ref.CameraDevice;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.Optional;
import java.util.ServiceConfigurationError;
import java.util.ServiceLoader;

/**
 * Discovery helpers for optional vision simulation providers.
 */
public final class VisionSimulations {
    private VisionSimulations() {
    }

    /**
     * Discovers vision simulation providers through {@link ServiceLoader}.
     *
     * @return discovered providers
     */
    public static List<VisionSimulationProvider> discover() {
        List<VisionSimulationProvider> providers = new ArrayList<>();
        Iterator<VisionSimulationProvider> iterator = ServiceLoader.load(VisionSimulationProvider.class).iterator();
        while (true) {
            try {
                if (!iterator.hasNext()) {
                    break;
                }
                providers.add(iterator.next());
            } catch (ServiceConfigurationError error) {
                // Provider jars may be present while their vendor simulation libraries are not.
            }
        }
        return List.copyOf(providers);
    }

    /**
     * Creates the first discovered simulation that supports the supplied cameras.
     *
     * @param cameras camera declarations
     * @return optional vision simulation
     */
    public static Optional<VisionSimulation> createDiscovered(List<CameraDevice> cameras) {
        return create(discover(), cameras, VisionSimulationField.EMPTY);
    }

    /**
     * Creates the first discovered simulation that supports the supplied cameras.
     *
     * @param cameras camera declarations
     * @param field simulation field
     * @return optional vision simulation
     */
    public static Optional<VisionSimulation> createDiscovered(
            List<CameraDevice> cameras,
            VisionSimulationField field) {
        return create(discover(), cameras, field);
    }

    /**
     * Creates the first simulation from the supplied providers that supports the supplied cameras.
     *
     * @param providers simulation providers
     * @param cameras camera declarations
     * @return optional vision simulation
     */
    public static Optional<VisionSimulation> create(List<VisionSimulationProvider> providers, List<CameraDevice> cameras) {
        return create(providers, cameras, VisionSimulationField.EMPTY);
    }

    /**
     * Creates the first simulation from the supplied providers that supports the supplied cameras.
     *
     * @param providers simulation providers
     * @param cameras camera declarations
     * @param field simulation field
     * @return optional vision simulation
     */
    public static Optional<VisionSimulation> create(
            List<VisionSimulationProvider> providers,
            List<CameraDevice> cameras,
            VisionSimulationField field) {
        List<CameraDevice> safeCameras = cameras == null ? List.of() : List.copyOf(cameras);
        List<VisionSimulationProvider> safeProviders = providers == null ? List.of() : List.copyOf(providers);
        VisionSimulationField safeField = field == null ? VisionSimulationField.EMPTY : field;
        for (VisionSimulationProvider provider : safeProviders) {
            try {
                if (provider != null && provider.supports(safeCameras)) {
                    return Optional.ofNullable(provider.create(safeCameras, safeField));
                }
            } catch (RuntimeException | ServiceConfigurationError error) {
                // Optional vendor simulation jars can be present without a usable runtime stack.
            }
        }
        return Optional.empty();
    }
}
