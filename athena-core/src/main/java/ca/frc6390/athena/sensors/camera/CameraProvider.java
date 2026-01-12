package ca.frc6390.athena.sensors.camera;

/**
 * Service-provider interface used to create {@link LocalizationCamera} instances from vendor
 * configuration objects. Vendor modules implement this and register via {@link java.util.ServiceLoader}.
 */
public interface CameraProvider {
    /**
     * Returns true if this provider can handle the given camera configuration.
     */
    boolean supports(ConfigurableCamera config);

    /**
     * Creates a localization-capable camera for the provided configuration.
     *
     * @param config     vendor-specific configuration implementing {@link ConfigurableCamera}
     * @param simulation true when running under WPILib simulation
     */
    LocalizationCamera create(ConfigurableCamera config, boolean simulation);
}
