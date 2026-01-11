package ca.frc6390.athena.sensors.camera;

import java.util.Optional;


/**
 * Enumeration of well-known {@link LocalizationCamera} capabilities. Centralising the type
 * associations in an enum keeps IntelliSense-friendly names while still giving us strongly-typed
 * lookups at runtime.
 */
public enum LocalizationCameraCapability {
    LED(LocalizationCamera.LedControl.class),
    PIPELINE(LocalizationCamera.PipelineControl.class),
    STREAM(LocalizationCamera.StreamControl.class),
    LIMELIGHT(LimelightCapability.class),
    PHOTON_VISION(PhotonVisionCapability.class);

    private final Class<?> type;

    LocalizationCameraCapability(Class<?> type) {
        this.type = type;
    }

    public Class<?> getType() {
        return type;
    }

    @SuppressWarnings("unchecked")
    public <T> Optional<T> cast(Object value) {
        if (value == null || !type.isInstance(value)) {
            return Optional.empty();
        }
        return Optional.of((T) type.cast(value));
    }
}
