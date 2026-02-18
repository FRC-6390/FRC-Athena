package ca.frc6390.athena.sensors.camera;

import java.util.Optional;


/**
 * Enumeration of well-known {@link VisionCamera} capabilities. Centralising the type
 * associations in an enum keeps IntelliSense-friendly names while still giving us strongly-typed
 * lookups at runtime.
 */
public enum VisionCameraCapability {
    LED(VisionCamera.LedControl.class),
    PIPELINE(VisionCamera.PipelineControl.class),
    STREAM(VisionCamera.StreamControl.class),
    PRIORITY_ID(VisionCamera.PriorityIdControl.class),
    LOCALIZATION_SOURCE(LocalizationSource.class),
    TARGETING_SOURCE(TargetingSource.class),
    FIDUCIAL_SOURCE(FiducialSource.class),
    LIMELIGHT_CAMERA(LimelightCamera.class),
    PHOTON_VISION_CAMERA(PhotonVisionCamera.class);

    private final Class<?> type;

    VisionCameraCapability(Class<?> type) {
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
