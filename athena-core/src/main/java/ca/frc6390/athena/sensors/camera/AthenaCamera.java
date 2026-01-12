package ca.frc6390.athena.sensors.camera;

/**
 * Team-facing camera catalog. Entries carry vendor keys and resolve to vendor-specific software via
 * {@link CameraRegistry}.
 */
public enum AthenaCamera {
    LIMELIGHT("camera:limelight"),
    PHOTON_VISION("camera:photonvision");

    private final String key;

    AthenaCamera(String key) {
        this.key = key;
    }

    public ConfigurableCamera.CameraSoftware resolve() {
        return CameraRegistry.get().camera(key);
    }
}
