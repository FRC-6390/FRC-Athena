package ca.frc6390.athena.sensors.camera.limelight;

import ca.frc6390.athena.sensors.camera.CameraRegistry;
import ca.frc6390.athena.sensors.camera.ConfigurableCamera;

/**
 * Registers Limelight camera software key with the central registry.
 */
public class LimelightProvider implements CameraRegistry.Provider {
    @Override
    public void register(CameraRegistry registry) {
        registry.add("camera:limelight", ConfigurableCamera.CameraSoftware.LimeLight);
    }
}
