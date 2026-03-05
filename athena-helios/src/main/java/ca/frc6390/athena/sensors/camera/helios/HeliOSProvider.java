package ca.frc6390.athena.sensors.camera.helios;

import ca.frc6390.athena.sensors.camera.CameraRegistry;
import ca.frc6390.athena.sensors.camera.ConfigurableCamera;

/**
 * Registers HeliOS camera software key with the central registry.
 */
public class HeliOSProvider implements CameraRegistry.Provider {
    @Override
    public void register(CameraRegistry registry) {
        registry.add("camera:helios", ConfigurableCamera.CameraSoftware.HeliOS);
    }
}
