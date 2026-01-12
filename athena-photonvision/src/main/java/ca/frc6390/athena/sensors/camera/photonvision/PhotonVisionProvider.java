package ca.frc6390.athena.sensors.camera.photonvision;

import ca.frc6390.athena.sensors.camera.CameraRegistry;
import ca.frc6390.athena.sensors.camera.ConfigurableCamera;

/**
 * Registers PhotonVision camera software key with the central registry.
 */
public class PhotonVisionProvider implements CameraRegistry.Provider {
    @Override
    public void register(CameraRegistry registry) {
        registry.add("camera:photonvision", ConfigurableCamera.CameraSoftware.PhotonVision);
    }
}
