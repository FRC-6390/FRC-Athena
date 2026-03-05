package ca.frc6390.athena.sensors.camera.helios;

import ca.frc6390.athena.sensors.camera.CameraProvider;
import ca.frc6390.athena.sensors.camera.ConfigurableCamera;
import ca.frc6390.athena.sensors.camera.VisionCamera;

/**
 * Service provider that builds HeliOS-backed {@link VisionCamera} instances.
 */
public class HeliOSCameraProvider implements CameraProvider {
    @Override
    public boolean supports(ConfigurableCamera config) {
        return config instanceof HeliOSConfig;
    }

    @Override
    public VisionCamera create(ConfigurableCamera config, boolean simulation) {
        if (!(config instanceof HeliOSConfig heliosConfig)) {
            return null;
        }
        return new HeliOSCameraAdapter(heliosConfig).getVisionCamera();
    }
}
