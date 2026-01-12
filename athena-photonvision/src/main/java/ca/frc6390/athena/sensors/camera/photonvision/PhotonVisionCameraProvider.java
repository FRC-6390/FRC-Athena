package ca.frc6390.athena.sensors.camera.photonvision;

import ca.frc6390.athena.sensors.camera.CameraProvider;
import ca.frc6390.athena.sensors.camera.ConfigurableCamera;
import ca.frc6390.athena.sensors.camera.LocalizationCamera;

/**
 * Service provider that builds PhotonVision-backed {@link LocalizationCamera} instances.
 */
public class PhotonVisionCameraProvider implements CameraProvider {
    @Override
    public boolean supports(ConfigurableCamera config) {
        return config instanceof PhotonVisionConfig;
    }

    @Override
    public LocalizationCamera create(ConfigurableCamera config, boolean simulation) {
        if (!(config instanceof PhotonVisionConfig pvConfig)) {
            return null;
        }
        PhotonVision photonVision = new PhotonVision(pvConfig);
        return photonVision.getLocalizationCamera();
    }
}
