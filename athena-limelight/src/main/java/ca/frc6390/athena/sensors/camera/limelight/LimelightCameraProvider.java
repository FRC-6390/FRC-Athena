package ca.frc6390.athena.sensors.camera.limelight;

import ca.frc6390.athena.sensors.camera.CameraProvider;
import ca.frc6390.athena.sensors.camera.ConfigurableCamera;
import ca.frc6390.athena.sensors.camera.LocalizationCamera;

/**
 * Service provider that builds Limelight-backed {@link LocalizationCamera} instances.
 */
public class LimelightCameraProvider implements CameraProvider {
    @Override
    public boolean supports(ConfigurableCamera config) {
        return config instanceof LimeLightConfig;
    }

    @Override
    public LocalizationCamera create(ConfigurableCamera config, boolean simulation) {
        if (!(config instanceof LimeLightConfig llConfig)) {
            return null;
        }

        if (simulation) {
            // In sim, reuse the Limelight behavior directly; the class already gates sim specifics.
            return new LimeLight(llConfig).getLocalizationCamera();
        }
        return new LimeLight(llConfig).getLocalizationCamera();
    }
}
