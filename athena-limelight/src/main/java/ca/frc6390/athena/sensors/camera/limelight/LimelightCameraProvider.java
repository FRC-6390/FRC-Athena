package ca.frc6390.athena.sensors.camera.limelight;

import ca.frc6390.athena.sensors.camera.CameraProvider;
import ca.frc6390.athena.sensors.camera.ConfigurableCamera;
import ca.frc6390.athena.sensors.camera.VisionCamera;

/**
 * Service provider that builds Limelight-backed {@link VisionCamera} instances.
 */
public class LimelightCameraProvider implements CameraProvider {
    @Override
    public boolean supports(ConfigurableCamera config) {
        return config instanceof LimeLightConfig;
    }

    @Override
    public VisionCamera create(ConfigurableCamera config, boolean simulation) {
        if (!(config instanceof LimeLightConfig llConfig)) {
            return null;
        }

        if (simulation) {
            // In sim, reuse the Limelight behavior directly; the class already gates sim specifics.
            return new LimeLight(llConfig).getVisionCamera();
        }
        return new LimeLight(llConfig).getVisionCamera();
    }
}
