package frc.robot.vision;

import ca.frc6390.athena.runtime.measurement.Measurements;
import ca.frc6390.athena.vision.device.CameraDevice;
import ca.frc6390.athena.vision.runtime.CameraAdapter;
import java.util.List;

/** Minimal adapter shape for a team-owned camera kind. Register it through ServiceLoader in a library. */
public final class ExampleCameraAdapter implements CameraAdapter {
    @Override
    public boolean supports(CameraDevice camera) {
        return camera != null && "team:target-camera".equals(camera.kind().key());
    }

    @Override
    public CameraDevice bind(CameraDevice camera) {
        return camera.bindTargets(() -> List.of(
                Measurements.target(42, 0.0, -10.0, 2.5, 0.9)));
    }
}
