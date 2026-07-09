package ca.frc6390.athena.vision.signal;

import java.util.Map;

import ca.frc6390.athena.runtime.measurement.MeasurementSignal;
import ca.frc6390.athena.vision.device.CameraDevice;

/**
 * Pose measurements produced from a camera.
 */
public interface PoseSignal extends MeasurementSignal {
    /**
     * Returns the camera producing this pose stream.
     *
     * @return camera
     */
    CameraDevice camera();

    /**
     * Returns pose-signal metadata for graph/runtime policy decisions.
     *
     * @return metadata
     */
    default Map<String, Object> metadata() {
        return Map.of();
    }
}
