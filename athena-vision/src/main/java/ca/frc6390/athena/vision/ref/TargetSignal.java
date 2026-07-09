package ca.frc6390.athena.vision.ref;

import java.util.Map;

import ca.frc6390.athena.runtime.measurement.MeasurementSignal;

/**
 * Target measurements produced from a camera.
 */
public interface TargetSignal extends MeasurementSignal {
    /**
     * Returns the camera producing this target stream.
     *
     * @return camera
     */
    CameraDevice camera();

    /**
     * Returns target-signal metadata for graph/runtime policy decisions.
     *
     * @return metadata
     */
    default Map<String, Object> metadata() {
        return Map.of();
    }
}
