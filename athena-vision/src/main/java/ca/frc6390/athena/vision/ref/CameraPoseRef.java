package ca.frc6390.athena.vision.ref;

import ca.frc6390.athena.runtime.measurement.MeasurementRef;

/**
 * Pose measurements produced from a camera.
 */
public interface CameraPoseRef extends MeasurementRef {
    /**
     * Returns the camera producing this pose measurement stream.
     *
     * @return camera
     */
    CameraRef camera();
}
