package ca.frc6390.athena.vision.ref;

import ca.frc6390.athena.runtime.measurement.MeasurementRef;

/**
 * Target measurements produced from a camera.
 */
public interface CameraTargetRef extends MeasurementRef {
    /**
     * Returns the camera producing this target measurement stream.
     *
     * @return camera
     */
    CameraRef camera();
}
