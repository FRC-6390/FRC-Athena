package ca.frc6390.athena.vision.runtime;

import ca.frc6390.athena.vision.device.CameraDevice;

/**
 * Service-loaded adapter that can bind runtime signals to a camera declaration.
 */
public interface CameraAdapter {
    /**
     * Returns true when this adapter can bind the camera.
     *
     * @param camera camera declaration
     * @return true if supported
     */
    boolean supports(CameraDevice camera);

    /**
     * Binds adapter-backed signals to the camera declaration.
     *
     * @param camera camera declaration
     * @return bound camera declaration
     */
    CameraDevice bind(CameraDevice camera);
}
