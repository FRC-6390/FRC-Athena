package ca.frc6390.athena.vision.runtime;

import ca.frc6390.athena.runtime.filter.PoseSnapshot;
import ca.frc6390.athena.vision.device.CameraDevice;

/**
 * Optional vision simulation bridge backed by a vendor implementation.
 */
public interface VisionSimulation {
    /**
     * Returns a camera declaration bound to this simulation's measurements.
     *
     * @param camera camera declaration
     * @return bound camera declaration
     */
    default CameraDevice bind(CameraDevice camera) {
        return camera;
    }

    /**
     * Updates simulated camera observations from the robot pose.
     *
     * @param robotPose field-relative robot pose
     */
    void update(PoseSnapshot robotPose);
}
