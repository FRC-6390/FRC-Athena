package ca.frc6390.athena.hardware.backend;

import ca.frc6390.athena.hardware.imu.ImuSpec;

/**
 * Runtime IMU created by a backend.
 */
public interface ImuDevice {
    /**
     * Returns the normalized spec used to create this device.
     *
     * @return IMU spec
     */
    ImuSpec spec();

    /**
     * Returns yaw in degrees.
     *
     * @return yaw degrees
     */
    double yawDegrees();
}
