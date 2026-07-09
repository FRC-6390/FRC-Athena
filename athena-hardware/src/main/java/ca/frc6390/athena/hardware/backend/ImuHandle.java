package ca.frc6390.athena.hardware.backend;

import ca.frc6390.athena.hardware.device.ImuDevice;

/**
 * Runtime IMU created by a backend.
 */
public interface ImuHandle {
    /**
     * Returns the declaration used to create this handle.
     *
     * @return IMU declaration
     */
    ImuDevice device();

    /**
     * Activates runtime configuration after the graph creates and caches the handle.
     */
    default void activate() {
        // default no-op
    }

    /**
     * Refreshes runtime input snapshots before graph consumers read this handle.
     */
    default void refreshInputs() {
        // default no-op
    }

    /**
     * Returns yaw in degrees.
     *
     * @return yaw degrees
     */
    double yawDegrees();

    /**
     * Returns accumulated angle in degrees.
     *
     * @return accumulated angle degrees
     */
    default double angleDegrees() {
        return yawDegrees();
    }

    /**
     * Zeros the current yaw reading.
     */
    default void zeroYaw() {
        throw new UnsupportedOperationException("IMU yaw zeroing is not supported by this handle.");
    }

    /**
     * Resets accumulated IMU state.
     */
    default void reset() {
        throw new UnsupportedOperationException("IMU reset is not supported by this handle.");
    }
}
