package ca.frc6390.athena.hardware.backend;

import ca.frc6390.athena.hardware.device.ImuDevice;

/**
 * Runtime IMU created by a backend.
 *
 * <p>Backend SPI: vendors implement these primitive methods. Robot code never calls them; it reads the
 * typed device accessors, or {@code raw()} for Athena's own per-cycle code. Not for robot code.</p>
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
     * Refreshes runtime input snapshots for one refresh pass. Handles that share a vendor batched read use the
     * pass token to read the shared bus once per pass; a token must not repeat across passes. The default ignores
     * the token.
     *
     * @param refreshPass token unique to this refresh pass
     */
    default void refreshInputs(long refreshPass) {
        refreshInputs();
    }

    /**
     * Returns yaw in degrees.
     *
     * @return yaw degrees
     */
    double yawDegrees();

    default double pitchDegrees() {
        throw new UnsupportedOperationException("IMU pitch is not supported by this handle.");
    }

    default double rollDegrees() {
        throw new UnsupportedOperationException("IMU roll is not supported by this handle.");
    }

    /**
     * Returns accumulated angle in degrees.
     *
     * @return accumulated angle degrees
     */
    default double angleDegrees() {
        return yawDegrees();
    }

    default double yawRateDegreesPerSecond() {
        throw new UnsupportedOperationException("IMU yaw rate is not supported by this handle.");
    }

    default double pitchRateDegreesPerSecond() {
        throw new UnsupportedOperationException("IMU pitch rate is not supported by this handle.");
    }

    default double rollRateDegreesPerSecond() {
        throw new UnsupportedOperationException("IMU roll rate is not supported by this handle.");
    }

    default double linearAccelerationXG() {
        throw new UnsupportedOperationException("IMU X acceleration is not supported by this handle.");
    }

    default double linearAccelerationYG() {
        throw new UnsupportedOperationException("IMU Y acceleration is not supported by this handle.");
    }

    default double linearAccelerationZG() {
        throw new UnsupportedOperationException("IMU Z acceleration is not supported by this handle.");
    }

    /** Returns whether the vendor currently reports this device connected. */
    default boolean isConnected() {
        return true;
    }

    /** Returns whether this device is calibrating and its orientation should not be trusted. */
    default boolean isCalibrating() {
        return false;
    }

    /** Returns the monotonic timestamp of the latest successful input refresh, in seconds. */
    default double lastUpdateSeconds() {
        return Double.NaN;
    }

    default void setYawDegrees(double yawDegrees) {
        if (Double.compare(yawDegrees, 0.0) != 0) {
            throw new UnsupportedOperationException("Setting arbitrary IMU yaw is not supported by this handle.");
        }
        zeroYaw();
    }

    /**
     * Zeros the current yaw reading.
     */
    default void zeroYaw() {
        throw new UnsupportedOperationException("IMU yaw zeroing is not supported by this handle.");
    }

    /**
     * Resets accumulated IMU Action.
     */
    default void reset() {
        throw new UnsupportedOperationException("IMU reset is not supported by this handle.");
    }
}
