package ca.frc6390.athena.hardware.backend;

import ca.frc6390.athena.hardware.device.MotorDevice;

/**
 * Runtime motor created by a backend.
 */
public interface MotorHandle {
    /**
     * Returns the declaration used to create this handle.
     *
     * @return motor declaration
     */
    MotorDevice device();

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

    /** Returns whether this backend can apply portable motor configuration at runtime. */
    default boolean supportsRuntimeConfiguration() {
        return false;
    }

    /**
     * Applies a temporary motor configuration without changing the immutable declaration or
     * persisting it across robot-program restarts.
     */
    default void applyRuntimeConfiguration(MotorRuntimeConfig configuration) {
        throw new UnsupportedOperationException(
                "Runtime configuration is not available for " + device().defaultName());
    }

    /** Returns the latest applied motor voltage. */
    default double appliedVoltage() {
        throw new UnsupportedOperationException("Applied voltage is not available for " + device().defaultName());
    }

    /** Returns the latest controller supply current. */
    default double supplyCurrentAmps() {
        throw new UnsupportedOperationException("Supply current is not available for " + device().defaultName());
    }

    /** Returns the latest motor stator current. */
    default double statorCurrentAmps() { return supplyCurrentAmps(); }

    /**
     * Configures this controller to follow another runtime motor controller.
     *
     * @param leader leader motor handle
     * @param inverted true to oppose the leader direction
     */
    default void follow(MotorHandle leader, boolean inverted) {
        throw new UnsupportedOperationException("Hardware following is not implemented by " + device().defaultName());
    }

    /**
     * Sets open-loop output.
     *
     * @param percent output from -1 to 1
     */
    default void setPercentOutput(double percent) {
        throw new UnsupportedOperationException("Percent output is not implemented by " + device().defaultName());
    }

    /**
     * Sets voltage output.
     *
     * @param volts output volts
     */
    default void setVoltage(double volts) {
        throw new UnsupportedOperationException("Voltage output is not implemented by " + device().defaultName());
    }

    /**
     * Sets a closed-loop position target.
     *
     * @param rotations target position in rotations
     */
    default void setPositionTargetRotations(double rotations) {
        throw new UnsupportedOperationException("Position closed-loop is not implemented by " + device().defaultName());
    }

    /**
     * Sets a closed-loop position target with explicit controller configuration.
     *
     * @param rotations target position in rotations
     * @param request closed-loop request metadata
     */
    default void setPositionTargetRotations(double rotations, MotorClosedLoopRequest request) {
        setPositionTargetRotations(rotations);
    }

    /**
     * Sets a closed-loop velocity target.
     *
     * @param rotationsPerSecond target velocity in rotations per second
     */
    default void setVelocityTargetRotationsPerSecond(double rotationsPerSecond) {
        throw new UnsupportedOperationException("Velocity closed-loop is not implemented by " + device().defaultName());
    }

    /**
     * Sets a closed-loop velocity target with explicit controller configuration.
     *
     * @param rotationsPerSecond target velocity in rotations per second
     * @param request closed-loop request metadata
     */
    default void setVelocityTargetRotationsPerSecond(
            double rotationsPerSecond,
            MotorClosedLoopRequest request) {
        setVelocityTargetRotationsPerSecond(rotationsPerSecond);
    }

    /**
     * Returns onboard control features supported by this handle.
     *
     * @return motor control capabilities
     */
    default MotorControlCapabilities controlCapabilities() {
        return MotorControlCapabilities.OPEN_LOOP_ONLY;
    }

    /**
     * Stops this motor.
     */
    default void stop() {
        setPercentOutput(0.0);
    }

    /**
     * Returns integrated encoder position in rotations.
     *
     * @return position rotations
     */
    default double integratedPositionRotations() {
        throw new UnsupportedOperationException("Integrated encoder position is not implemented by " + device().defaultName());
    }

    /**
     * Returns integrated encoder velocity in rotations per second.
     *
     * @return velocity rotations per second
     */
    default double integratedVelocityRotationsPerSecond() {
        throw new UnsupportedOperationException("Integrated encoder velocity is not implemented by " + device().defaultName());
    }

    /**
     * Sets the integrated encoder's relative position.
     *
     * @param rotations relative position rotations
     */
    default void setIntegratedPositionRotations(double rotations) {
        throw new UnsupportedOperationException(
                "Setting integrated encoder position is not implemented by " + device().defaultName());
    }

    /**
     * Returns whether the integrated encoder position can be set natively.
     *
     * @return true when native position setting is supported
     */
    default boolean supportsIntegratedPositionSetting() {
        return false;
    }

    /** Enables/configures the primary encoder when an integrated encoder is declared. */
    default void enableIntegratedEncoder() {
        // Most controllers expose their integrated encoder without extra configuration.
    }

    /** Enables/configures a controller-attached absolute encoder when one is declared. */
    default void enableAbsoluteEncoder() {
        // Default no-op for controllers whose absolute channel is always available.
    }

    /**
     * Returns controller-attached absolute encoder position in rotations.
     *
     * @return absolute position rotations
     */
    default double absolutePositionRotations() {
        throw new UnsupportedOperationException("Absolute encoder position is not implemented by " + device().defaultName());
    }

    /**
     * Returns controller-attached absolute encoder velocity in rotations per second.
     *
     * @return absolute velocity rotations per second
     */
    default double absoluteVelocityRotationsPerSecond() {
        throw new UnsupportedOperationException("Absolute encoder velocity is not implemented by " + device().defaultName());
    }
}
