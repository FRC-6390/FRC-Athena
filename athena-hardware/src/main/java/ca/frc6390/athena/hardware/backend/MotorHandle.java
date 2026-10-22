package ca.frc6390.athena.hardware.backend;

import ca.frc6390.athena.hardware.device.MotorDevice;

import java.util.OptionalDouble;

/**
 * Runtime motor created by a backend.
 *
 * <p>Backend SPI: vendors implement these primitive methods. Robot code never calls them; it reads the
 * typed device accessors, or {@code raw()} for Athena's own per-cycle code. Not for robot code.</p>
 */
public interface MotorHandle {
    /**
     * Returns the declaration used to create this handle.
     *
     * @return motor declaration
     */
    MotorDevice device();

    /** Cached readiness of required configuration; synchronous backends are ready immediately. */
    default ConfigurationStatus configurationStatus() { return ConfigurationStatus.READY; }

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
     * Returns whether the active controller enforces supply-current limits in battery amps natively.
     * Controllers with only a winding-current limit return false so Athena applies a software limit.
     * Wrappers must follow their active output route, including after motor-group promotion. This is a
     * cached capability query and must not contact the controller. Custom backends default to software
     * enforcement until they explicitly provide this guarantee.
     */
    default boolean enforcesSupplyCurrentLimit() {
        return false;
    }

    /**
     * Applies a temporary motor configuration without changing the immutable declaration or
     * persisting it across robot-program restarts. Implementations must enqueue potentially blocking
     * vendor work and expose its acknowledgement through {@link #configurationStatus()}; this method
     * runs on robot/telemetry threads. Drive output must remain gated while required work is pending.
     */
    default void applyRuntimeConfiguration(MotorRuntimeConfig configuration) {
        throw new UnsupportedOperationException(
                "Runtime configuration is not available for " + device().defaultName());
    }

    /**
     * Returns the controller-reported winding or motor temperature in Celsius. The default reports
     * nothing: Athena then uses its lumped thermal model alone.
     *
     * @return temperature, or empty when the controller does not report one
     */
    default OptionalDouble temperatureCelsius() {
        return OptionalDouble.empty();
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
     * Multiplier from a requested voltage to the voltage in this handle's integrated-velocity measurement frame.
     * Ordinary controller handles use their controller's phased feedback frame and return one. Wrappers that
     * reverse or derate commands must include that transformation here, without changing telemetry signs.
     *
     * @return signed voltage multiplier; zero when the wrapper suppresses voltage output
     */
    default double voltageCommandScale() { return 1.0; }

    /**
     * Voltage multiplier from a native followed leader's measurement frame to this follower's measurement
     * frame. Controller-phased feedback stays aligned with its applied output, including opposition, so the
     * default is one. Backends exposing raw shaft feedback must report the actual native follow direction.
     * Read only cached state; this method runs during host power clamping and must not query the bus.
     */
    default double hardwareFollowerVoltageScale() { return 1.0; }

    /**
     * Physical command destination of this handle, read from cached routing state. Command wrappers delegate
     * this so mirroring can deduplicate writes and retain only actual motors for later neutralization.
     */
    default MotorHandle commandTarget() { return this; }

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
     * Leaves hardware follower mode so the controller accepts direct commands again. Motor failover calls this
     * once when it promotes a follower or re-points it to a new leader. Afterwards the controller must run
     * uninverted: Athena applies the follower's direction in software. Controllers whose next direct request
     * replaces the follow request (CTRE) need not override it.
     */
    default void stopFollowing() {
        // default no-op
    }

    /**
     * Returns whether the controller reports a fault that stops it from driving the motor. Read from the cached
     * inputs of the latest {@link #refreshInputs()}; it must not touch the bus. A lost or stale CAN connection is
     * reported by {@link #refreshInputs()} throwing instead.
     *
     * @return true while the controller is faulted
     */
    default boolean faulted() {
        return false;
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
