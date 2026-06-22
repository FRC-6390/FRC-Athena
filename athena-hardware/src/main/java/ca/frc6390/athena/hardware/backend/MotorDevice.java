package ca.frc6390.athena.hardware.backend;

import ca.frc6390.athena.hardware.spec.MotorSpec;

/**
 * Runtime motor created by a backend.
 */
public interface MotorDevice {
    /**
     * Returns the normalized spec used to create this device.
     *
     * @return motor spec
     */
    MotorSpec spec();

    /**
     * Sets open-loop output.
     *
     * @param percent output from -1 to 1
     */
    default void setPercentOutput(double percent) {
        throw new UnsupportedOperationException("Percent output is not implemented by " + spec().path());
    }

    /**
     * Sets voltage output.
     *
     * @param volts output volts
     */
    default void setVoltage(double volts) {
        throw new UnsupportedOperationException("Voltage output is not implemented by " + spec().path());
    }

    /**
     * Sets a closed-loop position target.
     *
     * @param rotations target position in rotations
     */
    default void setPositionTargetRotations(double rotations) {
        throw new UnsupportedOperationException("Position closed-loop is not implemented by " + spec().path());
    }

    /**
     * Sets a closed-loop velocity target.
     *
     * @param rotationsPerSecond target velocity in rotations per second
     */
    default void setVelocityTargetRotationsPerSecond(double rotationsPerSecond) {
        throw new UnsupportedOperationException("Velocity closed-loop is not implemented by " + spec().path());
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
        throw new UnsupportedOperationException("Integrated encoder position is not implemented by " + spec().path());
    }

    /**
     * Returns integrated encoder velocity in rotations per second.
     *
     * @return velocity rotations per second
     */
    default double integratedVelocityRotationsPerSecond() {
        throw new UnsupportedOperationException("Integrated encoder velocity is not implemented by " + spec().path());
    }

    /**
     * Returns controller-attached absolute encoder position in rotations.
     *
     * @return absolute position rotations
     */
    default double absolutePositionRotations() {
        throw new UnsupportedOperationException("Absolute encoder position is not implemented by " + spec().path());
    }

    /**
     * Returns controller-attached absolute encoder velocity in rotations per second.
     *
     * @return absolute velocity rotations per second
     */
    default double absoluteVelocityRotationsPerSecond() {
        throw new UnsupportedOperationException("Absolute encoder velocity is not implemented by " + spec().path());
    }
}
