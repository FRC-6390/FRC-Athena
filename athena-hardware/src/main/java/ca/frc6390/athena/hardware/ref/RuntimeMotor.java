package ca.frc6390.athena.hardware.ref;

import ca.frc6390.athena.hardware.backend.MotorDevice;
import java.util.Objects;

/**
 * Runtime handle for a motor.
 */
public interface RuntimeMotor {
    /**
     * Stops the motor.
     */
    default void stop() {
        percent(0.0);
    }

    /**
     * Sets percent output.
     *
     * @param output output from -1 to 1
     */
    void percent(double output);

    /**
     * Sets voltage output.
     *
     * @param volts output volts
     */
    void voltage(double volts);

    /**
     * Sets closed-loop position target.
     *
     * @param position target position in mechanism units
     */
    default void position(double position) {
        throw new UnsupportedOperationException("Runtime motor position control is not available.");
    }

    /**
     * Sets closed-loop velocity target.
     *
     * @param velocity target velocity in mechanism units per second
     */
    default void velocity(double velocity) {
        throw new UnsupportedOperationException("Runtime motor velocity control is not available.");
    }

    /**
     * Switches to brake neutral mode.
     */
    void brake();

    /**
     * Switches to coast neutral mode.
     */
    void coast();

    /**
     * Wraps a backend motor device as a ref runtime handle.
     *
     * @param device backend motor device
     * @return runtime motor handle
     */
    static RuntimeMotor from(MotorDevice device) {
        Objects.requireNonNull(device, "device");
        return new RuntimeMotor() {
            @Override
            public void stop() {
                device.stop();
            }

            @Override
            public void percent(double output) {
                device.setPercentOutput(output);
            }

            @Override
            public void voltage(double volts) {
                device.setVoltage(volts);
            }

            @Override
            public void position(double position) {
                device.setPositionTargetRotations(position);
            }

            @Override
            public void velocity(double velocity) {
                device.setVelocityTargetRotationsPerSecond(velocity);
            }

            @Override
            public void brake() {
                throw new UnsupportedOperationException("Runtime motor brake mode is not available through MotorDevice.");
            }

            @Override
            public void coast() {
                throw new UnsupportedOperationException("Runtime motor coast mode is not available through MotorDevice.");
            }
        };
    }
}
