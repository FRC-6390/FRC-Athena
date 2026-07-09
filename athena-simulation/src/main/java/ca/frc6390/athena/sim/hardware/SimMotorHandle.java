package ca.frc6390.athena.sim.hardware;

import java.util.Objects;

import ca.frc6390.athena.hardware.backend.MotorHandle;
import ca.frc6390.athena.hardware.device.MotorDevice;

/**
 * In-memory simulation motor handle.
 */
public final class SimMotorHandle implements MotorHandle {
    private final MotorDevice device;
    private double percentOutput;
    private double positionRotations;
    private double velocityRotationsPerSecond;

    /**
     * Creates a simulation motor handle.
     *
     * @param device motor declaration
     */
    public SimMotorHandle(MotorDevice device) {
        this.device = Objects.requireNonNull(device, "device");
    }

    @Override
    public MotorDevice device() {
        return device;
    }

    @Override
    public void setPercentOutput(double percent) {
        percentOutput = clamp(percent, -1.0, 1.0);
        velocityRotationsPerSecond = percentOutput;
    }

    @Override
    public void setVoltage(double volts) {
        setPercentOutput(volts / 12.0);
    }

    @Override
    public void setPositionTargetRotations(double rotations) {
        positionRotations = finiteOrZero(rotations);
        velocityRotationsPerSecond = 0.0;
    }

    @Override
    public void setVelocityTargetRotationsPerSecond(double rotationsPerSecond) {
        velocityRotationsPerSecond = finiteOrZero(rotationsPerSecond);
    }

    @Override
    public double integratedPositionRotations() {
        return positionRotations;
    }

    @Override
    public double integratedVelocityRotationsPerSecond() {
        return velocityRotationsPerSecond;
    }

    /**
     * Advances integrated position from velocity.
     *
     * @param seconds timestep in seconds
     */
    public void step(double seconds) {
        if (Double.isFinite(seconds) && seconds > 0.0) {
            positionRotations += velocityRotationsPerSecond * seconds;
        }
    }

    private static double clamp(double value, double min, double max) {
        if (!Double.isFinite(value)) {
            return 0.0;
        }
        return Math.max(min, Math.min(max, value));
    }

    private static double finiteOrZero(double value) {
        return Double.isFinite(value) ? value : 0.0;
    }
}
