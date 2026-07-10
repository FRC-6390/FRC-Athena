package ca.frc6390.athena.sim.hardware;

import java.util.Objects;

import ca.frc6390.athena.hardware.backend.EncoderHandle;
import ca.frc6390.athena.hardware.device.EncoderDevice;

/**
 * In-memory simulation encoder handle.
 */
public final class SimEncoderHandle implements EncoderHandle {
    private final EncoderDevice device;
    private double positionRotations;
    private double absolutePositionRotations;
    private double velocityRotationsPerSecond;

    /**
     * Creates a simulation encoder handle.
     *
     * @param device encoder declaration
     */
    public SimEncoderHandle(EncoderDevice device) {
        this.device = Objects.requireNonNull(device, "device");
    }

    @Override
    public EncoderDevice device() {
        return device;
    }

    @Override
    public double positionRotations() {
        return positionRotations;
    }

    @Override
    public double absolutePositionRotations() {
        return absolutePositionRotations;
    }

    @Override
    public double velocityRotationsPerSecond() {
        return velocityRotationsPerSecond;
    }

    @Override
    public void setPositionRotations(double rotations) {
        positionRotations = finiteOrZero(rotations);
    }

    @Override
    public boolean supportsPositionSetting() {
        return true;
    }

    /**
     * Sets relative position.
     *
     * @param rotations rotations
     * @return this handle
     */
    public SimEncoderHandle positionRotations(double rotations) {
        positionRotations = finiteOrZero(rotations);
        return this;
    }

    /**
     * Sets absolute position.
     *
     * @param rotations rotations
     * @return this handle
     */
    public SimEncoderHandle absolutePositionRotations(double rotations) {
        absolutePositionRotations = finiteOrZero(rotations);
        return this;
    }

    /**
     * Sets velocity.
     *
     * @param rotationsPerSecond rotations per second
     * @return this handle
     */
    public SimEncoderHandle velocityRotationsPerSecond(double rotationsPerSecond) {
        velocityRotationsPerSecond = finiteOrZero(rotationsPerSecond);
        return this;
    }

    private static double finiteOrZero(double value) {
        return Double.isFinite(value) ? value : 0.0;
    }
}
