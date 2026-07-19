package ca.frc6390.athena.sim.hardware;

import java.util.Objects;

import ca.frc6390.athena.hardware.backend.ImuHandle;
import ca.frc6390.athena.hardware.device.ImuDevice;

/**
 * In-memory simulation IMU handle.
 */
public final class SimImuHandle implements ImuHandle {
    private final ImuDevice device;
    private double yawDegrees;
    private double pitchDegrees;
    private double rollDegrees;
    private double yawRateDegreesPerSecond;
    private double pitchRateDegreesPerSecond;
    private double rollRateDegreesPerSecond;
    private double linearAccelerationXG;
    private double linearAccelerationYG;
    private double linearAccelerationZG;
    private double lastUpdateSeconds = System.nanoTime() * 1.0e-9;

    /**
     * Creates a simulation IMU handle.
     *
     * @param device IMU declaration
     */
    public SimImuHandle(ImuDevice device) {
        this.device = Objects.requireNonNull(device, "device");
    }

    @Override
    public ImuDevice device() {
        return device;
    }

    @Override
    public double yawDegrees() {
        return yawDegrees;
    }

    @Override
    public double angleDegrees() {
        return yawDegrees;
    }

    @Override
    public double pitchDegrees() {
        return pitchDegrees;
    }

    @Override
    public double rollDegrees() {
        return rollDegrees;
    }

    @Override
    public double yawRateDegreesPerSecond() {
        return yawRateDegreesPerSecond;
    }

    @Override public double pitchRateDegreesPerSecond() { return pitchRateDegreesPerSecond; }
    @Override public double rollRateDegreesPerSecond() { return rollRateDegreesPerSecond; }
    @Override public double lastUpdateSeconds() { return lastUpdateSeconds; }

    @Override
    public void refreshInputs() {
        lastUpdateSeconds = System.nanoTime() * 1.0e-9;
    }

    @Override
    public double linearAccelerationXG() {
        return linearAccelerationXG;
    }

    @Override
    public double linearAccelerationYG() {
        return linearAccelerationYG;
    }

    @Override
    public double linearAccelerationZG() {
        return linearAccelerationZG;
    }

    @Override
    public void setYawDegrees(double yawDegrees) {
        this.yawDegrees = finiteOrZero(yawDegrees);
        touch();
    }

    @Override
    public void zeroYaw() {
        yawDegrees = 0.0;
        touch();
    }

    @Override
    public void reset() {
        yawDegrees = 0.0;
        pitchDegrees = 0.0;
        rollDegrees = 0.0;
        yawRateDegreesPerSecond = 0.0;
        pitchRateDegreesPerSecond = 0.0;
        rollRateDegreesPerSecond = 0.0;
        linearAccelerationXG = 0.0;
        linearAccelerationYG = 0.0;
        linearAccelerationZG = 0.0;
        touch();
    }

    /**
     * Sets yaw in degrees.
     *
     * @param yawDegrees yaw
     * @return this handle
     */
    public SimImuHandle yawDegrees(double yawDegrees) {
        this.yawDegrees = finiteOrZero(yawDegrees);
        touch();
        return this;
    }

    public SimImuHandle pitchDegrees(double pitchDegrees) {
        this.pitchDegrees = finiteOrZero(pitchDegrees);
        touch();
        return this;
    }

    public SimImuHandle rollDegrees(double rollDegrees) {
        this.rollDegrees = finiteOrZero(rollDegrees);
        touch();
        return this;
    }

    /**
     * Sets yaw rate in degrees per second.
     *
     * @param yawRateDegreesPerSecond yaw rate
     * @return this handle
     */
    public SimImuHandle yawRateDegreesPerSecond(double yawRateDegreesPerSecond) {
        this.yawRateDegreesPerSecond = finiteOrZero(yawRateDegreesPerSecond);
        touch();
        return this;
    }

    public SimImuHandle angularRateDegreesPerSecond(double roll, double pitch, double yaw) {
        rollRateDegreesPerSecond = finiteOrZero(roll);
        pitchRateDegreesPerSecond = finiteOrZero(pitch);
        yawRateDegreesPerSecond = finiteOrZero(yaw);
        touch();
        return this;
    }

    public SimImuHandle linearAccelerationG(double x, double y, double z) {
        linearAccelerationXG = finiteOrZero(x);
        linearAccelerationYG = finiteOrZero(y);
        linearAccelerationZG = finiteOrZero(z);
        touch();
        return this;
    }

    /**
     * Advances yaw from yaw rate.
     *
     * @param seconds timestep in seconds
     */
    public void step(double seconds) {
        if (Double.isFinite(seconds) && seconds > 0.0) {
            yawDegrees += yawRateDegreesPerSecond * seconds;
            touch();
        }
    }

    private void touch() {
        lastUpdateSeconds = System.nanoTime() * 1.0e-9;
    }

    private static double finiteOrZero(double value) {
        return Double.isFinite(value) ? value : 0.0;
    }
}
