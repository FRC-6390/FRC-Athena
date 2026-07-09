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
    private double yawRateDegreesPerSecond;

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
    public void zeroYaw() {
        yawDegrees = 0.0;
    }

    @Override
    public void reset() {
        yawDegrees = 0.0;
        yawRateDegreesPerSecond = 0.0;
    }

    /**
     * Sets yaw in degrees.
     *
     * @param yawDegrees yaw
     * @return this handle
     */
    public SimImuHandle yawDegrees(double yawDegrees) {
        this.yawDegrees = Double.isFinite(yawDegrees) ? yawDegrees : 0.0;
        return this;
    }

    /**
     * Sets yaw rate in degrees per second.
     *
     * @param yawRateDegreesPerSecond yaw rate
     * @return this handle
     */
    public SimImuHandle yawRateDegreesPerSecond(double yawRateDegreesPerSecond) {
        this.yawRateDegreesPerSecond = Double.isFinite(yawRateDegreesPerSecond) ? yawRateDegreesPerSecond : 0.0;
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
        }
    }
}
