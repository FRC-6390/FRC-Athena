package ca.frc6390.athena.sim.world;

/**
 * Mutable IMU simulation state.
 */
public final class SimImuState {
    private final String name;
    private double yawDegrees;
    private double yawRateDegreesPerSecond;

    /**
     * Creates an IMU simulation state.
     *
     * @param name IMU name
     */
    public SimImuState(String name) {
        this.name = name == null || name.isBlank() ? "imu" : name;
    }

    /**
     * Returns IMU name.
     *
     * @return IMU name
     */
    public String name() {
        return name;
    }

    /**
     * Returns yaw in degrees.
     *
     * @return yaw degrees
     */
    public double yawDegrees() {
        return yawDegrees;
    }

    /**
     * Sets yaw in degrees.
     *
     * @param yawDegrees yaw
     * @return this state
     */
    public SimImuState yawDegrees(double yawDegrees) {
        this.yawDegrees = Double.isFinite(yawDegrees) ? yawDegrees : 0.0;
        return this;
    }

    /**
     * Returns yaw rate in degrees per second.
     *
     * @return yaw rate
     */
    public double yawRateDegreesPerSecond() {
        return yawRateDegreesPerSecond;
    }

    /**
     * Sets yaw rate in degrees per second.
     *
     * @param yawRateDegreesPerSecond yaw rate
     * @return this state
     */
    public SimImuState yawRateDegreesPerSecond(double yawRateDegreesPerSecond) {
        this.yawRateDegreesPerSecond = Double.isFinite(yawRateDegreesPerSecond) ? yawRateDegreesPerSecond : 0.0;
        return this;
    }

    /**
     * Advances yaw from yaw rate.
     *
     * @param seconds timestep in seconds
     * @return this state
     */
    public SimImuState step(double seconds) {
        if (Double.isFinite(seconds) && seconds > 0.0) {
            yawDegrees += yawRateDegreesPerSecond * seconds;
        }
        return this;
    }
}
