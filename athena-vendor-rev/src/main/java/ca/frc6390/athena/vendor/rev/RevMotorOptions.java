package ca.frc6390.athena.vendor.rev;

/**
 * REV-specific motor options stored in Athena motor specs.
 */
public final class RevMotorOptions {
    private int smartCurrentLimitAmps;
    private double openLoopRampSeconds;
    private double closedLoopRampSeconds;

    /**
     * Sets smart current limit.
     *
     * @param amps limit in amps
     * @return this options object
     */
    public RevMotorOptions smartCurrentLimit(int amps) {
        smartCurrentLimitAmps = Math.max(0, amps);
        return this;
    }

    /**
     * Sets open-loop ramp time.
     *
     * @param seconds ramp seconds
     * @return this options object
     */
    public RevMotorOptions openLoopRampSeconds(double seconds) {
        openLoopRampSeconds = sanitize(seconds);
        return this;
    }

    /**
     * Sets closed-loop ramp time.
     *
     * @param seconds ramp seconds
     * @return this options object
     */
    public RevMotorOptions closedLoopRampSeconds(double seconds) {
        closedLoopRampSeconds = sanitize(seconds);
        return this;
    }

    /**
     * Returns smart current limit.
     *
     * @return amps, or zero when unset
     */
    public int smartCurrentLimitAmps() {
        return smartCurrentLimitAmps;
    }

    /**
     * Returns open-loop ramp seconds.
     *
     * @return seconds
     */
    public double openLoopRampSeconds() {
        return openLoopRampSeconds;
    }

    /**
     * Returns closed-loop ramp seconds.
     *
     * @return seconds
     */
    public double closedLoopRampSeconds() {
        return closedLoopRampSeconds;
    }

    private double sanitize(double seconds) {
        return Double.isFinite(seconds) ? Math.max(0.0, seconds) : 0.0;
    }
}
