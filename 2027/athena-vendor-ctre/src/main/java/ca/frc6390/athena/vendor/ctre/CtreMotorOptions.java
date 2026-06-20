package ca.frc6390.athena.vendor.ctre;

/**
 * CTRE-specific motor options stored in Athena motor specs.
 */
public final class CtreMotorOptions {
    private int supplyCurrentLimitAmps;
    private int statorCurrentLimitAmps;
    private int torqueCurrentLimitAmps;

    /**
     * Sets supply current limit.
     *
     * @param amps limit in amps
     * @return this options object
     */
    public CtreMotorOptions supplyCurrentLimit(int amps) {
        supplyCurrentLimitAmps = sanitize(amps);
        return this;
    }

    /**
     * Sets stator current limit.
     *
     * @param amps limit in amps
     * @return this options object
     */
    public CtreMotorOptions statorCurrentLimit(int amps) {
        statorCurrentLimitAmps = sanitize(amps);
        return this;
    }

    /**
     * Sets torque current limit.
     *
     * @param amps limit in amps
     * @return this options object
     */
    public CtreMotorOptions torqueCurrentLimit(int amps) {
        torqueCurrentLimitAmps = sanitize(amps);
        return this;
    }

    /**
     * Returns supply current limit.
     *
     * @return amps, or zero when unset
     */
    public int supplyCurrentLimitAmps() {
        return supplyCurrentLimitAmps;
    }

    /**
     * Returns stator current limit.
     *
     * @return amps, or zero when unset
     */
    public int statorCurrentLimitAmps() {
        return statorCurrentLimitAmps;
    }

    /**
     * Returns torque current limit.
     *
     * @return amps, or zero when unset
     */
    public int torqueCurrentLimitAmps() {
        return torqueCurrentLimitAmps;
    }

    private int sanitize(int amps) {
        return Math.max(0, amps);
    }
}
