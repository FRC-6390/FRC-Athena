package ca.frc6390.athena.vendor.ctre;

import ca.frc6390.athena.hardware.backend.FocPolicy;

/**
 * CTRE-specific motor options stored in Athena motor declarations. Prefer the
 * direct {@code MotorDevice} current-limit methods for ordinary configuration;
 * values set here override their direct equivalents.
 */
public final class CtreMotorOptions {
    private int supplyCurrentLimitAmps;
    private int statorCurrentLimitAmps;
    private int torqueCurrentLimitAmps;
    private FocPolicy focPolicy = FocPolicy.DISABLED;

    /**
     * Sets the supply-side current limit, overriding both the direct supply limit
     * and portable current limit on the motor declaration.
     *
     * @param amps limit in amps; negative values are treated as unset
     * @return this options object
     */
    public CtreMotorOptions supplyCurrentLimit(int amps) {
        supplyCurrentLimitAmps = sanitize(amps);
        return this;
    }

    /**
     * Sets the stator-side current limit, overriding the direct stator limit on the
     * motor declaration.
     *
     * @param amps limit in amps; negative values are treated as disabled
     * @return this options object
     */
    public CtreMotorOptions statorCurrentLimit(int amps) {
        statorCurrentLimitAmps = sanitize(amps);
        return this;
    }

    /**
     * Sets symmetric forward and reverse torque-current limits. This setting is
     * supported by TalonFX and rejected by TalonFXS.
     *
     * @param amps limit in amps; negative values are treated as disabled
     * @return this options object
     */
    public CtreMotorOptions torqueCurrentLimit(int amps) {
        torqueCurrentLimitAmps = sanitize(amps);
        return this;
    }

    /**
     * Sets desired FOC behavior for voltage closed-loop requests.
     *
     * @param policy FOC policy
     * @return this options object
     */
    public CtreMotorOptions foc(FocPolicy policy) {
        focPolicy = policy == null ? FocPolicy.DISABLED : policy;
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

    /**
     * Returns configured FOC policy.
     *
     * @return FOC policy
     */
    public FocPolicy focPolicy() {
        return focPolicy;
    }

    private int sanitize(int amps) {
        return Math.max(0, amps);
    }
}
