package ca.frc6390.athena.vendor.ctre;

import static edu.wpi.first.units.Units.Amps;

import ca.frc6390.athena.hardware.backend.FocPolicy;
import edu.wpi.first.units.measure.Current;

/**
 * CTRE-specific motor options stored in Athena motor declarations. Prefer the
 * direct {@code MotorDevice} current-limit methods for ordinary configuration;
 * values set here override their direct equivalents.
 */
public final class CtreMotorOptions {
    private double supplyCurrentLimitAmps;
    private double statorCurrentLimitAmps;
    private double torqueCurrentLimitAmps;
    private FocPolicy focPolicy = FocPolicy.DISABLED;

    /**
     * Sets the supply-side current limit, overriding both the direct supply limit
     * and portable current limit on the motor declaration.
     *
     * @param limit limit; negative values are treated as unset
     * @return this options object
     */
    public CtreMotorOptions supplyCurrentLimit(Current limit) {
        supplyCurrentLimitAmps = sanitize(limit);
        return this;
    }

    /**
     * Sets the stator-side current limit, overriding the direct stator limit on the
     * motor declaration.
     *
     * @param limit limit; negative values are treated as disabled
     * @return this options object
     */
    public CtreMotorOptions statorCurrentLimit(Current limit) {
        statorCurrentLimitAmps = sanitize(limit);
        return this;
    }

    /**
     * Sets symmetric forward and reverse torque-current limits. This setting is
     * supported by TalonFX and rejected by TalonFXS.
     *
     * @param limit limit; negative values are treated as disabled
     * @return this options object
     */
    public CtreMotorOptions torqueCurrentLimit(Current limit) {
        torqueCurrentLimitAmps = sanitize(limit);
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
     * @return limit, or zero when unset
     */
    public Current supplyCurrentLimit() {
        return Amps.of(supplyCurrentLimitAmps);
    }

    /**
     * Returns stator current limit.
     *
     * @return limit, or zero when unset
     */
    public Current statorCurrentLimit() {
        return Amps.of(statorCurrentLimitAmps);
    }

    /**
     * Returns torque current limit.
     *
     * @return limit, or zero when unset
     */
    public Current torqueCurrentLimit() {
        return Amps.of(torqueCurrentLimitAmps);
    }

    /**
     * Returns configured FOC policy.
     *
     * @return FOC policy
     */
    public FocPolicy focPolicy() {
        return focPolicy;
    }

    private static double sanitize(Current limit) {
        double amps = limit == null ? 0.0 : limit.in(Amps);
        return Double.isNaN(amps) ? 0.0 : Math.max(0.0, amps);
    }
}
