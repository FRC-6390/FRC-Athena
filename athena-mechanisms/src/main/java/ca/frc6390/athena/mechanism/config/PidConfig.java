package ca.frc6390.athena.mechanism.config;

import ca.frc6390.athena.mechanism.spec.PidSpec;

/**
 * Student-facing PID gain builder.
 */
public final class PidConfig {
    private double p;
    private double i;
    private double d;

    /**
     * Sets proportional gain.
     *
     * @param p gain
     * @return this config
     */
    public PidConfig p(double p) {
        this.p = p;
        return this;
    }

    /**
     * Sets integral gain.
     *
     * @param i gain
     * @return this config
     */
    public PidConfig i(double i) {
        this.i = i;
        return this;
    }

    /**
     * Sets derivative gain.
     *
     * @param d gain
     * @return this config
     */
    public PidConfig d(double d) {
        this.d = d;
        return this;
    }

    /**
     * Lowers to immutable PID gains.
     *
     * @return PID spec
     */
    public PidSpec toSpec() {
        return new PidSpec(p, i, d);
    }
}
