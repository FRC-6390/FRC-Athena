package ca.frc6390.athena.mechanism.config;

import ca.frc6390.athena.mechanism.spec.PidSpec;
import ca.frc6390.athena.mechanism.ref.PidRef;

/**
 * Student-facing PID gain builder.
 */
public final class PidConfig {
    private double p;
    private double i;
    private double d;
    private double iZone;
    private double tolerance;

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
     * Sets integral zone.
     *
     * @param iZone integral zone
     * @return this config
     */
    public PidConfig iZone(double iZone) {
        this.iZone = iZone;
        return this;
    }

    /**
     * Sets closed-loop tolerance.
     *
     * @param tolerance tolerance in mechanism units
     * @return this config
     */
    public PidConfig tolerance(double tolerance) {
        this.tolerance = tolerance;
        return this;
    }

    /**
     * Applies a reusable PID reference.
     *
     * @param pid PID reference
     * @return this config
     */
    public PidConfig apply(PidRef pid) {
        p = pid.p();
        i = pid.i();
        d = pid.d();
        iZone = pid.iZone();
        tolerance = pid.tolerance();
        return this;
    }

    /**
     * Lowers to immutable PID gains.
     *
     * @return PID spec
     */
    public PidSpec toSpec() {
        return new PidSpec(p, i, d, iZone, tolerance);
    }
}
