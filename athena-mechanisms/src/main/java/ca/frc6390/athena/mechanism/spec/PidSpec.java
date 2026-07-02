package ca.frc6390.athena.mechanism.spec;

/**
 * PID gain declaration.
 *
 * @param p proportional gain
 * @param i integral gain
 * @param d derivative gain
 * @param iZone integral zone, or 0 when unused
 * @param tolerance closed-loop tolerance, or 0 when unused
 */
public record PidSpec(double p, double i, double d, double iZone, double tolerance) {
    public PidSpec(double p, double i, double d) {
        this(p, i, d, 0.0, 0.0);
    }

    /**
     * Returns whether all gains are finite.
     *
     * @return true if valid
     */
    public boolean isFinite() {
        return Double.isFinite(p)
                && Double.isFinite(i)
                && Double.isFinite(d)
                && Double.isFinite(iZone)
                && Double.isFinite(tolerance);
    }
}
