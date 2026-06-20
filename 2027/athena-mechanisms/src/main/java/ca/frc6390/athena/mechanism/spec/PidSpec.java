package ca.frc6390.athena.mechanism.spec;

/**
 * PID gain declaration.
 *
 * @param p proportional gain
 * @param i integral gain
 * @param d derivative gain
 */
public record PidSpec(double p, double i, double d) {
    /**
     * Returns whether all gains are finite.
     *
     * @return true if valid
     */
    public boolean isFinite() {
        return Double.isFinite(p) && Double.isFinite(i) && Double.isFinite(d);
    }
}
