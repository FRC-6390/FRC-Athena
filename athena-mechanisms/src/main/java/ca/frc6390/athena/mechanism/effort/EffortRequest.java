package ca.frc6390.athena.mechanism.effort;

/**
 * Per-motor current limit requested by one control Action (B5).
 *
 * <p>The value is converted from the Action's declared effort when the Action is declared, so the
 * control path only compares doubles. {@link #NONE} means the Action sets no limit of its own; the
 * binding ceiling (if any) still applies.</p>
 *
 * @param currentLimitAmps per-motor current limit in amps, non-negative; infinite for no limit
 */
public record EffortRequest(double currentLimitAmps) {
    /** No effort limit on the Action. */
    public static final EffortRequest NONE = new EffortRequest(Double.POSITIVE_INFINITY);

    public EffortRequest {
        if (Double.isNaN(currentLimitAmps) || currentLimitAmps < 0.0) {
            throw new IllegalArgumentException("Effort current limit must be non-negative.");
        }
    }

    /** Returns whether this request limits current. */
    public boolean limited() {
        return Double.isFinite(currentLimitAmps);
    }
}
