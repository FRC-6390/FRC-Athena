package ca.frc6390.athena.mechanism.effort;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Current;
import java.util.Objects;

/**
 * Per-motor current limit requested by one control Action (B5).
 *
 * <p>The value is converted from the Action's declared effort when the Action is declared, so the
 * control path only compares doubles. {@link #NONE} means the Action sets no limit of its own; the
 * binding ceiling (if any) still applies.</p>
 *
 * @param currentLimit per-motor current limit, non-negative; infinite for no limit
 */
public record EffortRequest(Current currentLimit) {
    /** No effort limit on the Action. */
    public static final EffortRequest NONE = new EffortRequest(Units.Amps.of(Double.POSITIVE_INFINITY));

    public EffortRequest {
        Objects.requireNonNull(currentLimit, "currentLimit");
        double amps = currentLimit.in(Units.Amps);
        if (Double.isNaN(amps) || amps < 0.0) {
            throw new IllegalArgumentException("Effort current limit must be non-negative.");
        }
    }

    /** Returns whether this request limits current. */
    public boolean limited() {
        return Double.isFinite(currentLimit.in(Units.Amps));
    }

    /**
     * Returns the limit as a plain double in amps, for the per-cycle control path. Infinite for no limit.
     *
     * @return limit in amps
     */
    public double limitValue() {
        return currentLimit.in(Units.Amps);
    }
}
