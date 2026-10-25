package ca.frc6390.athena.mechanism.core;

/**
 * Per-Action motion policy override. A null component inherits the control binding default.
 *
 * @param retarget retarget policy for this Action's target, or null
 * @param release release policy used if this Action is the last one driving its binding, or null
 * @param arriveSeconds time a velocity control takes to reach a new target, NaN to use the binding's
 *     acceleration limit; see {@link Action#arriveWithin(edu.wpi.first.units.measure.Time)}
 */
public record MotionPolicy(Retarget retarget, Release release, double arriveSeconds) {
    /** Policy that inherits every binding default. */
    public static final MotionPolicy INHERIT = new MotionPolicy(null, null);

    public MotionPolicy {
        if (!Double.isNaN(arriveSeconds) && !(arriveSeconds > 0.0 && Double.isFinite(arriveSeconds))) {
            throw new IllegalArgumentException("Arrival time must be positive and finite.");
        }
    }

    /** Policy with retarget and release overrides only. */
    public MotionPolicy(Retarget retarget, Release release) {
        this(retarget, release, Double.NaN);
    }
}
