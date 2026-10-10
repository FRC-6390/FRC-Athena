package ca.frc6390.athena.mechanism.effort;

import edu.wpi.first.units.Measure;
import java.util.Objects;

/**
 * What a control does while its external load (B3 {@code load()}) is at or above the binding's effort
 * ceiling. Declared with {@link Effort#onLoad(LoadResponse...)}.
 *
 * <ul>
 *   <li>{@link #YIELD}: drops position authority. Current is driven to zero, so the mechanism follows
 *       the push (compliant). Once the load has been clear for {@link Yield#recoverAfter} the ceiling
 *       is restored over the same time.</li>
 *   <li>{@link #HOLD}: keeps the position command and the ceiling. This is the behaviour without any
 *       response.</li>
 *   <li>{@link #pushThrough(Measure, Measure)}: while loaded, allows {@code burst} for {@code duration}
 *       from the moment the load appears, then falls back to the ceiling.</li>
 *   <li>{@link #slow(double)}: while loaded, scales the motion profile speed and velocity targets.</li>
 * </ul>
 */
public sealed interface LoadResponse
        permits LoadResponse.Yield, LoadResponse.Hold, LoadResponse.PushThrough, LoadResponse.Slow {

    /** Yield without a recovery ramp; see {@link Yield#recoverAfter(Measure)} for a timed recovery. */
    Yield YIELD = new Yield(null);

    /** Keep the position command; only the ceiling applies. */
    Hold HOLD = new Hold();

    /**
     * Allows a higher effort while loaded.
     *
     * @param burst effort allowed while loaded, a torque, force or current measure of the binding
     * @param duration how long the burst may last after the load appears
     * @return response
     */
    static PushThrough pushThrough(Measure<?> burst, Measure<?> duration) {
        return new PushThrough(Objects.requireNonNull(burst, "burst"), Objects.requireNonNull(duration, "duration"));
    }

    /**
     * Scales profile speed while loaded.
     *
     * @param fraction speed multiplier in {@code (0, 1]}
     * @return response
     */
    static Slow slow(double fraction) {
        return new Slow(fraction);
    }

    /**
     * Compliant response. {@code recovery} is the time the load must stay clear before the ceiling
     * ramps back in (over the same time).
     *
     * @param recovery recovery time, or null for immediate recovery
     */
    record Yield(Measure<?> recovery) implements LoadResponse {
        /**
         * Returns this response with a recovery time.
         *
         * @param time load-clear time before the ceiling ramps back in
         * @return response
         */
        public Yield recoverAfter(Measure<?> time) {
            return new Yield(Objects.requireNonNull(time, "time"));
        }
    }

    /** Keep position and the ceiling. */
    record Hold() implements LoadResponse {
    }

    /**
     * Burst allowance while loaded.
     *
     * @param burst effort while the burst lasts
     * @param duration burst length
     */
    record PushThrough(Measure<?> burst, Measure<?> duration) implements LoadResponse {
    }

    /**
     * Speed scale while loaded.
     *
     * @param fraction multiplier in {@code (0, 1]}
     */
    record Slow(double fraction) implements LoadResponse {
        public Slow {
            if (!Double.isFinite(fraction) || fraction <= 0.0 || fraction > 1.0) {
                throw new IllegalArgumentException("slow(fraction) must be in (0, 1].");
            }
        }
    }
}
