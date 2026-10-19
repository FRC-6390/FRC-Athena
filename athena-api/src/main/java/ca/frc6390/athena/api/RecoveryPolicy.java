package ca.frc6390.athena.api;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Time;

/**
 * Controls how Athena retries and re-enables a declaration after a transient failure.
 *
 * @param retryInterval minimum time between retries of a failed declaration
 * @param healthySamples successful retries required before the declaration is re-enabled
 */
public record RecoveryPolicy(Time retryInterval, int healthySamples) {
    public static final RecoveryPolicy DEFAULT = new RecoveryPolicy(Seconds.of(0.0), 3);

    public RecoveryPolicy {
        if (retryInterval == null) {
            throw new IllegalArgumentException("Recovery retry interval must not be null.");
        }
        double seconds = retryInterval.in(Seconds);
        if (!Double.isFinite(seconds) || seconds < 0.0) {
            throw new IllegalArgumentException("Recovery retry interval must be finite and non-negative.");
        }
        if (healthySamples < 1) {
            throw new IllegalArgumentException("Recovery healthy samples must be positive.");
        }
    }

    /** Retries a failed declaration no more often than the supplied interval. */
    public static RecoveryPolicy retryEvery(Time interval) {
        return new RecoveryPolicy(interval, DEFAULT.healthySamples);
    }

    /** Returns a copy requiring this many successful retries before re-enabling the declaration. */
    public RecoveryPolicy healthySamples(int samples) {
        return new RecoveryPolicy(retryInterval, samples);
    }
}
