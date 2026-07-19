package ca.frc6390.athena.api;

/** Controls how Athena retries and re-enables a declaration after a transient failure. */
public record RecoveryPolicy(double retryIntervalSeconds, int healthySamples) {
    public static final RecoveryPolicy DEFAULT = new RecoveryPolicy(0.0, 3);

    public RecoveryPolicy {
        if (!Double.isFinite(retryIntervalSeconds) || retryIntervalSeconds < 0.0) {
            throw new IllegalArgumentException("Recovery retry interval must be finite and non-negative.");
        }
        if (healthySamples < 1) {
            throw new IllegalArgumentException("Recovery healthy samples must be positive.");
        }
    }

    /** Retries a failed declaration no more often than the supplied interval. */
    public static RecoveryPolicy retryEverySeconds(double seconds) {
        return new RecoveryPolicy(seconds, DEFAULT.healthySamples);
    }

    /** Returns a copy requiring this many successful retries before re-enabling the declaration. */
    public RecoveryPolicy healthySamples(int samples) {
        return new RecoveryPolicy(retryIntervalSeconds, samples);
    }
}
