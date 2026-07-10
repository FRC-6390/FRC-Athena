package ca.frc6390.athena.localization.pipeline;

/** Factory methods for uniform, inspectable localization nodes. */
public final class Localizations {
    private Localizations() {
    }

    /** Passes accepted pose measurements through unchanged. */
    public static Localization filter() {
        return new Localization(Localization.Strategy.FILTER);
    }

    /** Emits the newest accepted pose measurement. */
    public static Localization latestValid() {
        return new Localization(Localization.Strategy.LATEST_VALID);
    }

    /** Emits a variance-weighted pose average. */
    public static Localization weightedAverage() {
        return new Localization(Localization.Strategy.WEIGHTED_AVERAGE);
    }

    /** Conservatively fuses pose measurements with unknown cross-correlation. */
    public static Localization covarianceIntersection() {
        return new Localization(Localization.Strategy.COVARIANCE_INTERSECTION);
    }

    /** Creates a real timestamp-aware Kalman localization node. */
    public static Localization kalman() {
        return new Localization(Localization.Strategy.KALMAN);
    }
}
