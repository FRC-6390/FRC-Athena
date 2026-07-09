package ca.frc6390.athena.localization.pipeline;

/**
 * Factory methods for localization pipelines.
 */
public final class Localizations {
    private Localizations() {
    }

    public static LocalizationPipeline odometry() {
        return new LocalizationPipeline("odometry", LocalizationEstimators.odometry());
    }

    public static LocalizationPipeline vision() {
        return new LocalizationPipeline("vision", LocalizationEstimators.vision());
    }

    public static LocalizationPipeline weightedAverage() {
        return new LocalizationPipeline("weightedAverage", LocalizationEstimators.weightedAverage());
    }

    public static LocalizationPipeline latestValid() {
        return new LocalizationPipeline("latestValid", LocalizationEstimators.latestValid());
    }

    public static LocalizationPipeline kalman() {
        return new LocalizationPipeline("kalman", LocalizationEstimators.kalman());
    }
}
