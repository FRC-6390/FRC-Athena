package ca.frc6390.athena.localization.pipeline;

import ca.frc6390.athena.runtime.measurement.MeasurementSignal;
import java.util.Objects;

/**
 * Factory methods for localization pipelines.
 */
public final class Localizations {
    private Localizations() {
    }

    public static LocalizationPipeline odometry(MeasurementSignal odometry) {
        return new LocalizationPipeline("odometry", LocalizationEstimators.odometry())
                .input(Objects.requireNonNull(odometry, "odometry"));
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

}
