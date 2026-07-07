package ca.frc6390.athena.localization.config;

import java.util.function.Consumer;
import java.util.function.Supplier;

import ca.frc6390.athena.localization.ref.LocalizationEstimatorRef;
import ca.frc6390.athena.localization.ref.LocalizationEstimators;
import ca.frc6390.athena.localization.ref.LocalizationRef;
import ca.frc6390.athena.localization.spec.LocalizationSpec;

/**
 * Factory methods for localization declarations.
 */
public final class Localizations {
    private Localizations() {
    }

    /**
     * Creates and lowers a named localization declaration.
     *
     * @param name localization name
     * @param customizer declaration customizer
     * @return immutable localization spec
     */
    public static LocalizationSpec localization(String name, Consumer<LocalizationConfig> customizer) {
        LocalizationConfig config = LocalizationConfig.create(name);
        if (customizer != null) {
            customizer.accept(config);
        }
        return config.toSpec();
    }

    /**
     * Creates an odometry localization node.
     *
     * @return localization ref
     */
    public static LocalizationRef odometry() {
        return new LocalizationRef("odometry", LocalizationEstimators.odometry());
    }

    /**
     * Creates a vision localization node.
     *
     * @return localization ref
     */
    public static LocalizationRef vision() {
        return new LocalizationRef("vision", LocalizationEstimators.vision());
    }

    /**
     * Creates a weighted-average localization node.
     *
     * @return localization ref
     */
    public static LocalizationRef weightedAverage() {
        return new LocalizationRef("weightedAverage", LocalizationEstimators.weightedAverage());
    }

    /**
     * Creates a latest-valid localization node.
     *
     * @return localization ref
     */
    public static LocalizationRef latestValid() {
        return new LocalizationRef("latestValid", LocalizationEstimators.latestValid());
    }

    /**
     * Creates a Kalman-style localization node.
     *
     * @return localization ref
     */
    public static LocalizationRef kalman() {
        return new LocalizationRef("kalman", LocalizationEstimators.kalman());
    }

    /**
     * Creates a custom localization node.
     *
     * @param estimator estimator
     * @return localization ref
     */
    public static LocalizationRef custom(LocalizationEstimatorRef estimator) {
        return new LocalizationRef("custom", LocalizationEstimators.custom(estimator));
    }

    /**
     * Creates a custom localization node.
     *
     * @param estimator estimator supplier
     * @return localization ref
     */
    public static LocalizationRef custom(Supplier<LocalizationEstimatorRef> estimator) {
        return custom(estimator.get());
    }
}
