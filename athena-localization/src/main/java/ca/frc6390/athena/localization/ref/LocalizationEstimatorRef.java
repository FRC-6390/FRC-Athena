package ca.frc6390.athena.localization.ref;

import java.util.Optional;

/**
 * Strategy that turns localization inputs into one localization result.
 */
@FunctionalInterface
public interface LocalizationEstimatorRef {
    /**
     * Estimates localization from accepted inputs.
     *
     * @param context estimate context
     * @return result, if enough data was available
     */
    Optional<LocalizationResult> estimate(LocalizationEstimateContext context);
}
