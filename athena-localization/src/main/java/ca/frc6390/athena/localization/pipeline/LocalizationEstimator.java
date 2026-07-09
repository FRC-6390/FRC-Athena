package ca.frc6390.athena.localization.pipeline;

import java.util.Optional;

/**
 * Strategy that turns accepted localization inputs into one estimate.
 */
@FunctionalInterface
interface LocalizationEstimator {
    /**
     * Estimates localization from accepted inputs.
     *
     * @param estimate accepted inputs and previous estimate
     * @return result, if enough data was available
     */
    Optional<LocalizationResult> estimate(LocalizationEstimate estimate);
}
