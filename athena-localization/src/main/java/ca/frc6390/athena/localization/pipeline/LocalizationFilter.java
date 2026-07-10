package ca.frc6390.athena.localization.pipeline;

import ca.frc6390.athena.runtime.measurement.Measurement;
import ca.frc6390.athena.runtime.filter.PoseSnapshot;

/**
 * Predicate for localization pipeline inputs and estimates.
 */
@FunctionalInterface
public interface LocalizationFilter {
    /**
     * Returns whether a measurement or estimate should be accepted.
     *
     * @param localization localization being evaluated
     * @param measurement raw measurement, when evaluating one
     * @param pose estimated pose, when evaluating an estimate or pose measurement
     * @return true when accepted
     */
    boolean accept(Localization localization, Measurement measurement, PoseSnapshot pose);
}
