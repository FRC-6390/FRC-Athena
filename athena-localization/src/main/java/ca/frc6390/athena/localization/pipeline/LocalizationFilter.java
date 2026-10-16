package ca.frc6390.athena.localization.pipeline;

import ca.frc6390.athena.runtime.measurement.Measurement;
import ca.frc6390.athena.runtime.filter.PoseSnapshot;

/**
 * Predicate for localization pipeline inputs and estimates.
 *
 * <p>Filters may also name their rejections so that diagnostics can say why a camera frame was not used.
 * The two default methods keep existing lambdas and custom filters working unchanged: a custom filter that
 * does not describe its rejection is reported as {@code filter <name>} or {@code filter #<position>}.</p>
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

    /**
     * Returns a short human reason for a rejection, for example {@code ambiguity 0.31 > 0.20}. Called only after
     * {@link #accept} returned false. Empty when the filter does not describe its rejections.
     *
     * @param localization localization being evaluated
     * @param measurement raw measurement, when evaluating one
     * @param pose estimated pose, when evaluating an estimate or pose measurement
     * @return reason, or an empty string
     */
    default String rejectionReason(Localization localization, Measurement measurement, PoseSnapshot pose) {
        return "";
    }

    /**
     * Returns the name used in diagnostics, or an empty string for an unnamed filter.
     *
     * @return filter name
     */
    default String name() {
        return "";
    }
}
