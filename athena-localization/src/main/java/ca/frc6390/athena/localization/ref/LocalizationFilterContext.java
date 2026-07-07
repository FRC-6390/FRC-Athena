package ca.frc6390.athena.localization.ref;

import java.util.Optional;

import ca.frc6390.athena.runtime.measurement.Measurement;
import ca.frc6390.athena.runtime.measurement.MeasurementRef;

/**
 * Context evaluated by a localization filter.
 *
 * @param target target localization ref
 * @param input input being evaluated
 * @param measurement raw measurement being evaluated
 * @param result result being evaluated
 */
public record LocalizationFilterContext(
        LocalizationRef target,
        MeasurementRef input,
        Optional<Measurement> measurement,
        Optional<LocalizationResult> result) {
    public LocalizationFilterContext {
        measurement = measurement == null ? Optional.empty() : measurement;
        result = result == null ? Optional.empty() : result;
    }
}
