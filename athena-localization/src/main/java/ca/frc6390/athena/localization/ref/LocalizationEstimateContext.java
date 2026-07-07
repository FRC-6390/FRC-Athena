package ca.frc6390.athena.localization.ref;

import java.util.List;
import java.util.Optional;

import ca.frc6390.athena.runtime.measurement.Measurement;
import ca.frc6390.athena.runtime.measurement.MeasurementRef;

/**
 * Inputs visible to a localization estimator.
 *
 * @param target target localization ref
 * @param inputs declared inputs
 * @param inputResults accepted input results
 * @param measurements accepted raw measurements
 * @param rejectedMeasurements rejected raw measurements
 * @param previous previous result
 */
public record LocalizationEstimateContext(
        LocalizationRef target,
        List<MeasurementRef> inputs,
        List<LocalizationResult> inputResults,
        List<Measurement> measurements,
        List<Measurement> rejectedMeasurements,
        Optional<LocalizationResult> previous) {
    public LocalizationEstimateContext {
        inputs = inputs == null ? List.of() : List.copyOf(inputs);
        inputResults = inputResults == null ? List.of() : List.copyOf(inputResults);
        measurements = measurements == null ? List.of() : List.copyOf(measurements);
        rejectedMeasurements = rejectedMeasurements == null ? List.of() : List.copyOf(rejectedMeasurements);
        previous = previous == null ? Optional.empty() : previous;
    }
}
