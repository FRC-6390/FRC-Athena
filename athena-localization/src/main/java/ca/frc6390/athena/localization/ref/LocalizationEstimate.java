package ca.frc6390.athena.localization.ref;

import java.util.List;
import java.util.Optional;

import ca.frc6390.athena.runtime.measurement.Measurement;
import ca.frc6390.athena.runtime.measurement.MeasurementSignal;

/**
 * Accepted inputs visible to a localization estimator.
 */
record LocalizationEstimate(
        LocalizationPipeline target,
        List<MeasurementSignal> inputs,
        List<LocalizationResult> inputResults,
        List<Measurement> measurements,
        List<Measurement> rejectedMeasurements,
        Optional<LocalizationResult> previous) {
    LocalizationEstimate {
        inputs = inputs == null ? List.of() : List.copyOf(inputs);
        inputResults = inputResults == null ? List.of() : List.copyOf(inputResults);
        measurements = measurements == null ? List.of() : List.copyOf(measurements);
        rejectedMeasurements = rejectedMeasurements == null ? List.of() : List.copyOf(rejectedMeasurements);
        previous = previous == null ? Optional.empty() : previous;
    }
}
