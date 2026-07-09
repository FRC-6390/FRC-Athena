package ca.frc6390.athena.localization.ref;

import java.util.List;
import java.util.Objects;
import java.util.Optional;

import ca.frc6390.athena.runtime.control.RobotVelocity;
import ca.frc6390.athena.runtime.filter.PoseSnapshot;
import ca.frc6390.athena.runtime.measurement.Measurement;

/**
 * Internal output from a localization pipeline.
 */
record LocalizationResult(
        PoseSnapshot pose,
        RobotVelocity speeds,
        double timestampSeconds,
        List<Measurement> acceptedMeasurements,
        List<Measurement> rejectedMeasurements) {
    LocalizationResult {
        Objects.requireNonNull(pose, "pose");
        speeds = speeds == null ? RobotVelocity.zero() : speeds;
        timestampSeconds = Double.isFinite(timestampSeconds) ? timestampSeconds : 0.0;
        acceptedMeasurements = acceptedMeasurements == null ? List.of() : List.copyOf(acceptedMeasurements);
        rejectedMeasurements = rejectedMeasurements == null ? List.of() : List.copyOf(rejectedMeasurements);
    }

    public static LocalizationResult of(PoseSnapshot pose) {
        return new LocalizationResult(pose, RobotVelocity.zero(), 0.0, List.of(), List.of());
    }

    public static Optional<LocalizationResult> from(Measurement measurement) {
        return PoseSamples.from(measurement)
                .map(sample -> new LocalizationResult(
                        sample.pose(),
                        sample.speeds(),
                        sample.timestampSeconds(),
                        List.of(measurement),
                        List.of()));
    }

    public LocalizationResult withRejected(List<Measurement> rejected) {
        return new LocalizationResult(pose, speeds, timestampSeconds, acceptedMeasurements, rejected);
    }
}
