package ca.frc6390.athena.localization.pipeline;

import java.util.Comparator;
import java.util.List;
import java.util.Optional;

import ca.frc6390.athena.runtime.control.RobotVelocity;
import ca.frc6390.athena.runtime.filter.PoseSnapshot;
import ca.frc6390.athena.runtime.measurement.Measurement;

/**
 * Built-in localization estimator strategies.
 */
final class LocalizationEstimators {
    private LocalizationEstimators() {
    }

    static LocalizationEstimator latestValid() {
        return estimate -> latestCandidate(estimate.inputResults(), estimate.measurements());
    }

    static LocalizationEstimator odometry() {
        return latestValid();
    }

    static LocalizationEstimator vision() {
        return latestValid();
    }

    static LocalizationEstimator weightedAverage() {
        return estimate -> {
            List<PoseSample> measurements = poseSamples(estimate.measurements());
            if (!measurements.isEmpty()) {
                return Optional.of(weightMeasurements(measurements, estimate.rejectedMeasurements()));
            }
            if (!estimate.inputResults().isEmpty()) {
                List<PoseSample> nestedMeasurements = estimate.inputResults().stream()
                        .flatMap(result -> result.acceptedMeasurements().stream())
                        .flatMap(measurement -> PoseSamples.from(measurement).stream())
                        .toList();
                if (!nestedMeasurements.isEmpty()) {
                    return Optional.of(weightMeasurements(nestedMeasurements, estimate.rejectedMeasurements()));
                }
                return Optional.of(averageResults(estimate.inputResults(), estimate.rejectedMeasurements()));
            }
            return Optional.empty();
        };
    }

    private static Optional<LocalizationResult> latestCandidate(
            List<LocalizationResult> inputResults,
            List<Measurement> measurements) {
        Optional<LocalizationResult> result = inputResults.stream()
                .max(Comparator.comparingDouble(LocalizationResult::timestampSeconds));
        if (result.isPresent()) {
            return result;
        }
        return measurements.stream()
                .max(Comparator.comparingDouble(Measurement::timestampSeconds))
                .flatMap(LocalizationResult::from);
    }

    private static LocalizationResult weightMeasurements(
            List<PoseSample> measurements,
            List<Measurement> rejected) {
        double x = 0.0;
        double y = 0.0;
        double sin = 0.0;
        double cos = 0.0;
        double vx = 0.0;
        double vy = 0.0;
        double omega = 0.0;
        double totalWeight = 0.0;
        double timestamp = 0.0;
        for (PoseSample measurement : measurements) {
            double variance = measurement.stdDevs().translationVariance();
            double weight = Double.isFinite(variance) && variance > 1.0e-9 ? 1.0 / variance : 1.0;
            x += measurement.pose().xMeters() * weight;
            y += measurement.pose().yMeters() * weight;
            sin += Math.sin(measurement.pose().headingRadians()) * weight;
            cos += Math.cos(measurement.pose().headingRadians()) * weight;
            vx += measurement.speeds().xMetersPerSecond() * weight;
            vy += measurement.speeds().yMetersPerSecond() * weight;
            omega += measurement.speeds().angularRadiansPerSecond() * weight;
            totalWeight += weight;
            timestamp = Math.max(timestamp, measurement.timestampSeconds());
        }
        double safeWeight = totalWeight <= 0.0 ? 1.0 : totalWeight;
        return new LocalizationResult(
                new PoseSnapshot(x / safeWeight, y / safeWeight, Math.atan2(sin, cos)),
                new RobotVelocity(vx / safeWeight, vy / safeWeight, omega / safeWeight),
                timestamp,
                List.of(),
                rejected);
    }

    private static LocalizationResult averageResults(
            List<LocalizationResult> results,
            List<Measurement> rejected) {
        double x = 0.0;
        double y = 0.0;
        double sin = 0.0;
        double cos = 0.0;
        double vx = 0.0;
        double vy = 0.0;
        double omega = 0.0;
        double timestamp = 0.0;
        for (LocalizationResult result : results) {
            x += result.pose().xMeters();
            y += result.pose().yMeters();
            sin += Math.sin(result.pose().headingRadians());
            cos += Math.cos(result.pose().headingRadians());
            vx += result.speeds().xMetersPerSecond();
            vy += result.speeds().yMetersPerSecond();
            omega += result.speeds().angularRadiansPerSecond();
            timestamp = Math.max(timestamp, result.timestampSeconds());
        }
        double count = Math.max(1, results.size());
        return new LocalizationResult(
                new PoseSnapshot(x / count, y / count, Math.atan2(sin, cos)),
                new RobotVelocity(vx / count, vy / count, omega / count),
                timestamp,
                results.stream().flatMap(result -> result.acceptedMeasurements().stream()).toList(),
                rejected);
    }

    private static List<PoseSample> poseSamples(List<Measurement> measurements) {
        return measurements.stream()
                .flatMap(measurement -> PoseSamples.from(measurement).stream())
                .toList();
    }
}
