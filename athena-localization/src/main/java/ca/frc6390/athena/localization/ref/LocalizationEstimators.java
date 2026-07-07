package ca.frc6390.athena.localization.ref;

import java.util.Comparator;
import java.util.List;
import java.util.Objects;
import java.util.Optional;

import ca.frc6390.athena.runtime.control.RobotVelocity;
import ca.frc6390.athena.runtime.filter.PoseSnapshot;
import ca.frc6390.athena.runtime.measurement.Measurement;
import ca.frc6390.athena.runtime.measurement.PoseMeasurement;

/**
 * Built-in localization estimator strategies.
 */
public final class LocalizationEstimators {
    private LocalizationEstimators() {
    }

    /**
     * Uses the latest available input result or measurement.
     *
     * @return estimator
     */
    public static LocalizationEstimatorRef latestValid() {
        return context -> latestCandidate(context.inputResults(), context.measurements());
    }

    /**
     * Uses odometry-like inputs by taking the latest candidate.
     *
     * @return estimator
     */
    public static LocalizationEstimatorRef odometry() {
        return latestValid();
    }

    /**
     * Uses vision-like measurements by taking the latest candidate.
     *
     * @return estimator
     */
    public static LocalizationEstimatorRef vision() {
        return latestValid();
    }

    /**
     * Averages accepted inputs using measurement standard deviations when available.
     *
     * @return estimator
     */
    public static LocalizationEstimatorRef weightedAverage() {
        return context -> {
            List<PoseMeasurement> measurements = poseMeasurements(context.measurements());
            if (!measurements.isEmpty()) {
                return Optional.of(weightMeasurements(measurements, context.rejectedMeasurements()));
            }
            if (!context.inputResults().isEmpty()) {
                List<PoseMeasurement> nestedMeasurements = context.inputResults().stream()
                        .flatMap(result -> result.acceptedMeasurements().stream())
                        .filter(PoseMeasurement.class::isInstance)
                        .map(PoseMeasurement.class::cast)
                        .toList();
                if (!nestedMeasurements.isEmpty()) {
                    return Optional.of(weightMeasurements(nestedMeasurements, context.rejectedMeasurements()));
                }
                return Optional.of(averageResults(context.inputResults(), context.rejectedMeasurements()));
            }
            return Optional.empty();
        };
    }

    /**
     * Lightweight fusion placeholder for Kalman-style estimators.
     *
     * @return estimator
     */
    public static LocalizationEstimatorRef kalman() {
        return weightedAverage();
    }

    /**
     * Returns a custom estimator.
     *
     * @param estimator estimator
     * @return estimator
     */
    public static LocalizationEstimatorRef custom(LocalizationEstimatorRef estimator) {
        return Objects.requireNonNull(estimator, "estimator");
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
                .filter(PoseMeasurement.class::isInstance)
                .map(PoseMeasurement.class::cast)
                .max(Comparator.comparingDouble(PoseMeasurement::timestampSeconds))
                .map(LocalizationResult::from);
    }

    private static LocalizationResult weightMeasurements(
            List<PoseMeasurement> measurements,
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
        for (PoseMeasurement measurement : measurements) {
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
                measurements.stream().map(Measurement.class::cast).toList(),
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

    private static List<PoseMeasurement> poseMeasurements(List<Measurement> measurements) {
        return measurements.stream()
                .filter(PoseMeasurement.class::isInstance)
                .map(PoseMeasurement.class::cast)
                .toList();
    }
}
