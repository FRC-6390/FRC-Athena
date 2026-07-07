package ca.frc6390.athena.localization.ref;

import java.util.Objects;
import java.util.function.Predicate;

import ca.frc6390.athena.runtime.measurement.PoseMeasurement;
import ca.frc6390.athena.runtime.measurement.TargetMeasurement;

/**
 * Built-in localization filters.
 */
public final class LocalizationFilters {
    private LocalizationFilters() {
    }

    /**
     * Accepts measurements with latency below a limit.
     *
     * @param seconds latency limit
     * @return filter
     */
    public static LocalizationFilterRef latencyLessThan(double seconds) {
        double limit = Double.isFinite(seconds) ? seconds : Double.POSITIVE_INFINITY;
        return context -> context.measurement()
                .map(measurement -> measurement.latencySeconds() < limit)
                .orElse(true);
    }

    /**
     * Accepts measurements with timestamp age below a limit.
     *
     * @param nowSeconds current timestamp
     * @param seconds age limit
     * @return filter
     */
    public static LocalizationFilterRef maxAge(double nowSeconds, double seconds) {
        double now = Double.isFinite(nowSeconds) ? nowSeconds : 0.0;
        double limit = Double.isFinite(seconds) ? seconds : Double.POSITIVE_INFINITY;
        return context -> context.measurement()
                .map(measurement -> now - measurement.timestampSeconds() <= limit)
                .orElse(true);
    }

    /**
     * Accepts measurements with at least a tag count.
     *
     * @param count minimum tag count
     * @return filter
     */
    public static LocalizationFilterRef tagCountAtLeast(int count) {
        int minimum = Math.max(0, count);
        return context -> context.measurement()
                .map(measurement -> {
                    if (measurement instanceof PoseMeasurement poseMeasurement) {
                        return poseMeasurement.targetCount() >= minimum;
                    }
                    if (measurement instanceof TargetMeasurement targetMeasurement) {
                        return targetMeasurement.targetId() >= 0 && minimum <= 1;
                    }
                    return true;
                })
                .orElse(true);
    }

    /**
     * Accepts results close to a baseline localization result.
     *
     * @param baseline baseline
     * @param meters max distance
     * @return filter
     */
    public static LocalizationFilterRef maxPoseJump(LocalizationRef baseline, double meters) {
        Objects.requireNonNull(baseline, "baseline");
        double max = Double.isFinite(meters) ? Math.max(0.0, meters) : Double.POSITIVE_INFINITY;
        return context -> context.result()
                .map(result -> distance(result.pose(), baseline.pose()) <= max)
                .orElseGet(() -> context.measurement()
                        .filter(PoseMeasurement.class::isInstance)
                        .map(PoseMeasurement.class::cast)
                        .map(measurement -> distance(measurement.pose(), baseline.pose()) <= max)
                        .orElse(true));
    }

    /**
     * Creates a custom filter.
     *
     * @param predicate predicate
     * @return filter
     */
    public static LocalizationFilterRef custom(Predicate<LocalizationFilterContext> predicate) {
        Objects.requireNonNull(predicate, "predicate");
        return predicate::test;
    }

    private static double distance(ca.frc6390.athena.runtime.filter.PoseSnapshot a,
            ca.frc6390.athena.runtime.filter.PoseSnapshot b) {
        return Math.hypot(a.xMeters() - b.xMeters(), a.yMeters() - b.yMeters());
    }
}
