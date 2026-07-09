package ca.frc6390.athena.localization.ref;

import java.util.Objects;

/**
 * Built-in localization filters.
 */
public final class LocalizationFilters {
    private LocalizationFilters() {
    }

    public static LocalizationFilter latencyLessThan(double seconds) {
        double limit = Double.isFinite(seconds) ? seconds : Double.POSITIVE_INFINITY;
        return (pipeline, measurement, pose) -> measurement == null
                || measurement.latencySeconds() < limit;
    }

    public static LocalizationFilter maxAge(double nowSeconds, double seconds) {
        double now = Double.isFinite(nowSeconds) ? nowSeconds : 0.0;
        double limit = Double.isFinite(seconds) ? seconds : Double.POSITIVE_INFINITY;
        return (pipeline, measurement, pose) -> measurement == null
                || now - measurement.timestampSeconds() <= limit;
    }

    public static LocalizationFilter tagCountAtLeast(int count) {
        int minimum = Math.max(0, count);
        return (pipeline, measurement, pose) -> PoseSamples.from(measurement)
                .map(sample -> sample.targetCount() >= minimum)
                .orElse(true);
    }

    public static LocalizationFilter maxPoseJump(LocalizationPipeline baseline, double meters) {
        Objects.requireNonNull(baseline, "baseline");
        double max = Double.isFinite(meters) ? Math.max(0.0, meters) : Double.POSITIVE_INFINITY;
        return (pipeline, measurement, pose) -> {
            if (pose != null) {
                return distance(pose, baseline.pose()) <= max;
            }
            return PoseSamples.from(measurement)
                    .map(sample -> distance(sample.pose(), baseline.pose()) <= max)
                    .orElse(true);
        };
    }

    public static LocalizationFilter custom(LocalizationFilter filter) {
        return Objects.requireNonNull(filter, "filter");
    }

    private static double distance(ca.frc6390.athena.runtime.filter.PoseSnapshot a,
            ca.frc6390.athena.runtime.filter.PoseSnapshot b) {
        return Math.hypot(a.xMeters() - b.xMeters(), a.yMeters() - b.yMeters());
    }
}
