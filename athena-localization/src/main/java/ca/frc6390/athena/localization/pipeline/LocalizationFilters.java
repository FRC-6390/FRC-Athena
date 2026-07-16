package ca.frc6390.athena.localization.pipeline;

import java.util.Objects;
import ca.frc6390.athena.runtime.measurement.PoseSignal;
import ca.frc6390.athena.runtime.measurement.PoseMeasurementSample;
import ca.frc6390.athena.runtime.geometry.Geometry2d;

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
        return (pipeline, measurement, pose) -> !(measurement instanceof PoseMeasurementSample sample)
                || sample.targetCount() >= minimum;
    }

    public static LocalizationFilter maxPoseJump(PoseSignal baseline, double meters) {
        Objects.requireNonNull(baseline, "baseline");
        double max = Double.isFinite(meters) ? Math.max(0.0, meters) : Double.POSITIVE_INFINITY;
        return (pipeline, measurement, pose) -> {
            if (pose != null) {
                return distance(pose, baseline.pose()) <= max;
            }
            return !(measurement instanceof PoseMeasurementSample sample)
                    || distance(sample.pose(), baseline.pose()) <= max;
        };
    }

    /** Accepts only poses contained by the supplied geometry. */
    public static LocalizationFilter inside(Geometry2d geometry) {
        Objects.requireNonNull(geometry, "geometry");
        return (pipeline, measurement, pose) -> {
            if (pose != null) {
                return geometry.contains(pose);
            }
            return !(measurement instanceof PoseMeasurementSample sample) || geometry.contains(sample.pose());
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
