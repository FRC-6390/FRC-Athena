package ca.frc6390.athena.localization.pipeline;

import java.util.Locale;
import java.util.Objects;
import ca.frc6390.athena.runtime.measurement.PoseSignal;
import ca.frc6390.athena.runtime.measurement.PoseMeasurementSample;
import ca.frc6390.athena.runtime.geometry.Geometry2d;

/**
 * Built-in localization filters. Each names its rejection (see {@link LocalizationFilter#rejectionReason}).
 */
public final class LocalizationFilters {
    private LocalizationFilters() {
    }

    public static LocalizationFilter latencyLessThan(double seconds) {
        double limit = Double.isFinite(seconds) ? seconds : Double.POSITIVE_INFINITY;
        return new ReasonedFilter("latencyLessThan", (pipeline, measurement, pose) -> measurement == null
                || measurement.latencySeconds() < limit
                ? null
                : format("latency %.3f s >= %.3f s", measurement.latencySeconds(), limit));
    }

    public static LocalizationFilter maxAge(double nowSeconds, double seconds) {
        double now = Double.isFinite(nowSeconds) ? nowSeconds : 0.0;
        double limit = Double.isFinite(seconds) ? seconds : Double.POSITIVE_INFINITY;
        return new ReasonedFilter("maxAge", (pipeline, measurement, pose) -> {
            if (measurement == null) {
                return null;
            }
            double age = now - measurement.timestampSeconds();
            return age <= limit ? null : format("age %.3f s > %.3f s", age, limit);
        });
    }

    public static LocalizationFilter tagCountAtLeast(int count) {
        int minimum = Math.max(0, count);
        return new ReasonedFilter("tagCountAtLeast", (pipeline, measurement, pose) ->
                !(measurement instanceof PoseMeasurementSample sample) || sample.targetCount() >= minimum
                        ? null
                        : format("tags %d < %d", ((PoseMeasurementSample) measurement).targetCount(), minimum));
    }

    public static LocalizationFilter maxPoseJump(PoseSignal baseline, double meters) {
        Objects.requireNonNull(baseline, "baseline");
        double max = Double.isFinite(meters) ? Math.max(0.0, meters) : Double.POSITIVE_INFINITY;
        return new ReasonedFilter("maxPoseJump", (pipeline, measurement, pose) -> {
            if (pose != null) {
                return jumpReason(distance(pose, baseline.pose()), max);
            }
            if (!(measurement instanceof PoseMeasurementSample sample)) {
                return null;
            }
            return jumpReason(distance(sample.pose(), baseline.pose()), max);
        });
    }

    /** Accepts only poses contained by the supplied geometry. */
    public static LocalizationFilter inside(Geometry2d geometry) {
        Objects.requireNonNull(geometry, "geometry");
        return new ReasonedFilter("inside", (pipeline, measurement, pose) -> {
            if (pose != null) {
                return geometry.contains(pose) ? null : "pose outside the allowed area";
            }
            if (!(measurement instanceof PoseMeasurementSample sample)) {
                return null;
            }
            return geometry.contains(sample.pose()) ? null : "pose outside the allowed area";
        });
    }

    public static LocalizationFilter custom(LocalizationFilter filter) {
        return Objects.requireNonNull(filter, "filter");
    }

    private static String jumpReason(double jump, double max) {
        return jump <= max ? null : format("jump %.2f m > %.2f m", jump, max);
    }

    private static double distance(ca.frc6390.athena.runtime.filter.PoseSnapshot a,
            ca.frc6390.athena.runtime.filter.PoseSnapshot b) {
        return Math.hypot(a.xMeters() - b.xMeters(), a.yMeters() - b.yMeters());
    }

    static String format(String pattern, Object... values) {
        return String.format(Locale.ROOT, pattern, values);
    }
}
