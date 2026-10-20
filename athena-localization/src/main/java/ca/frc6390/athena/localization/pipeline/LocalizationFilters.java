package ca.frc6390.athena.localization.pipeline;

import ca.frc6390.athena.runtime.filter.PoseSnapshot;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import java.util.Locale;
import java.util.Objects;
import ca.frc6390.athena.runtime.measurement.PoseSignal;
import ca.frc6390.athena.runtime.measurement.PoseMeasurementSample;
import ca.frc6390.athena.runtime.geometry.Geometry2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Built-in localization filters. Each names its rejection (see {@link LocalizationFilter#rejectionReason}).
 */
public final class LocalizationFilters {
    private LocalizationFilters() {
    }

    public static LocalizationFilter latencyLessThan(Time limitTime) {
        double seconds = limitTime.in(Seconds);
        double limit = Double.isFinite(seconds) ? seconds : Double.POSITIVE_INFINITY;
        return new ReasonedFilter("latencyLessThan", (pipeline, measurement, pose) -> measurement == null
                || measurement.latencySeconds() < limit
                ? null
                : format("latency %.3f s >= %.3f s", measurement.latencySeconds(), limit));
    }

    public static LocalizationFilter maxAge(Time now, Time maximumAge) {
        double nowSeconds = now.in(Seconds);
        double seconds = maximumAge.in(Seconds);
        double nowValue = Double.isFinite(nowSeconds) ? nowSeconds : 0.0;
        double limit = Double.isFinite(seconds) ? seconds : Double.POSITIVE_INFINITY;
        return new ReasonedFilter("maxAge", (pipeline, measurement, pose) -> {
            if (measurement == null) {
                return null;
            }
            double age = nowValue - measurement.timestampSeconds();
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

    public static LocalizationFilter maxPoseJump(PoseSignal baseline, Distance maximumJump) {
        Objects.requireNonNull(baseline, "baseline");
        double meters = maximumJump.in(Meters);
        double max = Double.isFinite(meters) ? Math.max(0.0, meters) : Double.POSITIVE_INFINITY;
        return new ReasonedFilter("maxPoseJump", (pipeline, measurement, pose) -> {
            if (pose != null) {
                return jumpReason(distance(pose, baseline.poseSnapshot()), max);
            }
            if (!(measurement instanceof PoseMeasurementSample sample)) {
                return null;
            }
            return jumpReason(distance(sample.pose(), baseline.poseSnapshot()), max);
        });
    }

    /** Accepts only poses contained by the supplied geometry. */
    public static LocalizationFilter inside(Geometry2d geometry) {
        Objects.requireNonNull(geometry, "geometry");
        return new ReasonedFilter("inside", (pipeline, measurement, pose) -> {
            if (pose != null) {
                return geometry.contains(new Translation2d(pose.xMeters(), pose.yMeters())) ? null : "pose outside the allowed area";
            }
            if (!(measurement instanceof PoseMeasurementSample sample)) {
                return null;
            }
            return geometry.contains(new Translation2d(sample.pose().xMeters(), sample.pose().yMeters())) ? null : "pose outside the allowed area";
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

    /**
     * {@code String.format(Locale.ROOT, pattern, values)}. Rejection and status reasons are built every cycle, so the
     * patterns used here ({@code %s}, {@code %d} and {@code %.Nf} of non-negative doubles) are formatted directly;
     * anything else, and any value close to a rounding tie, goes through {@link String#format}.
     */
    static String format(String pattern, Object... values) {
        String formatted = formatDirect(pattern, values);
        return formatted != null ? formatted : String.format(Locale.ROOT, pattern, values);
    }

    private static final long[] POWERS_OF_TEN = {1L, 10L, 100L, 1_000L, 10_000L, 100_000L, 1_000_000L};

    /** Formats the supported subset of patterns, or returns null so the caller uses {@link String#format}. */
    private static String formatDirect(String pattern, Object[] values) {
        StringBuilder out = new StringBuilder(pattern.length() + 16);
        int next = 0;
        int length = pattern.length();
        for (int index = 0; index < length; index++) {
            char c = pattern.charAt(index);
            if (c != '%') {
                out.append(c);
                continue;
            }
            if (values == null || next >= values.length || index + 1 >= length) {
                return null;
            }
            Object value = values[next++];
            char conversion = pattern.charAt(++index);
            if (conversion == 's') {
                if (!(value instanceof String text)) {
                    return null;
                }
                out.append(text);
            } else if (conversion == 'd') {
                if (!(value instanceof Integer || value instanceof Long)) {
                    return null;
                }
                out.append(((Number) value).longValue());
            } else if (conversion == '.' && index + 2 < length && pattern.charAt(index + 2) == 'f') {
                int digits = pattern.charAt(index + 1) - '0';
                if (digits < 0 || digits >= POWERS_OF_TEN.length
                        || !(value instanceof Double number)
                        || !appendFixed(out, number, digits)) {
                    return null;
                }
                index += 2;
            } else {
                return null;
            }
        }
        return next == (values == null ? 0 : values.length) ? out.toString() : null;
    }

    /**
     * Appends {@code value} with {@code digits} decimals, rounded half-up like {@link java.util.Formatter}. Returns
     * false for negative, non-finite or very large values and for values within rounding error of a tie, whose
     * digits the formatter decides from the decimal representation.
     */
    private static boolean appendFixed(StringBuilder out, double value, int digits) {
        if (!(value >= 0.0) || value >= 1.0e9 || Double.doubleToRawLongBits(value) < 0L) {
            return false;
        }
        long scale = POWERS_OF_TEN[digits];
        double scaled = value * scale;
        double whole = Math.floor(scaled);
        double fraction = scaled - whole;
        if (Math.abs(fraction - 0.5) < 1.0e-6 + 4.0 * Math.ulp(scaled)) {
            return false;
        }
        long rounded = (long) whole + (fraction > 0.5 ? 1L : 0L);
        out.append(rounded / scale);
        if (digits > 0) {
            out.append('.');
            String decimals = Long.toString(rounded % scale);
            for (int pad = decimals.length(); pad < digits; pad++) {
                out.append('0');
            }
            out.append(decimals);
        }
        return true;
    }
}
