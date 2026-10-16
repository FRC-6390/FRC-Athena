package ca.frc6390.athena.localization.pipeline;

import ca.frc6390.athena.runtime.measurement.Measurement;
import ca.frc6390.athena.runtime.measurement.PoseMeasurementSample;

/** Filters for camera-derived pose measurements. Each names its rejection in diagnostics. */
public final class VisionFilters {
    private VisionFilters() {
    }

    /**
     * Rejects measurements whose total latency exceeds a limit.
     *
     * <p>Total latency is the larger of the reported pipeline latency and the frame age
     * ({@code now - capture timestamp}) when the localization is refreshed by the runtime. Frame age
     * also covers transport delay and frames that a camera keeps re-reporting while no new result
     * arrives. Frames captured in the future (more than 0.1 s ahead of the runtime timestamp) are
     * rejected too: they come from a camera whose clock is not synced to the robot yet. When no runtime
     * timestamp is available only the reported latency is checked.</p>
     *
     * @param seconds maximum latency in seconds
     * @return filter
     */
    public static LocalizationFilter maxLatencySeconds(double seconds) {
        double maximum = nonNegative(seconds, "Maximum latency");
        return new ReasonedFilter("maxLatencySeconds", (localization, measurement, pose) -> {
            if (measurement == null) {
                return null;
            }
            if (!(measurement.latencySeconds() <= maximum)) {
                return LocalizationFilters.format("latency %.3f s > %.3f s", measurement.latencySeconds(), maximum);
            }
            return ageReason(ageSeconds(localization, measurement), maximum);
        });
    }

    /**
     * Rejects measurements whose capture timestamp is older than a limit relative to the
     * runtime cycle timestamp, or captured more than 0.1 s in the future (camera clock not synced yet).
     * Measurements pass when no runtime timestamp is available.
     *
     * @param seconds maximum frame age in seconds
     * @return filter
     */
    public static LocalizationFilter maxAgeSeconds(double seconds) {
        double maximum = nonNegative(seconds, "Maximum age");
        return new ReasonedFilter("maxAgeSeconds", (localization, measurement, pose) -> measurement == null
                ? null
                : ageReason(ageSeconds(localization, measurement), maximum));
    }

    /** Clock skew allowed before a frame counts as coming from the future. */
    private static final double FUTURE_TOLERANCE_SECONDS = 0.1;

    /**
     * Null when the frame age is unknown (NaN), or between "slightly in the future" and the maximum. A
     * clearly negative age means the camera clock is not synced to the robot yet, so the frame is rejected.
     */
    private static String ageReason(double age, double maximum) {
        if (Double.isNaN(age)) {
            return null;
        }
        if (age < -FUTURE_TOLERANCE_SECONDS) {
            return LocalizationFilters.format("frame %.1f s in the future", -age);
        }
        return age <= maximum ? null : LocalizationFilters.format("frame age %.3f s > %.3f s", age, maximum);
    }

    private static double ageSeconds(Localization localization, Measurement measurement) {
        if (localization == null) {
            return Double.NaN;
        }
        double now = localization.cycleTimestampSeconds();
        double captured = measurement.timestampSeconds();
        return Double.isFinite(now) && Double.isFinite(captured) ? now - captured : Double.NaN;
    }

    public static LocalizationFilter maxAmbiguity(double ambiguity) {
        double maximum = nonNegative(ambiguity, "Maximum ambiguity");
        return new ReasonedFilter("maxAmbiguity", (localization, measurement, pose) ->
                !(measurement instanceof PoseMeasurementSample sample)
                        || sample.targetCount() > 1
                        || sample.ambiguity() <= maximum
                        ? null
                        : LocalizationFilters.format("ambiguity %.2f > %.2f",
                                ((PoseMeasurementSample) measurement).ambiguity(), maximum));
    }

    public static LocalizationFilter maxAverageTagDistanceMeters(double meters) {
        double maximum = nonNegative(meters, "Maximum average tag distance");
        return new ReasonedFilter("maxAverageTagDistanceMeters", (localization, measurement, pose) -> {
            if (!(measurement instanceof PoseMeasurementSample sample)) {
                return null;
            }
            double distance = sample.averageTargetDistanceMeters();
            return !Double.isFinite(distance) || distance <= maximum
                    ? null
                    : LocalizationFilters.format("tag distance %.2f m > %.2f m", distance, maximum);
        });
    }

    public static LocalizationFilter minimumTagCount(int count) {
        int minimum = Math.max(0, count);
        return new ReasonedFilter("minimumTagCount", (localization, measurement, pose) ->
                !(measurement instanceof PoseMeasurementSample sample) || sample.targetCount() >= minimum
                        ? null
                        : LocalizationFilters.format("tags %d < %d",
                                ((PoseMeasurementSample) measurement).targetCount(), minimum));
    }

    public static LocalizationFilter finitePoseAndTimestamp() {
        return new ReasonedFilter("finitePoseAndTimestamp", (localization, measurement, pose) -> {
            boolean finite = measurement != null
                    && pose != null
                    && Double.isFinite(measurement.timestampSeconds())
                    && Double.isFinite(pose.xMeters())
                    && Double.isFinite(pose.yMeters())
                    && Double.isFinite(pose.headingRadians());
            return finite ? null : "non-finite pose or timestamp";
        });
    }

    private static double nonNegative(double value, String name) {
        if (!Double.isFinite(value) || value < 0.0) {
            throw new IllegalArgumentException(name + " must be finite and non-negative.");
        }
        return value;
    }
}
