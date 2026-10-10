package ca.frc6390.athena.localization.pipeline;

import ca.frc6390.athena.runtime.measurement.Measurement;
import ca.frc6390.athena.runtime.measurement.PoseMeasurementSample;

/** Filters for camera-derived pose measurements. */
public final class VisionFilters {
    private VisionFilters() {
    }

    /**
     * Rejects measurements whose total latency exceeds a limit.
     *
     * <p>Total latency is the larger of the reported pipeline latency and the frame age
     * ({@code now - capture timestamp}) when the localization is refreshed by the runtime. Frame age
     * also covers transport delay and frames that a camera keeps re-reporting while no new result
     * arrives. When no runtime timestamp is available only the reported latency is checked.</p>
     *
     * @param seconds maximum latency in seconds
     * @return filter
     */
    public static LocalizationFilter maxLatencySeconds(double seconds) {
        double maximum = nonNegative(seconds, "Maximum latency");
        return (localization, measurement, pose) -> measurement == null
                || (measurement.latencySeconds() <= maximum
                        && !(ageSeconds(localization, measurement) > maximum));
    }

    /**
     * Rejects measurements whose capture timestamp is older than a limit relative to the
     * runtime cycle timestamp. Measurements pass when no runtime timestamp is available.
     *
     * @param seconds maximum frame age in seconds
     * @return filter
     */
    public static LocalizationFilter maxAgeSeconds(double seconds) {
        double maximum = nonNegative(seconds, "Maximum age");
        return (localization, measurement, pose) -> measurement == null
                || !(ageSeconds(localization, measurement) > maximum);
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
        return (localization, measurement, pose) -> !(measurement instanceof PoseMeasurementSample sample)
                || sample.targetCount() > 1
                || sample.ambiguity() <= maximum;
    }

    public static LocalizationFilter maxAverageTagDistanceMeters(double meters) {
        double maximum = nonNegative(meters, "Maximum average tag distance");
        return (localization, measurement, pose) -> {
            if (!(measurement instanceof PoseMeasurementSample sample)) {
                return true;
            }
            double distance = sample.averageTargetDistanceMeters();
            return !Double.isFinite(distance) || distance <= maximum;
        };
    }

    public static LocalizationFilter minimumTagCount(int count) {
        int minimum = Math.max(0, count);
        return (localization, measurement, pose) -> !(measurement instanceof PoseMeasurementSample sample)
                || sample.targetCount() >= minimum;
    }

    public static LocalizationFilter finitePoseAndTimestamp() {
        return (localization, measurement, pose) -> measurement != null
                && pose != null
                && Double.isFinite(measurement.timestampSeconds())
                && Double.isFinite(pose.xMeters())
                && Double.isFinite(pose.yMeters())
                && Double.isFinite(pose.headingRadians());
    }

    private static double nonNegative(double value, String name) {
        if (!Double.isFinite(value) || value < 0.0) {
            throw new IllegalArgumentException(name + " must be finite and non-negative.");
        }
        return value;
    }
}
