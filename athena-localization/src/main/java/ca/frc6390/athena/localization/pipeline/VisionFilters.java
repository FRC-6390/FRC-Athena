package ca.frc6390.athena.localization.pipeline;

import ca.frc6390.athena.runtime.measurement.PoseMeasurementSample;

/** Filters for camera-derived pose measurements. */
public final class VisionFilters {
    private VisionFilters() {
    }

    public static LocalizationFilter maxLatencySeconds(double seconds) {
        double maximum = nonNegative(seconds, "Maximum latency");
        return (localization, measurement, pose) -> measurement == null
                || measurement.latencySeconds() <= maximum;
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
