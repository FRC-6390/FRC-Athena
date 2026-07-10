package ca.frc6390.athena.runtime.measurement;

import java.util.List;
import java.util.Optional;
import java.util.function.BooleanSupplier;

/**
 * Typed runtime stream that emits measurement samples.
 */
public interface MeasurementSignal {
    /**
     * Returns current measurements from this signal.
     *
     * @return measurements
     */
    List<Measurement> measurements();

    /**
     * Returns the newest measurement if one exists.
     *
     * @return newest measurement
     */
    default Optional<Measurement> latestMeasurement() {
        Measurement latest = null;
        for (Measurement measurement : measurements()) {
            if (latest == null || measurement.timestampSeconds() > latest.timestampSeconds()) {
                latest = measurement;
            }
        }
        return Optional.ofNullable(latest);
    }

    /**
     * Applies standard deviations to pose measurements from this signal.
     *
     * @param stdDevs standard deviations
     * @return configured measurement signal
     */
    default MeasurementSignal stdDevs(MeasurementStdDevs stdDevs) {
        return new ConfiguredMeasurementSignal(this, stdDevs, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, null,
                "");
    }

    /**
     * Applies standard deviations with one translation value shared by both field axes.
     *
     * @param translationMeters translation standard deviation
     * @param headingRadians heading standard deviation
     * @return configured measurement signal
     */
    default MeasurementSignal stdDevs(double translationMeters, double headingRadians) {
        return stdDevs(translationMeters, translationMeters, headingRadians);
    }

    /**
     * Applies standard deviations directly without requiring a configuration object.
     *
     * @param xMeters x standard deviation
     * @param yMeters y standard deviation
     * @param headingRadians heading standard deviation
     * @return configured measurement signal
     */
    default MeasurementSignal stdDevs(double xMeters, double yMeters, double headingRadians) {
        return stdDevs(MeasurementStdDevs.of(xMeters, yMeters, headingRadians));
    }

    /**
     * Filters measurements with latency above a limit.
     *
     * @param seconds latency limit
     * @return configured measurement signal
     */
    default MeasurementSignal latencyLimit(double seconds) {
        return new ConfiguredMeasurementSignal(this, null, seconds, Double.POSITIVE_INFINITY, null, "");
    }

    /**
     * Filters measurements older than a limit.
     *
     * @param nowSeconds current timestamp
     * @param seconds max age
     * @return configured measurement signal
     */
    default MeasurementSignal maxAge(double nowSeconds, double seconds) {
        return new ConfiguredMeasurementSignal(this, null, Double.POSITIVE_INFINITY, seconds, () -> true, "",
                nowSeconds);
    }

    /**
     * Enables this measurement signal conditionally.
     *
     * @param enabled enabled supplier
     * @return configured measurement signal
     */
    default MeasurementSignal enabled(BooleanSupplier enabled) {
        return new ConfiguredMeasurementSignal(this, null, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, enabled,
                "");
    }

    /**
     * Adds a debug name to this signal.
     *
     * @param name debug name
     * @return configured measurement signal
     */
    default MeasurementSignal name(String name) {
        return new ConfiguredMeasurementSignal(this, null, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, null,
                name);
    }
}
