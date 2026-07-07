package ca.frc6390.athena.runtime.measurement;

import java.util.Comparator;
import java.util.List;
import java.util.Optional;
import java.util.function.BooleanSupplier;

/**
 * Source of runtime measurements that can feed localization, diagnostics, or custom estimators.
 */
public interface MeasurementRef {
    /**
     * Returns current measurements from this source.
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
        return measurements().stream().max(Comparator.comparingDouble(Measurement::timestampSeconds));
    }

    /**
     * Applies standard deviations to pose measurements from this source.
     *
     * @param stdDevs standard deviations
     * @return configured measurement ref
     */
    default MeasurementRef stdDevs(MeasurementStdDevs stdDevs) {
        return new ConfiguredMeasurementRef(this, stdDevs, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, null, "");
    }

    /**
     * Filters measurements with latency above a limit.
     *
     * @param seconds latency limit
     * @return configured measurement ref
     */
    default MeasurementRef latencyLimit(double seconds) {
        return new ConfiguredMeasurementRef(this, null, seconds, Double.POSITIVE_INFINITY, null, "");
    }

    /**
     * Filters measurements older than a limit.
     *
     * @param nowSeconds current timestamp
     * @param seconds max age
     * @return configured measurement ref
     */
    default MeasurementRef maxAge(double nowSeconds, double seconds) {
        return new ConfiguredMeasurementRef(this, null, Double.POSITIVE_INFINITY, seconds, () -> true, "", nowSeconds);
    }

    /**
     * Enables this measurement source conditionally.
     *
     * @param enabled enabled supplier
     * @return configured measurement ref
     */
    default MeasurementRef enabled(BooleanSupplier enabled) {
        return new ConfiguredMeasurementRef(this, null, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, enabled, "");
    }

    /**
     * Adds a debug name to this source.
     *
     * @param name debug name
     * @return configured measurement ref
     */
    default MeasurementRef name(String name) {
        return new ConfiguredMeasurementRef(this, null, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, null, name);
    }
}
