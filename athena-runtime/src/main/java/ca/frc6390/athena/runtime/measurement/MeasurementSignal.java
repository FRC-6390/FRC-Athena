package ca.frc6390.athena.runtime.measurement;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
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
     * Returns signals wrapped by this signal.
     *
     * <p>This preserves declaration provenance through filters and configuration so a
     * runtime can discover hardware-backed sources without evaluating measurements.</p>
     *
     * @return direct source signals
     */
    default List<MeasurementSignal> sources() {
        return List.of();
    }

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
     * @param translation translation standard deviation
     * @param heading heading standard deviation
     * @return configured measurement signal
     */
    default MeasurementSignal stdDevs(Distance translation, Angle heading) {
        return stdDevs(translation, translation, heading);
    }

    /**
     * Applies standard deviations directly without requiring a configuration object.
     *
     * @param xStdDev x standard deviation
     * @param yStdDev y standard deviation
     * @param headingStdDev heading standard deviation
     * @return configured measurement signal
     */
    default MeasurementSignal stdDevs(Distance xStdDev, Distance yStdDev, Angle headingStdDev) {
        return stdDevs(MeasurementStdDevs.of(xStdDev, yStdDev, headingStdDev));
    }

    /**
     * Filters measurements with latency above a limit.
     *
     * @param limit latency limit
     * @return configured measurement signal
     */
    default MeasurementSignal latencyLimit(Time limit) {
        return new ConfiguredMeasurementSignal(this, null, PoseSignals.seconds(limit), Double.POSITIVE_INFINITY, null, "");
    }

    /**
     * Filters measurements older than a limit.
     *
     * @param now current timestamp
     * @param maxAge max age
     * @return configured measurement signal
     */
    default MeasurementSignal maxAge(Time now, Time maxAge) {
        return new ConfiguredMeasurementSignal(this, null, Double.POSITIVE_INFINITY, PoseSignals.seconds(maxAge), () -> true, "",
                PoseSignals.seconds(now));
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
