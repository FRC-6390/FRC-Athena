package ca.frc6390.athena.runtime.measurement;

import java.util.List;

/**
 * Single runtime measurement sample.
 */
public interface Measurement extends MeasurementSignal {
    /**
     * Returns the measurement timestamp.
     *
     * @return timestamp in seconds
     */
    double timestampSeconds();

    /**
     * Returns the measurement latency.
     *
     * @return latency in seconds
     */
    double latencySeconds();

    /**
     * Returns the object that produced this measurement.
     *
     * @return source object
     */
    Object source();

    @Override
    default List<Measurement> measurements() {
        return List.of(this);
    }
}
