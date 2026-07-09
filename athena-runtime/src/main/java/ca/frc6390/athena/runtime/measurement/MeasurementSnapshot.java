package ca.frc6390.athena.runtime.measurement;

import java.util.List;
import java.util.Objects;
import java.util.Optional;

/**
 * Cached measurement signal snapshot owned by a runtime cycle.
 */
public final class MeasurementSnapshot implements MeasurementSignal {
    private final MeasurementSignal source;
    private List<Measurement> measurements = List.of();
    private Optional<Measurement> latest = Optional.empty();

    /**
     * Creates a snapshot for a signal.
     *
     * @param source source signal
     */
    public MeasurementSnapshot(MeasurementSignal source) {
        this.source = Objects.requireNonNull(source, "source");
    }

    /**
     * Refreshes this snapshot from the source signal.
     *
     * @return this snapshot
     */
    public MeasurementSnapshot refresh() {
        return refresh(source);
    }

    /**
     * Refreshes this snapshot from a temporary signal view.
     *
     * @param signal signal view
     * @return this snapshot
     */
    public MeasurementSnapshot refresh(MeasurementSignal signal) {
        Objects.requireNonNull(signal, "signal");
        List<Measurement> values = signal.measurements();
        measurements = values == null || values.isEmpty() ? List.of() : List.copyOf(values);
        Measurement newest = null;
        for (Measurement measurement : measurements) {
            if (newest == null || measurement.timestampSeconds() > newest.timestampSeconds()) {
                newest = measurement;
            }
        }
        latest = Optional.ofNullable(newest);
        return this;
    }

    @Override
    public List<Measurement> measurements() {
        return measurements;
    }

    @Override
    public Optional<Measurement> latestMeasurement() {
        return latest;
    }
}
