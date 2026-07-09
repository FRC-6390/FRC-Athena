package ca.frc6390.athena.runtime.measurement;

import java.util.List;
import java.util.Objects;
import java.util.Optional;
import java.util.function.Supplier;

final class SingleMeasurementSignal implements MeasurementSignal {
    private final Supplier<? extends Measurement> measurement;

    SingleMeasurementSignal(Supplier<? extends Measurement> measurement) {
        this.measurement = Objects.requireNonNull(measurement, "measurement");
    }

    @Override
    public List<Measurement> measurements() {
        Measurement value = measurement.get();
        return value == null ? List.of() : List.of(value);
    }

    @Override
    public Optional<Measurement> latestMeasurement() {
        return Optional.ofNullable(measurement.get());
    }
}
