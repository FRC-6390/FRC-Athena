package ca.frc6390.athena.localization.ref;

import java.util.Optional;

import ca.frc6390.athena.runtime.measurement.Measurement;
import ca.frc6390.athena.runtime.measurement.PoseMeasurementSample;

final class PoseSamples {
    private PoseSamples() {
    }

    static Optional<PoseSample> from(Measurement measurement) {
        if (!(measurement instanceof PoseMeasurementSample sample)) {
            return Optional.empty();
        }
        return Optional.of(new PoseSample(
                sample.pose(),
                sample.speeds(),
                sample.timestampSeconds(),
                sample.latencySeconds(),
                sample.targetCount(),
                sample.stdDevs()));
    }
}
