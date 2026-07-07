package ca.frc6390.athena.runtime.measurement;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.runtime.filter.PoseSnapshot;
import org.junit.jupiter.api.Test;

class MeasurementRefTest {
    @Test
    void singleMeasurementsAreMeasurementRefs() {
        MeasurementRef ref = Measurements.pose(new PoseSnapshot(1.0, 2.0, 0.5))
                .timing(4.0, 0.1)
                .stdDevs(MeasurementStdDevs.of(0.5, 0.5, 1.0));

        assertEquals(1, ref.measurements().size());
        PoseMeasurement measurement = (PoseMeasurement) ref.measurements().get(0);
        assertEquals(4.0, measurement.timestampSeconds(), 1.0e-9);
        assertEquals(0.25, measurement.stdDevs().translationVariance(), 1.0e-9);
    }

    @Test
    void configuredRefsFilterAndAnnotatePoseMeasurements() {
        MeasurementRef ref = Measurements.measurement(() -> Measurements
                .pose(new PoseSnapshot(1.0, 2.0, 0.0))
                .timing(10.0, 0.2))
                .latencyLimit(0.5)
                .stdDevs(MeasurementStdDevs.of(2.0, 2.0, 1.0));

        assertTrue(ref.latestMeasurement().orElseThrow() instanceof PoseMeasurement);
        PoseMeasurement measurement = (PoseMeasurement) ref.latestMeasurement().orElseThrow();
        assertEquals(4.0, measurement.stdDevs().translationVariance(), 1.0e-9);
    }
}
