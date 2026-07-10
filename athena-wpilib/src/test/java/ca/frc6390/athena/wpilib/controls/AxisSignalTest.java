package ca.frc6390.athena.wpilib.controls;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.function.DoubleSupplier;
import org.junit.jupiter.api.Test;

class AxisSignalTest {
    @Test
    void buildsAnImmutableAxisPipelineAndExposesItAsASupplier() {
        double[] raw = {0.6};
        AxisSignal rawAxis = new AxisSignal(() -> raw[0]);
        DoubleSupplier processed = rawAxis.deadband(0.2).squared().inverted().toSupplier();

        assertEquals(0.6, rawAxis.toSupplier().getAsDouble(), 1.0e-9);
        assertEquals(-0.25, processed.getAsDouble(), 1.0e-9);

        raw[0] = 0.1;
        assertEquals(0.0, processed.getAsDouble(), 1.0e-9);
    }

    @Test
    void clampsInvalidAndOutOfRangeInput() {
        double[] raw = {2.0};
        DoubleSupplier axis = new AxisSignal(() -> raw[0]).toSupplier();

        assertEquals(1.0, axis.getAsDouble(), 1.0e-9);
        raw[0] = Double.NaN;
        assertEquals(0.0, axis.getAsDouble(), 1.0e-9);
    }
}
