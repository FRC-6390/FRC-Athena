package ca.frc6390.athena.wpilib.controls;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

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

    @Test
    void squaringPreservesNegativeInputSign() {
        DoubleSupplier axis = new AxisSignal(() -> -0.6)
                .deadband(0.2)
                .squared()
                .toSupplier();

        assertEquals(-0.25, axis.getAsDouble(), 1.0e-9);
    }

    @Test
    void thresholdSignalsSupportHysteresis() {
        double[] raw = {0.0};
        ControlSignal threshold = new AxisSignal(() -> raw[0]).above(0.6, 0.4);

        assertFalse(ControlSignalTest.sample(threshold, 0.0));
        raw[0] = 0.7;
        assertTrue(ControlSignalTest.sample(threshold, 0.02));
        raw[0] = 0.5;
        assertTrue(ControlSignalTest.sample(threshold, 0.04));
        raw[0] = 0.3;
        assertFalse(ControlSignalTest.sample(threshold, 0.06));
    }

    @Test
    void belowThresholdSupportsHysteresisAtBothBoundaries() {
        double[] raw = {0.0};
        ControlSignal threshold = new AxisSignal(() -> raw[0]).below(-0.6, -0.4);

        assertFalse(ControlSignalTest.sample(threshold, 0.0));
        raw[0] = -0.6;
        assertFalse(ControlSignalTest.sample(threshold, 0.02));
        raw[0] = -0.61;
        assertTrue(ControlSignalTest.sample(threshold, 0.04));
        raw[0] = -0.4;
        assertFalse(ControlSignalTest.sample(threshold, 0.06));
    }

    @Test
    void insideOutsideAndProcessedThresholdsUseProcessedAxisValues() {
        double[] raw = {0.5};
        AxisSignal axis = new AxisSignal(() -> raw[0]).deadband(0.2).inverted();
        ControlSignal inside = axis.inside(0.4);
        ControlSignal outside = axis.outside(0.3);
        ControlSignal below = axis.below(-0.3);

        assertTrue(ControlSignalTest.sample(inside, 0.0));
        assertTrue(ControlSignalTest.sample(outside, 0.02));
        assertTrue(ControlSignalTest.sample(below, 0.04));
        raw[0] = 0.2;
        assertTrue(ControlSignalTest.sample(inside, 0.06));
        assertFalse(ControlSignalTest.sample(outside, 0.08));
        assertFalse(ControlSignalTest.sample(below, 0.10));
    }

    @Test
    void thresholdArgumentsRejectNonFiniteMagnitudeAndInvalidHysteresis() {
        AxisSignal axis = new AxisSignal(() -> 0.0);

        assertThrows(IllegalArgumentException.class, () -> axis.above(Double.NaN));
        assertThrows(IllegalArgumentException.class, () -> axis.below(Double.POSITIVE_INFINITY));
        assertThrows(IllegalArgumentException.class, () -> axis.above(0.4, 0.5));
        assertThrows(IllegalArgumentException.class, () -> axis.below(-0.4, -0.5));
        assertThrows(IllegalArgumentException.class, () -> axis.inside(-0.1));
        assertThrows(IllegalArgumentException.class, () -> axis.outside(1.1));
    }
}
