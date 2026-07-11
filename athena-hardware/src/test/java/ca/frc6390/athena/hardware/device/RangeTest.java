package ca.frc6390.athena.hardware.device;

import static org.junit.jupiter.api.Assertions.assertThrows;

import org.junit.jupiter.api.Test;

class RangeTest {
    @Test
    void rejectsNonFiniteBounds() {
        assertThrows(IllegalArgumentException.class, () -> Range.of(Double.NaN, 1.0));
        assertThrows(IllegalArgumentException.class, () -> Range.of(0.0, Double.NaN));
        assertThrows(IllegalArgumentException.class, () -> Range.of(Double.NEGATIVE_INFINITY, 1.0));
        assertThrows(IllegalArgumentException.class, () -> Range.of(0.0, Double.POSITIVE_INFINITY));
    }
}
