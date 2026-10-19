package ca.frc6390.athena.hardware.device;

import edu.wpi.first.units.Units;
import static org.junit.jupiter.api.Assertions.assertThrows;

import org.junit.jupiter.api.Test;

class RangeTest {
    @Test
    void rejectsNonFiniteBounds() {
        assertThrows(IllegalArgumentException.class, () -> Range.of(Units.Rotations.of(Double.NaN), Units.Rotations.of(1.0)));
        assertThrows(IllegalArgumentException.class, () -> Range.of(Units.Rotations.of(0.0), Units.Rotations.of(Double.NaN)));
        assertThrows(IllegalArgumentException.class, () -> Range.of(Units.Rotations.of(Double.NEGATIVE_INFINITY), Units.Rotations.of(1.0)));
        assertThrows(IllegalArgumentException.class, () -> Range.of(Units.Rotations.of(0.0), Units.Rotations.of(Double.POSITIVE_INFINITY)));
    }
}
