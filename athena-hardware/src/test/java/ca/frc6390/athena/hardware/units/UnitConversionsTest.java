package ca.frc6390.athena.hardware.units;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.units.Units;
import org.junit.jupiter.api.Test;

class UnitConversionsTest {

    @Test
    void sameDimensionCompareAcrossScalesOfOneQuantity() {
        assertTrue(UnitConversions.sameDimension(Units.Degrees, Units.Rotations));
        assertTrue(UnitConversions.sameDimension(Units.Inches, Units.Meters));
        assertFalse(UnitConversions.sameDimension(Units.Degrees, Units.Meters));
    }

    @Test
    void measureInConvertsAcrossUnitsOfOneDimension() {
        // Replaces the former UnitConversions.value/rate helpers: Measure.in is the single conversion path.
        assertEquals(0.3048, Units.Inches.of(12.0).in(Units.Meters), 1e-9);
        assertEquals(0.25, Units.Degrees.of(90.0).in(Units.Rotations), 1e-12);
        assertEquals(2.0, Units.RPM.of(120.0).in(Units.RotationsPerSecond), 1e-12);
        assertEquals(1.0, Units.DegreesPerSecond.of(360.0).in(Units.RotationsPerSecond), 1e-12);
    }

    @Test
    void sameDimensionRejectsMismatchedMeasures() {
        assertFalse(UnitConversions.sameDimension(Units.Degrees, Units.Meters));
    }

    @Test
    void factorMultipliesValuesBetweenUnitsOfOneDimension() {
        assertEquals(1.0 / 360.0, UnitConversions.factor(Units.Degrees, Units.Rotations), 1e-12);
        assertEquals(360.0, UnitConversions.factor(Units.Rotations, Units.Degrees), 1e-9);
        assertThrows(IllegalArgumentException.class,
                () -> UnitConversions.factor(Units.Degrees, Units.Inches));
    }

    @Test
    void motionKindFollowsTheUnitDimension() {
        assertEquals(MotionKind.ROTARY, MotionKind.of(Units.Degrees));
        assertEquals(MotionKind.LINEAR, MotionKind.of(Units.Inches));
        assertEquals(MotionKind.UNSPECIFIED, MotionKind.of(null));
        assertEquals(MotionKind.UNSPECIFIED, MotionKind.of(Units.Volts));
    }
}
