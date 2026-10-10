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
    void valueConvertsMeasuresAndRejectsOtherDimensions() {
        assertEquals(0.3048, UnitConversions.value(Units.Inches.of(12.0), Units.Meters), 1e-9);
        assertEquals(0.25, UnitConversions.value(Units.Degrees.of(90.0), Units.Rotations), 1e-12);
        assertThrows(IllegalArgumentException.class,
                () -> UnitConversions.value(Units.Degrees.of(90.0), Units.Meters));
    }

    @Test
    void rateConvertsPerSecondMeasures() {
        assertEquals(2.0, UnitConversions.rate(Units.RPM.of(120.0), Units.Rotations), 1e-12);
        assertEquals(1.0, UnitConversions.rate(Units.DegreesPerSecond.of(360.0), Units.Rotations), 1e-12);
        assertThrows(IllegalArgumentException.class,
                () -> UnitConversions.rate(Units.Degrees.of(90.0), Units.Rotations));
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
